#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
import csv
import os
import numpy as np
from time import time

class UGVController(Node):
    def __init__(self):
        super().__init__('ugv_control_node')

        # ------------------------
        # ROS 2 PARAMETERS
        # ------------------------
        self.declare_parameter("controller_type", "pd")
        self.declare_parameter("csv_name", "ugv_log.csv")
        self.declare_parameter("goal_pattern", "quadrado")

        self.controller_type = self.get_parameter("controller_type").value
        csv_name = self.get_parameter("csv_name").value
        goal_pattern = self.get_parameter("goal_pattern").value

        # ------------------------
        # CSV Logging
        # ------------------------
        csv_dir = os.path.expanduser('~/Desktop/UGV_ADRC/data')
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = os.path.join(csv_dir, csv_name)

        self.csv_fields = [
            'timestamp', 
            'x_real', 'y_real', 'yaw_real',        # estados reais
            'x_hat', 'y_hat', 'yaw_hat',           # estados estimados pelo Luenberger
            'x_ref', 'y_ref', 'yaw_ref',           # referência dos estados
            'dist_error', 'angle_error',           # erros
            'v_cmd', 'w_cmd',                      # comandos
            'eso_dist_x1', 'eso_dist_x2', 'eso_dist_x3',   # ESO linear
            'eso_ang_x1',  'eso_ang_x2',  'eso_ang_x3'    # ESO angular
        ]

        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_fields)

        self.get_logger().info(f"🟢 Controller Type: {self.controller_type}")
        self.get_logger().info(f"💾 CSV File: {self.csv_file}")
        self.get_logger().info(f"📍 Goal Pattern: {goal_pattern}")

        # ------------------------
        # Subscriptions & Publisher
        # ------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped, '/model/ugv/pose', self.pose_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/ugv/cmd_vel', 10)

        # ------------------------
        # States
        # ------------------------
        self.position = np.zeros(2)  # x, y
        self.yaw = 0.0

        # Luenberger Observer
        self.x_hat = np.zeros(3)
        self.L = np.array([0.8, 0.8, 1.0])
        self.dt = 0.01  # 20 Hz

        # ------------------------
        # Goals
        # ------------------------
        self.goals = self.define_goals(goal_pattern)
        self.current_goal_index = 0
        self.all_goals_reached = False

        # ------------------------
        # Controllers
        # ------------------------
        self.kp_linear = 1.0
        self.kd_linear = 0.1
        self.kp_angular = 2.0
        self.kd_angular = 0.2

        self.prev_dist_err = 0.0
        self.prev_angle_err = 0.0
        self.prev_time = None

        # ADRC ESO states
        self.eso_dist_x1 = 0.0
        self.eso_dist_x2 = 0.0
        self.eso_dist_x3 = 0.0
        self.eso_ang_x1  = 0.0
        self.eso_ang_x2  = 0.0
        self.eso_ang_x3  = 0.0
        self.prev_v_cmd  = 0.0
        self.prev_w_cmd  = 0.0

        # ------------------------
        # Control Loop Timer
        # ------------------------
        self.timer = self.create_timer(self.dt, self.control_loop)

    # ==============================
    # Goal definitions
    # ==============================
    def define_goals(self, pattern):
        if pattern == "quadrado":
            return [(5.0, -5.0), (5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0)]
        elif pattern == "zigzag":
            return [(2.0, 2.0), (4.0, -2.0), (6.0, 2.0), (8.0, -2.0)]
        elif pattern == "linha_reta":
            return [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0)]
        elif pattern == "circulo":
            return [(5.0*math.cos(theta), 5.0*math.sin(theta)) for theta in np.linspace(0, 2*math.pi, 8, endpoint=False)]
        else:
            self.get_logger().warn(f"⚠️ Goal pattern '{pattern}' não reconhecido. Usando quadrado.")
            return [(5.0, -5.0), (5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0)]

    # ==============================
    # Pose callback
    # ==============================
    def pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "variable_friction_world":
            return
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        q = msg.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    # ==============================
    # Control loop
    # ==============================
    def control_loop(self):
        if self.all_goals_reached or self.current_goal_index >= len(self.goals):
            self.all_goals_reached = True
            self.publish_stop()
            return

        goal = self.goals[self.current_goal_index]
        dx = goal[0] - self.position[0]
        dy = goal[1] - self.position[1]
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_err = self.shortest_angle(angle_to_goal, self.yaw)

        # --------------------------
        # Controller selection
        # --------------------------
        if self.controller_type == "pd":
            v_cmd, w_cmd = self.pd_controller(dist, angle_err)
        elif self.controller_type == "adrc":
            v_cmd, w_cmd = self.adrc_controller(dist, angle_err)
        else:
            self.get_logger().error(f"❌ Unknown controller type: {self.controller_type}")
            v_cmd, w_cmd = 0.0, 0.0

        # --------------------------
        # Reduce linear speed if angle error is high
        # --------------------------
        if abs(angle_err) > math.pi/6:
            v_cmd *= 0.5

        # --------------------------
        # Publish command
        # --------------------------
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        # --------------------------
        # Goal check
        # --------------------------
        if dist < 0.5:
            self.get_logger().info(f"🏁 Goal {self.current_goal_index} reached")
            self.current_goal_index += 1

        # --------------------------
        # Observer update
        # --------------------------
        u = np.array([v_cmd, w_cmd])
        y = np.array([self.position[0], self.position[1], self.yaw])
        self.luenberger_observer(u, y)

        # --------------------------
        # Logging
        # --------------------------
        self.log_csv(y, dist, angle_err)

    # ==============================
    # PD Controller
    # ==============================
    def pd_controller(self, dist, angle_err):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = now
            self.prev_dist_err = dist
            self.prev_angle_err = angle_err
            return 0.0, 0.0
        dt = now - self.prev_time
        if dt <= 0:
            dt = 1e-3

        d_dist = (dist - self.prev_dist_err) / dt
        d_angle = (angle_err - self.prev_angle_err) / dt

        v_cmd = np.clip(self.kp_linear*dist - self.kd_linear*d_dist, -1.0, 1.0)
        w_cmd = np.clip(self.kp_angular*angle_err - self.kd_angular*d_angle, -1.0, 1.0)

        self.prev_time = now
        self.prev_dist_err = dist
        self.prev_angle_err = angle_err
        return v_cmd, w_cmd

    # ==============================
    # ADRC Controller – corrigido para yaw
    # ==============================
    # ==============================
    # ADRC Controller – corrigido para yaw
    # ==============================
    def adrc_controller(self, dist, angle_err):
        # --------------------------
        # ESO parameters
        # --------------------------
        r_lin = 5.0
        beta1_lin = 3*r_lin
        beta2_lin = 3*r_lin**2
        beta3_lin = r_lin**3

        r_ang = 5.0
        beta1_ang = 3*r_ang
        beta2_ang = 3*r_ang**2
        beta3_ang = r_ang**3

        # --------------------------
        # ESO – linear
        # --------------------------
        e_v = dist - self.eso_dist_x1
        self.eso_dist_x1 += self.dt*(self.eso_dist_x2 + beta1_lin*e_v)
        self.eso_dist_x2 += self.dt*(self.prev_v_cmd + self.eso_dist_x3 + beta2_lin*e_v)
        self.eso_dist_x3 += self.dt*(beta3_lin*e_v)

        # --------------------------
        # ESO – angular (yaw) CORRIGIDO
        # --------------------------
        # Entrada: yaw real do robô (self.yaw)
        # x1 = yaw estimado, x2 = derivada estimada, x3 = distúrbio
        e_w = self.shortest_angle(self.yaw, self.eso_ang_x1)  # diferença entre yaw real e estimado
        self.eso_ang_x1 += self.dt * (self.eso_ang_x2 + beta1_ang * e_w)
        self.eso_ang_x2 += self.dt * (self.prev_w_cmd + self.eso_ang_x3 + beta2_ang * e_w)
        self.eso_ang_x3 += self.dt * (beta3_ang * e_w)
        
        # Opcional: limitar x3 para evitar saturação
        self.eso_ang_x3 = np.clip(self.eso_ang_x3, -1.0, 1.0)

        # --------------------------
        # Control law – linear
        # --------------------------
        u0_lin = self.kp_linear*dist + self.kd_linear*((dist - self.prev_dist_err)/self.dt)
        v_cmd = np.clip(u0_lin - self.eso_dist_x3, -1.0, 1.0)

        # --------------------------
        # Control law – angular
        # --------------------------
        # Use angle_err para PD, mas subtraia apenas o distúrbio estimado
        angle_error_derivative = self.shortest_angle(angle_err, self.prev_angle_err)/self.dt
        u0_ang = self.kp_angular*angle_err + self.kd_angular*angle_error_derivative
        w_cmd = np.clip(u0_ang - self.eso_ang_x3, -1.0, 1.0)

        # --------------------------
        # Update history
        # --------------------------
        self.prev_v_cmd = v_cmd
        self.prev_w_cmd = w_cmd
        self.prev_dist_err = dist
        self.prev_angle_err = angle_err

        return v_cmd, w_cmd



    # ==============================
    # Luenberger observer
    # ==============================
    def luenberger_observer(self, u, y):
        x, y_pos, yaw = self.x_hat
        v, w = u
        dx_hat = v*math.cos(yaw)
        dy_hat = v*math.sin(yaw)
        dyaw_hat = w

        y_hat = self.x_hat.copy()
        y_error = y - y_hat
        y_error[2] = self.shortest_angle(y[2], y_hat[2])

        self.x_hat[0] += self.dt*(dx_hat + self.L[0]*y_error[0])
        self.x_hat[1] += self.dt*(dy_hat + self.L[1]*y_error[1])
        self.x_hat[2] += self.dt*(dyaw_hat + self.L[2]*y_error[2])
        self.x_hat[2] = self.normalize_angle(self.x_hat[2])

    # ==============================
    # Utilities
    # ==============================
    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def log_csv(self, y_real, dist_err, angle_err):
        ts = time()

        # referência do estado
        if self.current_goal_index < len(self.goals):
            x_ref, y_ref = self.goals[self.current_goal_index]
            yaw_ref = math.atan2(y_ref - self.position[1], x_ref - self.position[0])
        else:
            x_ref, y_ref, yaw_ref = self.position[0], self.position[1], self.yaw

        row = [
            ts,
            *y_real,                  # x_real, y_real, yaw_real
            *self.x_hat,               # x_hat, y_hat, yaw_hat
            x_ref, y_ref, yaw_ref,     # referência dos estados
            dist_err, angle_err,       # erros
            self.prev_v_cmd, self.prev_w_cmd,
            self.eso_dist_x1, self.eso_dist_x2, self.eso_dist_x3,
            self.eso_ang_x1,  self.eso_ang_x2,  self.eso_ang_x3
        ]
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    # ==============================
    # Angle utilities
    # ==============================
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def shortest_angle(self, target, current):
        diff = target - current
        while diff > math.pi:
            diff -= 2*math.pi
        while diff < -math.pi:
            diff += 2*math.pi
        return diff

def main(args=None):
    rclpy.init(args=args)
    node = UGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
