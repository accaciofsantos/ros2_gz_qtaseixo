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
        super().__init__('ugv_adrc_node')

        # ------------------------
        # ROS 2 PARAMETERS
        # ------------------------
        self.declare_parameter("controller_type", "adrc")
        self.declare_parameter("csv_name", "ugv_log_adrc.csv")
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
            'x_real', 'y_real', 'yaw_real',
            'x_hat', 'y_hat', 'yaw_hat',
            'x_ref', 'y_ref', 'yaw_ref',
            'dist_error', 'angle_error',
            'v_cmd', 'w_cmd',
            'eso_v_x1', 'eso_v_x2',
            'eso_w_x1', 'eso_w_x2'
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
        self.position = np.zeros(2)
        self.yaw = 0.0

        # Luenberger observer
        self.x_hat = np.zeros(3)
        self.L = np.array([0.8, 0.8, 1.0])
        self.dt = 0.01

        # ------------------------
        # Goals
        # ------------------------
        self.goals = self.define_goals(goal_pattern)
        self.current_goal_index = 0
        self.all_goals_reached = False

        # ------------------------
        # PD gains for ADRC inner loop
        # ------------------------
        self.kp_linear = 1.0
        self.kd_linear = 0.1
        self.kp_angular = 2.0
        self.kd_angular = 0.2

        self.prev_dist_err = 0.0
        self.prev_angle_err = 0.0

        # ------------------------
        # ADRC ESO states
        # ------------------------
        # linear channel (v)
        self.eso_v_x1 = 0.0
        self.eso_v_x2 = 0.0
        # angular channel (w)
        self.eso_w_x1 = 0.0
        self.eso_w_x2 = 0.0

        # previous commands
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0

        # plant gain (cinemático)
        self.b0_v = 1.0
        self.b0_w = 1.0

        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)

    # ------------------------
    # Goal definitions
    # ------------------------
    def define_goals(self, pattern):
        if pattern == "quadrado":
            return [(5.0, -5.0), (5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0)]
        elif pattern == "zigzag":
            return [(4.0, 2.0), (8.0, -2.0), (12.0, 2.0), (16.0, -2.0)]
        elif pattern == "linha_reta":
            return [(2.0, 2.0), (4.0, 4.0), (6.0, 6.0), (8.0, 8.0)]
        elif pattern == "circulo":
            return [(5.0*math.cos(theta), 5.0*math.sin(theta)) 
                    for theta in np.linspace(0, 2*math.pi, 8, endpoint=False)]
        else:
            self.get_logger().warn(f"⚠️ Goal pattern '{pattern}' não reconhecido. Usando (0,0).")
            return [(0, 0)]

    # ------------------------
    # Pose callback
    # ------------------------
    def pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "variable_friction_world":
            return
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        q = msg.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    # ------------------------
    # Main control loop
    # ------------------------
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

        v_cmd, w_cmd = self.adrc_controller(dist, angle_err)

        # reduzir linear se ângulo grande
        if abs(angle_err) > math.pi/6:
            v_cmd *= 0.5

        # publicar comando
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        # verificar objetivo
        if dist < 0.5:
            self.get_logger().info(f"🏁 Goal {self.current_goal_index} reached")
            self.current_goal_index += 1

        # atualizar Luenberger observer
        self.luenberger_observer(np.array([v_cmd, w_cmd]),
                                 np.array([self.position[0], self.position[1], self.yaw]))

        # log CSV
        self.log_csv(np.array([self.position[0], self.position[1], self.yaw]),
                     dist, angle_err)

    # ------------------------
    # ADRC controller (ideal)
    # ------------------------
    def adrc_controller(self, dist, angle_err):
        # -------- ESO linear (v) 2ª ordem --------
        r_v = 2.0
        beta1_v = 2*r_v
        beta2_v = r_v**2

        e_v = dist - self.eso_v_x1
        self.eso_v_x1 += self.dt*(self.eso_v_x2 + beta1_v*e_v)
        self.eso_v_x2 += self.dt*(self.prev_v_cmd + beta2_v*e_v)

        # -------- ESO angular (w) 2ª ordem --------
        r_w = 10.0
        beta1_w = 2*r_w
        beta2_w = r_w**2

        e_w = self.shortest_angle(angle_err, self.eso_w_x1)
        self.eso_w_x1 += self.dt*(self.eso_w_x2 + beta1_w*e_w)
        self.eso_w_x2 += self.dt*(self.prev_w_cmd + beta2_w*e_w)
        self.eso_w_x1 = self.normalize_angle(self.eso_w_x1)

        # -------- controle PD interno --------
        angle_error_derivative = self.shortest_angle(angle_err, self.prev_angle_err)/self.dt
        u0_v = self.kp_linear*dist + self.kd_linear*((dist - self.prev_dist_err)/self.dt)
        u0_w = self.kp_angular*angle_err + self.kd_angular*angle_error_derivative

        # -------- lei ADRC ideal --------
        v_cmd = np.clip((u0_v - self.eso_v_x2)/self.b0_v, -1.0, 1.0)
        w_cmd = np.clip((u0_w - self.eso_w_x2)/self.b0_w, -1.0, 1.0)

        self.prev_v_cmd = v_cmd
        self.prev_w_cmd = w_cmd
        self.prev_dist_err = dist
        self.prev_angle_err = angle_err

        return v_cmd, w_cmd

    # ------------------------
    # Luenberger observer
    # ------------------------
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

    # ------------------------
    # CSV logging
    # ------------------------
    def log_csv(self, y_real, dist_err, angle_err):
        ts = time()
        if self.current_goal_index < len(self.goals):
            x_ref, y_ref = self.goals[self.current_goal_index]
            yaw_ref = math.atan2(y_ref - self.position[1], x_ref - self.position[0])
        else:
            x_ref, y_ref = self.position[0], self.position[1]
            yaw_ref = self.yaw

        row = [
            ts,
            *y_real,
            *self.x_hat,
            x_ref, y_ref, yaw_ref,
            dist_err, angle_err,
            self.prev_v_cmd, self.prev_w_cmd,
            self.eso_v_x1, self.eso_v_x2,
            self.eso_w_x1, self.eso_w_x2
        ]
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    # ------------------------
    # Helpers
    # ------------------------
    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2*math.pi
        while angle < -math.pi: angle += 2*math.pi
        return angle

    def shortest_angle(self, target, current):
        diff = target - current
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        return diff

# ------------------------
# Main
# ------------------------
def main(args=None):
    rclpy.init(args=args)
    node = UGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
