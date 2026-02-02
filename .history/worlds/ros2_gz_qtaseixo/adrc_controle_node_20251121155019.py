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

        self.controller_type = self.get_parameter("controller_type").value
        csv_name = self.get_parameter("csv_name").value

        # ------------------------
        # CSV Logging
        # ------------------------
        csv_dir = os.path.expanduser('~/Desktop/UGV_ADRC/data')
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = os.path.join(csv_dir, csv_name)

        self.csv_fields = [
            'timestamp','x_real','y_real','yaw_real',
            'x_hat','y_hat','yaw_hat','v_cmd','w_cmd'
        ]

        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_fields)

        self.get_logger().info(f"🟢 Controller Type: {self.controller_type}")
        self.get_logger().info(f"💾 CSV File: {self.csv_file}")

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
        self.dt = 0.1

        # ------------------------
        # Goals
        # ------------------------

        self.goals = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, -1.0),
            (3.0, 1.0),
            (4.0, -1.0),
            (5.0, 0.0)
        ]
        
        #self.goals = [
        #    (5.0, -5.0),
        #    (5.0, 5.0),
        #    (-5.0, 5.0),
        #    (-5.0, -5.0),
        #]
        self.current_goal_index = 0
        self.all_goals_reached = False

        # ------------------------
        # Controllers
        # ------------------------

        # PD Gains
        self.kp_linear = 1.0
        self.kd_linear = 0.1
        
        self.kp_angular = 2.0
        self.kd_angular = 0.2

        # Para derivada do PD
        self.prev_dist = None
        self.prev_time = None

        # para o ADRC
        self.prev_v_cmd = 0.0
        self.prev_w_cmd = 0.0

        # ADRC ESO states – linear
        self.eso_dist_x1 = 0.0
        self.eso_dist_x2 = 0.0
        self.eso_dist_z  = 0.0

        # ADRC ESO states – angular
        self.eso_ang_x1 = 0.0
        self.eso_ang_x2 = 0.0
        self.eso_ang_z  = 0.0

        # ------------------------
        # Control Loop Timer
        # ------------------------
        self.timer = self.create_timer(self.dt, self.control_loop)

    # ================================================================
    # Callbacks
    # ================================================================
    def pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "variable_friction_world":
            return

        self.position = np.array([msg.pose.position.x, msg.pose.position.y])

        q = msg.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    # ================================================================
    # Main Control Loop
    # ================================================================
    def control_loop(self):
        if self.all_goals_reached:
            self.publish_stop()
            return

        if self.current_goal_index >= len(self.goals):
            self.all_goals_reached = True
            self.publish_stop()
            return

        goal = self.goals[self.current_goal_index]
        dx = goal[0] - self.position[0]
        dy = goal[1] - self.position[1]
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_err = self.normalize_angle(angle_to_goal - self.yaw)

        # ============================================================
        # SELECT CONTROLLER TYPE
        # ============================================================

        if self.controller_type == "pd":
            v_cmd, w_cmd = self.pd_controller(dist, angle_err)

        elif self.controller_type == "pd_obs":
            v_cmd, w_cmd = self.pd_controller_with_observer(dist, angle_err)

        elif self.controller_type == "adrc":
            v_cmd, w_cmd = self.adrc_controller(dist, angle_err)

        else:
            self.get_logger().error(f"❌ Unknown controller type: {self.controller_type}")
            v_cmd, w_cmd = 0.0, 0.0

        # Publish command
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        # Check if goal reached
        if dist < 0.5:
            self.get_logger().info(f"🏁 Goal {self.current_goal_index} reached")
            self.current_goal_index += 1

        # Luenberger observer always running
        u = np.array([v_cmd, w_cmd])
        y = np.array([self.position[0], self.position[1], self.yaw])
        self.luenberger_observer(u, y)

        # Logging
        self.log_csv(y, self.x_hat, u)

    # ================================================================
    # CONTROLLERS IMPLEMENTATION
    # ================================================================
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

        # ----- ERROS -----
        d_dist = (dist - self.prev_dist_err) / dt
        d_angle = (angle_err - self.prev_angle_err) / dt

        # ----- CONTROLE PROPORCIONAL -----
        v_p = self.kp_linear * dist
        w_p = self.kp_angular * angle_err

        # ----- CONTROLE DERIVATIVO -----
        v_d = -self.kd_linear * d_dist
        w_d = -self.kd_angular * d_angle

        # ----- COMBINADO -----
        v_cmd = v_p + v_d
        w_cmd = w_p + w_d

        # ----- LÓGICA DE PRIORIZAÇÃO -----
        # Quando erro angular > 30°, pare de avançar
        #if abs(angle_err) > np.deg2rad(30):
        #    v_cmd = 0.0

        # ----- SATURAÇÃO -----
        v_cmd = np.clip(v_cmd, -1.0, 1.0)
        w_cmd = np.clip(w_cmd, -1.0, 1.0)

        # ----- ATUALIZAÇÃO -----
        self.prev_time = now
        self.prev_dist_err = dist
        self.prev_angle_err = angle_err

        return v_cmd, w_cmd


    def pd_controller_with_observer(self, dist, angle_err):
        """
        Faz o mesmo PD, mas usando a estimativa do observador (Luenberger)
        ao invés dos estados reais.
        """

        # ---- Estimativas ----
        x_hat, y_hat, yaw_hat = self.x_hat

        # ---- Posição do objetivo ----
        goal = self.goals[self.current_goal_index]
        gx, gy = goal

        # ---- Erros estimados ----
        dx_hat = gx - x_hat
        dy_hat = gy - y_hat
        dist_hat = math.hypot(dx_hat, dy_hat)
        angle_to_goal_hat = math.atan2(dy_hat, dx_hat)
        angle_err_hat = self.normalize_angle(angle_to_goal_hat - yaw_hat)

        # ---- Chama PD normal, mas com dist_hat e angle_err_hat ----
        return self.pd_controller(dist_hat, angle_err_hat)

    def adrc_controller(self, dist, angle_err):
        """
        ADRC para robô diferencial Husky:
        - v_cmd baseado na distância ao goal
        - w_cmd baseado no erro angular
        - ESO de 2ª ordem para linear e angular
        """

        # ================================
        # 1) PD desejado
        # ================================
        # Linear
        v0 = self.kp_linear * dist - self.kd_linear * ((dist - getattr(self, 'prev_dist_err', dist)) / self.dt)
        # Angular
        w0 = self.kp_angular * angle_err - self.kd_angular * ((angle_err - getattr(self, 'prev_angle_err', angle_err)) / self.dt)

        # ================================
        # 2) Frequência natural do ESO
        # ================================
        r_linear = 5.0   # ajustável
        beta1_linear = 2 * r_linear
        beta2_linear = r_linear ** 2

        r_angular = 8.0  # ajustável
        beta1_angular = 2 * r_angular
        beta2_angular = r_angular ** 2

        # ================================
        # 3) ESO – Linear
        # ================================
        e_v = v0 - self.eso_dist_x1
        self.eso_dist_x1 += self.dt * (self.eso_dist_z + beta1_linear * e_v)
        self.eso_dist_z  += self.dt * (beta2_linear * e_v)

        # ================================
        # 4) Controle ADRC – Linear
        # ================================
        v_cmd = v0 - self.eso_dist_z
        v_cmd = np.clip(v_cmd, -1.0, 1.0)

        # ================================
        # 5) ESO – Angular
        # ================================
        e_w = w0 - self.eso_ang_x1
        self.eso_ang_x1 += self.dt * (self.eso_ang_z + beta1_angular * e_w)
        self.eso_ang_z  += self.dt * (beta2_angular * e_w)

        # ================================
        # 6) Controle ADRC – Angular
        # ================================
        w_cmd = w0 - self.eso_ang_z
        w_cmd = np.clip(w_cmd, -1.0, 1.0)

        # ================================
        # 7) Atualiza erros anteriores
        # ================================
        self.prev_dist_err = dist
        self.prev_angle_err = angle_err

        return v_cmd, w_cmd

    # ================================================================
    # OBSERVER
    # ================================================================
    def luenberger_observer(self, u, y):
        x, y_pos, yaw = self.x_hat
        v, w = u

        dx_hat = v * math.cos(yaw)
        dy_hat = v * math.sin(yaw)
        dyaw_hat = w

        y_hat = self.x_hat.copy()
        y_error = y - y_hat
        y_error[2] = self.normalize_angle(y_error[2])

        self.x_hat[0] += self.dt * (dx_hat + self.L[0]*y_error[0])
        self.x_hat[1] += self.dt * (dy_hat + self.L[1]*y_error[1])
        self.x_hat[2] += self.dt * (dyaw_hat + self.L[2]*y_error[2])
        self.x_hat[2] = self.normalize_angle(self.x_hat[2])

    # ================================================================
    # UTILITIES
    # ================================================================
    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def log_csv(self, y_real, x_hat, u):
        ts = time()
        row = [ts, *y_real, *x_hat, *u]
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = UGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
