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

        # --- Subscriptions & Publisher ---
        self.pose_sub = self.create_subscription(PoseStamped, '/model/ugv/pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/ugv/cmd_vel', 10)

        # --- States ---
        self.position = np.zeros(2)      # [x, y]
        self.yaw = 0.0

        # --- Observer (Luenberger) ---
        self.x_hat = np.zeros(3)        # estimativa inicial [x, y, yaw]
        self.L = np.array([0.8, 0.8, 1.0])  # ganhos do observador (ajustáveis)
        self.dt = 0.1                    # mesmo que timer

        # --- Goals ---
        self.goals = [
            (5.0, 5.0),
            (5.0, -5.0),
            (-5.0, 5.0),
            (-5.0, -5.0),
        ]
        self.current_goal_index = 0
        self.all_goals_reached = False

        # --- Control gains ---
        self.kp_linear = 1.0
        self.kp_angular = 2.0

        # --- CSV Logging ---
        csv_dir = os.path.expanduser('~/Desktop/UGV_ADRC/data')
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = os.path.join(csv_dir, 'ugv_log2.csv')

        self.csv_fields = ['timestamp','x_real','y_real','yaw_real','x_hat','y_hat','yaw_hat','v_cmd','w_cmd']
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_fields)

        # --- Timer ---
        self.timer = self.create_timer(self.dt, self.control_loop)

    def pose_callback(self, msg: PoseStamped):
        # Filtra apenas mensagens do frame desejado
        if msg.header.frame_id != "variable_friction_world":
            return  # ignora outras mensagens

        # Atualiza estados reais
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        # --- Controle ---
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

        # --- Controle ---
        v_cmd = max(min(self.kp_linear * dist, 1.0), -1.0)
        w_cmd = max(min(self.kp_angular * angle_err, 1.0), -1.0)

        # Para distâncias muito pequenas, zera velocidade
        if dist < 0.01:
            v_cmd = 0.0
            w_cmd = 0.0

        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        if dist < 0.5:
            self.get_logger().info(f'🏁 Goal {self.current_goal_index} alcançado')
            self.current_goal_index += 1

        # --- Observador de Luenberger ---
        u = np.array([v_cmd, w_cmd])
        y = np.array([self.position[0], self.position[1], self.yaw])
        self.luenberger_observer(u, y)

        # --- Logging ---
        self.log_csv(y, self.x_hat, u)

    def luenberger_observer(self, u, y):
        """
        x_hat(k+1) = x_hat(k) + dt*( f(x_hat,u) + L*(y - y_hat) )
        Para robô diferencial simples (v, w):
        dx = v*cos(yaw)
        dy = v*sin(yaw)
        dyaw = w
        """
        x, y_pos, yaw = self.x_hat
        v, w = u

        # Dinâmica estimada
        dx_hat = v * math.cos(yaw)
        dy_hat = v * math.sin(yaw)
        dyaw_hat = w

        y_hat = self.x_hat.copy()  # evita alteração do vetor original
        y_error = y - y_hat
        y_error[2] = self.normalize_angle(y_error[2])  # normaliza erro de yaw


        # Atualiza estimativa
        self.x_hat[0] += self.dt * (dx_hat + self.L[0]*y_error[0])
        self.x_hat[1] += self.dt * (dy_hat + self.L[1]*y_error[1])
        self.x_hat[2] += self.dt * (dyaw_hat + self.L[2]*y_error[2])
        self.x_hat[2] = self.normalize_angle(self.x_hat[2])

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def log_csv(self, y_real, x_hat, u):
        ts = time()  # segundos desde epoch
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
