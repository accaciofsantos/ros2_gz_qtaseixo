#!/usr/bin/env python3
import numpy as np  # se ainda não estiver no topo
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
import csv
import os
import random
import math

class IdentifyUGVNode(Node):
    def __init__(self):
        super().__init__('identify_ugv_node')

        # --- Sementes fixas para PRBS reprodutível ---
        self.rng_lin = np.random.RandomState(42)   # semente do movimento linear
        self.rng_ang = np.random.RandomState(123)  # semente do movimento angular

        # --- Parâmetros configuráveis ---
        self.declare_parameter('tipo', 'prbs')            # tipo de sinal
        self.declare_parameter('output_file', '')         # nome do CSV
        self.declare_parameter('seed', 42)                # semente fixa

        # --- Lê parâmetros ---
        self.tipo = self.get_parameter('tipo').value
        output_file = self.get_parameter('output_file').value
        self.seed = self.get_parameter('seed').value

        # --- Configuração de tempo ---
        self.dt = 0.1       # 10 Hz
        self.duration = 30.0
        self.freq = 0.2

        # --- Sementes independentes ---
        self.rng_lin = random.Random(self.seed)
        self.rng_ang = random.Random(self.seed + 100)

        self.get_logger().info(f'Iniciando identificação com sinal "{self.tipo}"')
        self.get_logger().info(f'Sementes PRBS -> Linear: {self.seed}, Angular: {self.seed + 100}')

        # --- Define nome do CSV ---
        if output_file == '':
            output_file = f'data_ugv_{self.tipo}.csv'
        os.makedirs('data', exist_ok=True)
        self.file_path = os.path.join('data', output_file)

        # --- Arquivo CSV ---
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            't', 'u_linear', 'u_angular',
            'ugv_pos_x', 'ugv_pos_y', 'ugv_pos_z', 'ugv_yaw',
            'ugv_gps_lat', 'ugv_gps_lon', 'ugv_gps_alt',
            'imu_lin_acc_x', 'imu_lin_acc_y', 'imu_lin_acc_z',
            'imu_ang_vel_x', 'imu_ang_vel_y', 'imu_ang_vel_z'
        ])

        # --- Publisher e subscriptions ---
        self.publisher_ = self.create_publisher(Twist, '/ugv/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/model/ugv/pose', self.listener_ugv_pose, 10)
        self.create_subscription(Imu, '/ugv/imu', self.listener_imu, 10)
        self.create_subscription(NavSatFix, '/ugv/gps', self.listener_gps, 10)

        # --- Estados internos ---
        self.last_u_lin = 0.0
        self.last_u_ang = 0.0
        self.ugv_pos = [0.0, 0.0, 0.0]
        self.ugv_yaw = 0.0
        self.ugv_gps = [0.0, 0.0, 0.0]
        self.imu_lin_acc = [0.0, 0.0, 0.0]
        self.imu_ang_vel = [0.0, 0.0, 0.0]

        # --- Timer ---
        self.t = 0.0
        self.counter = 0
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(f'CSV: {self.file_path}')

    # --- Timer principal ---
    def timer_callback(self):
        self.t += self.dt
        if self.t > self.duration:
            self.get_logger().info('teste concluído! Parando o robô...')
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.publisher_.publish(stop)
            self.csv_file.close()
            self.destroy_node()
            return

        self.generate_excitation(self.counter)
        self.counter += 1

        msg = Twist()
        msg.linear.x = self.last_u_lin
        msg.angular.z = self.last_u_ang
        self.publisher_.publish(msg)

        self.writer.writerow([
            self.t, self.last_u_lin, self.last_u_ang,
            *self.ugv_pos, self.ugv_yaw,
            *self.ugv_gps,
            *self.imu_lin_acc,
            *self.imu_ang_vel
        ])

        print(f"[t={self.t:.2f}] x={self.ugv_pos[0]:.3f}, y={self.ugv_pos[1]:.3f}")

    # --- Callbacks ---
    def listener_ugv_pose(self, msg: PoseStamped):
        valid_frames = [
            "variable_friction_wolrd",
        ]
        if msg.header.frame_id not in valid_frames:
            return
        self.ugv_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.ugv_yaw = self.quaternion_to_yaw(msg.pose.orientation)

    def listener_imu(self, msg: Imu):
        self.imu_lin_acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        self.imu_ang_vel = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

    def listener_gps(self, msg: NavSatFix):
        self.ugv_gps = [msg.latitude, msg.longitude, msg.altitude]

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = IdentifyUGVNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()