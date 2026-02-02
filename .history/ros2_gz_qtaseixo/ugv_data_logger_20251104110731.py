#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
import csv
import os

class UGVDataLogger(Node):
    def __init__(self):
        super().__init__('ugv_data_logger')
        self.data_file = 'ugv_data.csv'
        self.start_time = self.get_clock().now().to_msg().sec_nanosec()[0]

        self.sub_pose = self.create_subscription(PoseStamped, '/model/ugv/pose', self.pose_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/ugv/imu', self.imu_cb, 10)
        self.sub_cmd = self.create_subscription(Twist, '/ugv/cmd_vel', self.cmd_cb, 10)

        self.last_pose = None
        self.last_imu = None
        self.last_cmd = None

        with open(self.data_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'vx', 'vy', 'ax', 'ay', 'yaw_rate', 'v_cmd', 'w_cmd'])

        self.timer = self.create_timer(0.1, self.save_data)
        self.get_logger().info(f"Registrando dados em {os.path.abspath(self.data_file)}")

    def pose_cb(self, msg): self.last_pose = msg.pose.position
    def imu_cb(self, msg): self.last_imu = msg
    def cmd_cb(self, msg): self.last_cmd = msg

    def save_data(self):
        if not (self.last_pose and self.last_imu and self.last_cmd):
            return
        t = self.get_clock().now().to_msg().sec_nanosec()[0] - self.start_time
        row = [
            t,
            self.last_pose.x, self.last_pose.y,
            self.last_imu.linear_acceleration.x,
            self.last_imu.linear_acceleration.y,
            self.last_imu.angular_velocity.z,
            self.last_cmd.linear.x,
            self.last_cmd.angular.z,
        ]
        with open(self.data_file, 'a', newline='') as f:
            csv.writer(f).writerow(row)

def main(args=None):
    rclpy.init(args=args)
    node = UGVDataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
