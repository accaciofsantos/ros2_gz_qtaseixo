#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

class UAVController(Node):
    def __init__(self):
        super().__init__('uav_control_node')

        # Pose do UAV
        self.uav_pose = None

        # Publisher e Subscriber
        self.uav_cmd_pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.uav_pose_sub = self.create_subscription(
            PoseStamped, '/model/uav/pose', self.uav_pose_callback, 10)

        # Waypoints (XY), UAV voa sobre eles com Z constante
        self.goals = [
            (-35.0, 0.0),
            (-35.0, -10.0),
            (-47.0, -10.0),
            (-47.0, 0.0),
        ]
        self.goal_index = 0
        self.all_goals_reached = False

        # Parâmetros do voo
        self.kp_uav = np.array([0.5, 0.5, 0.5])
        self.altitude_target = 60.0

        # Timer principal
        self.timer = self.create_timer(0.1, self.control_loop)

    def uav_pose_callback(self, msg):
        self.uav_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def control_loop(self):
        if self.all_goals_reached or self.uav_pose is None:
            self.stop_uav()
            return

        if self.goal_index >= len(self.goals):
            self.get_logger().info('✅ Todos os goals alcançados.')
            self.all_goals_reached = True
            self.stop_uav()
            return

        goal_xy = self.goals[self.goal_index]
        target = np.array([goal_xy[0], goal_xy[1], self.altitude_target])
        error = target - self.uav_pose
        dist = np.linalg.norm(error)

        v = np.clip(self.kp_uav * error, -2.0, 2.0)

        twist = Twist()
        twist.linear.x = float(v[0])
        twist.linear.y = float(v[1])
        twist.linear.z = float(v[2])
        self.uav_cmd_pub.publish(twist)

        if dist < 0.5:
            self.get_logger().info(f'🏁 Goal {self.goal_index} alcançado.')
            self.goal_index += 1

    def stop_uav(self):
        stop = Twist()
        self.uav_cmd_pub.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = UAVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
