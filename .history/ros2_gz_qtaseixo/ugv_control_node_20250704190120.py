#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class UGVController(Node):
    def __init__(self):
        super().__init__('ugv_control_node')

        # Subscription
        self.pose_sub = self.create_subscription(
            PoseStamped, '/model/ugv/pose', self.pose_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/ugv/cmd_vel', 10)

        # State
        self.position = None
        self.yaw = None

        # Goals
        self.goals = [
            (-35.0, 0.0),
            (-35.0, -10.0),    
            (-47.0, -10.0),
            (-47.0, 0.0),
        ]
        self.current_goal_index = 0
        self.all_goals_reached = False  # ✅ nova flag

        # Control gains
        self.kp_linear = 1.0
        self.kp_angular = 2.0

        # Timer loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg: PoseStamped):
        self.position = (msg.pose.position.x, msg.pose.position.y)

        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        self.get_logger().info(
            f"Pose recebida: pos=({self.position[0]:.2f}, {self.position[1]:.2f}), yaw={self.yaw:.2f}"
        )

    def control_loop(self):
        if self.position is None or self.yaw is None:
            return

        if self.all_goals_reached:
            # ✅ Para o robô ao final
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.cmd_pub.publish(stop_twist)
            self.get_logger().info('✅ Todos os goals alcançados. Parando o robô.')
            return

        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('✅ Todos os goals alcançados. Parando o robô.')
            self.all_goals_reached = True
            return

        goal = self.goals[self.current_goal_index]
        dx = goal[0] - self.position[0]
        dy = goal[1] - self.position[1]
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_err = self.normalize_angle(angle_to_goal - self.yaw)

        twist = Twist()
        twist.linear.x = self.kp_linear * dist
        twist.angular.z = self.kp_angular * angle_err

        twist.linear.x = max(min(twist.linear.x, 1.0), -1.0)
        twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(twist)

        if dist < 0.5:
            self.get_logger().info(f'🏁 Goal {self.current_goal_index} alcançado')
            self.current_goal_index += 1

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = UGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
