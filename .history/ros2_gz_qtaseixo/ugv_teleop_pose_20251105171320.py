#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import sys
import termios
import tty
import select

class UGVTeleopNode(Node):
    def __init__(self):
        super().__init__('ugv_teleop_node')

        # Publishers e subscribers
        self.publisher = self.create_publisher(Twist, '/ugv/cmd_vel', 10)
        self.subscription = self.create_subscription(
            PoseStamped, '/model/ugv/pose', self.ugv_pose_callback, 10)

        self.get_logger().info("UGV Teleop iniciado. Use W A S D para movimentar. SPACE para parar. Q para sair.")

        # Timer de loop de controle
        self.timer = self.create_timer(0.1, self.keyboard_loop)

        # Estado
        self.ugv_pose = None

    def ugv_pose_callback(self, msg):
        self.ugv_pose = (msg.pose.position.x, msg.pose.position.y)

    def keyboard_loop(self):
        key = self.get_key(timeout=0.1)
        if key is None:
            return

        twist = Twist()

        if key.lower() == 'w':
            twist.linear.x = 1.0
        elif key.lower() == 's':
            twist.linear.x = -1.0
        elif key.lower() == 'a':
            twist.angular.z = 1.0
        elif key.lower() == 'd':
            twist.angular.z = -1.0
        elif key == ' ':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif key.lower() == 'q':
            self.get_logger().info("Encerrando controle via teclado.")
            rclpy.shutdown()
            return

        self.publisher.publish(twist)

        if self.ugv_pose:
            x, y = self.ugv_pose
            self.get_logger().info(f"📍 Posição atual: x={x:.2f}, y={y:.2f}")

    def get_key(self, timeout=0.1):
        """Captura tecla com timeout sem bloquear o loop"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = UGVTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
