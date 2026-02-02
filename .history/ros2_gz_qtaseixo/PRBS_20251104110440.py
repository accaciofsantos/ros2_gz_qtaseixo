import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class PRBSPublisher(Node):
    def __init__(self):
        super().__init__('prbs_publisher')
        self.publisher_ = self.create_publisher(Twist, '/ugv/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.step_time = 1.0  # tempo entre trocas
        self.counter = 0
        self.sequence_v = self.generate_prbs(200, 3.0)
        self.sequence_w = self.generate_prbs(200, 1.0)

    def generate_prbs(self, length, amplitude):
        seq = (2*np.random.randint(0, 2, length) - 1) * amplitude
        return seq

    def timer_callback(self):
        idx = int(self.counter / (self.step_time / 0.1)) % len(self.sequence_v)
        msg = Twist()
        msg.linear.x = float(self.sequence_v[idx])
        msg.angular.z = float(self.sequence_w[idx])
        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = PRBSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
