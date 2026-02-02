import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import random

class UGVExcitation(Node):
    def __init__(self):
        super().__init__('identify_ugv_node')

        # --- Parâmetros ROS ---
        self.declare_parameter('tipo', 'prbs')  # Tipo de excitação
        self.declare_parameter('output_file', 'data.csv')

        self.tipo = self.get_parameter('tipo').value
        self.output_file = self.get_parameter('output_file').value

        # --- Sementes independentes ---
        self.seed_linear = 42
        self.seed_angular = 123
        self.rng_linear = random.Random(self.seed_linear)
        self.rng_angular = random.Random(self.seed_angular)

        # --- Publicador ---
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- Estados PRBS ---
        self.counter = 0
        self.period_lin = 10   # período (em amostras) de variação do PRBS linear
        self.period_ang = 15   # período (em amostras) de variação do PRBS angular
        self.last_u_lin = 0.0
        self.last_u_ang = 0.0

        self.get_logger().info(f'Iniciando identificação com sinal "{self.tipo}"')
        self.get_logger().info(f'CSV: {self.output_file}')

    def prbs_step(self, counter, period, amplitude_range, rng):
        """Gera passo PRBS para o contador e intervalo fornecidos"""
        if counter % period == 0:
            return rng.uniform(*amplitude_range)
        return None  # sem mudança

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'identify'

        # --- Geração PRBS independente para linear e angular ---
        if self.tipo == 'prbs':
            new_lin = self.prbs_step(self.counter, self.period_lin, (0.25, 0.75), self.rng_linear)
            new_ang = self.prbs_step(self.counter, self.period_ang, (-0.75, 0.75), self.rng_angular)

            if new_lin is not None:
                self.last_u_lin = new_lin
            if new_ang is not None:
                self.last_u_ang = new_ang

            msg.twist.linear.x = self.last_u_lin
            msg.twist.angular.z = self.last_u_ang

        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0

        # --- Publica o comando ---
        self.pub_cmd.publish(msg)
        self.counter += 1

        # --- Log no terminal ---
        t = self.counter * 0.1
        self.get_logger().info(f"[t={t:.1f}s] u_lin={self.last_u_lin:.2f} u_ang={self.last_u_ang:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = UGVExcitation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
