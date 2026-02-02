#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import argparse
import math

class UGVExcitationNode(Node):
    def __init__(self, mode='prbs', duration=60.0, freq=5.0, v_amp=1.0, w_amp=0.6):
        super().__init__('ugv_excitation_node')
        self.publisher = self.create_publisher(Twist, '/ugv/cmd_vel', 10)

        # --- Parâmetros gerais ---
        self.mode = mode
        self.duration = duration
        self.freq = freq
        self.dt = 1.0 / freq
        self.v_amp = v_amp
        self.w_amp = w_amp
        self.step = 0

        # --- Inicializa a sequência PRBS, se for o caso ---
        self.N = int(duration * freq)
        np.random.seed(42)
        self.prbs_v = self.v_amp * np.random.choice([-1, 1], self.N)
        self.prbs_w = self.w_amp * np.random.choice([-1, 1], self.N)

        self.get_logger().info(f"🚜 UGV Excitation Node iniciado - modo: {mode}")
        self.get_logger().info(f"Duração: {duration:.1f}s | Freq: {freq:.1f}Hz | v_amp={v_amp} | w_amp={w_amp}")
        self.timer = self.create_timer(self.dt, self.timer_callback)

    def timer_callback(self):
        t = self.step * self.dt
        if t >= self.duration:
            self.get_logger().info("✅ Excitação concluída. Enviando parada...")
            stop = Twist()
            self.publisher.publish(stop)
            rclpy.shutdown()
            return

        msg = Twist()

        if self.mode == 'prbs':
            msg.linear.x = float(self.prbs_v[self.step])
            msg.angular.z = float(self.prbs_w[self.step])

        elif self.mode == 'reta':
            msg.linear.x = self.v_amp
            msg.angular.z = 0.0

        elif self.mode == 'curva':
            msg.linear.x = self.v_amp
            msg.angular.z = self.w_amp

        elif self.mode == 'zigzag':
            # alterna sentido de giro a cada 5 segundos
            period = 5.0
            sign = 1 if (int(t / period) % 2 == 0) else -1
            msg.linear.x = self.v_amp
            msg.angular.z = sign * self.w_amp

        elif self.mode == 'senoidal':
            msg.linear.x = self.v_amp * math.sin(0.5 * t)
            msg.angular.z = self.w_amp * math.sin(0.2 * t)

        else:
            self.get_logger().warn(f"Modo '{self.mode}' não reconhecido!")
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f"[{t:.1f}s] v={msg.linear.x:.2f}, w={msg.angular.z:.2f}")
        self.step += 1


def main(args=None):
    parser = argparse.ArgumentParser(description='UGV Excitation Node - ROS 2')
    parser.add_argument('--mode', type=str, default='prbs',
                        help="Tipo de excitação: prbs | reta | curva | zigzag | senoidal")
    parser.add_argument('--duration', type=float, default=60.0, help="Duração do experimento [s]")
    parser.add_argument('--freq', type=float, default=5.0, help="Frequência de atualização [Hz]")
    parser.add_argument('--v_amp', type=float, default=1.0, help="Amplitude linear máxima [m/s]")
    parser.add_argument('--w_amp', type=float, default=0.6, help="Amplitude angular máxima [rad/s]")
    parsed_args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = UGVExcitationNode(
        mode=parsed_args.mode,
        duration=parsed_args.duration,
        freq=parsed_args.freq,
        v_amp=parsed_args.v_amp,
        w_amp=parsed_args.w_amp
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
