#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import csv
import os
import numpy as np
import random

class IdentifyUGVNode(Node):
    def __init__(self):
        super().__init__('identify_ugv_node')

        # --- Parâmetros configuráveis ---
        self.declare_parameter('tipo', 'prbs')       # prbs, senoide, degrau, ruido
        self.declare_parameter('amplitude', 0.3)
        self.declare_parameter('frequencia', 0.2)    # Hz, se aplicável
        self.declare_parameter('duration', 30.0)     # segundos
        self.declare_parameter('output_file', 'data_ugv.csv')

        # --- Ler parâmetros ---
        self.tipo = self.get_parameter('tipo').value
        self.amplitude = self.get_parameter('amplitude').value
        self.freq = self.get_parameter('frequencia').value
        self.duration = self.get_parameter('duration').value
        self.output_file = self.get_parameter('output_file').value

        # --- Publisher e Subscriber ---
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            TwistStamped, '/a200_0000/velocity', self.listener_callback, 10)

        # --- Timer ---
        self.dt = 0.05  # período de amostragem (20 Hz)
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # --- Arquivo CSV ---
        os.makedirs('data', exist_ok=True)
        self.file_path = os.path.join('data', self.output_file)
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['t', 'vx', 'vy', 'r', 'u_input'])

        # --- Estado interno ---
        self.last_u = 0.0
        self.sign = 1
        self.counter = 0

        self.get_logger().info(f'Iniciando identificação com sinal "{self.tipo}"')

    # --- Função para gerar o sinal de excitação ---
    def generate_excitation(self, t):
        if self.tipo == 'senoide':
            return self.amplitude * np.sin(2 * np.pi * self.freq * t)

        elif self.tipo == 'degrau':
            return self.amplitude if t % (1/self.freq) < (0.5/self.freq) else -self.amplitude

        elif self.tipo == 'ruido':
            return self.amplitude * (2 * random.random() - 1)

        elif self.tipo == 'prbs':
            # PRBS: alterna o sinal após N amostras pseudoaleatórias
            if self.counter % int(1.0/(self.freq*self.dt)) == 0:
                self.sign = random.choice([-1, 1])
            self.counter += 1
            return self.amplitude * self.sign

        else:
            return 0.0

    # --- Timer principal ---
    def timer_callback(self):
        self.t += self.dt
        if self.t > self.duration:
            self.get_logger().info('Identificação concluída!')
            self.csv_file.close()
            self.destroy_node()
            return

        u = self.generate_excitation(self.t)

        msg = TwistStamped()
        msg.twist.linear.x = u
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)

        self.last_u = u

    # --- Callback de leitura dos estados ---
    def listener_callback(self, msg):
        self.writer.writerow([
            self.t,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z,
            self.last_u
        ])

def main(args=None):
    rclpy.init(args=args)
    node = IdentifyUGVNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
