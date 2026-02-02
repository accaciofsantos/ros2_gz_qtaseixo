#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class ADRCControlNode(Node):
    def __init__(self):
        super().__init__('adrc_control_node')

        # --- Ganhos ---
        # Malha externa (posição)
        self.Kp_x = 1.0
        self.Kp_y = 1.0
        self.Kp_yaw = 1.0

        # Malha interna (velocidade) -> P ou PD
        self.Kp_vx = 1.0
        self.Kp_vy = 1.0
        self.Kp_omega = 1.0
        self.Kd_vx = 0.1
        self.Kd_vy = 0.1
        self.Kd_omega = 0.1

        # --- Estados internos ---
        self.pose = [0.0, 0.0, 0.0]   # x, y, yaw
        self.velocities = [0.0, 0.0, 0.0]  # vx, vy, omega

        # --- Waypoints ---
        self.waypoints = [
            (5.0, 5.0, 0.0),
            (5.0, -5.0, 0.0),
            (-5.0, -5.0, 0.0),
            (-5.0, 5.0, 0.0)
        ]
        self.current_wp = 0
        self.wp_tolerance = 0.2  # distância para considerar waypoint alcançado

        # --- Referência inicial ---
        self.x_ref, self.y_ref, self.yaw_ref = self.waypoints[self.current_wp]

        # --- Publisher e subscriber ---
        self.pub_cmd = self.create_publisher(Twist, '/ugv/cmd_vel', 10)
        self.create_subscription(PoseStamped, '/model/ugv/pose', self.pose_callback, 10)

        # --- Timer ---
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

    def pose_callback(self, msg: PoseStamped):
        self.pose[0] = msg.pose.position.x
        self.pose[1] = msg.pose.position.y
        self.pose[2] = self.quaternion_to_yaw(msg.pose.orientation)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # --- Verifica waypoint atual ---
        dx = self.x_ref - self.pose[0]
        dy = self.y_ref - self.pose[1]
        dist = math.hypot(dx, dy)
        if dist < self.wp_tolerance:
            # Vai para o próximo waypoint
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)
            self.x_ref, self.y_ref, self.yaw_ref = self.waypoints[self.current_wp]
            self.get_logger().info(f"Próximo waypoint: ({self.x_ref}, {self.y_ref})")

        # --- Erro de posição ---
        ex = self.x_ref - self.pose[0]
        ey = self.y_ref - self.pose[1]
        eyaw = self.yaw_ref - self.pose[2]

        # Limita yaw entre -pi e pi
        while eyaw > math.pi:
            eyaw -= 2*math.pi
        while eyaw < -math.pi:
            eyaw += 2*math.pi

        # --- Malha externa (gera velocidade de referência) ---
        vx_ref = self.Kp_x * ex
        vy_ref = self.Kp_y * ey
        omega_ref = self.Kp_yaw * eyaw

        # --- Malha interna (PD simples) ---
        u_vx = self.Kp_vx * (vx_ref - self.velocities[0])
        u_vy = self.Kp_vy * (vy_ref - self.velocities[1])
        u_omega = self.Kp_omega * (omega_ref - self.velocities[2])

        # --- Atualiza velocidades internas (simula leitura) ---
        self.velocities[0] += u_vx * self.dt
        self.velocities[1] += u_vy * self.dt
        self.velocities[2] += u_omega * self.dt

        # --- Publica comando ---
        msg = Twist()
        msg.linear.x = self.velocities[0]
        msg.linear.y = self.velocities[1]
        msg.angular.z = self.velocities[2]
        self.pub_cmd.publish(msg)

        self.get_logger().info(
            f"Pose: ({self.pose[0]:.2f}, {self.pose[1]:.2f}, {self.pose[2]:.2f}) "
            f"-> Cmd: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, omega={msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ADRCControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
