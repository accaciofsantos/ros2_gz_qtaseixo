#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np

class UGVUAVController(Node):
    def __init__(self):
        super().__init__('ugv_uav_control_node')

        # === UGV ===
        self.ugv_pose = None
        self.ugv_yaw = None
        self.ugv_cmd_pub = self.create_publisher(Twist, '/ugv/cmd_vel', 10)
        self.ugv_pose_sub = self.create_subscription(
            PoseStamped, '/model/ugv/pose', self.ugv_pose_callback, 10)

        # === UAV ===
        self.uav_pose = None
        self.uav_cmd_pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        self.uav_pose_sub = self.create_subscription(
            PoseStamped, '/model/uav/pose', self.uav_pose_callback, 10)

        # === Goals compartilhados (X, Y) === % quinta do Queixo
        #self.goals = [
        #    (-30.00, 6.00),
        #    (-26.80, 22.57),    
        #    (-22.45, 34.61),
        #    (-19.00, 43.10),
        #    (-32.00, 51.54),
        #    (-35.43, 49.50),
        #    (-38.70, 45.80),
        #    (-42.89, 36.92),
        #    (-46.76, 25.78),
        #    (-46.60, 14.22),
        #    (-30.00, 6.00),
        #]
        # === Goals compartilhados (X, Y) === % orchard
        #self.goals = [
        #    (-0.00,-7.40),
        #    (03.00,-8.00),
        #    (18.00,-8.30),
        #    (33.58,-8.52),
        #    (33.58,-4.72),    
        #    (33.50,-5.13),
        #    (23.19,-4.72),
        #    (19.07,-4.72),
        #    (12.19,-4.62),
        #    (8.00,-5.00),
        #     (00.00,-5.00),
        # ]
        # === Goals compartilhados (X, Y) === % scenario1 - canoppy dense
        self.goals = [
            (-0.26,-0.29),
            (29.00,-6.00),
            (29.00,3.70),
            (-6.00,3.93), 
            (-6.00,12.14),
            (30.0,11.00),
            (30.0,22),
            (-9.00,22),  
        ]
        # === Goals compartilhados (X, Y) === % scenario1 - paper ICARA 2026
        #self.goals = [
        #    (-2.00, -2.00),
        #    (10.00, -2.00),
        #    (10.00, 2.34),
        #    (-2.00, 2.34),
        #   (-2.00, 6.3),
        #   (10.00, 6.3),
        #    (-2.00, 10.00),
        #    (-2.00, 14.00),
        #    (11.00,  14.00),
        #    (11.00,  -2.00),
        #    (-2.00,  -2.00),
        #]

        # === Goals compartilhados (X, Y) === % scenario1 - paper WSEAS 
        self.goals = [
             (-1.74,-2.50), #0
            (13.00,-2.50), #1
            (13.00,2.49), #2
            (-3.00,1.70), #3
            (-3.00,5.70), #4
            (13.00,6.00),#5
            (13.00,10.00), #6
            (-3.00,10.00), #7 
            (-3.00,13.00), #8
            (10.00,16.00), #9
            (13.00,10.00), #10
            (13.00,-2.50), #11
            (-1.74,-2.50)  #12
        ]
        self.goal_index = 0
        self.all_goals_reached = False

        # === Ganhos ===
        self.kp_ugv_linear = 1.0
        self.kp_ugv_angular = 2.0
        self.kp_uav = np.array([0.5, 0.5, 0.5])
        self.altitude_target = 8.0 #53.0 (quinta) # 3.0 (orchard) # 8.0 (scenario1) # 3.0 (scenario1 - ICARA)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def ugv_pose_callback(self, msg):
        self.ugv_pose = (msg.pose.position.x, msg.pose.position.y)

        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.ugv_yaw = math.atan2(siny, cosy)

    def uav_pose_callback(self, msg):
        self.uav_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.uav_yaw = math.atan2(siny, cosy)


    def control_loop(self):
        if self.all_goals_reached:
            self.stop_robots()
            return

        if self.goal_index >= len(self.goals):
            self.get_logger().info('✅ Todos os goals alcançados.')
            self.all_goals_reached = True
            self.stop_robots()
            return

        goal_xy = self.goals[self.goal_index]

        self.control_ugv(goal_xy)
        self.control_uav(goal_xy)

    def control_ugv(self, goal):
        if self.ugv_pose is None or self.ugv_yaw is None:
            return

        dx = goal[0] - self.ugv_pose[0]
        dy = goal[1] - self.ugv_pose[1]
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_err = self.normalize_angle(angle_to_goal - self.ugv_yaw)

        twist = Twist()
        twist.linear.x = self.kp_ugv_linear * dist
        twist.angular.z = self.kp_ugv_angular * angle_err
        twist.linear.x = max(min(twist.linear.x, 0.75), -0.75)
        twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)

        self.ugv_cmd_pub.publish(twist)

        # Checagem de distância (somente do UGV)
        if dist < 1.5:
            self.get_logger().info(f'🏁 Goal {self.goal_index} alcançado.')
            self.goal_index += 1

    def control_uav(self, goal):
        if self.uav_pose is None or self.uav_yaw is None:
            return

        # Objetivo 3D
        target = np.array([goal[0], goal[1], self.altitude_target])
        error_world = target - self.uav_pose  # Erro no frame do mundo
        dx, dy, dz = error_world

        # Controle de orientação (yaw)
        angle_to_goal = math.atan2(dy, dx)
        angle_err = self.normalize_angle(angle_to_goal - self.uav_yaw)
        angular_z = max(min(2.0 * angle_err, 1.0), -1.0)

        # Converte erro (dx, dy) para o frame local do UAV
        cos_yaw = math.cos(-self.uav_yaw)
        sin_yaw = math.sin(-self.uav_yaw)
        dx_local = cos_yaw * dx - sin_yaw * dy
        # dy_local = sin_yaw * dx + cos_yaw * dy  # não será usado (como UGV)

        # Move apenas para frente se bem orientado
        vx = self.kp_uav[0] * dx_local if abs(angle_err) < 0.4 else 0.0
        vx = max(min(vx, 2.0), -2.0)

        # Altitude (independente da orientação)
        vz = max(min(self.kp_uav[2] * dz, 2.0), -2.0)

        # Comando final
        twist = Twist()
        twist.linear.x = float(vx)     # Apenas x no frame local
        twist.linear.y = 0.0           # Não usamos lateral (como UGV)
        twist.linear.z = float(vz)
        twist.angular.z = angular_z

        self.uav_cmd_pub.publish(twist)



    def stop_robots(self):
        stop = Twist()
        self.ugv_cmd_pub.publish(stop)
        self.uav_cmd_pub.publish(stop)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = UGVUAVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
