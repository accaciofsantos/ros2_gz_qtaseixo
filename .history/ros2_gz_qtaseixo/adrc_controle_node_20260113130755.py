#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
import csv
import os
import numpy as np
from time import time

class UGVController(Node):
    def __init__(self):
        super().__init__('ugv_pd_node')

        # ------------------------
        # ROS 2 PARAMETERS
        # ------------------------
        self.declare_parameter("controller_type", "pd")
        self.declare_parameter("csv_name", "ugv_pd_log.csv")
        self.declare_parameter("goal_pattern", "quadrado")

        self.controller_type = self.get_parameter("controller_type").value
        csv_name = self.get_parameter("csv_name").value
        goal_pattern = self.get_parameter("goal_pattern").value

        # ------------------------
        # CSV Logging
        # ------------------------
        csv_dir = os.path.expanduser('~/Desktop/UGV_ADRC/data')
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = os.path.join(csv_dir, csv_name)

        self.csv_fields = [
            'timestamp', 'x_real', 'y_real', 'yaw_real',
            'dist_error', 'angle_error',
            'v_cmd', 'w_cmd'
        ]

        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_fields)

        self.get_logger().info(f"🟢 Controller Type: {self.controller_type}")
        self.get_logger().info(f"💾 CSV File: {self.csv_file}")
        self.get_logger().info(f"📍 Goal Pattern: {goal_pattern}")

        # ------------------------
        # Subscriptions & Publisher
        # ------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped, '/model/ugv/pose', self.pose_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/ugv/cmd_vel', 10)

        # ------------------------
        # States
        # ------------------------
        self.position = np.zeros(2)  # x, y
        self.yaw = 0.0

        # ------------------------
        # Goals
        # ------------------------
        self.goals = self.define_goals(goal_pattern)
        self.current_goal_index = 0
        self.all_goals_reached = False

        # ------------------------
        # PD Gains
        # ------------------------
        self.kp_linear = 1.2
        self.kd_linear = 0.2
        self.kp_angular = 2.0
        self.kd_angular = 0.3

        self.prev_dist_err = 0.0
        self.prev_angle_err = 0.0
        self.prev_time = None

        # ------------------------
        # Control Loop Timer
        # ------------------------
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

    # ==============================
    # Goal definitions
    # ==============================
    def define_goals(self, pattern):
        if pattern == "quadrado":
            return [(5.0, -5.0), (5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0)]
        elif pattern == "zigzag":
            return [(2.0, 2.0), (4.0, -2.0), (6.0, 2.0), (8.0, -2.0)]
        elif pattern == "linha_reta":
            return [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0)]
        elif pattern == "circulo":
            return [(5.0*math.cos(theta), 5.0*math.sin(theta)) for theta in np.linspace(0, 2*math.pi, 8, endpoint=False)]
        else:
            self.get_logger().warn(f"⚠️ Goal pattern '{pattern}' não reconhecido. Usando quadrado.")
            return [(5.0, -5.0), (5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0)]

    # ==============================
    # Pose callback
    # ==============================
    def pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "variable_friction_world":
            return
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        q = msg.pose.orientation
        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    # ==============================
    # Control loop
    # ==============================
    def control_loop(self):
        if self.all_goals_reached or self.current_goal_index >= len(self.goals):
            self.all_goals_reached = True
            self.publish_stop()
            return

        goal = self.goals[self.current_goal_index]
        dx = goal[0] - self.position[0]
        dy = goal[1] - self.position[1]
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_err = self.shortest_angle(angle_to_goal, self.yaw)

        # --------------------------
        # PD Controller
        # --------------------------
        v_cmd, w_cmd = self.pd_controller(dist, angle_err)

        # --------------------------
        # Optional: reduce linear speed if angle error is very high
        # --------------------------
        if abs(angle_err) > math.pi/3:  # só reduz em curvas muito grandes
            v_cmd *= 0.8

        # --------------------------
        # Publish command
        # --------------------------
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        # --------------------------
        # Goal check
        # --------------------------
        if dist < 0.3:
            self.get_logger().info(f"🏁 Goal {self.current_goal_index} reached")
            self.current_goal_index += 1

        # --------------------------
        # Logging
        # --------------------------
        self.log_csv(dist, angle_err, v_cmd, w_cmd)

    # ==============================
    # PD Controller
    # ==============================
    def pd_controller(self, dist, angle_err):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = now
            self.prev_dist_err = dist
            self.prev_angle_err = angle_err
            return 0.0, 0.0

        dt = now - self.prev_time
        dt = max(dt, 1e-3)

        # Derivativos
        d_dist = (dist - self.prev_dist_err) / dt
        d_angle = (angle_err - self.prev_angle_err) / dt

        # PD clássico (derivativo positivo)
        v_cmd = self.kp_linear*dist + self.kd_linear*d_dist
        w_cmd = self.kp_angular*angle_err + self.kd_angular*d_angle

        # Limita velocidades
        v_cmd = np.clip(v_cmd, -1.0, 1.0)
        w_cmd = np.clip(w_cmd, -1.0, 1.0)

        # Atualiza histórico
        self.prev_time = now
        self.prev_dist_err = dist
        self.prev_angle_err = angle_err

        return v_cmd, w_cmd

    # ==============================
    # Utilities
    # ==============================
    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def log_csv(self, dist_err, angle_err, v_cmd, w_cmd):
        ts = time()
        row = [
            ts,
            *self.position,
            self.yaw,
            dist_err,
            angle_err,
            v_cmd,
            w_cmd
        ]
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

    # ==============================
    # Angle utilities
    # ==============================
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def shortest_angle(self, target, current):
        diff = target - current
        while diff > math.pi:
            diff -= 2*math.pi
        while diff < -math.pi:
            diff += 2*math.pi
        return diff

def main(args=None):
    rclpy.init(args=args)
    node = UGVController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
