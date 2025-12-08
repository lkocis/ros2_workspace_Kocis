#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
from lv4_mob_rob_navigation import NavigateToPose
from nav_msgs.msg import Odometry
import tf_transformations

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.collision_detected = False
        self.create_subscription(Bool, '/collision', self.collision_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.create_subscription(
            Odometry,           # PRAVI tip za odometriju
            '/odom_fake',       # tema na kojoj Gazebo objavljuje poziciju
            self.odom_callback, # tvoja callback funkcija
            10  
        )

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

    def collision_callback(self, msg: Bool):
        self.collision_detected = msg.data

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Pretvaranje quaternion u Euler za yaw (theta)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.theta = yaw

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        target_x = self.current_x + goal.x  
        target_y = self.current_y + goal.y
        target_theta = goal.theta  

        self.get_logger().info(f'Navigating to ({target_x:.2f}, {target_y:.2f}, {target_theta:.2f})')

        # --- Korak 1: rotacija prema cilju ---
        angle_to_goal = math.atan2(target_y - self.current_y, target_x - self.current_x)
        while abs(angle_to_goal - self.current_theta) > 0.05:
            twist = Twist()
            twist.angular.z = 0.5 * (angle_to_goal - self.current_theta)
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop_robot()

        # --- Korak 2: pravocrtno kretanje ---
        distance = math.hypot(target_x - self.current_x, target_y - self.current_y)
        while distance > 0.05:
            if self.collision_detected:
                self.stop_robot()
                goal_handle.abort()
                self.get_logger().warn('Collision detected! Navigation aborted.')
                return NavigateToPose.Result(success=False)

            twist = Twist()
            twist.linear.x = 0.2  # brzina naprijed
            self.cmd_pub.publish(twist)

            rclpy.spin_once(self, timeout_sec=0.1)
            distance = math.hypot(target_x - self.current_x, target_y - self.current_y)

        self.stop_robot()

        # --- Korak 3: rotacija na ciljanu orijentaciju ---
        while abs(target_theta - self.current_theta) > 0.05:
            twist = Twist()
            twist.angular.z = 0.5 * (target_theta - self.current_theta)
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_robot()
        goal_handle.succeed()
        self.get_logger().info('Navigation complete')
        return NavigateToPose.Result(success=True)

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
