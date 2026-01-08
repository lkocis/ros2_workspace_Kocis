#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import math
import tf_transformations
from rclpy.executors import MultiThreadedExecutor
from lv4_mob_rob_interfaces.action import Navigate
import time


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        # Publisher za brzinu robota
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Collision flag
        self.collision_detected = False
        self.create_subscription(
            Bool,
            '/collision',
            self.collision_callback,
            10
        )

        # Odometrija robota
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.create_subscription(
            Odometry,
            '/odom_fake',
            self.odom_callback,
            10
        )

        # Action server
        self._action_server = ActionServer(
            self,
            Navigate,
            '/navigate',
            self.execute_callback
        )

        self.get_logger().info('Navigator action server started')

    # -----------------------------------------------------

    def collision_callback(self, msg: Bool):
        self.collision_detected = msg.data

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.current_theta = yaw

    # -----------------------------------------------------

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # -----------------------------------------------------

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        # Relativni cilj → apsolutni
        target_x = self.current_x + goal.x
        target_y = self.current_y + goal.y
        target_theta = self.current_theta + math.radians(goal.theta)

        self.get_logger().info(
            f'New goal: x={goal.x:.2f}, y={goal.y:.2f}, theta={goal.theta:.1f}°'
        )

        feedback = Navigate.Feedback()

        # -------------------------------------------------
        # 1) ROTACIJA PREMA CILJU
        # -------------------------------------------------
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return Navigate.Result(success=False)

            angle_to_goal = math.atan2(
                target_y - self.current_y,
                target_x - self.current_x
            )
            error = self.normalize_angle(angle_to_goal - self.current_theta)

            if abs(error) < 0.05:
                break

            twist = Twist()
            twist.angular.z = 0.6 * error
            self.cmd_pub.publish(twist)

            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_robot()

        # -------------------------------------------------
        # 2) PRAVOCRTNO KRETANJE
        # -------------------------------------------------
        self.get_logger().info('Starting linear movement...')
        while rclpy.ok():
            # Provjera kolizije (Samo u ovom koraku!)
            if self.collision_detected:
                self.stop_robot()
                self.get_logger().error('COLLISION! Aborting action and returning False.')
                goal_handle.abort()
                result = Navigate.Result()
                result.success = False
                return result

            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return Navigate.Result(success=False)

            distance = math.hypot(target_x - self.current_x, target_y - self.current_y)
            feedback.distance_to_goal = distance
            goal_handle.publish_feedback(feedback)

            if distance < 0.05:
                break

            twist = Twist()
            twist.linear.x = 0.20 # Malo smanjite brzinu radi lakšeg testiranja
            self.cmd_pub.publish(twist)
            
            # Kratka pauza da se ne zaguši procesor
            time.sleep(0.05)

        # -------------------------------------------------
        # 3) ZAVRŠNA ROTACIJA
        # -------------------------------------------------
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()
                return Navigate.Result(success=False)

            error = self.normalize_angle(
                target_theta - self.current_theta
            )

            if abs(error) < 0.05:
                break

            twist = Twist()
            twist.angular.z = 0.6 * error
            self.cmd_pub.publish(twist)

            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_robot()

        # -------------------------------------------------
        goal_handle.succeed()
        self.get_logger().info('Navigation completed successfully')
        return Navigate.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    
    # Omogućuje istovremeno izvršavanje akcije i čitanje senzora
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
