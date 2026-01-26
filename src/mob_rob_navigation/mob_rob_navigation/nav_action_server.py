import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import time
from mob_rob_interfaces.action import Navigate
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool  

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('nav_action_server')
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(self, Navigate, 'navigate_to_pose', self.execute_callback, callback_group=self.callback_group)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.subscription = self.create_subscription(Odometry, 'odom_fake', self.odom_callback, 10, callback_group=self.callback_group)

        self.collision_detected = False
        self.collision_sub = self.create_subscription(Bool, '/collision', self.collision_callback, 10, callback_group=self.callback_group)
        
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_theta = 0.0

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.curr_theta = math.atan2(siny_cosp, cosy_cosp)

    def collision_callback(self, msg):
        self.collision_detected = msg.data

    def execute_callback(self, goal_handle):
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y
        target_theta = math.radians(goal_handle.request.theta)
        
        feedback_msg = Navigate.Feedback()
        move_msg = Twist()

        # 1) Orijentiraj robot prema cilju 
        self.get_logger().info('1) Rotation towards goal...')
        while rclpy.ok():
            angle_to_goal = math.atan2(target_y - self.curr_y, target_x - self.curr_x)
            angle_diff = angle_to_goal - self.curr_theta
            
            if abs(angle_diff) < 0.1: break
            move_msg.angular.z = 0.3 if angle_diff > 0 else -0.3
            self.publisher.publish(move_msg)
            time.sleep(0.1)

        # 2) Pravocrtno kretanje 
        self.get_logger().info('2) Moving towards goal...')
        while rclpy.ok():
            # PROVJERA KOLIZIJE 
            if self.collision_detected:
                self.get_logger().error('Collision detected! Aborting...')
                self.publisher.publish(Twist()) # Stani
                goal_handle.abort() # Nije uspjelo
                result = Navigate.Result()
                result.success = False
                return result

            dist = math.sqrt((target_x - self.curr_x)**2 + (target_y - self.curr_y)**2)
            feedback_msg.distance_to_goal = dist
            goal_handle.publish_feedback(feedback_msg)

            if dist < 0.2: break
            
            move_msg = Twist()
            move_msg.linear.x = 0.2
            self.publisher.publish(move_msg)
            time.sleep(0.1)

        # 3) Finalna orijentacija
        self.get_logger().info('3) Final orientation...')
        while rclpy.ok():
            angle_diff = target_theta - self.curr_theta
            if abs(angle_diff) < 0.1: break
            move_msg = Twist()
            move_msg.angular.z = 0.3 if angle_diff > 0 else -0.3
            self.publisher.publish(move_msg)
            time.sleep(0.1)

        self.publisher.publish(Twist()) 
        goal_handle.succeed()
        result = Navigate.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()