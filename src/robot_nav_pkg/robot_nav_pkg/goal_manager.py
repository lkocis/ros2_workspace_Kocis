#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        
        self.publisher_ = self.create_publisher(Pose, '/key_points', 10)
        
        self.navigator = BasicNavigator()

        self.get_logger().info('>>> Waiting for Nav2 to become active...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('>>> Nav2 is active. Starting path definition.')

        # 10 points for navigation
        self.points_data = [
            (2.0, 0.0), (3.0, 0.0), (4.0, 0.0), (4.0, 2.0), (4.0, 4.0),
            (2.0, 3.0), (2.0, 1.0), (0.0, 2.0), (-2.0, 2.0), (-4.0, 4.0)
        ]

        self.run_navigation()

    def run_navigation(self):
        goal_poses = []

        for pt in self.points_data:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pt[0]
            goal_pose.pose.position.y = pt[1]
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)

            msg = Pose()
            msg.position.x = pt[0]
            msg.position.y = pt[1]
            msg.position.z = 0.0
            msg.orientation.w = 1.0
            self.publisher_.publish(msg)

        self.get_logger().info('>>> Sending 10 points to the robot...')
        self.navigator.followWaypoints(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'>>> Current point in execution: {feedback.current_waypoint}')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('>>> Path successfully completed!')
        if result == TaskResult.CANCELED:
            self.get_logger().info('>>> Path was cancelled.')
        else:
            self.get_logger().info('>>>Path failed to complete.')
def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()