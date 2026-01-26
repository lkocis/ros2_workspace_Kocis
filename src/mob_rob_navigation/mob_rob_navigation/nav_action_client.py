import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
from mob_rob_interfaces.action import Navigate

class NavigationActionClient(Node):
    def __init__(self):
        super().__init__('nav_action_client')
        self._action_client = ActionClient(self, Navigate, 'navigate_to_pose')
        self.timer = None

    def send_goal(self, x, y, theta):
        goal_msg = Navigate.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.theta = theta

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # TIMER: Stop after 2 minutes
        self.timer = self.create_timer(120.0, self.timer_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(f'Euclidean distance to goal: {dist:.2f} m')

    def timer_callback(self):
        self.get_logger().error('Stopping navigation, timeout reached (2 minutes).')
        self.timer.cancel()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    client = NavigationActionClient()

    try:
        x = float(input("Input x (m): "))
        y = float(input("Input y (m): "))
        theta = float(input("Input theta (degrees): "))
    except ValueError:
        print("Error: Please enter a number.")
        return

    client.send_goal(x, y, theta)
    rclpy.spin(client)

if __name__ == '__main__':
    main()