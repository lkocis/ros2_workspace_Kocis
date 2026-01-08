#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('stop_distance', 0.5)
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  
            self.lidar_callback,
            10
        )

        self.collision_publisher = self.create_publisher(Bool, '/collision', 10)

        self.get_logger().info(f'Obstacle detector started with stop_distance={self.stop_distance} m')

    def lidar_callback(self, msg: LaserScan):
        obstacle_detected = False

        center_index = len(msg.ranges) // 2
        window_size = 10  

        for i in range(center_index - window_size, center_index + window_size):
            if 0 <= i < len(msg.ranges):
                distance = msg.ranges[i]
                if distance > msg.range_min and distance < msg.range_max:
                    if distance < self.stop_distance:
                        obstacle_detected = True
                        break

        collision_msg = Bool()
        collision_msg.data = obstacle_detected
        self.collision_publisher.publish(collision_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
