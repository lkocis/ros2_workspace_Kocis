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
        front_angles = range(-15, 16)
        obstacle_detected = False

        for angle in front_angles:
            index = int((angle - msg.angle_min) / msg.angle_increment)
            if 0 <= index < len(msg.ranges):
                distance = msg.ranges[index]
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
