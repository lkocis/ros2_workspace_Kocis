#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/hand_path', 10)

        # Pretplata na temu za kontrolu crtanja
        self.draw_sub = self.create_subscription(Bool, '/draw_enabled', self.draw_callback, 10)

        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz

        self.markers = []
        self.marker_id = 0
        self.draw_enabled = False

        self.get_logger().info('PathVisualizer started.')

    def draw_callback(self, msg):
        self.draw_enabled = msg.data

    def on_timer(self):
        if not self.draw_enabled:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'world',
                'hand',
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')
            return

        # Stvori novi marker (točka)
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hand_path'
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = t.transform.translation.x
        marker.pose.position.y = t.transform.translation.y
        marker.pose.position.z = t.transform.translation.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0  # trajno

        self.markers.append(marker)
        self.marker_id += 1

        # Objavi sve markere
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
