import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Bool
import rclpy.time


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/draw_path', 10)
        self.draw_sub = self.create_subscription(Bool, '/draw_active', self.draw_callback, 10)

        self.drawing = False
        self.marker_id = 0
        self.markers = MarkerArray()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('PathVisualizer pokrenut!')

    def draw_callback(self, msg):
        self.drawing = msg.data

    def timer_callback(self):
        if not self.drawing:
            return
        try:
            trans = self.tf_buffer.lookup_transform('world', 'hand', rclpy.time.Time())
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'lana_path'
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = trans.transform.translation.x
            marker.pose.position.y = trans.transform.translation.y
            marker.pose.position.z = trans.transform.translation.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.markers.markers.append(marker)
            self.marker_id += 1
            self.marker_pub.publish(self.markers)
        except Exception as e:
            pass


def main():
    rclpy.init()
    node = PathVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()