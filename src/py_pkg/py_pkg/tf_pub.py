import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
class TFFramePublisher(Node):
    def __init__(self):
        super().__init__('tf2_frame_publisher')
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info("TF publisher has been started.")

    def on_timer(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'new_frame'
        x = (self.get_clock().now().seconds_nanoseconds()[1] / 1e9) * 2 * math.pi
        t.transform.translation.x = math.sin(x)
        t.transform.translation.y = math.cos(x)
        t.transform.translation.z = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Publisherd transform: {t.header.frame_id} to {t.child_frame_id}: {t.transform}')

def main():
    rclpy.init()
    node = TFFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()