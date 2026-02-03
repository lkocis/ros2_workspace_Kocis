#!/usr/bin/env python3

import math
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# ADDED: QoS imports to fix the 0 messages issue
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CurrentPointManager(Node):
    def __init__(self):
        super().__init__('current_point_manager')
        
        self.declare_parameter('dist_threshold', 1.0) # 1 meter
        self.declare_parameter('angle_threshold', 30.0) # 30 degrees

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_pose = self.create_publisher(Pose, '/polozaj', 10)
        self.publisher_image = self.create_publisher(Image, '/slike', 10)
        
        self.subscription_img = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            image_qos
        )

        self.last_pose_x = None
        self.last_pose_y = None
        self.last_yaw = None
        self.current_image = None
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        self.current_image = msg

    def save_reference_and_publish_image(self, x, y, yaw):
        self.last_pose_x = x
        self.last_pose_y = y
        self.last_yaw = yaw
        
        if self.current_image is not None:
            self.publisher_image.publish(self.current_image)
            self.get_logger().info(">>> Image successfully published to /slike")
        else:
            self.get_logger().warn(">>> Trigger condition met, but no image received yet from camera.")

    def timer_callback(self):
            from_frame_rel = 'base_link'
            to_frame_rel = 'map'
            now = rclpy.time.Time()
            try:
                t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)

                # Pose message
                msg = Pose()
                msg.position.x = t.transform.translation.x
                msg.position.y = t.transform.translation.y
                msg.position.z = t.transform.translation.z

                msg.orientation.x = t.transform.rotation.x
                msg.orientation.y = t.transform.rotation.y
                msg.orientation.z = t.transform.rotation.z
                msg.orientation.w = t.transform.rotation.w

                self.publisher_pose.publish(msg)
                self.get_logger().info(f'>>> Robot pose now: X={msg.position.x:.2f}, Y={msg.position.y:.2f}', throttle_duration_sec=1.0)
                
            # ------------------------------------------------------------------------------

                curr_x = t.transform.translation.x
                curr_y = t.transform.translation.y
                curr_yaw = get_rotation_from_quaternion(t.transform.rotation)

                if self.last_pose_x is None:
                    self.save_reference_and_publish_image(t.transform.translation.x, t.transform.translation.y, get_rotation_from_quaternion(t.transform.rotation))
                    return
                
                dist = euclidean_distance(self.last_pose_x, self.last_pose_y, curr_x, curr_y)
                angle_diff = math.degrees(abs(curr_yaw - self.last_yaw)) 
                if angle_diff > 180: angle_diff = 360 - angle_diff  # Normalization
                
                dist_threshold = self.get_parameter('dist_threshold').value
                angle_threshold = self.get_parameter('angle_threshold').value

                if (dist >= dist_threshold) or (angle_diff >= angle_threshold):
                    self.save_reference_and_publish_image(t.transform.translation.x, t.transform.translation.y, get_rotation_from_quaternion(t.transform.rotation))
                    self.get_logger().info(f'>>> Taking a picture: \nDistance: {dist:.2f} m \nAngle: {angle_diff:.2f} degrees.', throttle_duration_sec=2.0)


            except TransformException as ex:
                self.get_logger().warn(
                    f'>>> Waiting for transformation {from_frame_rel} -> {to_frame_rel}...', throttle_duration_sec=2.0)
                return
            
def get_rotation_from_quaternion(rotation):
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPointManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()