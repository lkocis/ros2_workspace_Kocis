#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
class TFSubNode(Node):
    def __init__(self):
        super().__init__("tf_sub")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info("TF subscriber has been started.")

    def on_timer(self):
        from_frame_rel = 'base_link'
        to_frame_rel = 'left_wheel'
        try:
            t = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {from_frame_rel} to {to_frame_rel}: {ex}')
            return
        self.get_logger().info(f'Transform: {from_frame_rel} to {to_frame_rel}: {t.transform}')
    
def main(args=None):
    rclpy.init(args=args)
    node = TFSubNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#dva terminala: 
# 1) ros2 launch urdf_tutorial display.launch.py model:=/home/student/ros2_workspace_Kocis/src/mob_rob_description/urdf/mob_rob.urdf
# 2) ros2 run py_pkg tf_listen