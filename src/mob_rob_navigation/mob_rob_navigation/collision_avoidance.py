import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Bool, '/collision', 10)

    def scan_callback(self, msg):
        middle = len(msg.ranges) // 2
        front_ranges = msg.ranges[middle-20 : middle+20]
        min_dist = min([r for r in front_ranges if r > 0.05])
        
        collision_msg = Bool()
        collision_msg.data = True if min_dist < 0.7 else False
        self.publisher.publish(collision_msg)

def main():
    rclpy.init()
    rclpy.spin(CollisionDetector())