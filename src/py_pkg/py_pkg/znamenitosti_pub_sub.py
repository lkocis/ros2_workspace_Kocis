#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lv2_interface.msg import Znamenitost
from turtlesim.msg import Pose
import tkinter as tk
from threading import Thread

class ZnamenitostiNode(Node):
    def __init__(self):
        super().__init__("znamenitosti_pub_sub")

        # Subscriber
        self.pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Znamenitost, '/znamenitosti', 10)

        self.declare_parameter('periodic_enabled', 1)  # 0 = user input, 1 = periodic
        self.declare_parameter('period', 1.0)  

        self.counter = 1

    def pose_callback(self, msg):
        self.pose = msg

    def publish_callback(self, naziv):
        if self.pose is None:
            self.get_logger().warn('Pose not available')
            return
        
        msg = Znamenitost()
        msg.pose = self.pose
        msg.naziv = naziv
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {naziv}')


    def start_gui(self):
        root = tk.Tk()
        root.title("Znamenitosti publisher")
        naziv_entry = tk.Entry(root)
        naziv_entry.pack()

        def send():
            naziv = naziv_entry.get()
            if naziv:
                self.publish_callback(naziv)

        tk.Button(root, text="Send", command=send).pack()

        root.mainloop()

    def start_periodic(self):
        period = self.get_parameter('period').get_parameter_value().double_value
        self.create_timer(period, self.publish_periodic)

    def publish_periodic(self):
        naziv = f"Znamenitost_{self.counter}"
        self.publish_callback(naziv)
        self.counter += 1
   
def main(args=None):
    rclpy.init(args=args)
    node = ZnamenitostiNode()

    periodic = node.get_parameter('periodic_enabled').get_parameter_value().integer_value

    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if periodic:
            node.start_periodic()
            input("Press CTRL + C for exit...\n")
        else:
            node.start_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()