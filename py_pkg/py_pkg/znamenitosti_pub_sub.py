#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lv2_interface.msg import Znamenitost
from turtlesim.msg import Pose
import random
import tkinter as tk
from threading import Thread

class ZnamenitostiNode(Node):
    def __init__(self):
        super().__init__("znamenitosti_pub_sub")

        self.root = tk.Tk()
        self.root.title("Znamenitosti publisher.")

        self.entry = tk.Entry(self.root, width=40)
        self.entry.pack()

        self.textbox = tk.Text(self.root, height=10, width=50)
        self.textbox.pack()

        self.save_btn = tk.Button(self.root, text="Save", command=self.save_message)
        self.save_btn.pack()

        self.znamenitost_pub = self.create_publisher(Znamenitost, "znamenitosti", 10)

        self.znamenitost_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.status_timer = self.create_timer(1.0, self.publish_callback)

        self.turtle_pose = Pose() 

    def save_message(self):
        msg = self.entry.get()        
        self.textbox.insert(tk.END, msg + "\n")  
        self.entry.delete(0, tk.END)

    def publish_callback(self):
        msg = Znamenitost()
        msg.pose.x = random.uniform(0.0, 100.0)
        msg.pose.y = random.uniform(0.0, 100.0)
        msg.pose.theta = 0.0
        msg.naziv = "Katedrala"
        self.znamenitost_pub.publish(msg)
        #log_msg = f"Published: {msg.naziv} on ({msg.pose.x:.1f}, {msg.pose.y:.1f})"

    def pose_callback(self, msg):
        self.turtle_pose = msg
        # log_msg = f"Received: {msg.naziv} on ({msg.x:.1f}, {msg.y:.1f})"

def main(args=None):
    rclpy.init(args=args)
    node = ZnamenitostiNode()

    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    node.root.mainloop()
    

if __name__ == "__main__":
    main()