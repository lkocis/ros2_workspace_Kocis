#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lv4_mob_rob_navigation.action import NavigateToPose
from rclpy.action import ActionClient
import tkinter as tk
from tkinter import messagebox

class NavigatorClient(Node):
    def __init__(self):
        super().__init__('navigator_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.theta = float(theta)

        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending goal: x={x}, y={y}, theta={theta}")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return
        self.get_logger().info("Goal accepted!")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Navigation succeeded!")
        else:
            self.get_logger().warn("Navigation failed!")


class NavigatorGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node

        self.root = tk.Tk()
        self.root.title("Robot Navigator GUI")

        tk.Label(self.root, text="X:").grid(row=0, column=0)
        self.x_entry = tk.Entry(self.root)
        self.x_entry.insert(0, "0.0")
        self.x_entry.grid(row=0, column=1)

        tk.Label(self.root, text="Y:").grid(row=1, column=0)
        self.y_entry = tk.Entry(self.root)
        self.y_entry.insert(0, "0.0")
        self.y_entry.grid(row=1, column=1)

        tk.Label(self.root, text="Theta (rad):").grid(row=2, column=0)
        self.theta_entry = tk.Entry(self.root)
        self.theta_entry.insert(0, "0.0")
        self.theta_entry.grid(row=2, column=1)

        self.send_btn = tk.Button(self.root, text="Send Goal", command=self.send_goal)
        self.send_btn.grid(row=3, column=0, columnspan=2, pady=5)

        # Tkinter + ROS2 petlja
        self.update_ros()

        self.root.mainloop()

    def send_goal(self):
        x = self.x_entry.get()
        y = self.y_entry.get()
        theta = self.theta_entry.get()
        try:
            self.ros_node.send_goal(x, y, theta)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def update_ros(self):
        # Spin ROS node jednom, ne blokira GUI
        rclpy.spin_once(self.ros_node, timeout_sec=0.05)
        self.root.after(50, self.update_ros)  # pozovi opet nakon 50ms


def main():
    rclpy.init()
    ros_node = NavigatorClient()
    gui = NavigatorGUI(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
