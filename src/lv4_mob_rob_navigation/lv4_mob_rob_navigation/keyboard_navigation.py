#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lv4_mob_rob_interfaces.action import Navigate
from rclpy.action import ActionClient
import tkinter as tk
from tkinter import messagebox
from action_msgs.msg import GoalStatus

class NavigatorClient(Node):
    def __init__(self):
        super().__init__('navigator_client')
        self._action_client = ActionClient(self, Navigate, '/navigate')
        self.goal_handle = None
        self.timeout_timer = None
        self.timeout_sec = 120.0  # 2 minute
        self.on_navigation_finished = None


    def send_goal(self, x, y, theta):
        goal_msg = Navigate.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.theta = float(theta)

        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available")
            return

        self.get_logger().info(
            f"Sending goal: x={x}, y={y}, theta={theta}"
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_to_goal
        self.get_logger().info(
            f"Distance to goal: {distance:.2f} m"
        )

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        # pokreni timeout timer
        self.timeout_timer = self.create_timer(
            self.timeout_sec,
            self.timeout_callback
        )

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def timeout_callback(self):
        self.get_logger().warning("Navigation timeout! Cancelling goal...")
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        if self.on_navigation_finished:
            self.on_navigation_finished()

    def get_result_callback(self, future):
        # zaustavi timer
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_CANCELED:  
            self.get_logger().warning(
                "Navigation canceled."
            )
            return
        
        if result.success:
            self.get_logger().info("Navigation succeeded!")
        else:
            self.get_logger().warning("Navigation failed!")

        # obavijesti GUI
        if self.on_navigation_finished:
            self.on_navigation_finished()

        if self.goal_handle is None:
            return

class NavigatorGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.on_navigation_finished = None
        self.ros_node.on_navigation_finished = self.navigation_finished

        self.root = tk.Tk()
        self.root.title("Robot Navigator")

        tk.Label(self.root, text="X:").grid(row=0, column=0)
        self.x_entry = tk.Entry(self.root)
        self.x_entry.insert(0, "0.0")
        self.x_entry.grid(row=0, column=1)

        tk.Label(self.root, text="Y:").grid(row=1, column=0)
        self.y_entry = tk.Entry(self.root)
        self.y_entry.insert(0, "0.0")
        self.y_entry.grid(row=1, column=1)

        tk.Label(self.root, text="Theta (deg):").grid(row=2, column=0)
        self.theta_entry = tk.Entry(self.root)
        self.theta_entry.insert(0, "0.0")
        self.theta_entry.grid(row=2, column=1)

        self.send_btn = tk.Button(self.root, text="Send Goal", command=self.send_goal)
        self.send_btn.grid(row=3, column=0, columnspan=2, pady=5)

        self.update_ros()

        self.root.mainloop()

    def send_goal(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            theta = float(self.theta_entry.get())
            self.disable_inputs()
            self.ros_node.send_goal(x, y, theta)
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers")

    def disable_inputs(self):
        self.send_btn.config(state=tk.DISABLED)
        self.x_entry.config(state=tk.DISABLED)
        self.y_entry.config(state=tk.DISABLED)
        self.theta_entry.config(state=tk.DISABLED)

    def enable_inputs(self):
        self.send_btn.config(state=tk.NORMAL)
        self.x_entry.config(state=tk.NORMAL)
        self.y_entry.config(state=tk.NORMAL)
        self.theta_entry.config(state=tk.NORMAL)

    def navigation_finished(self):
        # poziva se iz ROS threada → sigurno prebaciti u Tkinter thread
        self.root.after(0, self.enable_inputs)

    def update_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.05)
        self.root.after(50, self.update_ros) 


def main():
    rclpy.init()
    ros_node = NavigatorClient()
    gui = NavigatorGUI(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
