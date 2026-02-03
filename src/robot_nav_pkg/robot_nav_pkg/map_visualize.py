#!/usr/bin/env python3
import rosbag2_py
import tkinter as tk
from PIL import Image, ImageTk
import yaml
import math
from cv_bridge import CvBridge
import cv2
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image as ROSImage

class BagVisualizer(tk.Tk):
    def __init__(self, yaml_path, pgm_path, bag_path, goal_points):
        super().__init__()
        self.title("Bag Visualizer")
       
        with open(yaml_path, 'r') as file:
            self.map_config = yaml.safe_load(file)
        
        self.res = self.map_config['resolution']
        self.origin = self.map_config['origin']
        
        self.map_img = Image.open(pgm_path)
        self.map_tk = ImageTk.PhotoImage(self.map_img)
        
        self.canvas = tk.Canvas(self, width=self.map_img.width, height=self.map_img.height)
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor="nw", image=self.map_tk)
        
        self.bridge = CvBridge()
        self.photos = [] 

        path_points, images_with_poses = self.read_data_from_bag(bag_path)

        self.display_robot_data(path_points, goal_points, images_with_poses)

    def world_to_pixel(self, x, y):
        px = (x - self.origin[0]) / self.res
        py = self.map_img.height - ((y - self.origin[1]) / self.res)
        return px, py

    def read_data_from_bag(self, bag_path):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr')
        reader.open(storage_options, converter_options)

        path_points = []
        images_with_poses = []
        last_x, last_y = 0.0, 0.0

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == '/polozaj':
                msg = deserialize_message(data, Pose)
                last_x, last_y = msg.position.x, msg.position.y
                path_points.append((last_x, last_y))
            elif topic == '/slike':
                msg = deserialize_message(data, ROSImage)
                images_with_poses.append({'pose': (last_x, last_y), 'msg': msg})
        
        return path_points, images_with_poses

    def display_robot_data(self, path_points, goal_points, images_data):
        # Blue path
        for i in range(len(path_points) - 1):
            x1, y1 = self.world_to_pixel(path_points[i][0], path_points[i][1])
            x2, y2 = self.world_to_pixel(path_points[i+1][0], path_points[i+1][1])
            self.canvas.create_line(x1, y1, x2, y2, fill="blue", width=2)

        for gx, gy in goal_points:
            # Magenta goal point
            px_goal, py_goal = self.world_to_pixel(gx, gy)
            self.canvas.create_oval(px_goal-6, py_goal-6, px_goal+6, py_goal+6, fill="magenta")

            sorted_imgs = sorted(images_data, key=lambda i: math.sqrt((gx-i['pose'][0])**2 + (gy-i['pose'][1])**2))
            
            for idx, img_entry in enumerate(sorted_imgs[:3]):
                ix, iy = img_entry['pose']
                px_robot, py_robot = self.world_to_pixel(ix, iy)
                
                img_x_pos = px_goal + (idx - 1) * 20   
                img_y_pos = py_goal - 100     
                
                tk_img = self.ros_to_tk_image(img_entry['msg'])
                self.photos.append(tk_img) 
                self.canvas.create_line(px_robot, py_robot, img_x_pos, img_y_pos, fill="gray", dash=(4, 4), arrow=tk.LAST)
                self.canvas.create_image(img_x_pos, img_y_pos, image=tk_img)

    def ros_to_tk_image(self, ros_msg):
        cv_img = self.bridge.imgmsg_to_cv2(ros_msg, desired_encoding='bgr8')
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb_img).resize((30, 10)) 
        return ImageTk.PhotoImage(pil_img)

if __name__ == "__main__":
    points = [
        (2.0, 0.0), (3.0, 0.0), (4.0, 0.0), (4.0, 2.0), (4.0, 4.0),
        (2.0, 3.0), (2.0, 1.0), (0.0, 2.0), (-2.0, 2.0), (-4.0, 4.0)
    ]
    
    bag_path = "/home/lana/ros2_workspace_Kocis/rosbag2_2026_02_03-12_35_32"

    depot_pgm_path = "/home/lana/ros2_workspace_Kocis/src/robot_nav_pkg/maps/depot.pgm"

    depot_yaml_path = "/home/lana/ros2_workspace_Kocis/src/robot_nav_pkg/maps/depot.yaml"
    
    app = BagVisualizer(depot_yaml_path, depot_pgm_path, bag_path, points)
    app.mainloop()

    # starting: python3 map_visualize.py