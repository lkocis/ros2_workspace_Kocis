#!/usr/bin/env python3
import rosbag2_py
from rclpy.serialization import deserialize_message
from turtlesim.msg import Pose
from lv2_interface.msg import Znamenitost
import matplotlib.pyplot as plt

def main():
    bag_path = "/home/lana/ros2_workspace_KocisL/rosbag2_2025_11_12-21_10_24"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    robot_positions = []
    landmarks = []

    topics_to_read = ['/turtle1/pose', '/znamenitosti']

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic not in topics_to_read:
            continue

        if topic == '/turtle1/pose':
            msg = deserialize_message(data, Pose)
            robot_positions.append((msg.x, msg.y))
        elif topic == '/znamenitosti':
            msg = deserialize_message(data, Znamenitost)
            landmarks.append((msg.pose.x, msg.pose.y, msg.naziv))

    del reader

    plt.figure(figsize=(7,7))
    
    if robot_positions:
        xs, ys = zip(*robot_positions)
        plt.plot(xs, ys, '-b', label='Putanja robota')

    if landmarks:
        xs, ys, nazivi = zip(*landmarks)
        plt.scatter(xs, ys, c='r', marker='.', label='Znamenitosti')
        for x, y, naziv in landmarks:
            plt.text(x + 0.1, y + 0.1, naziv, fontsize=9, color='green')

    plt.show()

if __name__ == "__main__":
    main()
