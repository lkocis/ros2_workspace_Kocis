from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lv2_interface'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='py_pkg',
            executable='znamenitosti_pub_sub',  
            name='znamenitosti_pub_sub',
            output='screen',
            parameters=[config]
        ),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'
        )
    ])

#Naredbe:
# pokreće se sa: ros2 launch lv2_interface znamenitosti.launch.py
# ros2 run turtlesim turtle_teleop_key