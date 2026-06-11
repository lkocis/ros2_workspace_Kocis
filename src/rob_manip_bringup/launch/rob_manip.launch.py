from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rob_manip_moveit = get_package_share_directory('rob_manip_moveit')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rob_manip_bringup = get_package_share_directory('rob_manip_bringup')

    return LaunchDescription([

        # RSP
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rob_manip_moveit, 'launch', 'rsp.launch.py')
            )
        ),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': 'empty.sdf -r'}.items()
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description'],
        ),

        # ROS-Gazebo bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(rob_manip_bringup, 'config', 'gz_bridge.yaml')
            }]
        ),

        # MoveGroup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rob_manip_moveit, 'launch', 'move_group.launch.py')
            )
        ),

        # RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rob_manip_moveit, 'launch', 'moveit_rviz.launch.py')
            )
        ),

        # Controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--param-file',
                os.path.join(rob_manip_moveit, 'config', 'ros2_controllers.yaml')],
        ),

    ])
