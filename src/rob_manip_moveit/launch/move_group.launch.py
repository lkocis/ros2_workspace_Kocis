from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rob_manip", package_name="rob_manip_moveit").to_moveit_configs()
    
    ld = generate_move_group_launch(moveit_config)
    
    # Dodaj use_sim_time=true svim čvorovima
    for action in ld.entities:
        if hasattr(action, 'node_parameters'):
            action.node_parameters.append({'use_sim_time': True})
    
    return ld