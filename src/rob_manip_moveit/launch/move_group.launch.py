from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rob_manip", package_name="rob_manip_moveit")
        .parameter("use_sim_time", True)
        .to_moveit_configs()
    )
    
    return generate_move_group_launch(moveit_config)
