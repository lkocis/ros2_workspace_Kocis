import os
import math
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.utils import create_params_file_from_dict
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def make_pose(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    q = quaternion_from_euler(0.0, 3.14, 0.0)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def move_to(arm, mpy, pose, draw_pub, draw=True):
    """Pomakni ruku na zadanu pozu i kontroliraj vizualizaciju."""
    # Uključi/isključi crtanje
    msg = Bool()
    msg.data = draw
    draw_pub.publish(msg)

    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link="hand")
    plan = arm.plan()
    if plan:
        mpy.execute(plan.trajectory, controllers=[])
    else:
        get_logger("draw_lana").error(f"Planiranje nije uspjelo za pozu ({pose.pose.position.x:.2f}, {pose.pose.position.z:.2f})")


def draw_letter_L(arm, mpy, draw_pub, ox, oy, oz, h, w):
    """Crta slovo L. ox,oy,oz = origin, h = visina, w = sirina."""
    # Premjesti se na vrh bez crtanja
    move_to(arm, mpy, make_pose(ox, oy, oz + h), draw_pub, draw=False)
    # Povuci liniju prema dolje
    move_to(arm, mpy, make_pose(ox, oy, oz), draw_pub, draw=True)
    # Povuci vodoravnu liniju prema desno
    move_to(arm, mpy, make_pose(ox + w, oy, oz), draw_pub, draw=True)


def draw_letter_A(arm, mpy, draw_pub, ox, oy, oz, h, w):
    """Crta slovo A."""
    mid_h = oz + h / 2
    # Lijeva dijagonala gore
    move_to(arm, mpy, make_pose(ox, oy, oz), draw_pub, draw=False)
    move_to(arm, mpy, make_pose(ox + w / 2, oy, oz + h), draw_pub, draw=True)
    # Desna dijagonala dolje
    move_to(arm, mpy, make_pose(ox + w, oy, oz), draw_pub, draw=True)
    # Srednja horizontalna linija
    move_to(arm, mpy, make_pose(ox + w * 0.25, oy, mid_h), draw_pub, draw=False)
    move_to(arm, mpy, make_pose(ox + w * 0.75, oy, mid_h), draw_pub, draw=True)


def draw_letter_N(arm, mpy, draw_pub, ox, oy, oz, h, w):
    """Crta slovo N."""
    # Lijeva vertikala
    move_to(arm, mpy, make_pose(ox, oy, oz), draw_pub, draw=False)
    move_to(arm, mpy, make_pose(ox, oy, oz + h), draw_pub, draw=True)
    # Dijagonala
    move_to(arm, mpy, make_pose(ox + w, oy, oz), draw_pub, draw=True)
    # Desna vertikala
    move_to(arm, mpy, make_pose(ox + w, oy, oz + h), draw_pub, draw=True)


def main():
    # Konfiguracija MoveIt
    moveit_config = (
        MoveItConfigsBuilder(robot_name="rob_manip", package_name="rob_manip_moveit")
        .robot_description(file_path="config/rob_manip.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("rob_manip_moveit"),
                "config",
                "py_node_config.yaml",
            )
        )
        .to_moveit_configs()
    ).to_dict()

    moveit_config.update({"use_sim_time": True})
    file = create_params_file_from_dict(moveit_config, "/**")

    rclpy.init()
    logger = get_logger("draw_lana")

    # Publisher za kontrolu vizualizacije
    node = rclpy.create_node("draw_lana_helper")
    draw_pub = node.create_publisher(Bool, '/draw_active', 10)

    # MoveItPy setup
    mpy = MoveItPy(node_name="draw_lana", launch_params_filepaths=[file])
    arm = mpy.get_planning_component("arm")
    logger.info("MoveItPy instanca stvorena")

    import time
    time.sleep(2.0)  # Čekaj da se sustav inicijalizira

    # Parametri slova
    y = 0.0       # Fiksna y koordinata ravnine crtanja
    h = 0.15      # Visina slova
    w = 0.08      # Širina slova
    gap = 0.04    # Razmak između slova
    start_x = 0.25  # Početna x koordinata
    start_z = 0.15  # Početna z koordinata

    # Crtaj svako slovo
    logger.info("Crtam: L")
    draw_letter_L(arm, mpy, draw_pub, start_x, y, start_z, h, w)

    logger.info("Crtam: A")
    draw_letter_A(arm, mpy, draw_pub, start_x + w + gap, y, start_z, h, w)

    logger.info("Crtam: N")
    draw_letter_N(arm, mpy, draw_pub, start_x + 2 * (w + gap), y, start_z, h, w)

    logger.info("Crtam: A")
    draw_letter_A(arm, mpy, draw_pub, start_x + 3 * (w + gap), y, start_z, h, w)

    # Ugasi crtanje
    msg = Bool()
    msg.data = False
    draw_pub.publish(msg)

    logger.info("Gotovo! LANA nacrtana.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()