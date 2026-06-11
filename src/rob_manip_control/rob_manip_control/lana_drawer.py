#!/usr/bin/env python3

import os
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.utils import create_params_file_from_dict
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

# x = horizontalno, z = vertikalno
# y = dubina (fiksna za 2D crtanje)

def get_letter_L(x_offset, scale=0.1):
    points = [
        (0.0, 1.0),   # vrh
        (0.0, 0.0),   # dno lijevo
        (0.5, 0.0),   # dno desno
    ]
    return [(x_offset + p[0]*scale, p[1]*scale) for p in points]

def get_letter_A(x_offset, scale=0.1):
    points = [
        (0.0, 0.0),   # dno lijevo
        (0.25, 1.0),  # vrh
        (0.5, 0.0),   # dno desno
        (0.25, 1.0),  # nazad na vrh 
        # srednja crta
        (0.1, 0.5),   # lijevo sredina
        (0.4, 0.5),   # desno sredina
    ]
    return [(x_offset + p[0]*scale, p[1]*scale) for p in points]

def get_letter_N(x_offset, scale=0.1):
    points = [
        (0.0, 0.0),   # dno lijevo
        (0.0, 1.0),   # vrh lijevo
        (0.5, 0.0),   # dno desno 
        (0.5, 1.0),   # vrh desno
    ]
    return [(x_offset + p[0]*scale, p[1]*scale) for p in points]

def get_letter_A2(x_offset, scale=0.1):
    return get_letter_A(x_offset, scale)

class LanaDrawer(Node):
    def __init__(self):
        super().__init__('lana_drawer')

        # Publisher za kontrolu vizualizatora
        self.draw_pub = self.create_publisher(Bool, '/draw_enabled', 10)

        self.get_logger().info('LanaDrawer: inicijalizacija MoveIt...')

        # MoveIt konfiguracija
        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="rob_manip",
                package_name="rob_manip_moveit"
            )
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
        params_file = create_params_file_from_dict(moveit_config, "/**")

        rclpy.init() if not rclpy.ok() else None

        self.mpy = MoveItPy(
            node_name="lana_moveit_py",
            launch_params_filepaths=[params_file]
        )
        self.arm = self.mpy.get_planning_component("arm")
        self.get_logger().info('MoveIt inicijaliziran!')

    def set_draw(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.draw_pub.publish(msg)

    def move_to_pose(self, x, y, z):
        """Pomakni hand na zadanu poziciju"""
        self.arm.set_start_state_to_current_state()

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        q = quaternion_from_euler(0.0, math.pi, 0.0)
        pose_goal.pose.orientation.x = q[0]
        pose_goal.pose.orientation.y = q[1]
        pose_goal.pose.orientation.z = q[2]
        pose_goal.pose.orientation.w = q[3]

        self.arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link="hand"
        )

        plan = self.arm.plan()
        if plan:
            self.mpy.execute(plan.trajectory, controllers=[])
            time.sleep(1.0)
            return True
        else:
            self.get_logger().warn(f'Planiranje nije uspjelo za ({x:.2f}, {y:.2f}, {z:.2f})')
            return False

    def draw_letter(self, points, y_plane=0.2):
        """Crta slovo prolazeći kroz zadane točke"""
        if not points:
            return

        # Idi na prvu točku bez crtanja
        self.set_draw(False)
        self.move_to_pose(points[0][0], y_plane, points[0][1] + 0.5)

        # Uključi crtanje
        self.set_draw(True)

        # Prođi kroz sve točke
        for x, z in points[1:]:
            self.move_to_pose(x, y_plane, z + 0.5)

        # Isključi crtanje
        self.set_draw(False)

    def go_home(self):
        self.set_draw(False)
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="home")
        plan = self.arm.plan()
        if plan:
            self.mpy.execute(plan.trajectory, controllers=[])
            time.sleep(2.0)

    def draw_lana(self):
        scale = 0.08
        y_plane = 0.35  # dubina na kojoj crtamo

        self.get_logger().info('Crtanje: L')
        self.draw_letter(get_letter_L(0.0, scale), y_plane)
        self.go_home()

        self.get_logger().info('Crtanje: A')
        self.draw_letter(get_letter_A(0.1, scale), y_plane)
        self.go_home()

        self.get_logger().info('Crtanje: N')
        self.draw_letter(get_letter_N(0.2, scale), y_plane)
        self.go_home()

        self.get_logger().info('Crtanje: A')
        self.draw_letter(get_letter_A2(0.3, scale), y_plane)
        self.go_home()

        self.get_logger().info('Gotovo! Ime LANA nacrtano.')


def main(args=None):
    rclpy.init(args=args)
    node = LanaDrawer()
    node.draw_lana()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
