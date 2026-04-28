from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wait_for_match_start = LaunchConfiguration("wait_for_match_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wait_for_match_start",
                default_value="false",
                description=(
                    "If true, wait for referee/game_status match-in-progress (game_progress==4) "
                    "before sending Nav2 goals. If false, start immediately (still needs robot_status)."
                ),
            ),
            Node(
                package="simple_sentry_control",
                executable="simple_sentry_control_node_exec",
                name="simple_sentry_control_node",
                output="screen",
                parameters=[
                {"wait_for_match_start": wait_for_match_start},
                {"hp_low_threshold": 200},
                {"hp_recover_threshold": 380},
                # 裁判机满血上限；若 hp_recover_threshold 填得比满血还大，会永远卡在「回血中」
                {"hp_max": 600},

                {'origin_x': 0.0},
                {'origin_y': 0.0},
                {'origin_yaw': 0.0},
                {'origin_chassis_mode': 1},

                {'heal_x': -0.34},
                {'heal_y': -7.74},
                {'heal_yaw': 0.0},
                {'heal_chassis_mode': 1},

                {'point_a_x': 4.8},
                {'point_a_y': -6.15},
                {'point_a_yaw': 0.0},
                {'point_a_chassis_mode': 1},

                {'point_b_x': 1.2},
                {'point_b_y': -6.58},
                {'point_b_yaw': 0.0},
                {'point_b_chassis_mode': 1},

                {'point_c_x': 10.91},
                {'point_c_y': 1.36},
                {'point_c_yaw': 0.0},
                {'point_c_chassis_mode': 3},
                ],
            ),
        ]
    )



