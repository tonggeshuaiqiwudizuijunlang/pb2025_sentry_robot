from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_sentry_control',
            executable='simple_sentry_control_node_exec',
            name='simple_sentry_control_node',
            output='screen',
            parameters=[
                {'hp_low_threshold': -1}, # 设置为-1，确保不会因为血量逻辑打断建图巡逻
                {'hp_recover_threshold': 400},

                {'origin_x': 0.0},
                {'origin_y': 0.0},
                {'origin_yaw': 0.0},

                {'point_a_x': 1.0},
                {'point_a_y': 4.34},
                {'point_a_yaw': 0.0},

                {'point_b_x': -1.9},
                {'point_b_y': 5.2},
                {'point_b_yaw': 0.0},

                {'point_c_x': -5.0},
                {'point_c_y': 3.27},
                {'point_c_yaw': 0.0},
            ]
        )
    ])
