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
                {'hp_low_threshold': 200},
                {'hp_recover_threshold': 380},

                {'origin_x': 0.0},
                {'origin_y': 0.0},
                {'origin_yaw': 0.0},

                {'point_a_x': 6.0},
                {'point_a_y': 0.0},
                {'point_a_yaw': 0.0},

                {'point_b_x': 6.0},
                {'point_b_y': 6.0},
                {'point_b_yaw': 0.0},

                {'point_c_x': 5.0},
                {'point_c_y': 6.0},
                {'point_c_yaw': 0.0},
            ]
        )
    ])
