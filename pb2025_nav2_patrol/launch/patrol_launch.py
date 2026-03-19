from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('pb2025_nav2_patrol')
    params_file = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    node = Node(
        package='pb2025_nav2_patrol',
        executable='patrol_node',
        name='pb2025_nav2_patrol',
        output='screen',
        parameters=[params_file]
    )
    return LaunchDescription([node])
