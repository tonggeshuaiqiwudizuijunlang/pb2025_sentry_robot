# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_hik_camera = LaunchConfiguration("use_hik_camera")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    rm_bringup_dir = get_package_share_directory("rm_bringup")
    hik_camera_dir = get_package_share_directory("hik_camera")

    armor_detector_params = os.path.join(
        rm_bringup_dir, "config", "node_params", "armor_detector_params.yaml"
    )
    armor_solver_params = os.path.join(
        rm_bringup_dir, "config", "node_params", "armor_solver_params.yaml"
    )
    camera_driver_params = os.path.join(
        rm_bringup_dir, "config", "node_params", "camera_driver_params.yaml"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    declare_use_hik_camera_cmd = DeclareLaunchArgument(
        "use_hik_camera",
        default_value="True",
        description="Whether to launch hik_camera for FYT vision",
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes",
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    camera_include_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hik_camera_dir, "launch", "hik_camera.launch.py")
        ),
        condition=IfCondition(use_hik_camera),
        launch_arguments={"params_file": camera_driver_params}.items(),
    )

    detector_container_cmd = ComposableNodeContainer(
        name="camera_detector_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package="armor_detector",
                plugin="fyt::auto_aim::ArmorDetectorNode",
                name="armor_detector",
                parameters=[armor_detector_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
    )

    armor_solver_cmd = Node(
        package="armor_solver",
        executable="armor_solver_node",
        name="armor_solver",
        output="screen",
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[armor_solver_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    vision_group_cmd = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            camera_include_cmd,
            detector_container_cmd,
            armor_solver_cmd,
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_hik_camera_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(vision_group_cmd)

    return ld
