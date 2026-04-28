import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

    SetParameter(name='rune',value=launch_params['rune']),
    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])
    
    robot_navigation_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'sentry.urdf.xacro')])

    robot_gimbal_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_gimbal_description,
                    'publish_frequency': 1000.0}]
    )
    
    robot_navigation_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_navigation_description,}]
                    # 'publish_frequency': 1000.0}]
    )

    # 确保存在 push_namespace，避免在后面使用时出现 NameError
    push_namespace = PushRosNamespace('')

    def get_params(name):
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))

    # 图像
    if launch_params['video_play']: 
        image_node  = ComposableNode(
            package='rm_camera_driver',
            plugin='fyt::camera_driver::VideoPlayerNode',
            name='video_player',
            parameters=[get_params('video_player')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
    else:
        # Include external hik_camera launch instead of composing Hik as component
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        hik_launch_path = os.path.join(get_package_share_directory('hik_camera'), 'launch', 'hik_camera.launch.py')
        camera_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hik_launch_path),
            launch_arguments={'params_file': get_params('camera_driver')}.items()
        )
        # do not provide image_node to container (camera launched separately)
        image_node = None
        # add camera_include to the delayed actions list later

    # 串口
    if launch_params['virtual_serial']:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='virtual_serial_node',
            name='virtual_serial',
            output='both',
            emulate_tty=True,
            parameters=[get_params('virtual_serial')],
            ros_arguments=['--ros-args', '-p', 'has_rune:=true' if launch_params['rune'] else 'has_rune:=false'],
        )
    else:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            name='serial_driver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('serial_driver')],
            ros_arguments=['--ros-args', ],
        )
        
    # 装甲板识别
    armor_detector_node = ComposableNode(
        package='armor_detector', 
        plugin='fyt::auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[get_params('armor_detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # 装甲板解算
    if launch_params['hero_solver']:
        armor_solver_node = Node(
            package='hero_armor_solver',
            executable='hero_armor_solver_node',
            name='armor_solver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('armor_solver')],
            ros_arguments=[],
        )
    else:
        armor_solver_node = Node(
            package='armor_solver',
            executable='armor_solver_node',
            name='armor_solver',
            output='both',
            emulate_tty=True,
            parameters=[get_params('armor_solver')],
            ros_arguments=[],
        )

    # 打符
    rune_detector_node = ComposableNode(    
        package='rune_detector',
        plugin='fyt::rune::RuneDetectorNode',
        name='rune_detector',
        parameters=[get_params('rune_detector')],
        extra_arguments=[{'use_intra_process_comms': True}]
        )
    rune_solver_node = Node(
        package='rune_solver',
        executable='rune_solver_node',
        name='rune_solver',
        output='both',
        emulate_tty=True,
        parameters=[get_params('rune_solver')],
        arguments=['--ros-args',], 
        )

    # 使用intra cmmunication提高图像的传输速度
    def get_camera_detector_container(*detector_nodes):
        nodes_list = list(detector_nodes)
        # image_node may be None when camera is launched via separate launch include
        if image_node is not None:
            nodes_list.append(image_node)
        container = ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=nodes_list,
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', ],
        )
        return TimerAction(
            period=2.0,
            actions=[container],
        )

    # 延迟启动
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_armor_solver_node = TimerAction(
        period=2.0,
        actions=[armor_solver_node],
    )
    
    delay_rune_solver_node = TimerAction(
        period=2.0,
        actions=[rune_solver_node],
    )
    
    if launch_params['rune']:
        cam_detector_node = get_camera_detector_container(armor_detector_node, rune_detector_node)
    else:
        cam_detector_node = get_camera_detector_container(armor_detector_node)

    delay_cam_detector_node = TimerAction(
        period=2.0,
        actions=[cam_detector_node],
        ) 
    # if camera launched via include, insert its include into launch sequence after a short delay
    try:
        camera_include  # exists when hik include used
    except NameError:
        camera_include = None
    if camera_include is not None:
        delay_camera_include = TimerAction(period=1.0, actions=[camera_include])
        # insert camera include before container so it starts earlier
        launch_description_list = [ push_namespace, delay_camera_include, delay_cam_detector_node, delay_armor_solver_node]
    else:
        launch_description_list = [ push_namespace, delay_cam_detector_node, delay_armor_solver_node]
    
    if launch_params['rune']:
        launch_description_list.append(delay_rune_solver_node)
    
    if launch_params['navigation']:
        launch_description_list.append(robot_navigation_publisher)

    # [Method A] Connect vision TF tree root (imu_world) to the known TF tree root (odom).
    # This is an identity transform — imu_world IS odom for the vision system's purposes.
    # Without this, imu_world is an orphaned root that tf2 and Foxglove cannot resolve,
    # causing all vision markers (armor_solver debug, target sphere, etc.) to be invisible.
    imu_world_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_world_to_odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'imu_world'],
    )
    launch_description_list.append(imu_world_static_tf)

    return LaunchDescription(launch_description_list)
