import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():
    from common import create_tracker_node, node_params, launch_params, robot_state_publisher
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch_ros.descriptions import ComposableNode

    serial_port = LaunchConfiguration('serial_port')
    can_bridge_params = LaunchConfiguration('can_bridge_params')

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:=' + launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    if launch_params['camera'] == 'hik':
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif launch_params['camera'] == 'mv':
        cam_detector = get_camera_detector_container(mv_camera_node)
    else:
        raise RuntimeError(f"Unsupported camera type: {launch_params['camera']}")

    can_bridge_node = Node(
        package='rm_can_bridge',
        executable='rm_can_bridge_node',
        name='rm_can_bridge',
        output='both',
        emulate_tty=True,
        parameters=[
            can_bridge_params,
            {
                'serial_port': serial_port,
            }
        ],
        on_exit=Shutdown(),
    )

    delay_can_bridge_node = TimerAction(
        period=1.5,
        actions=[can_bridge_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[create_tracker_node({'target_frame': 'gimbal_link'})],
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'can_bridge_params',
            default_value=os.path.join(
                get_package_share_directory('rm_can_bridge'),
                'config',
                'tracker_target_bridge.yaml',
            ),
        ),
        robot_state_publisher,
        cam_detector,
        delay_can_bridge_node,
        delay_tracker_node,
    ])
