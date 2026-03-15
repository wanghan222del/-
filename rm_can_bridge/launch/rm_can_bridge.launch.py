from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    reconnect_interval_ms = LaunchConfiguration("reconnect_interval_ms")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    armors_topic = LaunchConfiguration("armors_topic")
    target_topic = LaunchConfiguration("target_topic")
    input_mode = LaunchConfiguration("input_mode")
    angle_unit = LaunchConfiguration("angle_unit")
    target_timeout_ms = LaunchConfiguration("target_timeout_ms")
    publish_feedback = LaunchConfiguration("publish_feedback")
    feedback_can_id = LaunchConfiguration("feedback_can_id")
    feedback_frame_id = LaunchConfiguration("feedback_frame_id")
    expected_target_frame = LaunchConfiguration("expected_target_frame")

    declare_args = [
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("reconnect_interval_ms", default_value="1000"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("armors_topic", default_value="/detector/armors"),
        DeclareLaunchArgument("target_topic", default_value="/tracker/target"),
        DeclareLaunchArgument("input_mode", default_value="detector_armors"),
        DeclareLaunchArgument("angle_unit", default_value="deg"),
        DeclareLaunchArgument("target_timeout_ms", default_value="300"),
        DeclareLaunchArgument("publish_feedback", default_value="true"),
        DeclareLaunchArgument("feedback_can_id", default_value="1026"),
        DeclareLaunchArgument("feedback_frame_id", default_value="gimbal_link"),
        DeclareLaunchArgument("expected_target_frame", default_value="gimbal_link"),
    ]

    node = Node(
        package="rm_can_bridge",
        executable="rm_can_bridge_node",
        name="rm_can_bridge",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "reconnect_interval_ms": reconnect_interval_ms,
                "cmd_vel_topic": cmd_vel_topic,
                "armors_topic": armors_topic,
                "target_topic": target_topic,
                "input_mode": input_mode,
                "angle_unit": angle_unit,
                "target_timeout_ms": target_timeout_ms,
                "publish_feedback": publish_feedback,
                "feedback_can_id": feedback_can_id,
                "feedback_frame_id": feedback_frame_id,
                "expected_target_frame": expected_target_frame,
            }
        ],
    )

    return LaunchDescription(declare_args + [node])
