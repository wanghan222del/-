import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))

robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
    ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'publish_frequency': 1000.0}]
)

node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

def create_tracker_node(parameter_overrides=None):
    parameters = [node_params]
    if parameter_overrides:
        parameters.append(parameter_overrides)
    return Node(
        package='armor_tracker',
        executable='armor_tracker_node',
        output='both',
        emulate_tty=True,
        parameters=parameters,
        ros_arguments=['--log-level', 'armor_tracker:='+launch_params['tracker_log_level']],
    )


tracker_node = create_tracker_node()
