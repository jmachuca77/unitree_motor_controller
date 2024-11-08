import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    declare_motor_config_file_cmd = DeclareLaunchArgument(
        'motor_config_file',
        default_value=os.path.join(
            get_package_share_directory('unitree_motor_controller'),
            'config',
            'unitree_motor_config.yaml'
        ),
        description='Full path to the motor config file to load'
    )

    declare_joint_state_publish_rate_cmd = DeclareLaunchArgument(
            'joint_state_publish_rate',
            default_value='10',
            description='Rate at which to publish joint states')

    declare_motor_command_publish_rate_cmd = DeclareLaunchArgument(
            'motor_command_publish_rate',
            default_value='100',
            description='Rate at which to send motor commands')

    # Node configuration
    motor_controller_node = Node(
        package='unitree_motor_controller',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('motor_config_file'),
            'joint_state_publish_rate': LaunchConfiguration('joint_state_publish_rate'),
            'motor_command_publish_rate': LaunchConfiguration('motor_command_publish_rate'),
        }]
    )

    return LaunchDescription([
        declare_motor_config_file_cmd,
        declare_joint_state_publish_rate_cmd,
        declare_motor_command_publish_rate_cmd,
        motor_controller_node
    ])
