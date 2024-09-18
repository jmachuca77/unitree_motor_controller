import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    declare_publish_rate_cmd = DeclareLaunchArgument(
        'publish_rate',
        default_value='100',
        description='Publish rate for the motor controller')

    declare_motor_type_cmd = DeclareLaunchArgument(
        'motor_type',
        default_value='GO_M8010_6',
        description='Type of the motor')

    declare_motor_id_cmd = DeclareLaunchArgument(
        'motor_id',
        default_value='0',
        description='ID of the motor')

    declare_joint_name_cmd = DeclareLaunchArgument(
        'joint_name',
        default_value='unitree_motor_joint',
        description='Name of the joint')

    # Node configuration
    motor_controller_node = Node(
        package='unitree_motor_controller',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'motor_type': LaunchConfiguration('motor_type'),
            'motor_id': LaunchConfiguration('motor_id'),
            'joint_name': LaunchConfiguration('joint_name')
        }]
    )

    return LaunchDescription([
        declare_publish_rate_cmd,
        declare_motor_type_cmd,
        declare_motor_id_cmd,
        declare_joint_name_cmd,
        motor_controller_node
    ])