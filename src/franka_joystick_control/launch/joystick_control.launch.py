#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    robot_pc_ip_arg = DeclareLaunchArgument(
        'robot_pc_ip',
        default_value='192.168.18.1',
        description='IP address of the realtime PC running the Franka controller'
    )
    
    robot_pc_port_arg = DeclareLaunchArgument(
        'robot_pc_port',
        default_value='8888',
        description='Port number for communication with realtime PC'
    )
    
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='true',
        description='Use Python version (true) or C++ version (false)'
    )
    
    # Joy node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 50.0,
        }]
    )
    
    # Python joystick publisher (default)
    joystick_node = Node(
        package='franka_joystick_control',
        executable='franka_joystick_publisher',
        name='franka_joystick_publisher',
        parameters=[{
            'robot_pc_ip': LaunchConfiguration('robot_pc_ip'),
            'robot_pc_port': LaunchConfiguration('robot_pc_port'),
        }]
    )
    
    return LaunchDescription([
        robot_pc_ip_arg,
        robot_pc_port_arg,
        joy_node,
        joystick_node,
    ])