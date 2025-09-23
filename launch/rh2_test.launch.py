#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    rs485_port_arg = DeclareLaunchArgument(
        'rs485_port',
        default_value='/dev/ttyUSB0',
        description='RS485 serial device path'
    )
    
    rs485_baudrate_arg = DeclareLaunchArgument(
        'rs485_baudrate', 
        default_value='115200',
        description='RS485 baudrate'
    )
    
    motor_ids_arg = DeclareLaunchArgument(
        'motor_ids',
        default_value='[1,2,3,4,5,6]',
        description='Motor ID list'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100.0',
        description='Control loop frequency (Hz)'
    )
    
    ruiyan_hand_node = Node(
        package='ruiyan_hand_driver',
        executable='ruiyan_hand_node',
        name='ruiyan_hand_test',
        parameters=[{
            'rs485_port': LaunchConfiguration('rs485_port'),
            'rs485_baudrate': LaunchConfiguration('rs485_baudrate'),
            'motor_ids': LaunchConfiguration('motor_ids'),
            'control_rate': LaunchConfiguration('control_rate'),
        }],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        rs485_port_arg,
        rs485_baudrate_arg,
        motor_ids_arg,
        control_rate_arg,
        ruiyan_hand_node
    ])
