#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    left_port_arg = DeclareLaunchArgument(
        'left_rs485_port',
        default_value='/dev/ttyUSB0',
        description='RS485 serial port for the left hand'
    )
    left_baudrate_arg = DeclareLaunchArgument(
        'left_rs485_baudrate',
        default_value='115200',
        description='RS485 baudrate for the left hand'
    )
    left_motor_ids_arg = DeclareLaunchArgument(
        'left_motor_ids',
        default_value='[1, 2, 3, 4, 5, 6]',
        description='Motor IDs for the left hand'
    )

    right_port_arg = DeclareLaunchArgument(
        'right_rs485_port',
        default_value='/dev/ttyUSB1',
        description='RS485 serial port for the right hand'
    )
    right_baudrate_arg = DeclareLaunchArgument(
        'right_rs485_baudrate',
        default_value='115200',
        description='RS485 baudrate for the right hand'
    )
    right_motor_ids_arg = DeclareLaunchArgument(
        'right_motor_ids',
        default_value='[1, 2, 3, 4, 5, 6]',
        description='Motor IDs for the right hand'
    )

    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100.0',
        description='Control frequency in Hz'
    )

    return LaunchDescription([
        left_port_arg,
        left_baudrate_arg,
        left_motor_ids_arg,
        right_port_arg,
        right_baudrate_arg,
        right_motor_ids_arg,
        control_rate_arg,
        Node(
            package='ruiyan_hand_driver',
            executable='ruiyan_hand_node',
            name='ruiyan_hand_node',
            output='screen',
            parameters=[
                {'left_rs485_port': LaunchConfiguration('left_rs485_port')},
                {'left_rs485_baudrate': LaunchConfiguration('left_rs485_baudrate')},
                {'left_motor_ids': LaunchConfiguration('left_motor_ids')},
                {'right_rs485_port': LaunchConfiguration('right_rs485_port')},
                {'right_rs485_baudrate': LaunchConfiguration('right_rs485_baudrate')},
                {'right_motor_ids': LaunchConfiguration('right_motor_ids')},
                {'control_rate': LaunchConfiguration('control_rate')}
            ]
        )
    ])
