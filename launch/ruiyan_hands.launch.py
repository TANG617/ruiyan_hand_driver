#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'left_rs485_port',
            default_value='/dev/ttyACM0',
            description='左手RS485串口端口'
        ),
        DeclareLaunchArgument(
            'left_rs485_baudrate',
            default_value='115200',
            description='左手RS485波特率'
        ),
        DeclareLaunchArgument(
            'left_motor_ids',
            default_value='[1,2,3,4,5,6]',
            description='左手电机ID列表'
        ),
        DeclareLaunchArgument(
            'right_rs485_port',
            default_value='/dev/ttyACM1',
            description='右手RS485串口端口'
        ),
        DeclareLaunchArgument(
            'right_rs485_baudrate',
            default_value='115200',
            description='右手RS485波特率'
        ),
        DeclareLaunchArgument(
            'right_motor_ids',
            default_value='[1,2,3,4,5,6]',
            description='右手电机ID列表'
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='100.0',
            description='控制频率(Hz)'
        ),
        
        # 启动瑞眼灵巧手控制节点
        Node(
            package='ruiyan_hand_driver',
            executable='ruiyan_hand_node',
            name='ruiyan_hand_node',
            output='screen',
            parameters=[{
                'left_rs485_port': LaunchConfiguration('left_rs485_port'),
                'left_rs485_baudrate': LaunchConfiguration('left_rs485_baudrate'),
                'left_motor_ids': LaunchConfiguration('left_motor_ids'),
                'right_rs485_port': LaunchConfiguration('right_rs485_port'),
                'right_rs485_baudrate': LaunchConfiguration('right_rs485_baudrate'),
                'right_motor_ids': LaunchConfiguration('right_motor_ids'),
                'control_rate': LaunchConfiguration('control_rate'),
            }]
        ),
    ])
