#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成瑞眼灵巧手测试启动配置"""
    
    # 声明启动参数
    rs485_port_arg = DeclareLaunchArgument(
        'rs485_port',
        default_value='/dev/ttyUSB0',
        description='RS485串口设备路径'
    )
    
    rs485_baudrate_arg = DeclareLaunchArgument(
        'rs485_baudrate', 
        default_value='115200',
        description='RS485波特率'
    )
    
    motor_ids_arg = DeclareLaunchArgument(
        'motor_ids',
        default_value='[1,2,3,4,5,6]',
        description='电机ID列表'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100.0',
        description='控制循环频率 (Hz)'
    )
    
    # 瑞眼灵巧手控制器节点
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
