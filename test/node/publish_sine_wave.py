#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import logging

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_publisher')
        
        # 发布器：向左手设置角度话题发送指令
        self.left_hand_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/left/set_angles',
            10
        )
        
        # 发布器：向右手设置角度话题发送指令
        self.right_hand_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/right/set_angles',
            10
        )
        
        # 定时器：10Hz频率发送指令
        self.timer = self.create_timer(0.01, self.publish_sine_wave)
        
        # 时间计数器
        self.start_time = time.time()
        
        # 正弦波参数
        self.amplitude = 2047.5  # 幅度 (0-4095范围的一半)
        self.frequency = 0.5     # 频率 (Hz) - 10秒一个循环
        self.num_joints = 6      # 关节数量
        self.controlled_joints = [2, 3, 4, 5]  # 控制ID 3-6 (索引2-5)
        
        self.get_logger().info('正弦波发布器已启动')
        self.get_logger().info(f'幅度: {self.amplitude}, 频率: {self.frequency} Hz')
        self.get_logger().info('开始发送正弦波指令...')

    def publish_sine_wave(self):
        """发布正弦波指令"""
        current_time = time.time() - self.start_time
        
        # 创建JointState消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "left_hand_base_link"
        
        # 设置关节名称
        msg.name = [f'left_hand_joint_{i+1}' for i in range(self.num_joints)]
        
        # 计算每个关节的正弦波位置
        positions = [0.0] * self.num_joints
        velocities = [1000.0] * self.num_joints
        efforts = [1000.0] * self.num_joints
        
        for i, joint_idx in enumerate(self.controlled_joints):
            # 每个关节有不同的相位偏移，创造波浪效果
            phase_offset = (i * 2 * math.pi) / len(self.controlled_joints)
            
            # 计算正弦波位置 (0-4095范围)
            sine_value = math.sin(2 * math.pi * self.frequency * current_time + phase_offset)
            position = float(int(self.amplitude * sine_value + self.amplitude + 0.5))  # 四舍五入到整数并转换为浮点数
            
            positions[joint_idx] = position
        
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        
        # 发布消息
        self.left_hand_publisher.publish(msg)
        
        # 打印当前状态（每秒打印一次）
        if int(current_time * 10) % 1 == 0:
            controlled_positions = [positions[i] for i in self.controlled_joints]
            self.get_logger().info(f'时间: {current_time:.1f}s, 控制关节位置: {controlled_positions}')

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SineWavePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('用户中断程序')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
