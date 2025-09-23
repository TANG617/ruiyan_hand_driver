#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import logging

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        
        # 订阅左手关节状态话题
        self.left_hand_subscription = self.create_subscription(
            JointState,
            '/ruiyan_hand/left/joint_states',
            self.left_hand_callback,
            10
        )
        
        # 订阅右手关节状态话题
        self.right_hand_subscription = self.create_subscription(
            JointState,
            '/ruiyan_hand/right/joint_states',
            self.right_hand_callback,
            10
        )
        
        self.get_logger().info('关节状态订阅器已启动')
        self.get_logger().info('等待接收关节状态数据...')

    def left_hand_callback(self, msg: JointState):
        """左手关节状态回调函数"""
        self.get_logger().info('=== 左手关节状态 ===')
        self.get_logger().info(f'时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.get_logger().info(f'坐标系: {msg.header.frame_id}')
        self.get_logger().info(f'关节名称: {msg.name}')
        self.get_logger().info(f'关节位置: {msg.position}')
        self.get_logger().info(f'关节速度: {msg.velocity}')
        self.get_logger().info(f'关节力矩: {msg.effort}')
        self.get_logger().info('==================')

    def right_hand_callback(self, msg: JointState):
        """右手关节状态回调函数"""
        self.get_logger().info('=== 右手关节状态 ===')
        self.get_logger().info(f'时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.get_logger().info(f'坐标系: {msg.header.frame_id}')
        self.get_logger().info(f'关节名称: {msg.name}')
        self.get_logger().info(f'关节位置: {msg.position}')
        self.get_logger().info(f'关节速度: {msg.velocity}')
        self.get_logger().info(f'关节力矩: {msg.effort}')
        self.get_logger().info('==================')

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = JointStateSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('用户中断程序')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
