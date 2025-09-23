#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import logging

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        
        self.left_hand_subscription = self.create_subscription(
            JointState,
            '/ruiyan_hand/left/joint_states',
            self.left_hand_callback,
            10
        )
        
        self.right_hand_subscription = self.create_subscription(
            JointState,
            '/ruiyan_hand/right/joint_states',
            self.right_hand_callback,
            10
        )
        
        self.get_logger().info('Joint state subscriber started')
        self.get_logger().info('Waiting to receive joint state data...')

    def left_hand_callback(self, msg: JointState):
        self.get_logger().info('=== Left Hand Joint State ===')
        self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        self.get_logger().info(f'Joint names: {msg.name}')
        self.get_logger().info(f'Joint positions: {msg.position}')
        self.get_logger().info(f'Joint velocities: {msg.velocity}')
        self.get_logger().info(f'Joint efforts: {msg.effort}')
        self.get_logger().info('==========================')

    def right_hand_callback(self, msg: JointState):
        self.get_logger().info('=== Right Hand Joint State ===')
        self.get_logger().info(f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.get_logger().info(f'Frame ID: {msg.header.frame_id}')
        self.get_logger().info(f'Joint names: {msg.name}')
        self.get_logger().info(f'Joint positions: {msg.position}')
        self.get_logger().info(f'Joint velocities: {msg.velocity}')
        self.get_logger().info(f'Joint efforts: {msg.effort}')
        self.get_logger().info('==========================')

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = JointStateSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('User interrupted program')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
