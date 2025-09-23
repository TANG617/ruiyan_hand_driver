#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import logging

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class SineWavePublisher(Node):
    def __init__(self):
        super().__init__('sine_wave_publisher')
        
        self.left_hand_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/left/set_angles',
            10
        )
        
        self.right_hand_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/right/set_angles',
            10
        )
        
        self.timer = self.create_timer(0.01, self.publish_sine_wave)
        
        self.start_time = time.time()
        
        self.amplitude = 2047.5
        self.frequency = 0.5
        self.num_joints = 6
        self.controlled_joints = [2, 3, 4, 5]
        
        self.get_logger().info('Sine wave publisher started')
        self.get_logger().info(f'Amplitude: {self.amplitude}, Frequency: {self.frequency} Hz')
        self.get_logger().info('Starting to send sine wave commands...')

    def publish_sine_wave(self):
        current_time = time.time() - self.start_time
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "left_hand_base_link"
        
        msg.name = [f'left_hand_joint_{i+1}' for i in range(self.num_joints)]
        
        positions = [0.0] * self.num_joints
        velocities = [1000.0] * self.num_joints
        efforts = [1000.0] * self.num_joints
        
        for i, joint_idx in enumerate(self.controlled_joints):
            phase_offset = (i * 2 * math.pi) / len(self.controlled_joints)
            
            sine_value = math.sin(2 * math.pi * self.frequency * current_time + phase_offset)
            position = float(int(self.amplitude * sine_value + self.amplitude + 0.5))
            
            positions[joint_idx] = position
        
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        
        self.left_hand_publisher.publish(msg)
        
        if int(current_time * 10) % 1 == 0:
            controlled_positions = [positions[i] for i in self.controlled_joints]
            self.get_logger().info(f'Time: {current_time:.1f}s, Controlled joint positions: {controlled_positions}')

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SineWavePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('User interrupted program')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
