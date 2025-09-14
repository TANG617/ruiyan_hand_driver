#!/usr/bin/env python3
"""
瑞眼灵巧手简单测试脚本
快速测试手部控制功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class SimpleRuiyanTest(Node):
    """简单测试节点"""
    
    def __init__(self):
        super().__init__('simple_ruiyan_test')
        
        # 创建发布者
        self.left_pub = self.create_publisher(JointState, '/ruiyan_hand/left/set_angles', 10)
        self.right_pub = self.create_publisher(JointState, '/ruiyan_hand/right/set_angles', 10)
        
        # 关节名称
        self.joint_names = [f'joint_{i}' for i in range(1, 7)]
        
        self.get_logger().info("简单测试节点已启动")
    
    def send_command(self, left_angles=None, right_angles=None):
        """发送控制命令"""
        if left_angles is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'left_{name}' for name in self.joint_names]
            msg.position = left_angles
            self.left_pub.publish(msg)
            self.get_logger().info(f"左手: {left_angles}")
        
        if right_angles is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'right_{name}' for name in self.joint_names]
            msg.position = right_angles
            self.right_pub.publish(msg)
            self.get_logger().info(f"右手: {right_angles}")
    
    def run_quick_test(self):
        """运行快速测试"""
        self.get_logger().info("开始快速测试...")
        
        # 1. 张开
        self.get_logger().info("1. 张开")
        open_angles = [0.0] * 6
        self.send_command(left_angles=open_angles, right_angles=open_angles)
        time.sleep(2.0)
        
        # 2. 握拳
        self.get_logger().info("2. 握拳")
        fist_angles = [90.0] * 6
        self.send_command(left_angles=fist_angles, right_angles=fist_angles)
        time.sleep(2.0)
        
        # 3. 回到张开
        self.get_logger().info("3. 回到张开")
        self.send_command(left_angles=open_angles, right_angles=open_angles)
        time.sleep(1.0)
        
        self.get_logger().info("快速测试完成！")


def main():
    rclpy.init()
    
    try:
        test_node = SimpleRuiyanTest()
        test_node.run_quick_test()
        
        # 保持节点运行一段时间
        rclpy.spin_once(test_node, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        print("测试被中断")
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
