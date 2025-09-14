#!/usr/bin/env python3
"""
使用USB端口测试瑞眼灵巧手
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class USBTestController(Node):
    """USB端口测试控制器"""
    
    def __init__(self):
        super().__init__('usb_test_controller')
        
        # 创建发布者 - 使用USB端口
        self.left_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/left/set_angles',
            10
        )
        
        self.right_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/right/set_angles',
            10
        )
        
        # 创建订阅者
        self.left_joint_states_sub = self.create_subscription(
            JointState,
            '/ruiyan_hand/left/joint_states',
            self.left_joint_states_callback,
            10
        )
        
        self.right_joint_states_sub = self.create_subscription(
            JointState,
            '/ruiyan_hand/right/joint_states',
            self.right_joint_states_callback,
            10
        )
        
        # 关节名称
        self.joint_names = [f'joint_{i}' for i in range(1, 7)]
        
        # 状态跟踪
        self.left_states = None
        self.right_states = None
        self.left_last_time = None
        self.right_last_time = None
        
        self.get_logger().info("USB测试控制器已启动")
        self.get_logger().info("注意: 请确保ruiyan_hand_node使用USB端口配置")
    
    def left_joint_states_callback(self, msg: JointState):
        """左手关节状态回调"""
        self.left_states = msg
        self.left_last_time = time.time()
        self.get_logger().info(f"左手状态: {[f'{pos:.2f}' for pos in msg.position]}")
    
    def right_joint_states_callback(self, msg: JointState):
        """右手关节状态回调"""
        self.right_states = msg
        self.right_last_time = time.time()
        self.get_logger().info(f"右手状态: {[f'{pos:.2f}' for pos in msg.position]}")
    
    def send_angles(self, left_angles=None, right_angles=None):
        """发送角度命令"""
        if left_angles is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'left_{name}' for name in self.joint_names]
            msg.position = left_angles
            self.left_publisher.publish(msg)
            self.get_logger().info(f"发送左手角度: {[f'{angle:.2f}' for angle in left_angles]}")
        
        if right_angles is not None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'right_{name}' for name in self.joint_names]
            msg.position = right_angles
            self.right_publisher.publish(msg)
            self.get_logger().info(f"发送右手角度: {[f'{angle:.2f}' for angle in right_angles]}")
    
    def test_basic_movements(self):
        """测试基本动作"""
        self.get_logger().info("=== 开始基本动作测试 ===")
        
        # 1. 完全张开
        self.get_logger().info("1. 完全张开")
        open_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_angles(left_angles=open_angles, right_angles=open_angles)
        time.sleep(3.0)
        
        # 2. 轻微弯曲
        self.get_logger().info("2. 轻微弯曲")
        slight_bend = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
        self.send_angles(left_angles=slight_bend, right_angles=slight_bend)
        time.sleep(3.0)
        
        # 3. 中等弯曲
        self.get_logger().info("3. 中等弯曲")
        medium_bend = [60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
        self.send_angles(left_angles=medium_bend, right_angles=medium_bend)
        time.sleep(3.0)
        
        # 4. 握拳
        self.get_logger().info("4. 握拳")
        fist_angles = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        self.send_angles(left_angles=fist_angles, right_angles=fist_angles)
        time.sleep(3.0)
        
        # 5. 回到张开
        self.get_logger().info("5. 回到张开")
        self.send_angles(left_angles=open_angles, right_angles=open_angles)
        time.sleep(2.0)
        
        self.get_logger().info("=== 基本动作测试完成 ===")
    
    def monitor_feedback(self, duration=10):
        """监控反馈"""
        self.get_logger().info(f"=== 监控反馈 {duration}秒 ===")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.left_states and self.left_last_time:
                current_time = time.time()
                if current_time - self.left_last_time < 2.0:
                    self.get_logger().info(f"左手状态: {[f'{pos:.2f}' for pos in self.left_states.position]}")
                else:
                    self.get_logger().warn("左手状态未更新")
            
            if self.right_states and self.right_last_time:
                current_time = time.time()
                if current_time - self.right_last_time < 2.0:
                    self.get_logger().info(f"右手状态: {[f'{pos:.2f}' for pos in self.right_states.position]}")
                else:
                    self.get_logger().warn("右手状态未更新")
            
            rclpy.spin_once(self, timeout_sec=1.0)
    
    def run_test(self):
        """运行测试"""
        self.get_logger().info("开始USB端口测试...")
        
        # 先监控一下当前状态
        self.monitor_feedback(5)
        
        # 运行基本动作测试
        self.test_basic_movements()
        
        # 再次监控反馈
        self.monitor_feedback(5)
        
        self.get_logger().info("USB端口测试完成")


def main():
    rclpy.init()
    
    try:
        test_controller = USBTestController()
        test_controller.run_test()
        
    except KeyboardInterrupt:
        print("测试被中断")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
    finally:
        if 'test_controller' in locals():
            test_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
