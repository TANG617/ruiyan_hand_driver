#!/usr/bin/env python3
"""
测试ROS2节点是否能正常控制灵巧手
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import logging

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RuiyanHandTestNode(Node):
    """瑞眼灵巧手测试节点"""
    
    def __init__(self):
        super().__init__('ruiyan_hand_test_node')
        
        # 创建发布者
        self.left_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/left/set_angles',
            10
        )
        
        # 创建订阅者
        self.left_joint_states_sub = self.create_subscription(
            JointState,
            '/ruiyan_hand/left/joint_states',
            self.left_joint_states_callback,
            10
        )
        
        # 关节名称
        self.joint_names = [f'left_joint_{i}' for i in range(1, 7)]
        
        # 状态跟踪
        self.left_current_states = None
        self.left_last_time = None
        
        self.get_logger().info("瑞眼灵巧手测试节点已启动")
        self.get_logger().info("等待ruiyan_hand_node启动...")
        
        # 等待一段时间让ruiyan_hand_node启动
        time.sleep(3.0)
    
    def left_joint_states_callback(self, msg: JointState):
        """左手关节状态回调"""
        self.left_current_states = msg
        self.left_last_time = time.time()
        self.get_logger().info(f"左手状态: {[f'{pos:.2f}' for pos in msg.position]}")
    
    def send_angles(self, angles):
        """发送角度命令"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = angles
        msg.velocity = [0.0] * len(angles)
        msg.effort = [0.0] * len(angles)
        
        self.left_publisher.publish(msg)
        self.get_logger().info(f"发送左手角度: {[f'{angle:.2f}' for angle in angles]}")
    
    def test_basic_movements(self):
        """测试基本动作"""
        self.get_logger().info("=== 开始基本动作测试 ===")
        
        # 1. 完全张开
        self.get_logger().info("1. 完全张开")
        open_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_angles(open_angles)
        time.sleep(3.0)
        
        # 2. 轻微弯曲
        self.get_logger().info("2. 轻微弯曲")
        slight_bend = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
        self.send_angles(slight_bend)
        time.sleep(3.0)
        
        # 3. 中等弯曲
        self.get_logger().info("3. 中等弯曲")
        medium_bend = [60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
        self.send_angles(medium_bend)
        time.sleep(3.0)
        
        # 4. 握拳
        self.get_logger().info("4. 握拳")
        fist_angles = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        self.send_angles(fist_angles)
        time.sleep(3.0)
        
        # 5. 回到张开
        self.get_logger().info("5. 回到张开")
        self.send_angles(open_angles)
        time.sleep(2.0)
        
        self.get_logger().info("=== 基本动作测试完成 ===")
    
    def monitor_feedback(self, duration=5):
        """监控反馈"""
        self.get_logger().info(f"=== 监控反馈 {duration}秒 ===")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.left_current_states and self.left_last_time:
                current_time = time.time()
                if current_time - self.left_last_time < 2.0:
                    self.get_logger().info(f"左手状态: {[f'{pos:.2f}' for pos in self.left_current_states.position]}")
                else:
                    self.get_logger().warn("左手状态未更新")
            
            rclpy.spin_once(self, timeout_sec=1.0)
    
    def run_test(self):
        """运行测试"""
        self.get_logger().info("开始ROS2节点测试...")
        
        # 先监控一下当前状态
        self.monitor_feedback(3)
        
        # 运行基本动作测试
        self.test_basic_movements()
        
        # 再次监控反馈
        self.monitor_feedback(3)
        
        self.get_logger().info("ROS2节点测试完成")


def main():
    """主函数"""
    rclpy.init()
    
    try:
        test_node = RuiyanHandTestNode()
        test_node.run_test()
        
    except KeyboardInterrupt:
        print("测试被中断")
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
