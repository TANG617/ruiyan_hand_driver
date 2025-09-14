#!/usr/bin/env python3
"""
瑞眼灵巧手交互式测试脚本
允许用户手动输入角度值进行测试
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class InteractiveRuiyanTest(Node):
    """交互式测试节点"""
    
    def __init__(self):
        super().__init__('interactive_ruiyan_test')
        
        # 创建发布者
        self.left_pub = self.create_publisher(JointState, '/ruiyan_hand/left/set_angles', 10)
        self.right_pub = self.create_publisher(JointState, '/ruiyan_hand/right/set_angles', 10)
        
        # 关节名称
        self.joint_names = [f'joint_{i}' for i in range(1, 7)]
        
        self.get_logger().info("交互式测试节点已启动")
        self.print_help()
    
    def print_help(self):
        """打印帮助信息"""
        print("\n" + "="*60)
        print("瑞眼灵巧手交互式测试")
        print("="*60)
        print("命令:")
        print("  l <角度1> <角度2> ... <角度6>  - 设置左手角度")
        print("  r <角度1> <角度2> ... <角度6>  - 设置右手角度")
        print("  b <角度1> <角度2> ... <角度6>  - 设置双手角度")
        print("  open                            - 张开双手")
        print("  fist                            - 握拳")
        print("  peace                           - 和平手势")
        print("  ok                              - OK手势")
        print("  point                           - 指向手势")
        print("  help                            - 显示帮助")
        print("  quit                            - 退出")
        print("="*60)
        print("角度范围: 0-180度")
        print("示例: l 90 90 90 90 90 90  (左手握拳)")
        print("="*60)
    
    def send_left_angles(self, angles):
        """发送左手角度"""
        if len(angles) != 6:
            print("错误: 需要6个角度值")
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'left_{name}' for name in self.joint_names]
        msg.position = angles
        self.left_pub.publish(msg)
        print(f"左手角度已设置: {angles}")
    
    def send_right_angles(self, angles):
        """发送右手角度"""
        if len(angles) != 6:
            print("错误: 需要6个角度值")
            return
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'right_{name}' for name in self.joint_names]
        msg.position = angles
        self.right_pub.publish(msg)
        print(f"右手角度已设置: {angles}")
    
    def send_both_angles(self, angles):
        """发送双手角度"""
        self.send_left_angles(angles)
        self.send_right_angles(angles)
    
    def open_hands(self):
        """张开双手"""
        angles = [0.0] * 6
        self.send_both_angles(angles)
        print("双手已张开")
    
    def make_fist(self):
        """握拳"""
        angles = [90.0] * 6
        self.send_both_angles(angles)
        print("双手已握拳")
    
    def peace_gesture(self):
        """和平手势"""
        angles = [0.0, 90.0, 0.0, 90.0, 0.0, 0.0]
        self.send_both_angles(angles)
        print("和平手势")
    
    def ok_gesture(self):
        """OK手势"""
        angles = [0.0, 0.0, 90.0, 90.0, 90.0, 0.0]
        self.send_both_angles(angles)
        print("OK手势")
    
    def point_gesture(self):
        """指向手势"""
        angles = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]
        self.send_both_angles(angles)
        print("指向手势")
    
    def process_command(self, command):
        """处理命令"""
        parts = command.strip().split()
        if not parts:
            return True
        
        cmd = parts[0].lower()
        
        if cmd == 'quit' or cmd == 'exit':
            return False
        elif cmd == 'help':
            self.print_help()
        elif cmd == 'open':
            self.open_hands()
        elif cmd == 'fist':
            self.make_fist()
        elif cmd == 'peace':
            self.peace_gesture()
        elif cmd == 'ok':
            self.ok_gesture()
        elif cmd == 'point':
            self.point_gesture()
        elif cmd == 'l':  # 左手
            try:
                angles = [float(x) for x in parts[1:]]
                self.send_left_angles(angles)
            except (ValueError, IndexError):
                print("错误: 请输入6个有效的角度值")
        elif cmd == 'r':  # 右手
            try:
                angles = [float(x) for x in parts[1:]]
                self.send_right_angles(angles)
            except (ValueError, IndexError):
                print("错误: 请输入6个有效的角度值")
        elif cmd == 'b':  # 双手
            try:
                angles = [float(x) for x in parts[1:]]
                self.send_both_angles(angles)
            except (ValueError, IndexError):
                print("错误: 请输入6个有效的角度值")
        else:
            print(f"未知命令: {cmd}")
            print("输入 'help' 查看帮助")
        
        return True
    
    def run(self):
        """运行交互式测试"""
        print("\n开始交互式测试...")
        print("输入命令控制手部，输入 'quit' 退出")
        
        try:
            while True:
                command = input("\n> ").strip()
                if not self.process_command(command):
                    break
                
                # 处理ROS2消息
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            print("\n测试被中断")
        except EOFError:
            print("\n输入结束")


def main():
    rclpy.init()
    
    try:
        test_node = InteractiveRuiyanTest()
        test_node.run()
        
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
