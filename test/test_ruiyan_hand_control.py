#!/usr/bin/env python3
"""
瑞眼灵巧手控制测试脚本
向set_angles话题发送JointState消息来控制手部
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class RuiyanHandTestController(Node):
    """瑞眼灵巧手测试控制器"""
    
    def __init__(self):
        super().__init__('ruiyan_hand_test_controller')
        
        # 创建发布者 - 左手
        self.left_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/left/set_angles',
            10
        )
        
        # 创建发布者 - 右手
        self.right_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/right/set_angles',
            10
        )
        
        # 创建订阅者 - 监听关节状态
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
        self.joint_names = [
            'left_joint_1', 'left_joint_2', 'left_joint_3',
            'left_joint_4', 'left_joint_5', 'left_joint_6'
        ]
        
        self.right_joint_names = [
            'right_joint_1', 'right_joint_2', 'right_joint_3',
            'right_joint_4', 'right_joint_5', 'right_joint_6'
        ]
        
        # 状态跟踪
        self.left_current_states = None
        self.right_current_states = None
        
        self.get_logger().info("瑞眼灵巧手测试控制器已启动")
        self.get_logger().info("Topics:")
        self.get_logger().info("  发布: /ruiyan_hand/left/set_angles")
        self.get_logger().info("  发布: /ruiyan_hand/right/set_angles")
        self.get_logger().info("  订阅: /ruiyan_hand/left/joint_states")
        self.get_logger().info("  订阅: /ruiyan_hand/right/joint_states")
    
    def left_joint_states_callback(self, msg: JointState):
        """左手关节状态回调"""
        self.left_current_states = msg
        self.get_logger().debug(f"左手状态: {[f'{pos:.2f}' for pos in msg.position]}")
    
    def right_joint_states_callback(self, msg: JointState):
        """右手关节状态回调"""
        self.right_current_states = msg
        self.get_logger().debug(f"右手状态: {[f'{pos:.2f}' for pos in msg.position]}")
    
    def create_joint_state_msg(self, angles, joint_names, hand_type="left"):
        """创建JointState消息"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{hand_type}_hand"
        msg.name = joint_names
        msg.position = angles
        msg.velocity = [0.0] * len(angles)
        msg.effort = [0.0] * len(angles)
        return msg
    
    def send_angles(self, left_angles=None, right_angles=None):
        """发送角度命令"""
        if left_angles is not None:
            msg = self.create_joint_state_msg(left_angles, self.joint_names, "left")
            self.left_publisher.publish(msg)
            self.get_logger().info(f"发送左手角度: {[f'{angle:.2f}' for angle in left_angles]}")
        
        if right_angles is not None:
            msg = self.create_joint_state_msg(right_angles, self.right_joint_names, "right")
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
    
    def test_individual_fingers(self):
        """测试单个手指"""
        self.get_logger().info("=== 开始单个手指测试 ===")
        
        # 初始状态
        base_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_angles(left_angles=base_angles, right_angles=base_angles)
        time.sleep(2.0)
        
        # 逐个测试每个手指
        for finger in range(6):
            self.get_logger().info(f"测试手指 {finger + 1}")
            test_angles = base_angles.copy()
            test_angles[finger] = 90.0
            self.send_angles(left_angles=test_angles, right_angles=test_angles)
            time.sleep(2.0)
            
            # 回到初始状态
            self.send_angles(left_angles=base_angles, right_angles=base_angles)
            time.sleep(1.0)
        
        self.get_logger().info("=== 单个手指测试完成 ===")
    
    def test_hand_gestures(self):
        """测试手势"""
        self.get_logger().info("=== 开始手势测试 ===")
        
        # 1. 和平手势 (V字手势)
        self.get_logger().info("1. 和平手势")
        peace_angles = [0.0, 90.0, 0.0, 90.0, 0.0, 0.0]
        self.send_angles(left_angles=peace_angles, right_angles=peace_angles)
        time.sleep(3.0)
        
        # 2. OK手势
        self.get_logger().info("2. OK手势")
        ok_angles = [0.0, 0.0, 90.0, 90.0, 90.0, 0.0]
        self.send_angles(left_angles=ok_angles, right_angles=ok_angles)
        time.sleep(3.0)
        
        # 3. 指向手势
        self.get_logger().info("3. 指向手势")
        point_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]
        self.send_angles(left_angles=point_angles, right_angles=point_angles)
        time.sleep(3.0)
        
        # 4. 回到张开
        self.get_logger().info("4. 回到张开")
        open_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_angles(left_angles=open_angles, right_angles=open_angles)
        time.sleep(2.0)
        
        self.get_logger().info("=== 手势测试完成 ===")
    
    def test_smooth_movement(self):
        """测试平滑运动"""
        self.get_logger().info("=== 开始平滑运动测试 ===")
        
        # 从张开到握拳的平滑过渡
        steps = 20
        for i in range(steps + 1):
            angle = (i / steps) * 90.0
            angles = [angle] * 6
            self.send_angles(left_angles=angles, right_angles=angles)
            time.sleep(0.1)
        
        # 从握拳到张开的平滑过渡
        for i in range(steps + 1):
            angle = 90.0 - (i / steps) * 90.0
            angles = [angle] * 6
            self.send_angles(left_angles=angles, right_angles=angles)
            time.sleep(0.1)
        
        self.get_logger().info("=== 平滑运动测试完成 ===")
    
    def test_individual_hands(self):
        """测试单手控制"""
        self.get_logger().info("=== 开始单手控制测试 ===")
        
        # 只控制左手
        self.get_logger().info("只控制左手")
        left_angles = [45.0, 45.0, 45.0, 45.0, 45.0, 45.0]
        self.send_angles(left_angles=left_angles)
        time.sleep(3.0)
        
        # 只控制右手
        self.get_logger().info("只控制右手")
        right_angles = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0]
        self.send_angles(right_angles=right_angles)
        time.sleep(3.0)
        
        # 回到初始状态
        self.get_logger().info("回到初始状态")
        self.send_angles(left_angles=[0.0] * 6, right_angles=[0.0] * 6)
        time.sleep(2.0)
        
        self.get_logger().info("=== 单手控制测试完成 ===")
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info("开始运行所有测试...")
        
        try:
            self.test_basic_movements()
            time.sleep(2.0)
            
            self.test_individual_fingers()
            time.sleep(2.0)
            
            self.test_hand_gestures()
            time.sleep(2.0)
            
            self.test_smooth_movement()
            time.sleep(2.0)
            
            self.test_individual_hands()
            
            self.get_logger().info("所有测试完成！")
            
        except KeyboardInterrupt:
            self.get_logger().info("测试被用户中断")
        except Exception as e:
            self.get_logger().error(f"测试过程中发生错误: {e}")
        finally:
            # 确保回到安全位置
            self.get_logger().info("回到安全位置")
            self.send_angles(left_angles=[0.0] * 6, right_angles=[0.0] * 6)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        # 创建测试控制器
        test_controller = RuiyanHandTestController()
        
        print("\n瑞眼灵巧手控制测试脚本")
        print("=" * 50)
        print("可用测试:")
        print("1. 基本动作测试")
        print("2. 单个手指测试")
        print("3. 手势测试")
        print("4. 平滑运动测试")
        print("5. 单手控制测试")
        print("6. 运行所有测试")
        print("=" * 50)
        
        # 等待用户选择
        while True:
            try:
                choice = input("\n请选择测试 (1-6, 或按 Ctrl+C 退出): ").strip()
                
                if choice == '1':
                    test_controller.test_basic_movements()
                elif choice == '2':
                    test_controller.test_individual_fingers()
                elif choice == '3':
                    test_controller.test_hand_gestures()
                elif choice == '4':
                    test_controller.test_smooth_movement()
                elif choice == '5':
                    test_controller.test_individual_hands()
                elif choice == '6':
                    test_controller.run_all_tests()
                else:
                    print("无效选择，请输入 1-6")
                    continue
                
                # 运行ROS2循环
                rclpy.spin_once(test_controller, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                print("\n退出测试")
                break
            except Exception as e:
                print(f"测试过程中发生错误: {e}")
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        if 'test_controller' in locals():
            # 确保回到安全位置
            test_controller.send_angles(left_angles=[0.0] * 6, right_angles=[0.0] * 6)
            time.sleep(1.0)
            test_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
