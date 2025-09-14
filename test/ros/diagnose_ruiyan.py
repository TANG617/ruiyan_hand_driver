#!/usr/bin/env python3
"""
瑞眼灵巧手诊断脚本
检查通信状态和协议问题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import sys
import os

# 添加项目路径 - 使用相对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(current_dir, '..', '..')
sys.path.insert(0, project_root)

# 尝试添加安装路径（如果存在）
install_path = os.path.join(project_root, '..', '..', 'install', 'ruiyan_hand_driver', 'lib', 'python3.10', 'site-packages')
if os.path.exists(install_path):
    sys.path.append(install_path)

from ruiyan_hand_driver.communication_interface import DexhandMessage, RS485Interface, CommunicationConfig
from ruiyan_hand_driver.ruiyan_hand_controller import DexhandController

class RuiyanDiagnostic(Node):
    """瑞眼灵巧手诊断节点"""
    
    def __init__(self):
        super().__init__('ruiyan_diagnostic')
        
        # 创建订阅者监听关节状态
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
        
        # 状态跟踪
        self.left_states = None
        self.right_states = None
        self.left_last_time = None
        self.right_last_time = None
        
        self.get_logger().info("瑞眼灵巧手诊断节点已启动")
    
    def left_joint_states_callback(self, msg: JointState):
        """左手关节状态回调"""
        self.left_states = msg
        self.left_last_time = time.time()
        self.get_logger().info(f"左手状态更新: {[f'{pos:.2f}' for pos in msg.position]}")
    
    def right_joint_states_callback(self, msg: JointState):
        """右手关节状态回调"""
        self.right_states = msg
        self.right_last_time = time.time()
        self.get_logger().info(f"右手状态更新: {[f'{pos:.2f}' for pos in msg.position]}")
    
    def check_communication(self):
        """检查通信状态"""
        self.get_logger().info("=== 检查通信状态 ===")
        
        # 检查串口设备
        import glob
        serial_devices = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        self.get_logger().info(f"可用串口设备: {serial_devices}")
        
        # 检查默认设备
        default_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for port in default_ports:
            if os.path.exists(port):
                self.get_logger().info(f"✅ 设备存在: {port}")
                # 检查权限
                if os.access(port, os.R_OK | os.W_OK):
                    self.get_logger().info(f"✅ 设备可读写: {port}")
                else:
                    self.get_logger().warn(f"❌ 设备权限不足: {port}")
            else:
                self.get_logger().warn(f"❌ 设备不存在: {port}")
    
    def test_direct_communication(self):
        """测试直接通信"""
        self.get_logger().info("=== 测试直接通信 ===")
        
        # 测试RS485接口
        try:
            rs485 = RS485Interface(port='/dev/ttyACM0', auto_connect=False)
            self.get_logger().info("RS485接口创建成功")
            
            # 尝试连接
            if rs485.connect():
                self.get_logger().info("✅ RS485连接成功")
                
                # 测试发送消息
                test_message = DexhandMessage(
                    command=0x00,
                    motor_id=0x00,
                    data_payload=[0xA0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                )
                
                if rs485.send_message(test_message):
                    self.get_logger().info("✅ 测试消息发送成功")
                    
                    # 尝试接收响应
                    response = rs485.receive_message(timeout=1.0)
                    if response:
                        self.get_logger().info(f"✅ 收到响应: {[hex(b) for b in response]}")
                    else:
                        self.get_logger().warn("❌ 未收到响应")
                else:
                    self.get_logger().error("❌ 测试消息发送失败")
                
                rs485.disconnect()
            else:
                self.get_logger().error("❌ RS485连接失败")
                
        except Exception as e:
            self.get_logger().error(f"❌ 直接通信测试失败: {e}")
    
    def test_controller(self):
        """测试控制器"""
        self.get_logger().info("=== 测试控制器 ===")
        
        try:
            # 创建配置
            config = CommunicationConfig.create_rs485_config(
                motor_ids=[1, 2, 3, 4, 5, 6],
                port='/dev/ttyACM0',
                auto_connect=False
            )
            
            # 创建控制器
            controller = DexhandController(config)
            self.get_logger().info("控制器创建成功")
            
            # 尝试连接
            if controller.connect():
                self.get_logger().info("✅ 控制器连接成功")
                
                # 测试读取电机信息
                self.get_logger().info("测试读取电机信息...")
                results = controller.get_motors_info(timeout=2.0)
                self.get_logger().info(f"读取结果: {results}")
                
                # 测试控制电机
                self.get_logger().info("测试控制电机...")
                positions = [100, 200, 300, 400, 500, 600]
                speeds = [100, 100, 100, 100, 100, 100]
                currents = [500, 500, 500, 500, 500, 500]
                
                control_results = controller.move_motors(positions, speeds, currents, timeout=2.0)
                self.get_logger().info(f"控制结果: {control_results}")
                
                controller.disconnect()
            else:
                self.get_logger().error("❌ 控制器连接失败")
                
        except Exception as e:
            self.get_logger().error(f"❌ 控制器测试失败: {e}")
    
    def monitor_joint_states(self, duration=10):
        """监控关节状态"""
        self.get_logger().info(f"=== 监控关节状态 {duration}秒 ===")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.left_states:
                current_time = time.time()
                if self.left_last_time and current_time - self.left_last_time < 1.0:
                    self.get_logger().info(f"左手状态: {[f'{pos:.2f}' for pos in self.left_states.position]}")
                else:
                    self.get_logger().warn("左手状态未更新")
            
            if self.right_states:
                current_time = time.time()
                if self.right_last_time and current_time - self.right_last_time < 1.0:
                    self.get_logger().info(f"右手状态: {[f'{pos:.2f}' for pos in self.right_states.position]}")
                else:
                    self.get_logger().warn("右手状态未更新")
            
            rclpy.spin_once(self, timeout_sec=1.0)
    
    def run_diagnosis(self):
        """运行完整诊断"""
        self.get_logger().info("开始瑞眼灵巧手诊断...")
        
        # 1. 检查通信状态
        self.check_communication()
        time.sleep(1.0)
        
        # 2. 测试直接通信
        self.test_direct_communication()
        time.sleep(1.0)
        
        # 3. 测试控制器
        self.test_controller()
        time.sleep(1.0)
        
        # 4. 监控关节状态
        self.monitor_joint_states(5)
        
        self.get_logger().info("诊断完成")


def main():
    rclpy.init()
    
    try:
        diagnostic = RuiyanDiagnostic()
        diagnostic.run_diagnosis()
        
    except KeyboardInterrupt:
        print("诊断被中断")
    except Exception as e:
        print(f"诊断过程中发生错误: {e}")
    finally:
        if 'diagnostic' in locals():
            diagnostic.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
