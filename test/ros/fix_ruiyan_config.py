#!/usr/bin/env python3
"""
修复瑞眼灵巧手配置脚本
重新配置节点使用正确的端口
"""

import rclpy
from rclpy.node import Node
import time
import os

class RuiyanConfigFixer(Node):
    """瑞眼灵巧手配置修复节点"""
    
    def __init__(self):
        super().__init__('ruiyan_config_fixer')
        
        self.get_logger().info("瑞眼灵巧手配置修复节点已启动")
    
    def check_available_ports(self):
        """检查可用端口"""
        print("=== 检查可用端口 ===")
        
        import glob
        all_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        
        available_ports = []
        for port in all_ports:
            if os.path.exists(port) and os.access(port, os.R_OK | os.W_OK):
                available_ports.append(port)
                print(f"✅ 可用端口: {port}")
            else:
                print(f"❌ 端口不可用: {port}")
        
        return available_ports
    
    def test_port_communication(self, port):
        """测试端口通信"""
        print(f"\n=== 测试端口通信: {port} ===")
        
        try:
            import serial
            
            # 测试不同波特率
            baudrates = [115200, 9600, 38400, 57600]
            
            for baudrate in baudrates:
                print(f"测试波特率: {baudrate}")
                
                try:
                    ser = serial.Serial(
                        port=port,
                        baudrate=baudrate,
                        timeout=1.0,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS
                    )
                    
                    # 清空缓冲区
                    ser.flushInput()
                    ser.flushOutput()
                    
                    # 发送简单测试命令
                    test_data = b'\x01\x02\x03\x04'
                    ser.write(test_data)
                    time.sleep(0.1)
                    
                    # 尝试读取响应
                    response = ser.read(10)
                    if response:
                        print(f"  收到响应: {' '.join([f'{b:02X}' for b in response])}")
                        ser.close()
                        return baudrate
                    else:
                        print("  无响应")
                    
                    ser.close()
                    
                except Exception as e:
                    print(f"  错误: {e}")
            
            return None
            
        except ImportError:
            print("pyserial未安装")
            return None
    
    def create_working_config(self, port, baudrate):
        """创建工作配置"""
        print(f"\n=== 创建工作配置 ===")
        print(f"端口: {port}")
        print(f"波特率: {baudrate}")
        
        # 创建launch文件
        launch_content = f'''#!/usr/bin/env python3
"""
瑞眼灵巧手工作配置启动文件
自动检测到的配置: 端口={port}, 波特率={baudrate}
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ruiyan_hand_driver',
            executable='ruiyan_hand_node',
            name='ruiyan_hand_node',
            output='screen',
            parameters=[
                {{'left_rs485_port': '{port}'}},
                {{'left_rs485_baudrate': {baudrate}}},
                {{'left_motor_ids': [1, 2, 3, 4, 5, 6]}},
                {{'right_rs485_port': '{port}'}},  # 使用同一端口
                {{'right_rs485_baudrate': {baudrate}}},
                {{'right_motor_ids': [1, 2, 3, 4, 5, 6]}},
                {{'control_rate': 100.0}}
            ]
        )
    ])
'''
        
        # 写入launch文件
        # 使用相对路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        launch_file = os.path.join(current_dir, '..', '..', 'launch', 'ruiyan_hands_working.launch.py')
        with open(launch_file, 'w') as f:
            f.write(launch_content)
        
        os.chmod(launch_file, 0o755)
        print(f"✅ 工作配置已保存到: {launch_file}")
        
        return launch_file
    
    def run_fix(self):
        """运行修复流程"""
        print("开始修复瑞眼灵巧手配置...")
        
        # 1. 检查可用端口
        available_ports = self.check_available_ports()
        
        if not available_ports:
            print("❌ 没有找到可用的串口设备")
            return False
        
        # 2. 测试每个端口的通信
        working_configs = []
        
        for port in available_ports:
            baudrate = self.test_port_communication(port)
            if baudrate:
                working_configs.append((port, baudrate))
                print(f"✅ 找到工作配置: {port} @ {baudrate}")
        
        if not working_configs:
            print("❌ 没有找到工作的端口配置")
            print("\n可能的原因:")
            print("1. 瑞眼灵巧手设备未连接")
            print("2. 设备驱动未正确安装")
            print("3. 设备需要特殊的初始化序列")
            print("4. 协议不匹配")
            return False
        
        # 3. 创建工作配置
        best_config = working_configs[0]  # 使用第一个工作配置
        launch_file = self.create_working_config(best_config[0], best_config[1])
        
        print(f"\n✅ 修复完成!")
        print(f"工作配置: 端口={best_config[0]}, 波特率={best_config[1]}")
        print(f"启动文件: {launch_file}")
        print(f"\n使用方法:")
        print(f"ros2 launch ruiyan_hand_driver ruiyan_hands_working.launch.py")
        
        return True

def main():
    rclpy.init()
    
    try:
        fixer = RuiyanConfigFixer()
        fixer.run_fix()
        
    except KeyboardInterrupt:
        print("修复被中断")
    except Exception as e:
        print(f"修复过程中发生错误: {e}")
    finally:
        if 'fixer' in locals():
            fixer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
