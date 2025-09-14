#!/usr/bin/env python3
"""
瑞眼灵巧手协议调试脚本
尝试不同的协议参数和格式
"""

import sys
import os
import time
import serial

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

def test_raw_serial():
    """测试原始串口通信"""
    print("=== 测试原始串口通信 ===")
    
    ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    baudrates = [115200, 9600, 38400, 57600]
    
    for port in ports:
        if not os.path.exists(port):
            continue
            
        print(f"\n测试端口: {port}")
        
        for baudrate in baudrates:
            try:
                print(f"  测试波特率: {baudrate}")
                
                # 创建串口连接
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
                
                # 发送测试数据 - 根据您提供的协议示例
                test_data = bytes.fromhex("A5 01 00 18 A0 01 00 00 00 00 00 00 A0 02 00 00 00 00 00 00 A0 03 00 00 00 00 00 00 1C")
                print(f"  发送数据: {' '.join([f'{b:02X}' for b in test_data])}")
                
                ser.write(test_data)
                time.sleep(0.1)
                
                # 尝试读取响应
                response = ser.read(64)
                if response:
                    print(f"  收到响应: {' '.join([f'{b:02X}' for b in response])}")
                else:
                    print("  未收到响应")
                
                ser.close()
                
            except Exception as e:
                print(f"  错误: {e}")

def test_different_motor_ids():
    """测试不同的电机ID"""
    print("\n=== 测试不同的电机ID ===")
    
    # 尝试不同的电机ID组合
    motor_id_combinations = [
        [0, 0, 0, 0, 0, 0],  # 全部为0
        [1, 2, 3, 4, 5, 6],  # 标准ID
        [0, 1, 2, 3, 4, 5],  # 从0开始
        [1, 1, 1, 1, 1, 1],  # 全部为1
    ]
    
    for motor_ids in motor_id_combinations:
        print(f"\n测试电机ID: {motor_ids}")
        
        try:
            # 创建配置
            config = CommunicationConfig.create_rs485_config(
                motor_ids=motor_ids,
                port='/dev/ttyACM0',
                auto_connect=False
            )
            
            # 创建控制器
            controller = DexhandController(config)
            
            if controller.connect():
                print("  连接成功")
                
                # 测试读取电机信息
                results = controller.get_motors_info(timeout=2.0)
                print(f"  读取结果: {results}")
                
                if results:
                    print("  ✅ 找到响应！")
                    break
                else:
                    print("  ❌ 无响应")
                
                controller.disconnect()
            else:
                print("  连接失败")
                
        except Exception as e:
            print(f"  错误: {e}")

def test_different_commands():
    """测试不同的命令格式"""
    print("\n=== 测试不同的命令格式 ===")
    
    try:
        rs485 = RS485Interface(port='/dev/ttyACM0', auto_connect=False)
        
        if rs485.connect():
            print("RS485连接成功")
            
            # 测试不同的命令格式
            test_commands = [
                # 原始协议示例格式
                {
                    'name': '原始读取命令',
                    'data': [0xA0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
                },
                {
                    'name': '原始控制命令',
                    'data': [0xAA, 0x01, 0x00, 0x00, 0xE8, 0x03, 0xF4, 0x01]
                },
                # 简化的单电机命令
                {
                    'name': '单电机读取',
                    'data': [0xA0, 0x01]
                },
                {
                    'name': '单电机控制',
                    'data': [0xAA, 0x01, 0x00, 0x00, 0xE8, 0x03, 0xF4, 0x01]
                },
                # 广播命令
                {
                    'name': '广播读取',
                    'data': [0xA0, 0x00]
                },
                {
                    'name': '广播控制',
                    'data': [0xAA, 0x00, 0x00, 0x00, 0xE8, 0x03, 0xF4, 0x01]
                }
            ]
            
            for cmd in test_commands:
                print(f"\n测试: {cmd['name']}")
                
                # 创建消息
                message = DexhandMessage(
                    command=0x00,
                    motor_id=0x00,
                    data_payload=cmd['data']
                )
                
                # 构建帧
                frame = rs485._build_rs485_frame(message)
                print(f"  发送帧: {' '.join([f'{b:02X}' for b in frame])}")
                
                # 发送消息
                if rs485.send_message(message):
                    print("  发送成功")
                    
                    # 尝试接收响应
                    response = rs485.receive_message(timeout=2.0)
                    if response:
                        print(f"  收到响应: {' '.join([f'{b:02X}' for b in response])}")
                    else:
                        print("  未收到响应")
                else:
                    print("  发送失败")
            
            rs485.disconnect()
        else:
            print("RS485连接失败")
            
    except Exception as e:
        print(f"错误: {e}")

def test_baudrate_scan():
    """扫描波特率"""
    print("\n=== 扫描波特率 ===")
    
    baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    
    for baudrate in baudrates:
        print(f"\n测试波特率: {baudrate}")
        
        try:
            config = CommunicationConfig.create_rs485_config(
                motor_ids=[1, 2, 3, 4, 5, 6],
                port='/dev/ttyACM0',
                baudrate=baudrate,
                auto_connect=False
            )
            
            controller = DexhandController(config)
            
            if controller.connect():
                print("  连接成功")
                
                # 测试读取
                results = controller.get_motors_info(timeout=1.0)
                if results:
                    print(f"  ✅ 找到响应: {results}")
                    break
                else:
                    print("  无响应")
                
                controller.disconnect()
            else:
                print("  连接失败")
                
        except Exception as e:
            print(f"  错误: {e}")

def main():
    print("瑞眼灵巧手协议调试")
    print("=" * 50)
    
    # 1. 测试原始串口通信
    test_raw_serial()
    
    # 2. 测试不同的电机ID
    test_different_motor_ids()
    
    # 3. 测试不同的命令格式
    test_different_commands()
    
    # 4. 扫描波特率
    test_baudrate_scan()
    
    print("\n调试完成")

if __name__ == '__main__':
    main()
