#!/usr/bin/env python3
"""
测试USB端口上的瑞眼灵巧手设备
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

def test_usb_port(port, baudrate=115200):
    """测试USB端口"""
    print(f"\n=== 测试端口: {port} (波特率: {baudrate}) ===")
    
    try:
        # 创建配置
        config = CommunicationConfig.create_rs485_config(
            motor_ids=[1, 2, 3, 4, 5, 6],
            port=port,
            baudrate=baudrate,
            auto_connect=False
        )
        
        # 创建控制器
        controller = DexhandController(config)
        
        if controller.connect():
            print("✅ 控制器连接成功")
            
            # 测试读取电机信息
            print("测试读取电机信息...")
            results = controller.get_motors_info(timeout=3.0)
            print(f"读取结果: {results}")
            
            if results:
                print("✅ 找到电机响应！")
                for motor_id, info in results.items():
                    if 'error' not in info:
                        print(f"  电机{motor_id}: 位置={info.get('current_position', 0)}, "
                              f"速度={info.get('current_speed', 0)}, "
                              f"电流={info.get('current_ma', 0)}mA")
                    else:
                        print(f"  电机{motor_id}: {info['error']}")
                
                # 测试控制电机
                print("\n测试控制电机...")
                positions = [100, 200, 300, 400, 500, 600]
                speeds = [100, 100, 100, 100, 100, 100]
                currents = [500, 500, 500, 500, 500, 500]
                
                control_results = controller.move_motors(positions, speeds, currents, timeout=3.0)
                print(f"控制结果: {control_results}")
                
                if control_results:
                    print("✅ 电机控制成功！")
                    return True
                else:
                    print("❌ 电机控制失败")
            else:
                print("❌ 未找到电机响应")
            
            controller.disconnect()
        else:
            print("❌ 控制器连接失败")
            
    except Exception as e:
        print(f"❌ 测试失败: {e}")
    
    return False

def test_different_motor_ids_on_usb(port):
    """在USB端口上测试不同的电机ID"""
    print(f"\n=== 在{port}上测试不同电机ID ===")
    
    motor_id_combinations = [
        [1, 2, 3, 4, 5, 6],  # 标准ID
        [0, 1, 2, 3, 4, 5],  # 从0开始
        [1, 1, 1, 1, 1, 1],  # 全部为1
        [0, 0, 0, 0, 0, 0],  # 全部为0
        [1],                 # 单个电机
        [0],                 # 单个电机ID=0
    ]
    
    for motor_ids in motor_id_combinations:
        print(f"\n测试电机ID: {motor_ids}")
        
        try:
            config = CommunicationConfig.create_rs485_config(
                motor_ids=motor_ids,
                port=port,
                baudrate=115200,
                auto_connect=False
            )
            
            controller = DexhandController(config)
            
            if controller.connect():
                results = controller.get_motors_info(timeout=2.0)
                print(f"  读取结果: {results}")
                
                if results:
                    print("  ✅ 找到响应！")
                    return motor_ids
                else:
                    print("  ❌ 无响应")
                
                controller.disconnect()
            else:
                print("  连接失败")
                
        except Exception as e:
            print(f"  错误: {e}")
    
    return None

def main():
    print("测试USB端口上的瑞眼灵巧手设备")
    print("=" * 50)
    
    # 测试USB端口
    usb_ports = ['/dev/ttyACM0']
    baudrates = [115200]
    
    working_ports = []
    
    for port in usb_ports:
        if not os.path.exists(port):
            print(f"端口 {port} 不存在")
            continue
        
        print(f"\n检查端口: {port}")
        
        # 测试不同波特率
        for baudrate in baudrates:
            if test_usb_port(port, baudrate):
                working_ports.append((port, baudrate))
                break
        
        # 如果找到了工作端口，测试不同电机ID
        if working_ports:
            last_port = working_ports[-1][0]
            working_motor_ids = test_different_motor_ids_on_usb(last_port)
            if working_motor_ids:
                print(f"\n✅ 找到工作配置:")
                print(f"  端口: {last_port}")
                print(f"  波特率: {working_ports[-1][1]}")
                print(f"  电机ID: {working_motor_ids}")
                break
    
    if not working_ports:
        print("\n❌ 未找到工作的USB端口配置")
        print("\n建议:")
        print("1. 检查瑞眼灵巧手是否正确连接到USB端口")
        print("2. 检查设备驱动是否正确安装")
        print("3. 尝试使用不同的USB端口")
        print("4. 检查设备是否需要特殊的初始化序列")
    else:
        print(f"\n✅ 找到 {len(working_ports)} 个工作配置")

if __name__ == '__main__':
    main()
