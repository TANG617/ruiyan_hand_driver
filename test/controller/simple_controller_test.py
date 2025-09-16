#!/usr/bin/env python3
"""
最简单的瑞眼灵巧手控制器测试
"""

import sys
import os
import time

# 添加项目路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(current_dir, '..', '..')
sys.path.insert(0, project_root)

from ruiyan_hand_driver.communication_interface import CommunicationConfig
from ruiyan_hand_driver.ruiyan_hand_controller import DexhandController

def main():
    print("最简单的瑞眼灵巧手控制器测试")
    print("=" * 40)
    
    # 创建RS485配置
    config = CommunicationConfig.create_rs485_config(
        motor_ids=[1, 2, 3, 4, 5, 6],
        port='/dev/ttyACM0',
        baudrate=115200
    )
    
    # 创建控制器
    controller = DexhandController(config)
    
    try:
        # 连接设备
        print("正在连接设备...")
        if not controller.connect():
            print("❌ 设备连接失败")
            return
        print("✅ 设备连接成功")
        
        # 读取电机状态
        print("\n读取电机状态...")
        results = controller.get_motors_info_simple(timeout=0.1)
        
        for motor_id, info in results.items():
            if 'error' in info:
                print(f"电机{motor_id}: ❌ {info['error']}")
            else:
                position = info.get('current_position', 0)
                angle = info.get('position_normalized', 0.0) * 180.0
                print(f"电机{motor_id}: 位置={position}, 角度={angle:.1f}°")
        
        # 控制电机到中间位置
        print("\n控制电机到中间位置...")
        positions = [2048, 2048, 2048, 2048, 2048, 2048]  # 中间位置
        speeds = [100, 100, 100, 100, 100, 100]
        current_limits = [1000, 1000, 1000, 1000, 1000, 1000]
        
        success = controller.move_motors_fast(positions, speeds, current_limits)
        if success:
            print("✅ 控制命令发送成功")
        else:
            print("❌ 控制命令发送失败")
        
        # 等待2秒
        print("等待2秒...")
        time.sleep(2)
        
        # 再次读取状态
        print("\n再次读取电机状态...")
        results = controller.get_motors_info_simple(timeout=0.1)
        
        for motor_id, info in results.items():
            if 'error' in info:
                print(f"电机{motor_id}: ❌ {info['error']}")
            else:
                position = info.get('current_position', 0)
                angle = info.get('position_normalized', 0.0) * 180.0
                print(f"电机{motor_id}: 位置={position}, 角度={angle:.1f}°")
        
        print("\n✅ 测试完成")
        
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
    finally:
        # 断开连接
        controller.disconnect()
        print("设备连接已断开")

if __name__ == "__main__":
    main()
