#!/usr/bin/env python3
"""
测试状态读取功能的脚本
验证电机状态读取是否正常工作
"""

import sys
import os
import time
import logging

# 添加项目路径 - 使用相对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(current_dir, '..', '..')
sys.path.insert(0, project_root)

from ruiyan_hand_driver.communication_interface import CommunicationConfig, CommunicationType
from ruiyan_hand_driver.ruiyan_hand_controller import DexhandController

# 配置日志
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_status_reading():
    """测试状态读取功能"""
    logger.info("开始测试状态读取功能...")
    
    # 创建RS485配置
    config = CommunicationConfig.create_rs485_config(
        motor_ids=[1, 2, 3, 4, 5, 6],
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=0.1
    )
    
    # 创建控制器
    controller = DexhandController(config)
    
    try:
        # 连接
        if not controller.connect():
            logger.error("无法连接到设备")
            return False
        
        logger.info("设备连接成功，开始状态读取测试...")
        
        # 测试1: 简单状态读取
        logger.info("测试1: 简单状态读取")
        for i in range(5):
            logger.info(f"第{i+1}次读取:")
            results = controller.get_motors_info_simple(timeout=0.1)
            for motor_id, info in results.items():
                if 'error' in info:
                    logger.warning(f"电机{motor_id}: {info['error']}")
                else:
                    logger.info(f"电机{motor_id}: 位置={info.get('current_position', 0)}, "
                              f"角度={info.get('position_normalized', 0.0)*180:.2f}°, "
                              f"速度={info.get('current_speed', 0)}, "
                              f"电流={info.get('current_ma', 0)}mA")
            time.sleep(0.5)
        
        # 测试2: 多电机状态读取
        logger.info("测试2: 多电机状态读取")
        for i in range(3):
            logger.info(f"第{i+1}次多电机读取:")
            results = controller.get_motors_info(timeout=0.2)
            for motor_id, info in results.items():
                if 'error' in info:
                    logger.warning(f"电机{motor_id}: {info['error']}")
                else:
                    logger.info(f"电机{motor_id}: 位置={info.get('current_position', 0)}, "
                              f"角度={info.get('position_normalized', 0.0)*180:.2f}°, "
                              f"速度={info.get('current_speed', 0)}, "
                              f"电流={info.get('current_ma', 0)}mA")
            time.sleep(0.5)
        
        # 测试3: 控制命令后的状态读取
        logger.info("测试3: 控制命令后的状态读取")
        positions = [2048, 2048, 2048, 2048, 2048, 2048]  # 中间位置
        speeds = [100, 100, 100, 100, 100, 100]
        current_limits = [1000, 1000, 1000, 1000, 1000, 1000]
        
        # 发送控制命令
        logger.info("发送控制命令...")
        success = controller.move_motors_fast(positions, speeds, current_limits)
        logger.info(f"控制命令发送结果: {success}")
        
        # 等待一段时间后读取状态
        time.sleep(0.1)
        logger.info("读取控制后的状态:")
        results = controller.get_motors_info_simple(timeout=0.1)
        for motor_id, info in results.items():
            if 'error' in info:
                logger.warning(f"电机{motor_id}: {info['error']}")
            else:
                logger.info(f"电机{motor_id}: 位置={info.get('current_position', 0)}, "
                          f"角度={info.get('position_normalized', 0.0)*180:.2f}°, "
                          f"速度={info.get('current_speed', 0)}, "
                          f"电流={info.get('current_ma', 0)}mA")
        
        return True
        
    except Exception as e:
        logger.error(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        controller.disconnect()
        logger.info("设备连接已断开")

def test_communication_debug():
    """测试通信调试"""
    logger.info("开始通信调试测试...")
    
    # 创建RS485配置
    config = CommunicationConfig.create_rs485_config(
        motor_ids=[1, 2, 3, 4, 5, 6],
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=0.1
    )
    
    # 创建控制器
    controller = DexhandController(config)
    
    try:
        # 连接
        if not controller.connect():
            logger.error("无法连接到设备")
            return False
        
        logger.info("设备连接成功，开始通信调试...")
        
        # 测试原始通信
        logger.info("测试原始通信...")
        from ruiyan_hand_driver.communication_interface import DexhandMessage
        
        # 创建测试消息
        test_message = DexhandMessage(
            command=0xA0,  # 读取电机信息
            motor_id=1,
            data_payload=[1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            timeout=0.1
        )
        
        # 发送消息
        logger.info("发送测试消息...")
        success = controller.comm_manager.send_message(test_message)
        logger.info(f"消息发送结果: {success}")
        
        # 接收响应
        logger.info("接收响应...")
        response = controller.comm_manager.receive_message(timeout=0.2)
        if response:
            logger.info(f"收到响应: {[hex(x) for x in response]}")
        else:
            logger.warning("未收到响应")
        
        return True
        
    except Exception as e:
        logger.error(f"通信调试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        controller.disconnect()
        logger.info("设备连接已断开")

if __name__ == "__main__":
    print("瑞眼灵巧手状态读取测试")
    print("=" * 50)
    
    # 检查设备权限
    if not os.access('/dev/ttyACM0', os.R_OK | os.W_OK):
        print("错误: 没有访问/dev/ttyACM0的权限")
        print("请运行: sudo chmod 666 /dev/ttyACM0")
        sys.exit(1)
    
    try:
        # 测试状态读取
        if test_status_reading():
            print("\n✅ 状态读取测试完成")
        else:
            print("\n❌ 状态读取测试失败")
        
        print("\n" + "=" * 50)
        
        # 测试通信调试
        if test_communication_debug():
            print("\n✅ 通信调试测试完成")
        else:
            print("\n❌ 通信调试测试失败")
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
