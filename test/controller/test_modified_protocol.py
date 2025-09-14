#!/usr/bin/env python3
"""
测试修改后的RS485协议
验证新的帧格式是否能正常控制灵巧手
"""

import sys
import os
import logging

# 添加项目路径 - 使用相对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(current_dir, '..', '..')
sys.path.insert(0, project_root)

from ruiyan_hand_driver.communication_interface import CommunicationConfig, CommunicationType, DexhandMessage
from ruiyan_hand_driver.ruiyan_hand_controller import DexhandController

# 配置日志
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_rs485_protocol():
    """测试修改后的RS485协议"""
    logger.info("开始测试修改后的RS485协议")
    
    # 创建RS485配置
    config = CommunicationConfig.create_rs485_config(
        motor_ids=[1, 2, 3, 4, 5, 6],
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=2.0
    )
    
    # 创建控制器
    controller = DexhandController(config)
    
    try:
        # 连接
        if not controller.connect():
            logger.error("无法连接到RS485设备")
            return False
        
        logger.info("RS485设备连接成功")
        
        # 测试1: 发送与test_rs485.py相同的命令
        logger.info("=" * 60)
        logger.info("测试1: 发送与test_rs485.py相同的命令")
        logger.info("=" * 60)
        
        # 解析测试命令: A5 01 00 18 AA 00 00 E8 03 F4 01 00 AA 00 08 E8 03 F4 01 00 AA FF 0F E8 03 F4 01 00 72
        # A5 - header
        # 01 00 - ID (电机1)
        # 18 - 数据长度 (24字节)
        # AA 00 00 E8 03 F4 01 00 AA 00 08 E8 03 F4 01 00 AA FF 0F E8 03 F4 01 00 - 数据
        # 72 - 校验和
        
        # 构建相同的数据
        test_data = [
            0xAA, 0x00, 0x00, 0xE8, 0x03, 0xF4, 0x01, 0x00,  # 电机1数据
            0xAA, 0x00, 0x08, 0xE8, 0x03, 0xF4, 0x01, 0x00,  # 电机2数据  
            0xAA, 0xFF, 0x0F, 0xE8, 0x03, 0xF4, 0x01, 0x00   # 电机3数据
        ]
        
        message = DexhandMessage(
            command=0xAA,
            motor_id=1,  # 使用电机1的ID
            data_payload=test_data,
            timeout=2.0
        )
        
        # 发送消息
        if controller.comm_manager.send_message(message):
            logger.info("测试命令发送成功")
            
            # 尝试接收响应
            response = controller.comm_manager.receive_message(timeout=2.0)
            if response:
                logger.info(f"收到响应: {[hex(x) for x in response]}")
            else:
                logger.warning("未收到响应")
        else:
            logger.error("测试命令发送失败")
        
        # 测试2: 使用控制器API发送控制命令
        logger.info("=" * 60)
        logger.info("测试2: 使用控制器API发送控制命令")
        logger.info("=" * 60)
        
        # 设置一些测试位置
        positions = [1000, 2000, 3000, 1000, 2000, 3000]  # 6个电机的位置
        speeds = [100, 100, 100, 100, 100, 100]           # 6个电机的速度
        current_limits = [1000, 1000, 1000, 1000, 1000, 1000]  # 6个电机的电流限制
        
        logger.info(f"发送控制命令 - 位置: {positions}")
        logger.info(f"发送控制命令 - 速度: {speeds}")
        logger.info(f"发送控制命令 - 电流限制: {current_limits}")
        
        results = controller.move_motors(positions, speeds, current_limits)
        logger.info(f"控制结果: {results}")
        
        # 测试3: 获取电机信息
        logger.info("=" * 60)
        logger.info("测试3: 获取电机信息")
        logger.info("=" * 60)
        
        motor_info = controller.get_motors_info()
        logger.info(f"电机信息: {motor_info}")
        
        return True
        
    except Exception as e:
        logger.error(f"测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        controller.disconnect()
        logger.info("RS485连接已断开")


def test_frame_building():
    """测试帧构建功能"""
    logger.info("=" * 60)
    logger.info("测试帧构建功能")
    logger.info("=" * 60)
    
    # 创建RS485接口
    from ruiyan_hand_driver.communication_interface import RS485Interface
    
    rs485 = RS485Interface(port='/dev/ttyACM0', auto_connect=False)
    
    # 测试消息
    test_message = DexhandMessage(
        command=0xAA,
        motor_id=1,
        data_payload=[0xAA, 0x00, 0x00, 0xE8, 0x03, 0xF4, 0x01, 0x00],
        timeout=1.0
    )
    
    # 构建帧
    frame = rs485._build_rs485_frame(test_message)
    frame_hex = ' '.join([f'{byte:02X}' for byte in frame])
    
    logger.info(f"构建的帧: {frame_hex}")
    logger.info(f"帧长度: {len(frame)} 字节")
    
    # 验证帧格式
    if len(frame) >= 6:
        header = frame[0]
        id_low = frame[1]
        id_high = frame[2]
        length = frame[3]
        data = frame[4:-1]
        checksum = frame[-1]
        
        logger.info(f"Header: 0x{header:02X}")
        logger.info(f"ID: 0x{id_low:02X} 0x{id_high:02X} (={id_low | (id_high << 8)})")
        logger.info(f"Length: {length}")
        logger.info(f"Data: {' '.join([f'{byte:02X}' for byte in data])}")
        logger.info(f"Checksum: 0x{checksum:02X}")
        
        # 验证校验和
        calculated_checksum = sum(frame[:-1]) & 0xFF
        logger.info(f"计算的校验和: 0x{calculated_checksum:02X}")
        
        if calculated_checksum == checksum:
            logger.info("✓ 校验和正确")
        else:
            logger.error("✗ 校验和错误")
    else:
        logger.error("帧长度不足")


def main():
    """主函数"""
    print("修改后的RS485协议测试")
    print("=" * 50)
    
    # 测试帧构建
    test_frame_building()
    
    print("\n" + "=" * 50)
    print("开始实际通信测试...")
    print("=" * 50)
    
    # 测试实际通信
    success = test_rs485_protocol()
    
    if success:
        print("\n✓ 所有测试完成")
    else:
        print("\n✗ 测试失败")


if __name__ == '__main__':
    main()
