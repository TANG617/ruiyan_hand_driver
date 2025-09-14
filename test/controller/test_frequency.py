#!/usr/bin/env python3
"""
测试控制频率的脚本
验证修改后的代码是否能达到100Hz的控制频率
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
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_control_frequency():
    """测试控制频率"""
    logger.info("开始测试控制频率...")
    
    # 创建RS485配置
    config = CommunicationConfig.create_rs485_config(
        motor_ids=[1, 2, 3, 4, 5, 6],
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=0.01  # 10ms超时
    )
    
    # 创建控制器
    controller = DexhandController(config)
    
    try:
        # 连接
        if not controller.connect():
            logger.error("无法连接到设备")
            return False
        
        logger.info("设备连接成功，开始频率测试...")
        
        # 测试参数
        positions = [2048, 2048, 2048, 2048, 2048, 2048]  # 中间位置
        speeds = [100, 100, 100, 100, 100, 100]
        current_limits = [1000, 1000, 1000, 1000, 1000, 1000]
        
        # 测试快速控制方法
        logger.info("测试快速控制方法...")
        start_time = time.time()
        test_duration = 10.0  # 测试10秒
        count = 0
        
        while time.time() - start_time < test_duration:
            success = controller.move_motors_fast(positions, speeds, current_limits)
            if success:
                count += 1
            else:
                logger.warn(f"第{count+1}次快速控制失败")
        
        actual_duration = time.time() - start_time
        frequency = count / actual_duration
        
        logger.info(f"快速控制测试结果:")
        logger.info(f"  测试时间: {actual_duration:.2f}秒")
        logger.info(f"  执行次数: {count}")
        logger.info(f"  实际频率: {frequency:.2f}Hz")
        
        # 测试普通控制方法
        logger.info("测试普通控制方法...")
        start_time = time.time()
        count = 0
        
        while time.time() - start_time < test_duration:
            results = controller.move_motors(positions, speeds, current_limits, timeout=0.01)
            if results:
                count += 1
            else:
                logger.warn(f"第{count+1}次普通控制失败")
        
        actual_duration = time.time() - start_time
        frequency = count / actual_duration
        
        logger.info(f"普通控制测试结果:")
        logger.info(f"  测试时间: {actual_duration:.2f}秒")
        logger.info(f"  执行次数: {count}")
        logger.info(f"  实际频率: {frequency:.2f}Hz")
        
        return True
        
    except Exception as e:
        logger.error(f"测试过程中发生错误: {e}")
        return False
    finally:
        controller.disconnect()
        logger.info("设备连接已断开")

def test_communication_timeout():
    """测试通信超时设置"""
    logger.info("开始测试通信超时设置...")
    
    # 创建RS485配置
    config = CommunicationConfig.create_rs485_config(
        motor_ids=[1, 2, 3, 4, 5, 6],
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=0.01  # 10ms超时
    )
    
    # 创建控制器
    controller = DexhandController(config)
    
    try:
        # 连接
        if not controller.connect():
            logger.error("无法连接到设备")
            return False
        
        logger.info("设备连接成功，开始超时测试...")
        
        # 测试不同超时时间
        timeout_values = [0.001, 0.01, 0.05, 0.1, 0.5, 1.0]
        positions = [2048, 2048, 2048, 2048, 2048, 2048]
        speeds = [100, 100, 100, 100, 100, 100]
        current_limits = [1000, 1000, 1000, 1000, 1000, 1000]
        
        for timeout in timeout_values:
            logger.info(f"测试超时时间: {timeout}秒")
            start_time = time.time()
            
            # 执行10次控制命令
            for i in range(10):
                results = controller.move_motors(positions, speeds, current_limits, timeout=timeout)
            
            actual_duration = time.time() - start_time
            avg_time = actual_duration / 10
            
            logger.info(f"  10次控制总时间: {actual_duration:.3f}秒")
            logger.info(f"  平均每次时间: {avg_time:.3f}秒")
            logger.info(f"  理论最大频率: {1.0/avg_time:.2f}Hz")
        
        return True
        
    except Exception as e:
        logger.error(f"测试过程中发生错误: {e}")
        return False
    finally:
        controller.disconnect()
        logger.info("设备连接已断开")

if __name__ == "__main__":
    print("瑞眼灵巧手控制频率测试")
    print("=" * 50)
    
    # 检查设备权限
    if not os.access('/dev/ttyACM0', os.R_OK | os.W_OK):
        print("错误: 没有访问/dev/ttyACM0的权限")
        print("请运行: sudo chmod 666 /dev/ttyACM0")
        sys.exit(1)
    
    try:
        # 测试控制频率
        if test_control_frequency():
            print("\n✅ 控制频率测试完成")
        else:
            print("\n❌ 控制频率测试失败")
        
        print("\n" + "=" * 50)
        
        # 测试通信超时
        if test_communication_timeout():
            print("\n✅ 通信超时测试完成")
        else:
            print("\n❌ 通信超时测试失败")
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
