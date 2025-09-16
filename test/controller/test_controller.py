#!/usr/bin/env python3
"""
测试控制器代码
使用ruiyan_hand_controller控制瑞眼灵巧手
提供交互式控制界面和自动化测试功能
"""

import sys
import os
import time
import logging
import threading
from typing import List, Dict, Optional

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

class RuiyanHandTester:
    """瑞眼灵巧手测试控制器"""
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200, 
                 motor_ids: List[int] = None):
        """
        初始化测试控制器
        
        Args:
            port: RS485端口
            baudrate: 波特率
            motor_ids: 电机ID列表
        """
        if motor_ids is None:
            motor_ids = [1, 2, 3, 4, 5, 6]
        
        self.motor_ids = motor_ids
        self.port = port
        self.baudrate = baudrate
        
        # 创建通信配置
        self.config = CommunicationConfig.create_rs485_config(
            motor_ids=motor_ids,
            port=port,
            baudrate=baudrate,
            timeout=0.1
        )
        
        # 创建控制器
        self.controller = DexhandController(self.config)
        self.connected = False
        
        # 状态监控
        self.monitoring = False
        self.monitor_thread = None
        self.current_status = {}
        
        logger.info(f"瑞眼灵巧手测试控制器初始化完成")
        logger.info(f"端口: {port}, 波特率: {baudrate}, 电机ID: {motor_ids}")
    
    def connect(self) -> bool:
        """连接设备"""
        try:
            if self.controller.connect():
                self.connected = True
                logger.info("设备连接成功")
                return True
            else:
                logger.error("设备连接失败")
                return False
        except Exception as e:
            logger.error(f"连接过程中发生错误: {e}")
            return False
    
    def disconnect(self):
        """断开设备连接"""
        try:
            self.stop_monitoring()
            self.controller.disconnect()
            self.connected = False
            logger.info("设备连接已断开")
        except Exception as e:
            logger.error(f"断开连接时发生错误: {e}")
    
    def get_motor_status(self) -> Dict[int, Dict]:
        """获取所有电机状态"""
        if not self.connected:
            logger.error("设备未连接")
            return {}
        
        try:
            # 使用简单方法逐个读取电机状态
            results = self.controller.get_motors_info_simple(timeout=0.1)
            self.current_status = results
            return results
        except Exception as e:
            logger.error(f"获取电机状态时发生错误: {e}")
            return {}
    
    def print_motor_status(self):
        """打印电机状态"""
        status = self.get_motor_status()
        if not status:
            logger.warning("无法获取电机状态")
            return
        
        print("\n" + "="*60)
        print("电机状态信息")
        print("="*60)
        
        for motor_id in self.motor_ids:
            if motor_id in status:
                info = status[motor_id]
                if 'error' in info:
                    print(f"电机{motor_id}: ❌ {info['error']}")
                else:
                    position = info.get('current_position', 0)
                    angle = info.get('position_normalized', 0.0) * 180.0
                    speed = info.get('current_speed', 0)
                    current = info.get('current_ma', 0)
                    status_desc = info.get('status_description', 'unknown')
                    
                    print(f"电机{motor_id}: 位置={position:4d} | 角度={angle:6.2f}° | "
                          f"速度={speed:4d} | 电流={current:4d}mA | 状态={status_desc}")
            else:
                print(f"电机{motor_id}: ❌ 无数据")
        
        print("="*60)
    
    def move_motors(self, positions: List[int], speeds: List[int] = None, 
                   current_limits: List[int] = None) -> bool:
        """
        控制电机位置
        
        Args:
            positions: 目标位置列表 (0-4095)
            speeds: 速度列表 (默认100)
            current_limits: 电流限制列表 (默认1000mA)
        """
        if not self.connected:
            logger.error("设备未连接")
            return False
        
        if len(positions) != len(self.motor_ids):
            logger.error(f"位置数量({len(positions)})与电机数量({len(self.motor_ids)})不匹配")
            return False
        
        if speeds is None:
            speeds = [100] * len(self.motor_ids)
        if current_limits is None:
            current_limits = [1000] * len(self.motor_ids)
        
        try:
            # 使用快速控制方法
            success = self.controller.move_motors_fast(positions, speeds, current_limits)
            if success:
                logger.info(f"控制命令已发送: 位置={positions}")
            else:
                logger.error("控制命令发送失败")
            return success
        except Exception as e:
            logger.error(f"控制电机时发生错误: {e}")
            return False
    
    def move_motors_by_angles(self, angles: List[float], speeds: List[int] = None,
                             current_limits: List[int] = None) -> bool:
        """
        通过角度控制电机
        
        Args:
            angles: 目标角度列表 (0-180度)
            speeds: 速度列表 (默认100)
            current_limits: 电流限制列表 (默认1000mA)
        """
        if len(angles) != len(self.motor_ids):
            logger.error(f"角度数量({len(angles)})与电机数量({len(self.motor_ids)})不匹配")
            return False
        
        # 角度转位置 (0-180度 -> 0-4095)
        positions = []
        for angle in angles:
            position = int(angle * 4095 / 180.0)
            position = max(0, min(4095, position))  # 限制范围
            positions.append(position)
        
        logger.info(f"角度转位置: {angles}° -> {positions}")
        return self.move_motors(positions, speeds, current_limits)
    
    def clear_motor_errors(self) -> bool:
        """清除电机故障"""
        if not self.connected:
            logger.error("设备未连接")
            return False
        
        try:
            success = self.controller.clear_motors_error()
            if success:
                logger.info("清除电机故障命令已发送")
            else:
                logger.error("清除电机故障命令发送失败")
            return success
        except Exception as e:
            logger.error(f"清除电机故障时发生错误: {e}")
            return False
    
    def start_monitoring(self, interval: float = 1.0):
        """开始状态监控"""
        if self.monitoring:
            logger.warning("状态监控已在运行")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop, 
            args=(interval,),
            daemon=True
        )
        self.monitor_thread.start()
        logger.info(f"状态监控已启动，间隔: {interval}秒")
    
    def stop_monitoring(self):
        """停止状态监控"""
        if not self.monitoring:
            return
        
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        logger.info("状态监控已停止")
    
    def _monitor_loop(self, interval: float):
        """监控循环"""
        while self.monitoring:
            try:
                self.print_motor_status()
                time.sleep(interval)
            except Exception as e:
                logger.error(f"监控循环中发生错误: {e}")
                time.sleep(interval)
    
    def interactive_control(self):
        """交互式控制界面"""
        print("\n" + "="*60)
        print("瑞眼灵巧手交互式控制界面")
        print("="*60)
        print("命令:")
        print("  status  - 显示电机状态")
        print("  move <pos1> <pos2> ... <pos6>  - 设置电机位置 (0-4095)")
        print("  angle <angle1> <angle2> ... <angle6>  - 设置电机角度 (0-180°)")
        print("  clear   - 清除电机故障")
        print("  monitor <interval>  - 开始状态监控 (间隔秒数)")
        print("  stop    - 停止状态监控")
        print("  test    - 运行自动化测试")
        print("  quit    - 退出")
        print("="*60)
        
        while True:
            try:
                command = input("\n请输入命令: ").strip().split()
                if not command:
                    continue
                
                cmd = command[0].lower()
                
                if cmd == 'quit' or cmd == 'q':
                    break
                elif cmd == 'status':
                    self.print_motor_status()
                elif cmd == 'move':
                    if len(command) != 7:
                        print("错误: 需要6个位置值 (0-4095)")
                        continue
                    try:
                        positions = [int(x) for x in command[1:7]]
                        self.move_motors(positions)
                    except ValueError:
                        print("错误: 位置值必须是整数")
                elif cmd == 'angle':
                    if len(command) != 7:
                        print("错误: 需要6个角度值 (0-180°)")
                        continue
                    try:
                        angles = [float(x) for x in command[1:7]]
                        self.move_motors_by_angles(angles)
                    except ValueError:
                        print("错误: 角度值必须是数字")
                elif cmd == 'clear':
                    self.clear_motor_errors()
                elif cmd == 'monitor':
                    interval = 1.0
                    if len(command) > 1:
                        try:
                            interval = float(command[1])
                        except ValueError:
                            print("错误: 间隔时间必须是数字")
                            continue
                    self.start_monitoring(interval)
                elif cmd == 'stop':
                    self.stop_monitoring()
                elif cmd == 'test':
                    self.run_automated_test()
                else:
                    print(f"未知命令: {cmd}")
                    
            except KeyboardInterrupt:
                print("\n收到中断信号，退出交互式控制...")
                break
            except Exception as e:
                print(f"执行命令时发生错误: {e}")
    
    def run_automated_test(self):
        """运行自动化测试"""
        print("\n开始自动化测试...")
        
        # 测试1: 清除故障
        print("测试1: 清除电机故障")
        self.clear_motor_errors()
        time.sleep(1)
        
        # 测试2: 读取状态
        print("测试2: 读取电机状态")
        self.print_motor_status()
        time.sleep(1)
        
        # 测试3: 位置控制测试
        print("测试3: 位置控制测试")
        test_positions = [
            [2048, 2048, 2048, 2048, 2048, 2048],  # 中间位置
            [1000, 1000, 1000, 1000, 1000, 1000],  # 低位置
            [3000, 3000, 3000, 3000, 3000, 3000],  # 高位置
            [2048, 2048, 2048, 2048, 2048, 2048],  # 回到中间
        ]
        
        for i, positions in enumerate(test_positions):
            print(f"  步骤{i+1}: 位置 {positions}")
            self.move_motors(positions)
            time.sleep(2)
            self.print_motor_status()
            time.sleep(1)
        
        # 测试4: 角度控制测试
        print("测试4: 角度控制测试")
        test_angles = [
            [90, 90, 90, 90, 90, 90],   # 90度
            [45, 45, 45, 45, 45, 45],   # 45度
            [135, 135, 135, 135, 135, 135],  # 135度
            [90, 90, 90, 90, 90, 90],   # 回到90度
        ]
        
        for i, angles in enumerate(test_angles):
            print(f"  步骤{i+1}: 角度 {angles}°")
            self.move_motors_by_angles(angles)
            time.sleep(2)
            self.print_motor_status()
            time.sleep(1)
        
        print("自动化测试完成！")
    
    def __enter__(self):
        """上下文管理器入口"""
        if not self.connect():
            raise RuntimeError("无法连接到设备")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.disconnect()


def main():
    """主函数"""
    print("瑞眼灵巧手控制器测试程序")
    print("="*50)
    
    # 检查设备权限
    if not os.access('/dev/ttyACM0', os.R_OK | os.W_OK):
        print("错误: 没有访问/dev/ttyACM0的权限")
        print("请运行: sudo chmod 666 /dev/ttyACM0")
        sys.exit(1)
    
    try:
        # 创建测试控制器
        with RuiyanHandTester() as tester:
            print("设备连接成功！")
            
            # 显示初始状态
            tester.print_motor_status()
            
            # 进入交互式控制
            tester.interactive_control()
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("程序结束")


if __name__ == "__main__":
    main()
