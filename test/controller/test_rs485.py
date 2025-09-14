#!/usr/bin/env python3
"""
RS485通信测试脚本
向/dev/ttyACM0发送指定的十六进制指令并读取回复
"""

import serial
import time
import logging
from typing import Optional, List

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RS485Tester:
    """RS485通信测试器"""
    
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200, timeout: float = 2.0):
        """
        初始化RS485测试器
        
        Args:
            port: 串口设备路径
            baudrate: 波特率
            timeout: 超时时间（秒）
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.connected = False
    
    def connect(self) -> bool:
        """连接RS485串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.connected = True
            logger.info(f"RS485串口连接成功: {self.port} @ {self.baudrate} bps")
            return True
        except Exception as e:
            self.connected = False
            logger.error(f"RS485串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开RS485串口连接"""
        if self.serial_conn and self.connected:
            self.serial_conn.close()
            self.connected = False
            logger.info("RS485串口连接已断开")
    
    def hex_string_to_bytes(self, hex_string: str) -> bytes:
        """
        将十六进制字符串转换为字节数组
        
        Args:
            hex_string: 十六进制字符串，如 "A5 01 00 18 AA"
            
        Returns:
            字节数组
        """
        # 移除空格并转换为字节
        hex_clean = hex_string.replace(' ', '').replace('\n', '').replace('\r', '')
        return bytes.fromhex(hex_clean)
    
    def bytes_to_hex_string(self, data: bytes, separator: str = ' ') -> str:
        """
        将字节数组转换为十六进制字符串
        
        Args:
            data: 字节数组
            separator: 分隔符
            
        Returns:
            十六进制字符串
        """
        return separator.join([f'{byte:02X}' for byte in data])
    
    def send_command(self, hex_command: str) -> bool:
        """
        发送十六进制命令
        
        Args:
            hex_command: 十六进制命令字符串
            
        Returns:
            发送是否成功
        """
        if not self.connected:
            logger.error("RS485串口未连接")
            return False
        
        try:
            # 转换十六进制字符串为字节
            command_bytes = self.hex_string_to_bytes(hex_command)
            
            # 发送命令
            self.serial_conn.write(command_bytes)
            logger.info(f"已发送命令: {self.bytes_to_hex_string(command_bytes)}")
            logger.info(f"命令长度: {len(command_bytes)} 字节")
            return True
        except Exception as e:
            logger.error(f"发送命令失败: {e}")
            return False
    
    def read_response(self, timeout: float = None) -> Optional[bytes]:
        """
        读取响应数据
        
        Args:
            timeout: 超时时间，None表示使用默认超时
            
        Returns:
            响应字节数据，如果超时或出错则返回None
        """
        if not self.connected:
            logger.error("RS485串口未连接")
            return None
        
        try:
            # 设置超时
            original_timeout = self.serial_conn.timeout
            if timeout is not None:
                self.serial_conn.timeout = timeout
            
            # 读取数据
            response = self.serial_conn.read(1024)  # 读取最多1024字节
            
            # 恢复原始超时
            self.serial_conn.timeout = original_timeout
            
            if response:
                logger.info(f"已接收响应: {self.bytes_to_hex_string(response)}")
                logger.info(f"响应长度: {len(response)} 字节")
                return response
            else:
                logger.warning("未接收到响应数据")
                return None
        except Exception as e:
            logger.error(f"读取响应失败: {e}")
            return None
    
    def send_and_receive(self, hex_command: str, response_timeout: float = 2.0) -> Optional[bytes]:
        """
        发送命令并接收响应
        
        Args:
            hex_command: 十六进制命令字符串
            response_timeout: 响应超时时间
            
        Returns:
            响应字节数据
        """
        if not self.send_command(hex_command):
            return None
        
        # 等待一小段时间确保命令发送完成
        time.sleep(0.1)
        
        # 读取响应
        return self.read_response(response_timeout)
    
    def test_specific_command(self):
        """测试指定的命令"""
        # 指定的十六进制命令
        command = "A5 01 00 18 AA 00 00 E8 03 F4 01 00 AA 00 08 E8 03 F4 01 00 AA FF 0F E8 03 F4 01 00 72"
        
        logger.info("=" * 60)
        logger.info("开始测试指定的RS485命令")
        logger.info("=" * 60)
        logger.info(f"目标设备: {self.port}")
        logger.info(f"波特率: {self.baudrate}")
        logger.info(f"命令: {command}")
        logger.info("=" * 60)
        
        # 发送命令并接收响应
        response = self.send_and_receive(command, response_timeout=3.0)
        
        if response:
            logger.info("=" * 60)
            logger.info("测试结果:")
            logger.info(f"发送的命令: {command}")
            logger.info(f"接收的响应: {self.bytes_to_hex_string(response)}")
            logger.info(f"响应长度: {len(response)} 字节")
            logger.info("=" * 60)
            return True
        else:
            logger.error("=" * 60)
            logger.error("测试失败: 未接收到响应")
            logger.error("=" * 60)
            return False
    
    def test_multiple_attempts(self, num_attempts: int = 3):
        """多次尝试测试"""
        logger.info(f"开始多次尝试测试 (共{num_attempts}次)")
        
        success_count = 0
        for i in range(num_attempts):
            logger.info(f"\n--- 第 {i+1} 次尝试 ---")
            if self.test_specific_command():
                success_count += 1
            time.sleep(1.0)  # 每次尝试间隔1秒
        
        logger.info(f"\n测试总结: {success_count}/{num_attempts} 次成功")
        return success_count > 0
    
    def test_connection(self):
        """测试连接状态"""
        logger.info("测试RS485连接状态...")
        
        if not self.connected:
            logger.error("RS485未连接")
            return False
        
        # 尝试发送一个简单的测试命令
        test_command = "A5 01 00 01 00 01"  # 简单的测试命令
        logger.info(f"发送测试命令: {test_command}")
        
        response = self.send_and_receive(test_command, response_timeout=1.0)
        if response:
            logger.info("连接测试成功")
            return True
        else:
            logger.warning("连接测试失败 - 可能设备未响应或命令格式不正确")
            return False


def main():
    """主函数"""
    print("RS485通信测试脚本")
    print("=" * 50)
    
    # 创建RS485测试器
    tester = RS485Tester(port='/dev/ttyACM0', baudrate=115200, timeout=2.0)
    
    try:
        # 连接串口
        if not tester.connect():
            print("无法连接到RS485设备，请检查:")
            print("1. 设备是否连接到 /dev/ttyACM0")
            print("2. 设备是否被其他程序占用")
            print("3. 用户是否有访问权限")
            return
        
        # 测试连接
        print("\n1. 测试连接状态...")
        tester.test_connection()
        
        # 测试指定命令
        print("\n2. 测试指定命令...")
        success = tester.test_specific_command()
        
        if not success:
            print("\n3. 尝试多次测试...")
            tester.test_multiple_attempts(3)
        
        print("\n测试完成")
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
    finally:
        tester.disconnect()


if __name__ == '__main__':
    main()
