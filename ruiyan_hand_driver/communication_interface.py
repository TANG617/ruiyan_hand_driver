#!/usr/bin/env python3
"""
通信接口模块
提供CAN和485通信协议的抽象接口
"""

from dis import Instruction
import time
import logging
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class CommunicationType(Enum):
    """通信类型枚举"""
    CAN = "can" # not implementated 
    SERIAL = "serial"

class CommunicationDirectionType(Enum):
    """通信类型枚举"""
    SEND = "send"
    RECEIVE = "receive"

@dataclass
class RuiyanHandMessage:
    """RuiyanHand消息数据结构"""
    motor_id: int
    instruction: int
    payload: List[int] = None


class CommunicationInterface(ABC):
    """通信接口抽象基类"""
    
    def __init__(self, auto_connect: bool):
        self.connected = False
        if auto_connect:
            self.connect()
    
    @abstractmethod
    def connect(self) -> bool:
        """连接通信接口"""
        pass
    
    @abstractmethod
    def disconnect(self):
        """断开通信接口"""
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """检查连接状态"""
        pass
    
    @abstractmethod
    def send_and_receive(self, message: RuiyanHandMessage) -> RuiyanHandMessage:
        """发送消息并接收响应"""
        pass


class SerialInterface(CommunicationInterface):
    """RS485通信接口实现"""
    
    def __init__(self, port: str , baudrate: int ,timeout: float = 1.0, auto_connect: bool = False, mock:bool=False):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_controller = None
        self.mock = mock
        super().__init__(auto_connect)
    
    def connect(self) -> bool:
        """连接串口"""
        if self.mock == True:
            self.connected = True
            return True
        try:
            import serial
            self.serial_controller = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.connected = True
            logger.info(f"串口打开成功: {self.port}")
            return True
        except ImportError:
            self.connected = False
            logger.error("串口库未安装，请安装pyserial: pip install pyserial")
            return False
        except Exception as e:
            self.connected = False
            logger.error(f"串口打开失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_controller and self.connected:
            self.serial_controller.close()
            self.connected = False
            logger.info("串口连接已断开")
    
    def is_connected(self) -> bool:
        """检查RS485串口连接状态"""
        return self.connected
    
    def _send_message(self, message: RuiyanHandMessage) -> bool:
        """发送RS485消息"""
        if not self.connected:
            logger.error("RS485串口未连接")
            return False

        if self.mock == True:
            serial_frame = self._build_serial_frame(message)
            logger.debug(f"Mock消息已发送: 电机{message.motor_id}, 命令{hex(message.instruction)}")
            logger.debug(f"发送的完整帧数据: {' '.join([f'{byte:02X}' for byte in serial_frame])}")
            logger.debug(f"帧长度: {len(serial_frame)} 字节")
            return True

        if message.direction == CommunicationDirectionType.SEND:
            try:
                # 构建RS485消息帧
                serial_frame = self._build_serial_frame(message)
                self.serial_controller.write(serial_frame)
                logger.debug(f"RS485消息已发送: 电机{message.motor_id}, 命令{hex(message.command)}")
                logger.debug(f"发送的完整帧数据: {' '.join([f'{byte:02X}' for byte in serial_frame])}")
                logger.debug(f"帧长度: {len(serial_frame)} 字节")
                return True
            except Exception as e:
                logger.error(f"RS485消息发送失败: {e}")
                return False
        else:
            logger.error(f"不支持的通信方向: {message.direction}")
            return False
    
    def _receive_message(self) -> RuiyanHandMessage:
        """接收RS485消息"""
        if not self.connected:
            return None
        try:
            data = self.serial_controller.read(64)  # 假设最大消息长度为64字节
            received_message = self._parse_serial_frame(data)
            return received_message
        except Exception as e:
            logger.error(f"RS485消息接收失败: {e}")
            return None
    
    def _build_serial_frame(self, message: RuiyanHandMessage) -> bytes:
        """构建RS485消息帧 - 符合瑞眼灵巧手协议"""
        # 正确的RS485帧格式: [header 0xA5][motor_id 2字节][len 1字节][data n字节][check 1字节]
        frame = bytearray()
        
        # 起始符 (header)
        frame.append(0xA5)
        
        # ID (2字节) - 对应CAN/CANFD原生ID，使用电机ID
        motor_id = message.motor_id
        frame.append(motor_id & 0xFF)        # ID低字节
        frame.append((motor_id >> 8) & 0xFF) # ID高字节
        
        # 数据部分
        data_segment = []
        if message.payload:
            data_segment.extend(message.payload)
        
        # 数据长度 (len)
        frame.append(len(data_segment)+1)
        frame.append(message.instruction)
        
        # 数据 (data[])
        frame.extend(data_segment)
        
        # 校验和 (check) - 从header到data[]的所有数据的累加和低8位
        checksum = 0
        for byte in frame:
            checksum += byte
        frame.append(checksum & 0xFF)  # 累加和低8位
        
        return bytes(frame)
    
    def _parse_serial_frame(self, data: bytes) -> RuiyanHandMessage:
        """解析RS485消息帧 - 符合瑞眼灵巧手协议"""
        if len(data) < 6:  # 最小帧长度: header(1) + motor_id(2) + len(1) + data(0) + check(1) = 5
            return None
        
        if data[0] != 0xA5:
            return None
        
        # 验证校验和 - 累加和低8位
        checksum = 0
        for byte in data[:-1]:  # 除了最后一个字节（校验和）
            checksum += byte
        
        if (checksum & 0xFF) != data[-1]:
            logger.debug(f"校验和错误: 计算值={checksum & 0xFF:02X}, 接收值={data[-1]:02X}")
            # 暂时忽略校验和错误，继续解析
            # return None
        
        # 提取帧信息
        motor_id = data[1] | (data[2] << 8)  # ID (2字节)
        data_length = data[3]                # 数据长度
        instruction = data[4]
        data_segment = list(data[5:-1])         # 数据部分（不包括校验和）
        
        # 调试信息
        logger.debug(f"解析RS485帧: ID={motor_id} (0x{data[1]:02X} 0x{data[2]:02X}), 长度={data_length}, 数据长度={len(data_part)}")
        
        return RuiyanHandMessage(
            type=CommunicationType.SERIAL,
            direction=CommunicationDirectionType.RECEIVE,
            motor_id=motor_id,
            instruction=instruction,
            payload=data_segment,
            timeout=list(data)
        )
        # return {
        #     'motor_id': motor_id,
        #     'data_length': data_length,
        #     'data_payload': data_part,
        #     'raw_data': list(data)
        # }
    
    def _collect_responses(self, timeout: float = 1.0) -> Dict[int, Optional[Any]]:
        """收集多个电机的响应消息"""
        if not self.connected:
            return {}
        
        results = {}
        start_time = time.time()
        
        # 在超时时间内收集响应
        while time.time() - start_time < timeout:
            # 使用更短的接收超时时间以提高响应速度
            response_data = self.receive_message(timeout=0.01)  # 10ms接收超时
            if response_data is None:
                # 如果没有数据，短暂等待后继续
                time.sleep(0.001)  # 1ms等待
                continue
            
            parsed = self._parse_rs485_frame(response_data)
            if parsed and 'motor_id' in parsed:
                motor_id = parsed['motor_id']
                # 如果ID是257（0x01 0x01），可能是设备返回的响应ID，我们将其映射为1
                if motor_id == 257:
                    motor_id = 1
                if motor_id not in results:
                    results[motor_id] = parsed
        
        return results
    
    def send_and_receive(self, message: RuiyanHandMessage) -> RuiyanHandMessage:
        """发送消息并接收响应"""
        if not self._send_message(message):
            return {}
        
        # 接收响应
        if(self.mock == True):
            logger.debug("Mock模式，无需接收数据")
            return {}
        received_message = self._receive_message()
        return received_message
