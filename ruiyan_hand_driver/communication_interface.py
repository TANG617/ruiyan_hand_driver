#!/usr/bin/env python3
"""
通信接口模块
提供CAN和485通信协议的抽象接口
"""

import time
import logging
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class CommunicationType(Enum):
    """通信类型枚举"""
    CAN = "can"
    SERIAL = "serial"

class CommunicationDirectionType(Enum):
    """通信类型枚举"""
    SEND = "send"
    RECEIVE = "receive"


@dataclass
class CommunicationConfig:
    """通信配置类"""
    # 通用配置
    communication_type: CommunicationType
    auto_connect: bool = True
    motor_ids: List[int] = None
    
    # CAN配置
    can_interface: str = 'socketcan'
    can_channel: str = 'can0'
    can_bitrate: int = 1000000
    
    # RS485配置
    serial_port: str = '/dev/ttyACM0'
    serial_baudrate: int = 115200
    serial_timeout: float = 1.0
    
    def __post_init__(self):
        if self.motor_ids is None:
            self.motor_ids = [1, 2, 3, 4, 5, 6]
    
    @classmethod
    def create_can_config(cls, motor_ids: List[int] = None, 
                         interface: str = None, 
                         channel: str = None, 
                         bitrate: int = None,
                         auto_connect: bool = False) -> 'CommunicationConfig':
        """创建CAN通信配置"""
        return cls(
            communication_type=CommunicationType.CAN,
            motor_ids=motor_ids,
            can_interface=interface,
            can_channel=channel,
            can_bitrate=bitrate,
            auto_connect=auto_connect
        )
    
    @classmethod
    def create_serial_config(cls, motor_ids: List[int] = None,
                           port: str = None,
                           baudrate: int = None,
                           timeout: float = 1.0,
                           auto_connect: bool = False) -> 'CommunicationConfig':
        """创建RS485通信配置"""
        return cls(
            communication_type=CommunicationType.SERIAL,
            motor_ids=motor_ids,
            serial_port=port,
            serial_baudrate=baudrate,
            serial_timeout=timeout,
            auto_connect=auto_connect
        )
    
    def get_interface_kwargs(self) -> Dict[str, Any]:
        """获取接口初始化参数"""
        if self.communication_type == CommunicationType.CAN:
            return {
                'interface': self.can_interface,
                'channel': self.can_channel,
                'bitrate': self.can_bitrate,
                'motor_ids': self.motor_ids,
                'auto_connect': False  # 由CommunicationManager控制
            }
        elif self.communication_type == CommunicationType.SERIAL:
            return {
                'port': self.serial_port,
                'baudrate': self.serial_baudrate,
                'timeout': self.serial_timeout,
                'auto_connect': False  # 由CommunicationManager控制
            }
        else:
            raise ValueError(f"不支持的通信类型: {self.communication_type}")


@dataclass
class RuiyanHandMessage:
    """RuiyanHand消息数据结构"""
    type: CommunicationType
    direction: CommunicationDirectionType
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
    
    def __init__(self, port: str , baudrate: int ,timeout: float = 1.0, auto_connect: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_controller = None
        super().__init__(auto_connect)
    
    def connect(self) -> bool:
        """连接R串口"""
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
        if self.serial_conn and self.connected:
            self.serial_conn.close()
            self.connected = False
            logger.info("串口连接已断开")
    
    def is_connected(self) -> bool:
        """检查RS485串口连接状态"""
        return self.connected
    
    def send_message(self, message: RuiyanHandMessage) -> bool:
        """发送RS485消息"""
        if not self.connected:
            logger.error("RS485串口未连接")
            return False

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
    
    def receive_message(self) -> RuiyanHandMessage:
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
        frame.append(len(data_segment))
        
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
        data_part = list(data[4:-1])         # 数据部分（不包括校验和）
        
        # 调试信息
        logger.debug(f"解析RS485帧: ID={motor_id} (0x{data[1]:02X} 0x{data[2]:02X}), 长度={data_length}, 数据长度={len(data_part)}")
        
        return RuiyanHandMessage(
            type=CommunicationType.SERIAL,
            direction=CommunicationDirectionType.RECEIVE,
            motor_id=motor_id,
            instruction=data_length,
            payload=data_part,
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
        if not self.send_message(message):
            return {}
        
        # 接收响应
        received_message = self.receive_message()
        return received_message


class CommunicationManager:
    """通信管理器，统一管理不同的通信接口"""
    
    def __init__(self, config: CommunicationConfig):
        self.config = config
        self.interface = None
        self._create_interface()
    
    def _create_interface(self):
        """创建通信接口实例"""
        kwargs = self.config.get_interface_kwargs()
        
        if self.config.communication_type == CommunicationType.CAN:
            self.interface = CANInterface(**kwargs)
        elif self.config.communication_type == CommunicationType.RS485:
            # 过滤掉RS485不支持的参数
            rs485_kwargs = {k: v for k, v in kwargs.items() if k in ['port', 'baudrate', 'timeout', 'auto_connect']}
            self.interface = RS485Interface(**rs485_kwargs)
        else:
            raise ValueError(f"不支持的通信接口类型: {self.config.communication_type}")
    
    def connect(self) -> bool:
        """连接通信接口"""
        return self.interface.connect() if self.interface else False
    
    def disconnect(self):
        """断开通信接口"""
        if self.interface:
            self.interface.disconnect()
    
    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.interface.is_connected() if self.interface else False
    
    def send_message(self, message: DexhandMessage) -> bool:
        """发送消息"""
        return self.interface.send_message(message) if self.interface else False
    
    def receive_message(self, timeout: float = 1.0) -> Optional[Any]:
        """接收消息"""
        return self.interface.receive_message(timeout) if self.interface else None
    
    def send_and_receive(self, message: DexhandMessage) -> Dict[int, Any]:
        """发送消息并接收响应"""
        return self.interface.send_and_receive(message) if self.interface else {}
    
    def __enter__(self):
        if not self.is_connected():
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
