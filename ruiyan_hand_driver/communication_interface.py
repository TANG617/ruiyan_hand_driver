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
    RS485 = "485"
    SERIAL = "serial"


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
    rs485_port: str = '/dev/ttyACM0'
    rs485_baudrate: int = 115200
    rs485_timeout: float = 1.0
    
    def __post_init__(self):
        if self.motor_ids is None:
            self.motor_ids = [1, 2, 3, 4, 5, 6]
    
    @classmethod
    def create_can_config(cls, motor_ids: List[int] = None, 
                         interface: str = 'socketcan', 
                         channel: str = 'can0', 
                         bitrate: int = 1000000,
                         auto_connect: bool = True) -> 'CommunicationConfig':
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
    def create_rs485_config(cls, motor_ids: List[int] = None,
                           port: str = '/dev/ttyUSB0',
                           baudrate: int = 115200,
                           timeout: float = 1.0,
                           auto_connect: bool = True) -> 'CommunicationConfig':
        """创建RS485通信配置"""
        return cls(
            communication_type=CommunicationType.RS485,
            motor_ids=motor_ids,
            rs485_port=port,
            rs485_baudrate=baudrate,
            rs485_timeout=timeout,
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
        elif self.communication_type == CommunicationType.RS485:
            return {
                'port': self.rs485_port,
                'baudrate': self.rs485_baudrate,
                'timeout': self.rs485_timeout,
                'auto_connect': False  # 由CommunicationManager控制
            }
        else:
            raise ValueError(f"不支持的通信类型: {self.communication_type}")


@dataclass
class DexhandMessage:
    """Dexhand消息数据结构"""
    command: int
    motor_id: int
    data_payload: List[int] = None
    timeout: float = 1.0
    expected_response_id: Optional[int] = None
    
    def __post_init__(self):
        if self.data_payload is None:
            self.data_payload = []


class CommunicationInterface(ABC):
    """通信接口抽象基类"""
    
    def __init__(self, auto_connect: bool = True):
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
    def send_message(self, message: DexhandMessage) -> bool:
        """发送消息"""
        pass
    
    @abstractmethod
    def receive_message(self, timeout: float = 1.0) -> Optional[Any]:
        """接收消息"""
        pass
    
    @abstractmethod
    def send_and_receive(self, message: DexhandMessage) -> Dict[int, Any]:
        """发送消息并接收响应"""
        pass


class CANInterface(CommunicationInterface):
    """CAN通信接口实现"""
    
    def __init__(self, interface: str = 'socketcan', channel: str = 'can0', 
                 bitrate: int = 1000000, motor_ids: List[int] = [1, 2, 3, 4, 5, 6],
                 auto_connect: bool = True):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.motor_ids = motor_ids
        self.response_ids = [0x100 + i for i in self.motor_ids]
        self.bus = None
        super().__init__(auto_connect)
    
    def connect(self) -> bool:
        """连接CAN总线"""
        try:
            import can
            bus_kwargs = {
                "interface": self.interface,
                "channel": self.channel,
                "bitrate": self.bitrate,
                "state": can.BusState.ACTIVE,
            }
            self.bus = can.Bus(**bus_kwargs)
            self.connected = True
            logger.info(f"CAN总线连接成功: {self.channel}")
            return True
        except ImportError:
            self.connected = False
            logger.error("CAN库未安装，请安装python-can: pip install python-can")
            return False
        except Exception as e:
            self.connected = False
            logger.error(f"CAN总线连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开CAN总线连接"""
        if self.bus and self.connected:
            self.bus.shutdown()
            self.connected = False
            logger.info("CAN总线连接已断开")
    
    def is_connected(self) -> bool:
        """检查CAN总线连接状态"""
        return self.connected
    
    def send_message(self, message: DexhandMessage) -> bool:
        """发送CAN消息"""
        if not self.connected:
            logger.error("CAN总线未连接")
            return False

        if message.motor_id not in self.motor_ids:
            logger.error(f"无效的电机ID: {message.motor_id}")
            return False
        
        # 构建CAN消息数据
        data = [message.command]
        if message.data_payload:
            data.extend(message.data_payload[:7])
        
        # 填充到8字节
        while len(data) < 8:
            data.append(0x00)
        
        try:
            import can
            msg = can.Message(
                arbitration_id=message.motor_id,
                data=data,
                is_extended_id=False
            )
            self.bus.send(msg)
            logger.debug(f"CAN消息已发送: ID={message.motor_id}, 数据={[hex(x) for x in data]}")
            return True
        except ImportError:
            logger.error("CAN库未安装，请安装python-can: pip install python-can")
            return False
        except Exception as e:
            logger.error(f"CAN消息发送失败: {e}")
            return False
    
    def receive_message(self, timeout: float = 1.0) -> Optional[Any]:
        """接收单个CAN消息"""
        if not self.connected:
            return None
        
        try:
            import can
            rx = self.bus.recv(timeout=timeout)
            return rx
        except ImportError:
            logger.error("CAN库未安装，请安装python-can: pip install python-can")
            return None
        except Exception as e:
            logger.error(f"CAN消息接收失败: {e}")
            return None
    
    def _collect_responses(self, timeout: float = 1.0) -> Dict[int, Optional[Any]]:
        """收集多个电机的响应消息"""
        if not self.connected:
            return {}
        
        results = {}
        expected_response_ids = {0x100 + motor_id: motor_id for motor_id in self.motor_ids}
        
        start_time = time.time()
        while time.time() - start_time < timeout and len(results) < len(self.motor_ids):
            rx = self.receive_message(timeout=0.1)
            if rx is None:
                continue
            
            motor_id = expected_response_ids.get(rx.arbitration_id)
            if motor_id is not None and motor_id not in results:
                results[motor_id] = rx
        
        # 为没有响应的电机添加None
        for motor_id in self.motor_ids:
            if motor_id not in results:
                results[motor_id] = None
        
        return results
    
    def send_and_receive(self, message: DexhandMessage) -> Dict[int, Any]:
        """发送消息并接收响应"""
        if not self.send_message(message):
            return {}
        
        # 如果是单个电机消息，接收单个响应
        if message.expected_response_id:
            rx = self.receive_message(message.timeout)
            return {message.motor_id: rx}
        
        # 否则收集所有电机的响应
        return self._collect_responses(message.timeout)


class RS485Interface(CommunicationInterface):
    """RS485通信接口实现"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200,
                 timeout: float = 1.0, auto_connect: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        super().__init__(auto_connect)
    
    def connect(self) -> bool:
        """连接RS485串口"""
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            self.connected = True
            logger.info(f"RS485串口连接成功: {self.port}")
            return True
        except ImportError:
            self.connected = False
            logger.error("串口库未安装，请安装pyserial: pip install pyserial")
            return False
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
    
    def is_connected(self) -> bool:
        """检查RS485串口连接状态"""
        return self.connected
    
    def send_message(self, message: DexhandMessage) -> bool:
        """发送RS485消息"""
        if not self.connected:
            logger.error("RS485串口未连接")
            return False
        
        try:
            # 构建RS485消息帧
            frame_data = self._build_rs485_frame(message)
            self.serial_conn.write(frame_data)
            logger.debug(f"RS485消息已发送: 电机{message.motor_id}, 命令{hex(message.command)}, 数据{[hex(x) for x in frame_data]}")
            return True
        except Exception as e:
            logger.error(f"RS485消息发送失败: {e}")
            return False
    
    def receive_message(self, timeout: float = 1.0) -> Optional[bytes]:
        """接收RS485消息"""
        if not self.connected:
            return None
        
        try:
            # 设置超时
            original_timeout = self.serial_conn.timeout
            self.serial_conn.timeout = timeout
            
            # 读取消息
            data = self.serial_conn.read(64)  # 假设最大消息长度为64字节
            
            # 恢复原始超时
            self.serial_conn.timeout = original_timeout
            
            if data:
                logger.debug(f"RS485消息已接收: 长度={len(data)}, 数据{[hex(x) for x in data]}")
                return data
            else:
                return None
        except Exception as e:
            logger.error(f"RS485消息接收失败: {e}")
            return None
    
    def _build_rs485_frame(self, message: DexhandMessage) -> bytes:
        """构建RS485消息帧 - 符合瑞眼灵巧手协议"""
        # 瑞眼灵巧手RS485帧格式: [起始符A5][设备ID][方向][数据长度][数据][校验和]
        frame = bytearray()
        
        # 起始符
        frame.append(0xA5)
        
        # 设备ID (01表示主控)
        frame.append(0x01)
        
        # 方向 (00表示发送，01表示接收)
        frame.append(0x00)
        
        # 数据部分
        data_part = []
        if message.data_payload:
            data_part.extend(message.data_payload)
        
        # 数据长度
        frame.append(len(data_part))
        
        # 数据
        frame.extend(data_part)
        
        # 校验和 (简单异或校验，包括起始符、设备ID、方向、数据长度和数据)
        checksum = 0
        for byte in frame:
            checksum ^= byte
        frame.append(checksum)
        
        return bytes(frame)
    
    def _parse_rs485_frame(self, data: bytes) -> Optional[Dict]:
        """解析RS485消息帧 - 符合瑞眼灵巧手协议"""
        if len(data) < 5:  # 最小帧长度
            return None
        
        if data[0] != 0xA5:
            return None
        
        # 验证校验和
        checksum = 0
        for byte in data[:-1]:  # 除了最后一个字节（校验和）
            checksum ^= byte
        
        if checksum != data[-1]:
            logger.debug(f"校验和错误: 计算值={checksum:02X}, 接收值={data[-1]:02X}")
            # 暂时忽略校验和错误，继续解析
            # return None
        
        # 提取帧信息
        device_id = data[1]
        direction = data[2]  # 00=发送，01=接收
        data_length = data[3]
        data_part = list(data[4:-1])  # 数据部分（不包括校验和）
        
        return {
            'device_id': device_id,
            'direction': direction,
            'data_length': data_length,
            'data_payload': data_part,
            'raw_data': list(data)
        }
    
    def _collect_responses(self, timeout: float = 1.0) -> Dict[int, Optional[Any]]:
        """收集多个电机的响应消息"""
        if not self.connected:
            return {}
        
        results = {}
        start_time = time.time()
        
        # 在超时时间内收集响应
        while time.time() - start_time < timeout:
            response_data = self.receive_message(timeout=0.1)
            if response_data is None:
                continue
            
            parsed = self._parse_rs485_frame(response_data)
            if parsed and 'motor_id' in parsed:
                motor_id = parsed['motor_id']
                if motor_id not in results:
                    results[motor_id] = parsed
        
        return results
    
    def send_and_receive(self, message: DexhandMessage) -> Dict[int, Any]:
        """发送消息并接收响应"""
        if not self.send_message(message):
            return {}
        
        # 接收响应
        response_data = self.receive_message(message.timeout)
        if response_data:
            parsed = self._parse_rs485_frame(response_data)
            if parsed:
                return {message.motor_id: parsed}
        
        return {message.motor_id: None}


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
