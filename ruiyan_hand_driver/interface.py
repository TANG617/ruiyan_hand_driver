#!/usr/bin/env python3
"""
通信接口模块
提供CAN和485通信协议的抽象接口
"""

from dis import Instruction
import time
import logging
import struct
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from enum import Enum, IntEnum

logger = logging.getLogger(__name__)

class RuiyanHandInstructionType(IntEnum):
    READ_MOTOR_INFO = 0xA0
    CTRL_MOTOR_POSITION_VELOCITY_CURRENT = 0xAA
    CLEAR_MOTOR_ERROR = 0xA5

class RuiyanHandStatusCode(IntEnum):
    """瑞眼灵巧手状态码枚举"""
    SERVO_OK = 0                    # 电机响应函数操作成功
    SERVO_TEMPERATURE_HIGH_W = 1    # 电机过温告警
    SERVO_TEMPERATURE_HIGH_E = 2    # 电机过温保护
    SERVO_VOLTAGE_LOW_E = 3         # 电机低压保护
    SERVO_VOLTAGE_HIGH_E = 4        # 电机过压保护
    SERVO_CURRENT_OVER_E = 5        # 电机过流保护
    SERVO_TORQUE_OVER_E = 6         # 电机力矩保护
    SERVO_FUSE_E = 7                # 电机熔丝位错保护
    SERVO_PWM_E = 8                 # 电机堵转保护
    SERVO_DRIVE_E = 9               # 驱动器异常保护
    SERVO_HALL_E = 10               # 电机hall错保护
    SERVO_FAIL = 250                # 伺服未响应函数操作或响应超时
    SERVO_PARAM_ERR = 251           # 伺服响应数据出错
    SERVO_LIB_INIT_ERR = 252        # 伺服组群结构体读写函数未初始化或失败
    CAN_FORMAT_ERROR = 253          # 异步解析时can数据格式不符
    CAN_MSG_SENT_FAIL = 254         # can消息发送失败
    LIB_HOOK_APPLY_FAILED = 255     # hook申请失败

    @classmethod
    def get_description(cls, status_code: int) -> str:
        """获取状态码的中文描述"""
        descriptions = {
            cls.SERVO_OK: "操作成功",
            cls.SERVO_TEMPERATURE_HIGH_W: "电机过温告警",
            cls.SERVO_TEMPERATURE_HIGH_E: "电机过温保护",
            cls.SERVO_VOLTAGE_LOW_E: "电机低压保护", 
            cls.SERVO_VOLTAGE_HIGH_E: "电机过压保护",
            cls.SERVO_CURRENT_OVER_E: "电机过流保护",
            cls.SERVO_TORQUE_OVER_E: "电机力矩保护",
            cls.SERVO_FUSE_E: "电机熔丝位错保护",
            cls.SERVO_PWM_E: "电机堵转保护",
            cls.SERVO_DRIVE_E: "驱动器异常保护",
            cls.SERVO_HALL_E: "电机hall错保护",
            cls.SERVO_FAIL: "伺服未响应或超时",
            cls.SERVO_PARAM_ERR: "伺服响应数据出错",
            cls.SERVO_LIB_INIT_ERR: "伺服组群结构体未初始化",
            cls.CAN_FORMAT_ERROR: "CAN数据格式不符",
            cls.CAN_MSG_SENT_FAIL: "CAN消息发送失败",
            cls.LIB_HOOK_APPLY_FAILED: "Hook申请失败"
        }
        return descriptions.get(status_code, f"未知状态码: {status_code}")

class CommunicationType(Enum):
    """通信类型枚举"""
    CAN = "can" # not implementated 
    SERIAL = "serial"


@dataclass
class RuiyanHandControlMessage:
    """RuiyanHand消息数据结构"""
    motor_id: int
    instruction: RuiyanHandInstructionType
    position: Optional[int]
    velocity: Optional[int]
    current: Optional[int]

    def print(self):
        print(f"motor_id: {self.motor_id}, instruction: {self.instruction}, position: {self.position}, velocity: {self.velocity}, current: {self.current}")

@dataclass
class RuiyanHandStatusMessage:
    """RuiyanHand消息数据结构"""
    motor_id: int
    instruction: RuiyanHandInstructionType
    position:Optional[int]
    velocity:Optional[int]
    current:Optional[int]
    def print(self):
        print(f"电机ID: {self.motor_id}, 指令: {self.instruction}, 位置: {self.position}, 速度: {self.velocity}, 电流: {self.current}")

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
    def send_and_receive(self, message: RuiyanHandControlMessage) -> RuiyanHandStatusMessage:
        """发送消息并接收响应"""
        pass


class SerialInterface(CommunicationInterface):
    """RS485通信接口实现"""
    
    def __init__(self, port: str , baudrate: int ,timeout: float = 0.006, auto_connect: bool = False, mock:bool=False): #6ms is the fastes speed
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
    
    def _send_message(self, message: RuiyanHandControlMessage) -> bool:
        """发送RS485消息"""
        if not self.connected:
            logger.error("RS485串口未连接")
            return False

        if self.mock == True:
            serial_frame = self._build_serial_frame(message)
            logger.debug(f"【发送】电机ID: {message.motor_id}, 指令: {hex(message.instruction)}, 帧数据: {' '.join([f'{byte:02X}' for byte in serial_frame])}")
            return True

    
        try:
            # 构建RS485消息帧
            serial_frame = self._build_serial_frame(message)
            self.serial_controller.write(serial_frame)
            logger.debug(f"【发送】电机ID: {message.motor_id}, 指令: {hex(message.instruction)}, 帧数据: {' '.join([f'{byte:02X}' for byte in serial_frame])}")
            return True
        except Exception as e:
            logger.error(f"串口消息发送失败: {e}")
            return False

    
    def _receive_message(self) -> bytes:
        """接收RS485消息"""
        if not self.connected:
            return None
        try:
            response = self.serial_controller.read(64)  # 假设最大消息长度为64字节
            return response
        except Exception as e:
            logger.error(f"RS485消息接收失败: {e}")
            return None
    
    def _build_serial_frame(self, message: RuiyanHandControlMessage) -> bytes:
        """构建RS485消息帧 - 符合瑞眼灵巧手协议"""
        # 正确的RS485帧格式: [header 0xA5][motor_id 2字节][len 1字节][data n字节][check 1字节]
        serial_frame = struct.pack('<B B B 2B 3H 1B', 
                         0xA5, 
                         message.motor_id, 
                         0x00, 
                         0x08, 
                         message.instruction, 
                         message.position, 
                         message.velocity, 
                         message.current, 
                         0x00)

        checksum = 0
        for byte in serial_frame:
            checksum += byte
        
        serial_frame = struct.pack('<B B B 2B 3H 1B 1B', 
                         0xA5, 
                         message.motor_id, 
                         0x00, 
                         0x08, 
                         message.instruction, 
                         message.position, 
                         message.velocity, 
                         message.current, 
                         0x00,
                         checksum & 0xFF
                         )

        # checksum = 0
        # for byte in serial_frame:
        #     checksum += byte
        # serial_frame.append(checksum & 0xFF)  # 累加和低8位

        return serial_frame
    
    
    def send_and_receive(self, message: RuiyanHandControlMessage) -> bytes:
        """发送消息并接收响应"""
        if not self._send_message(message):
            return {}
        
        # 接收响应
        if(self.mock == True):
            logger.debug("Mock模式，无需接收数据")
            return {}
        response = self._receive_message()
        return response
