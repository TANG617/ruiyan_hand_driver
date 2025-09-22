#!/usr/bin/env python3
"""
Dexhand控制器模块
根据API函数封装对应的DexhandMessage数组，传递给communication_interface
"""

from dis import Instruction
from readline import insert_text
import time
import struct
import logging
from turtle import pos
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from .interface import RuiyanHandControlMessage, CommunicationInterface, RuiyanHandControlMessage, RuiyanHandStatusMessage, RuiyanHandInstructionType, RuiyanHandStatusCode

logger = logging.getLogger(__name__)

class RuiyanHandController:
    """Dexhand控制器，封装所有API函数"""
    
    
    def __init__(self, communication_interface:CommunicationInterface, motors_id:[int], instruction:RuiyanHandInstructionType = None):

        self.motor_ids = motors_id
        self.communication_interface = communication_interface
        self.instruction = instruction
        self.position_list=[0,0,0,0,0,0]
        self.velocity_list=[0,0,0,0,0,0]
        self.current_list= [0,0,0,0,0,0]
        self.command_names = {
            0xA0: "读取电机信息",
            0xAA: "位置速度电流混合控制",
            0xA5: "清除电机故障"
        }
    
    def connect(self) -> bool:
        """连接通信接口"""
        return self.communication_interface.connect()
    
    def disconnect(self):
        """断开通信接口"""
        return self.communication_interface.disconnect()
    
    def is_connected(self) -> bool:
        """检查连接状态"""
        return self.communication_interface.is_connected()

    
    def _validate_motor_ids(self, motor_ids: List[int]) -> bool:
        """验证电机ID列表"""
        for motor_id in motor_ids:
            if motor_id not in self.motor_ids:
                return False
        return True

    def _set_motor(self, motor_id:int, position:Optional[int], velocity:Optional[int], current:Optional[int]) -> RuiyanHandStatusMessage:
        request_message = RuiyanHandControlMessage(motor_id=motor_id,instruction=self.instruction,position=position,velocity=velocity,current=current)
        logger.debug(f"【发送】电机ID: {request_message.motor_id}, 指令: {request_message.instruction}, 位置: {request_message.position}, 速度: {request_message.velocity}, 电流: {request_message.current}")
        status = self._parse_response(self.communication_interface.send_and_receive(message=request_message))
        return status

    def set(self, position_list:Optional[List[int]] = None, velocity_list:Optional[List[int]] = None, current_list:Optional[List[int]] = None) -> bool:
        if position_list is not None:
            self.position_list = position_list
        if velocity_list is not None:
            self.velocity_list = velocity_list
        if current_list is not None:
            self.current_list = current_list
        return True

    def loop(self) -> List[RuiyanHandStatusMessage]:
        status_list = []
        for index, motor_id in enumerate(self.motor_ids):
            status_list.append(
                self._set_motor(
                    motor_id=motor_id,
                    position=self.position_list[index],
                    velocity=self.velocity_list[index],
                    current=self.current_list[index]))
        return status_list


    def _parse_response(self, raw_bytes: bytes) -> RuiyanHandStatusMessage:
        """
        按照新协议解析响应数据:
        typedef struct {
            uint64_t :8;          // CMD (前面的命令字)
            uint64_t status:8;    // 故障状态，0表示无故障
            uint64_t P:12;        // 当前位置，0-4095对应0到满行程
            uint64_t V:12;        // 当前速度，-2048~2047单位0.001行程/s
            uint64_t I:12;        // 当前电流，-2048~2047单位0.001A
        }MFingerInfo_t;
        """

            
        # 解析帧头部分: [0xA5][motor_id][0x00][data_len][instruction]
        header_data = struct.unpack('<5B', raw_bytes[:5])
        header, motor_id, _, data_length, instruction = header_data
        
        if header != 0xA5:
            logger.error(f"帧头错误: 期望0xA5, 收到0x{header:02X}")
            return None
        
        finger_data = raw_bytes[5:13]  # 取8字节数据
        
        # 按小端序解析为64位整数
        data_uint64 = struct.unpack('>Q', finger_data)[0]
        
        # 按位解析各个字段
        cmd = (data_uint64 >> 0) & 0xFF        # 最低8位
        status = (data_uint64 >> 8) & 0xFF     # 第9-16位
        position = (data_uint64 >> 16) & 0xFFF # 第17-28位(12位)
        velocity = (data_uint64 >> 28) & 0xFFF # 第29-40位(12位)
        current = (data_uint64 >> 40) & 0xFFF  # 第41-52位(12位)
        
        # 将12位有符号数转换为Python int (速度和电流是有符号的)
        if velocity & 0x800:  # 检查符号位(第12位)
            velocity = velocity - 0x1000  # 转换为负数
        if current & 0x800:   # 检查符号位(第12位)
            current = current - 0x1000   # 转换为负数
            
        response_message = RuiyanHandStatusMessage(
            motor_id=motor_id,
            instruction=RuiyanHandInstructionType(instruction),
            status=status,
            position=position,
            velocity=velocity,
            current=current
        )

        if status != 0:
            status_desc = RuiyanHandStatusCode.get_description(status)
            logger.error(f"【错误】电机ID: {motor_id}, 状态码: {status}, 错误信息: {status_desc}")
        else:
            logger.debug(f"【接收】电机ID: {response_message.motor_id}, 指令: {response_message.instruction}, "
                        f"状态: {response_message.status}, 位置: {response_message.position}, "
                        f"速度: {response_message.velocity}, 电流: {response_message.current}")
                
        return response_message
        

