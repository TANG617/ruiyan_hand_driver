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
from .interface import RuiyanHandControlMessage, CommunicationInterface, RuiyanHandControlMessage, RuiyanHandStatusMessage, RuiyanHandInstructionType

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
        logger.info(f"【发送】电机ID: {request_message.motor_id}, 指令: {request_message.instruction}, 位置: {request_message.position}, 速度: {request_message.velocity}, 电流: {request_message.current}")
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

    def loop(self):
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
        # A5 02 01 08 AA 
        # 00 E7 
        # 03 00 
        # 04 00 
        # 00 48
        raw = struct.unpack('>B B B 2B 3H 1B 1B', raw_bytes)
        header, _, motor_id, data_length, instruction = raw[0:5]

        match instruction:
            case RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT:
                position, velocity, current, validation = raw[6:]
                response_message =  RuiyanHandStatusMessage(
                motor_id=motor_id,
                instruction=RuiyanHandInstructionType(instruction),
                position=position,
                velocity=velocity,
                current=current
                )
                logger.info(f"【接收】电机ID: {response_message.motor_id}, 指令: {response_message.instruction}, 位置: {response_message.position}, 速度: {response_message.velocity}, 电流: {response_message.current}")
                return response_message
        

