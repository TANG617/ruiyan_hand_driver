#!/usr/bin/env python3
"""
Dexhand控制器模块
根据API函数封装对应的DexhandMessage数组，传递给communication_interface
"""

import time
import struct
import logging
from turtle import pos
from typing import Dict, List, Optional, Any
from enum import IntEnum
from .communication_interface import RuiyanHandMessage, CommunicationInterface, CommunicationType

logger = logging.getLogger(__name__)


class RuiyanHandInstructionType(IntEnum):
    READ_MOTOR_INFO = 0xA0
    CTRL_MOTOR_POSITION_VELOCITY_CURRENT = 0xAA
    CLEAR_MOTOR_ERROR = 0xA5

class RuiyanHandController:
    """Dexhand控制器，封装所有API函数"""
    
    
    def __init__(self, communication_interface:CommunicationInterface, motors_id:[int]):

        self.motor_ids = motors_id
        self.communication_interface = communication_interface
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

    # def _create_message(self, command: int, motor_id: int, data_payload: List[int] = None, 
    #                    timeout: float = 1.0, expected_response_id: Optional[int] = None) -> DexhandMessage:
    #     """创建DexhandMessage对象"""
    #     return DexhandMessage(
    #         command=command,
    #         motor_id=motor_id,
    #         data_payload=data_payload or [],
    #         timeout=timeout,
    #         expected_response_id=expected_response_id
    #     )
    
    def _validate_motor_ids(self, motor_ids: List[int]) -> bool:
        """验证电机ID列表"""
        for motor_id in motor_ids:
            if motor_id not in self.motor_ids:
                return False
        return True

    def _set_motor(self, motor_id:int, instruction:RuiyanHandInstructionType, position:Optional[int], velocity:Optional[int], current:Optional[int]) -> RuiyanHandMessage:#TODO: Use RuiyanHandInfo instead
        payload=[]
        match instruction:
            case RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT:
                payload.extend(
                    [position & 0xFF,              # 位置低字节
                    (position >> 8) & 0xFF,       # 位置高字节
                    velocity & 0xFF,                 # 速度低字节
                    (velocity >> 8) & 0xFF,          # 速度高字节
                    current & 0xFF,         # 电流低字节
                    (current >> 8) & 0xFF,   # 电流高字节
                    0x00
                    ]
                )

        message = RuiyanHandMessage(motor_id=motor_id,instruction=instruction,payload=payload)
        received_message = self.communication_interface.send_and_receive(message=message)
        return received_message

    def set_motors(self, instruction:RuiyanHandInstructionType, position:Optional[List[int]], velocity:Optional[List[int]], current:Optional[List[int]]) -> List[RuiyanHandMessage]:
        received_message_list = []
        for index, motor_id in enumerate(self.motor_ids):
            received_message_list.append(
                self._set_motor(
                    motor_id=motor_id,
                    instruction=instruction,
                    position=position[index],
                    velocity=velocity[index],
                    current=current[index]))
        return received_message_list

    
    # def get_motors_info_messages(self, timeout: float = 1.0) -> List[DexhandMessage]:
    #     """生成读取所有电机信息的消息列表 - 使用多电机协议"""
    #     # 根据协议示例，一次发送所有电机的读取命令
    #     # 数据格式: [命令A0][电机1数据8字节][命令A0][电机2数据8字节][命令A0][电机3数据8字节]
    #     data_payload = []
        
    #     for motor_id in self.motor_ids:
    #         # 每个电机8字节数据: [命令][电机ID][6个0字节]
    #         data_payload.extend([
    #             self.COMMAND_READ_MOTOR_INFO,  # 0xA0
    #             motor_id,                      # 电机ID
    #             0x00, 0x00, 0x00, 0x00, 0x00, 0x00  # 6个0字节
    #         ])
        
    #     # 使用第一个电机ID作为消息ID（用于RS485帧的ID字段）
    #     message = self._create_message(
    #         command=self.COMMAND_READ_MOTOR_INFO,  # 使用读取命令
    #         motor_id=self.motor_ids[0],  # 使用第一个电机ID作为消息ID
    #         data_payload=data_payload,
    #         timeout=timeout
    #     )
    #     return [message]
    
    
    # def move_motors_messages(self, positions: List[int], speeds: List[int], 
    #                        current_limits: List[int], timeout: float = 1.0) -> List[DexhandMessage]:
    #     """生成控制电机位置的消息列表 - 使用多电机协议"""
    #     if not (len(positions) == len(speeds) == len(current_limits) <= len(self.motor_ids)):
    #         logger.error("位置、速度、电流限制列表长度不匹配或超出电机数量")
    #         return []
        
    #     # 根据测试命令格式，构建多电机控制数据
    #     # 数据格式: [命令AA][电机ID][位置低字节][位置高字节][速度低字节][速度高字节][电流低字节][电流高字节] (每个电机8字节)
    #     data_payload = []
        
    #     for i, motor_id in enumerate(self.motor_ids):
    #         if i < len(positions):
    #             # 每个电机8字节数据: [命令AA][电机ID][位置低字节][位置高字节][速度低字节][速度高字节][电流低字节][电流高字节]
    #             data_payload.extend([
    #                 self.COMMAND_CTRL_MOTOR_POSITION,  # 0xAA
    #                 motor_id,                          # 电机ID
    #                 positions[i] & 0xFF,              # 位置低字节
    #                 (positions[i] >> 8) & 0xFF,       # 位置高字节
    #                 speeds[i] & 0xFF,                 # 速度低字节
    #                 (speeds[i] >> 8) & 0xFF,          # 速度高字节
    #                 current_limits[i] & 0xFF,         # 电流低字节
    #                 (current_limits[i] >> 8) & 0xFF   # 电流高字节
    #             ])
        
    #     # 使用第一个电机ID作为消息ID（用于RS485帧的ID字段）
    #     message = self._create_message(
    #         command=self.COMMAND_CTRL_MOTOR_POSITION,  # 使用控制命令
    #         motor_id=self.motor_ids[0],  # 使用第一个电机ID作为消息ID
    #         data_payload=data_payload,
    #         timeout=timeout
    #     )
    #     return [message]
    
    
    
    # def send_messages(self, messages: List[DexhandMessage]) -> Dict[int, Any]:
    #     """发送消息列表并收集响应"""
    #     if not self.is_connected():
    #         logger.error("通信接口未连接")
    #         return {}
        
    #     results = {}
        
    #     # 发送所有消息
    #     for message in messages:
    #         if self.comm_manager.send_message(message):
    #             logger.debug(f"消息已发送: 电机{message.motor_id}, 命令{hex(message.command)}, 数据{message.data_payload}")
    #         else:
    #             logger.error(f"消息发送失败: 电机{message.motor_id}, 命令{hex(message.command)}")
        
    #     # 收集响应
    #     if messages:
    #         # 使用第一个消息的超时时间
    #         timeout = messages[0].timeout
    #         logger.debug(f"开始收集响应，超时时间: {timeout}秒")
    #         responses = self.comm_manager.interface._collect_responses(timeout)
    #         logger.debug(f"收集到响应: {list(responses.keys())}")
    #         results.update(responses)
        
    #     return results
    
    
    # def get_motors_info(self, timeout: float = 1.0) -> Dict[int, Dict]:
    #     """获取所有电机信息"""
    #     messages = self.get_motors_info_messages(timeout)
    #     if not messages:
    #         return {}
        
    #     responses = self.send_messages(messages)
    #     results = {}
        
    #     for motor_id, response in responses.items():
    #         results[motor_id] = self._parse_motor_info(response, motor_id)
        
    #     return results
    
    # def get_motors_info_simple(self, timeout: float = 0.1) -> Dict[int, Dict]:
    #     """简单获取电机信息 - 逐个电机读取"""
    #     results = {}
        
    #     for motor_id in self.motor_ids:
    #         # 为每个电机创建单独的消息
    #         message = self._create_message(
    #             command=self.COMMAND_READ_MOTOR_INFO,
    #             motor_id=motor_id,
    #             data_payload=[motor_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    #             timeout=timeout
    #         )
            
    #         # 发送并接收响应
    #         response = self.comm_manager.send_and_receive(message)
    #         if motor_id in response and response[motor_id] is not None:
    #             results[motor_id] = self._parse_motor_info(response[motor_id], motor_id)
    #         else:
    #             results[motor_id] = {'error': f"电机{motor_id}无响应"}
        
    #     return results
    
    # def move_motors(self, positions: List[int], speeds: List[int], 
    #                current_limits: List[int], timeout: float = 1.0) -> Dict[int, Dict]:
    #     """控制多个电机位置"""
    #     messages = self.move_motors_messages(positions, speeds, current_limits, timeout)
    #     if not messages:
    #         return {}
        
    #     responses = self.send_messages(messages)
    #     results = {}
        
    #     for motor_id, response in responses.items():
    #         results[motor_id] = self._parse_motor_info(response, motor_id)
        
    #     return results
    
    # def move_motors_fast(self, positions: List[int], speeds: List[int], 
    #                     current_limits: List[int]) -> bool:
    #     """快速控制多个电机位置 - 仅发送命令，不等待响应"""
    #     messages = self.move_motors_messages(positions, speeds, current_limits, timeout=0.01)
    #     if not messages:
    #         return False
        
    #     # 仅发送消息，不等待响应
    #     success = True
    #     for message in messages:
    #         if not self.comm_manager.send_message(message):
    #             success = False
    #             logger.error(f"快速发送消息失败: 电机{message.motor_id}, 命令{hex(message.command)}")
        
    #     return success
    
    # def clear_motors_error(self, timeout: float = 1.0) -> bool:
    #     """清除所有电机故障，向ID=0广播清除故障命令"""
    #     if not self.is_connected():
    #         logger.error("通信接口未连接")
    #         return False
        
    #     # 创建广播消息，motor_id=0表示广播
    #     message = self._create_message(
    #         command=self.COMMAND_CLEAR_MOTOR_ERROR,
    #         motor_id=0,  # 广播ID
    #         data_payload=[],
    #         timeout=timeout
    #     )
        
    #     # 发送消息
    #     if self.comm_manager.send_message(message):
    #         logger.info("清除电机故障命令已发送（广播）")
    #         return True
    #     else:
    #         logger.error("清除电机故障命令发送失败")
    #         return False
    
    # def _parse_response_bitfields(self, data: bytes) -> Dict:
    #     """解析响应位域数据"""
    #     raw_uint64 = struct.unpack('<Q', data)[0]
    #     raw_uint64 >>= 8
        
    #     status = raw_uint64 & 0xFF
    #     raw_uint64 >>= 8
        
    #     position = raw_uint64 & 0xFFF
    #     raw_uint64 >>= 12
        
    #     velocity_raw = raw_uint64 & 0xFFF
    #     if velocity_raw & 0x800:
    #         velocity = velocity_raw - 0x1000
    #     else:
    #         velocity = velocity_raw
    #     raw_uint64 >>= 12
        
    #     current_raw = raw_uint64 & 0xFFF
    #     if current_raw & 0x800:
    #         current = current_raw - 0x1000
    #     else:
    #         current = current_raw
    #     raw_uint64 >>= 12
        
    #     force_sensor = raw_uint64 & 0xFFF
        
    #     return {
    #         'status': status,
    #         'position': position,
    #         'velocity': velocity,
    #         'current': current,
    #         'force_sensor': force_sensor
    #     }
    
    # def _get_status_description(self, status_code: int) -> str:
    #     """获取状态描述"""
    #     status_descriptions = {
    #         0: "电机正常",
    #         1: "电机过温告警", 
    #         2: "电机过温保护",
    #         3: "电机低压保护",
    #         4: "电机过压保护",
    #         5: "电机过流保护",
    #         6: "电机力矩保护",
    #         7: "电机熔丝位错保护",
    #         8: "电机堵转保护"
    #     }
    #     return status_descriptions.get(status_code, f"未知状态码({status_code})")
    
    # def _parse_motor_info(self, response: Any, motor_id: int) -> Dict:
    #     """解析电机信息响应 - 支持多电机协议"""
    #     if response is None:
    #         return {'error': f"电机{motor_id}未收到应答"}
        
    #     # 提取数据
    #     if hasattr(response, 'data'):
    #         # CAN消息
    #         data = response.data
    #     elif isinstance(response, dict) and 'data_payload' in response:
    #         # RS485消息 - 多电机协议
    #         data = response['data_payload']
    #     elif isinstance(response, dict) and 'raw_data' in response:
    #         # RS485消息 - 旧格式
    #         data = response['raw_data']
    #     else:
    #         return {'error': f"电机{motor_id}响应格式错误"}
        
    #     # 多电机协议解析
    #     if isinstance(response, dict) and 'data_payload' in response:
    #         return self._parse_multi_motor_response(response, motor_id)
        
    #     # 单电机协议解析（保持兼容性）
    #     if len(data) < 8:
    #         return {'error': f"电机{motor_id}数据长度不足"}
        
    #     decoded = {
    #         'motor_id': motor_id,
    #         'command_type': hex(data[0]),
    #         'command_name': self.command_names.get(data[0], f"未知指令({hex(data[0])})"),
    #         'timestamp': time.time(),
    #         'raw_data': [hex(x) for x in data],
    #         'raw_bytes': list(data)
    #     }
        
    #     try:
    #         if data[0] in [self.COMMAND_READ_MOTOR_INFO, self.COMMAND_CTRL_MOTOR_POSITION]:
    #             parsed_fields = self._parse_response_bitfields(data)
                
    #             position = parsed_fields['position']
    #             velocity_raw = parsed_fields['velocity']
    #             current_raw = parsed_fields['current']
    #             force_sensor = parsed_fields['force_sensor']
    #             status = parsed_fields['status']
                
    #             velocity_physical = velocity_raw * 0.001
    #             current_physical = current_raw * 0.001
    #             current_ma = current_raw
                
    #             decoded.update({
    #                 'status_code': status,
    #                 'position_raw': position,
    #                 'velocity_raw': velocity_raw,
    #                 'current_raw': current_raw,
    #                 'force_sensor_raw': force_sensor,
    #                 'current_position': position,
    #                 'current_speed': velocity_raw,
    #                 'current_current': current_ma,
    #                 'status': status,
    #                 'position_normalized': position / 4095.0,
    #                 'velocity_physical': velocity_physical,
    #                 'current_physical': current_physical,
    #                 'current_ma': current_ma,
    #                 'force_sensor_adc': force_sensor,
    #                 'force_sensor_normalized': force_sensor / 4095.0,
    #                 'status_description': self._get_status_description(status),
    #                 'is_normal': status == 0,
    #                 'has_error': status != 0,
    #                 'units': {
    #                     'position': '0-4095 (满行程)',
    #                     'velocity': '0.001行程/s',
    #                     'current': 'mA',
    #                     'force_sensor': 'ADC原始值 (0-4095)'
    #                 }
    #             })
                
    #         else:
    #             decoded['error'] = f"意外的指令类型: 0x{data[0]:02X}"
                
    #     except Exception as e:
    #         decoded['error'] = f"解码过程中发生错误: {e}"
    #         decoded['exception_details'] = str(e)
        
    #     return decoded
    
    # def _parse_multi_motor_response(self, response: Dict, target_motor_id: int) -> Dict:
    #     """解析多电机响应数据"""
    #     data_payload = response['data_payload']
        
    #     logger.debug(f"解析多电机响应: 目标电机ID={target_motor_id}, 数据长度={len(data_payload)}")
    #     logger.debug(f"响应数据: {[hex(x) for x in data_payload]}")
        
    #     # 查找目标电机的数据
    #     # 每个电机8字节: [命令][电机ID][6字节数据]
    #     # 如果设备返回的电机ID都是0，我们根据位置来推断电机ID
    #     motor_index = 0
    #     for i in range(0, len(data_payload), 8):
    #         if i + 7 < len(data_payload):
    #             command = data_payload[i]
    #             motor_id = data_payload[i + 1]
    #             motor_data = data_payload[i + 2:i + 8]
                
    #             # 根据位置推断电机ID（从1开始）
    #             inferred_motor_id = motor_index + 1
                
    #             logger.debug(f"检查电机数据: 位置{i}, 命令={hex(command)}, 电机ID={motor_id}, 推断ID={inferred_motor_id}, 数据={[hex(x) for x in motor_data]}")
                
    #             # 匹配目标电机ID（使用推断的ID）
    #             if inferred_motor_id == target_motor_id:
    #                 # 找到目标电机数据
    #                 decoded = {
    #                     'motor_id': motor_id,
    #                     'command_type': hex(command),
    #                     'command_name': self.command_names.get(command, f"未知指令({hex(command)})"),
    #                     'timestamp': time.time(),
    #                     'raw_data': [hex(x) for x in data_payload],
    #                     'raw_bytes': list(data_payload)
    #                 }
                    
    #                 try:
    #                     if command in [self.COMMAND_READ_MOTOR_INFO, self.COMMAND_CTRL_MOTOR_POSITION]:
    #                         # 解析电机数据（6字节）
    #                         if len(motor_data) >= 6:
    #                             # 根据协议示例解析数据
    #                             # 数据格式: [位置低字节][位置高字节][速度低字节][速度高字节][电流低字节][电流高字节]
    #                             position = motor_data[0] | (motor_data[1] << 8)
    #                             velocity_raw = motor_data[2] | (motor_data[3] << 8)
    #                             current_raw = motor_data[4] | (motor_data[5] << 8)
                                
    #                             # 处理有符号数
    #                             if velocity_raw & 0x8000:
    #                                 velocity_raw -= 0x10000
    #                             if current_raw & 0x8000:
    #                                 current_raw -= 0x10000
                                
    #                             decoded.update({
    #                                 'status_code': 0,  # 假设正常状态
    #                                 'position_raw': position,
    #                                 'velocity_raw': velocity_raw,
    #                                 'current_raw': current_raw,
    #                                 'force_sensor_raw': 0,  # 协议示例中没有力传感器数据
    #                                 'current_position': position,
    #                                 'current_speed': velocity_raw,
    #                                 'current_current': current_raw,
    #                                 'status': 0,
    #                                 'position_normalized': position / 4095.0,
    #                                 'velocity_physical': velocity_raw * 0.001,
    #                                 'current_physical': current_raw * 0.001,
    #                                 'current_ma': current_raw,
    #                                 'force_sensor_adc': 0,
    #                                 'force_sensor_normalized': 0.0,
    #                                 'status_description': self._get_status_description(0),
    #                                 'is_normal': True,
    #                                 'has_error': False,
    #                                 'units': {
    #                                     'position': '0-4095 (满行程)',
    #                                     'velocity': '0.001行程/s',
    #                                     'current': 'mA',
    #                                     'force_sensor': 'ADC原始值 (0-4095)'
    #                                 }
    #                             })
    #                         else:
    #                             decoded['error'] = f"电机{motor_id}数据长度不足"
    #                     else:
    #                         decoded['error'] = f"意外的指令类型: 0x{command:02X}"
                            
    #                 except Exception as e:
    #                     decoded['error'] = f"解码过程中发生错误: {e}"
    #                     decoded['exception_details'] = str(e)
                    
    #                 return decoded
                
    #             motor_index += 1
        
    #     return {'error': f"未找到电机{target_motor_id}的响应数据"}
    
    
    # def __enter__(self):
    #     if not self.is_connected():
    #         self.connect()
    #     return self
    
    # def __exit__(self, exc_type, exc_val, exc_tb):
    #     self.disconnect()
    
    # def __del__(self):
    #     if hasattr(self, 'comm_manager'):
    #         self.disconnect()
