#!/usr/bin/env python3
# Copyright (c) 2025 PSI Robot Team
# Licensed under the Apache License, Version 2.0

import logging
import struct
from typing import List, Optional

from .interface import (
    CommunicationInterface,
    RuiyanHandControlMessage,
    RuiyanHandInstructionType,
    RuiyanHandStatusCode,
    RuiyanHandStatusMessage,
)

logger = logging.getLogger(__name__)


class RuiyanHandController:

    def __init__(
        self,
        communication_interface: CommunicationInterface,
        motors_id: [int],
        instruction: RuiyanHandInstructionType = None,
    ):

        self.motor_ids = motors_id
        self.communication_interface = communication_interface
        self.instruction = instruction
        self.position_list = [0, 0, 0, 0, 0, 0]
        self.velocity_list = [0, 0, 0, 0, 0, 0]
        self.current_list = [0, 0, 0, 0, 0, 0]
        self.command_names = {
            0xA0: "Read motor information",
            0xAA: "Position velocity current hybrid control",
            0xA5: "Clear motor error",
        }

    def connect(self) -> bool:
        return self.communication_interface.connect()

    def disconnect(self):
        return self.communication_interface.disconnect()

    def is_connected(self) -> bool:
        return self.communication_interface.is_connected()

    def _validate_motor_ids(self, motor_ids: List[int]) -> bool:
        for motor_id in motor_ids:
            if motor_id not in self.motor_ids:
                return False
        return True

    def _set_motor(
        self,
        motor_id: int,
        position: Optional[int],
        velocity: Optional[int],
        current: Optional[int],
    ) -> RuiyanHandStatusMessage:
        request_message = RuiyanHandControlMessage(
            motor_id=motor_id,
            instruction=self.instruction,
            position=position,
            velocity=velocity,
            current=current,
        )
        logger.debug(
            f"Motor ID: {request_message.motor_id}, "
            f"Instruction: {request_message.instruction}, "
            f"Position: {request_message.position}, "
            f"Velocity: {request_message.velocity}, "
            f"Current: {request_message.current}"
        )
        status = self._parse_response(
            self.communication_interface.send_and_receive(
                message=request_message
            )
        )
        return status

    def set(
        self,
        position_list: Optional[List[int]] = None,
        velocity_list: Optional[List[int]] = None,
        current_list: Optional[List[int]] = None,
    ) -> bool:
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
                    current=self.current_list[index],
                )
            )
        return status_list

    def _parse_response(self, raw_bytes: bytes) -> RuiyanHandStatusMessage:

        header, motor_id, _, data_length = struct.unpack("<4B", raw_bytes[:4])

        if header != 0xA5:
            logger.error(
                f"Header error: expected 0xA5, received 0x{header:02X}"
            )
            return None

        finger_data = raw_bytes[4:12]

        data_uint64 = struct.unpack("<Q", finger_data)[0]

        instruction = (data_uint64 >> 0) & 0xFF
        status = (data_uint64 >> 8) & 0xFF
        position = (data_uint64 >> 16) & 0xFFF
        velocity = (data_uint64 >> 28) & 0xFFF
        current = (data_uint64 >> 40) & 0xFFF

        if velocity & 0x800:
            velocity = velocity - 0x1000
        if current & 0x800:
            current = current - 0x1000

        response_message = RuiyanHandStatusMessage(
            motor_id=motor_id,
            instruction=RuiyanHandInstructionType(instruction),
            status=status,
            position=position,
            velocity=velocity,
            current=current,
        )

        if status != 0:
            status_desc = RuiyanHandStatusCode.get_description(status)
            logger.error(
                f"Error - Motor ID: {motor_id}, "
                f"Status code: {status}, "
                f"Error message: {status_desc}"
            )
        else:
            if response_message.motor_id == 3:
                logger.debug(
                    f"Received - Motor ID: {response_message.motor_id}, "
                    f"Instruction: {response_message.instruction}, "
                    f"Status: {response_message.status}, "
                    f"Position: {response_message.position}, "
                    f"Velocity: {response_message.velocity}, "
                    f"Current: {response_message.current}"
                )

        return response_message
