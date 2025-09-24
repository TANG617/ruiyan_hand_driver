#!/usr/bin/env python3
# Copyright (c) 2025 PSI Robot Team
# Licensed under the Apache License, Version 2.0

import logging
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Optional

logger = logging.getLogger(__name__)


class RuiyanHandInstructionType(IntEnum):
    READ_MOTOR_INFO = 0xA0
    CTRL_MOTOR_POSITION_VELOCITY_CURRENT = 0xAA
    CLEAR_MOTOR_ERROR = 0xA5


class RuiyanHandStatusCode(IntEnum):
    SERVO_OK = 0
    SERVO_TEMPERATURE_HIGH_WARN = 1
    SERVO_TEMPERATURE_HIGH_ERROR = 2
    SERVO_VOLTAGE_LOW_ERROR = 3
    SERVO_VOLTAGE_HIGH_ERROR = 4
    SERVO_CURRENT_OVER_ERROR = 5
    SERVO_TORQUE_OVER_ERROR = 6
    SERVO_FUSE_ERROR = 7
    SERVO_PWM_ERROR = 8
    SERVO_DRIVE_ERROR = 9
    SERVO_HALL_ERROR = 10
    SERVO_FAIL = 250
    SERVO_PARAM_ERROR = 251
    SERVO_LIB_INIT_ERROR = 252
    CAN_FORMAT_ERROR = 253
    CAN_MSG_SENT_FAIL = 254
    LIB_HOOK_APPLY_FAILED = 255

    @classmethod
    def get_description(cls, status_code: int) -> str:
        descriptions = {
            cls.SERVO_OK: "Operation successful",
            cls.SERVO_TEMPERATURE_HIGH_W: "Motor temperature high warning",
            cls.SERVO_TEMPERATURE_HIGH_E: "Motor temperature high protection",
            cls.SERVO_VOLTAGE_LOW_E: "Motor low voltage protection",
            cls.SERVO_VOLTAGE_HIGH_E: "Motor high voltage protection",
            cls.SERVO_CURRENT_OVER_E: "Motor overcurrent protection",
            cls.SERVO_TORQUE_OVER_E: "Motor torque protection",
            cls.SERVO_FUSE_E: "Motor fuse error protection",
            cls.SERVO_PWM_E: "Motor stall protection",
            cls.SERVO_DRIVE_E: "Driver exception protection",
            cls.SERVO_HALL_E: "Motor hall error protection",
            cls.SERVO_FAIL: "Servo no response or timeout",
            cls.SERVO_PARAM_ERR: "Servo response data error",
            cls.SERVO_LIB_INIT_ERR: "Servo group structure not initialized",
            cls.CAN_FORMAT_ERROR: "CAN data format error",
            cls.CAN_MSG_SENT_FAIL: "CAN message send failed",
            cls.LIB_HOOK_APPLY_FAILED: "Hook application failed",
        }
        return descriptions.get(
            status_code, f"Unknown status code: {status_code}"
        )


class CommunicationType(Enum):
    CAN = "can"
    SERIAL = "serial"


@dataclass
class RuiyanHandControlMessage:
    motor_id: int
    instruction: RuiyanHandInstructionType
    position: Optional[int]
    velocity: Optional[int]
    current: Optional[int]

    def print(self):
        print(
            f"motor_id: {self.motor_id}, "
            f"instruction: {self.instruction}, "
            f"position: {self.position}, "
            f"velocity: {self.velocity}, "
            f"current: {self.current}"
        )


@dataclass
class RuiyanHandStatusMessage:
    motor_id: int
    instruction: RuiyanHandInstructionType
    status: int
    position: Optional[int]
    velocity: Optional[int]
    current: Optional[int]

    def print(self):
        print(
            f"Motor ID: {self.motor_id}, "
            f"Instruction: {self.instruction}, "
            f"Status: {self.status}, "
            f"Position: {self.position}, "
            f"Velocity: {self.velocity}, "
            f"Current: {self.current}"
        )


class CommunicationInterface(ABC):

    def __init__(self, auto_connect: bool):
        self.connected = False
        if auto_connect:
            self.connect()

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        pass

    @abstractmethod
    def send_and_receive(self, message: RuiyanHandControlMessage) -> bytes:
        pass


class SerialInterface(CommunicationInterface):

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float = 0.01,
        auto_connect: bool = False,
        mock: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_controller = None
        self.mock = mock
        super().__init__(auto_connect)

    def connect(self) -> bool:
        if self.mock:
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
                bytesize=serial.EIGHTBITS,
            )
            self.connected = True
            logger.info(f"Serial port opened successfully: {self.port}")
            return True
        except ImportError:
            self.connected = False
            logger.error(
                "Serial library not installed, please install pyserial: "
                "pip install pyserial"
            )
            return False
        except Exception as e:
            self.connected = False
            logger.error(f"Serial port open failed: {e}")
            return False

    def disconnect(self):
        if self.serial_controller and self.connected:
            self.serial_controller.close()
            self.connected = False
            logger.info("Serial connection disconnected")

    def is_connected(self) -> bool:
        return self.connected

    def _send_message(self, message: RuiyanHandControlMessage) -> bool:
        if not self.connected:
            logger.error("Serial port not connected")
            return False

        if self.mock:
            serial_frame = self._build_serial_frame(message)
            logger.debug(
                f"Send - Motor ID: {message.motor_id}, "
                f"Instruction: {hex(message.instruction)}, "
                f"Frame data: {' '.join([f'{byte:02X}' for byte in serial_frame])}" # noqa E999
            )
            return True

        try:
            serial_frame = self._build_serial_frame(message)
            self.serial_controller.write(serial_frame)
            logger.debug(
                f"Send - Motor ID: {message.motor_id}, "
                f"Instruction: {hex(message.instruction)}, "
                f"Frame data: {' '.join([f'{byte:02X}' for byte in serial_frame])}" # noqa
            )
            return True
        except Exception as e:
            logger.error(f"Serial message send failed: {e}")
            return False

    def _receive_message(self) -> bytes:
        if not self.connected:
            return None
        try:
            response = self.serial_controller.read(64)
            return response
        except Exception as e:
            logger.error(f"Serial message receive failed: {e}")
            return None

    def _build_serial_frame(self, message: RuiyanHandControlMessage) -> bytes:
        serial_frame = struct.pack(
            "<B B B 2B 3H 1B",
            0xA5,
            message.motor_id,
            0x00,
            0x08,
            message.instruction,
            message.position,
            message.velocity,
            message.current,
            0x00,
        )

        checksum = 0
        for byte in serial_frame:
            checksum += byte

        serial_frame = struct.pack(
            "<B B B 2B 3H 1B 1B",
            0xA5,
            message.motor_id,
            0x00,
            0x08,
            message.instruction,
            message.position,
            message.velocity,
            message.current,
            0x00,
            checksum & 0xFF,
        )

        return serial_frame

    def send_and_receive(self, message: RuiyanHandControlMessage) -> bytes:
        if not self._send_message(message):
            return b""

        if self.mock:
            logger.debug("Mock mode, no need to receive data")
            mock_response = struct.pack(
                "<5B 8B",
                0xA5,
                message.motor_id,
                0x00,
                0x08,
                int(message.instruction),
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
            )
            return mock_response
        response = self._receive_message()
        return response if response is not None else b""
