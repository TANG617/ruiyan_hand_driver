# Copyright (c) 2025 PSI Robot Team
# Licensed under the Apache License, Version 2.0

import logging
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from ruiyan_hand_driver.interface import (
    RuiyanHandControlMessage,
    RuiyanHandInstructionType,
    SerialInterface,
)

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)




message = RuiyanHandControlMessage(
    motor_id=2,
    instruction=RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT,
    position=0,
    velocity=3000,
    current=1000,
)

interface = SerialInterface(port="/dev/ttyACM0", baudrate=115200)
interface.connect()
response = interface.send_and_receive(message=message)
