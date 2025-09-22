import sys
import os
import logging
from typing import Container

import rclpy

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

sys.path.append(os.path.join(os.path.dirname(__file__),"..",".."))


from ruiyan_hand_driver.interface import   SerialInterface, RuiyanHandInstructionType
from ruiyan_hand_driver.controller import  RuiyanHandController
import time

from ruiyan_hand_driver.node import RuiyanHandNode


interface = SerialInterface(port="/dev/ttyACM2", baudrate=115200, mock=False, auto_connect=True)
controller = RuiyanHandController(communication_interface=interface,motors_id=[1,2,3,4,5,6], instruction=RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT)
node = RuiyanHandNode(left_hand=controller,right_hand=None)
rclpy.spin(node)