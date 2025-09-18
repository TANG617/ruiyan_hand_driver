import sys
import os
import logging
from typing import Container

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

sys.path.append(os.path.join(os.path.dirname(__file__),"..",".."))


from ruiyan_hand_driver.communication_interface import   SerialInterface, RuiyanHandInstructionType
from ruiyan_hand_driver.ruiyan_hand_controller import  RuiyanHandController


interface = SerialInterface(port="/dev/tty.usbmodemBD65E8ABCD1", baudrate=115200, mock=False)
controller = RuiyanHandController(interface,motors_id=[4,5,6,3,1,2])
# interface.connect()
# interface.send_message(message=message)
controller.connect()
position_list = [4095,4095,4095,4095,4095,4095]
velocity_list = [1000,1000,1000,1000,1000,1000]
current_list = [1000,1000,1000,1000,1000,1000]
response = controller.set_motors(RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT,position=position_list,velocity=velocity_list,current=current_list)
# response = bytearray([0xA5, 0x02, 0x01, 0x08, 0xAA, 0x00, 0xE7, 0x03, 0x00, 0x04, 0x00, 0x00, 0x48])
# print(controller._parse_response(response))


