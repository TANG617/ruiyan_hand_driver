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


from ruiyan_hand_driver.communication_interface import  CommunicationDirectionType, CommunicationType, SerialInterface
from ruiyan_hand_driver.ruiyan_hand_controller import  RuiyanHandController, RuiyanHandInstructionType


interface = SerialInterface(port="/dev/ttyACM0", baudrate=115200, mock=True)
controller = RuiyanHandController(interface,motors_id=[1,2,3,4,5,6])
# interface.connect()
# interface.send_message(message=message)
controller.connect()
position_list = [4095,2047,1023,511,255,127]
velocity_list = [4095,2047,1023,511,255,127]
current_list = [4095,2047,1023,511,255,127]
# controller.set_motors(RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT,position=position_list,velocity=velocity_list,current=current_list)
response = bytearray([0xA5, 0x02, 0x01, 0x08, 0xAA, 0x00, 0xE7, 0x03, 0x00, 0x04, 0x00, 0x00, 0x48])
controller._parse_response(response)


