import sys
import os
import logging
from typing import Container

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

sys.path.append(os.path.join(os.path.dirname(__file__),"..",".."))


from ruiyan_hand_driver.interface import   SerialInterface, RuiyanHandInstructionType
from ruiyan_hand_driver.controller import  RuiyanHandController
import time


interface = SerialInterface(port="/dev/ttyACM0", baudrate=115200, mock=False,auto_connect=True)
controller = RuiyanHandController(interface,motors_id=[1,2,3,4,5,6],instruction=RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT)
# interface.connect()
# interface.send_message(message=message)
# controller.connect()

while 1:
    position_list = [0,0,3840,3840,3840,3840]
    velocity_list = [1000,1000,1000,1000,1000,1000]
    current_list = [500, 500, 500, 500, 500, 500]
    success = controller.set(position_list=position_list, velocity_list=velocity_list, current_list=current_list)
    responses = controller.loop()
    time.sleep(1)
    for response in responses:
        response.print()

    # response = bytearray([0xA5, 0x02, 0x01, 0x08, 0xAA, 0x00, 0xE7, 0x03, 0x00, 0x04, 0x00, 0x00, 0x48])
    
    position_list = [0, 0, 0, 0, 0, 0]
    velocity_list = [1000, 1000, 1000, 1000, 1000, 1000]
    current_list = [500, 500, 500, 500, 500, 500]
    success = controller.set(position_list=position_list,velocity_list=velocity_list,current_list=current_list)
    responses = controller.loop()
    time.sleep(1)
    for response in responses:
        response.print()
# print(controller._parse_response(response))


