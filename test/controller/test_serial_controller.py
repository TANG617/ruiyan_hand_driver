import sys
import os
import logging

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

sys.path.append(os.path.join(os.path.dirname(__file__),"..",".."))


from ruiyan_hand_driver.communication_interface import  CommunicationDirectionType, CommunicationType, SerialInterface, RuiyanHandMessage
from ruiyan_hand_driver.ruiyan_hand_controller import  RuiyanHandController, RuiyanHandInstructionType


interface = SerialInterface(port="/dev/ttyACM0", baudrate=115200, mock=True)
controller = RuiyanHandController(interface,motors_id=[1,2,3,4,5,6])
# interface.connect()
# interface.send_message(message=message)
controller.connect()
position_list = [4095,2047,1023,511,255,127]
velocity_list = [4095,2047,1023,511,255,127]
current_list = [4095,2047,1023,511,255,127]
controller.set_motors(RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT,position=position_list,velocity=velocity_list,current=current_list)


