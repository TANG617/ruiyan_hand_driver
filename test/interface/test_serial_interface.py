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

message = RuiyanHandMessage(
    motor_id=2,
    instruction=0xAA,
    payload=[0xE8, 0x03, 0xE8, 0x03, 0xF4, 0x01, 0x00]
)

interface = SerialInterface(port="/dev/ttyACM0", baudrate=115200, mock=True)
interface.connect()
interface.send_message(message=message)


