from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .controller import RuiyanHandController, RuiyanHandInstructionType

class RuiyanHandNode(Node):
    def __init__(self, left_hand:Optional[RuiyanHandController], right_hand: Optional[RuiyanHandController], topic_prefix:str="ruiyan", frequency:int=100):
        self.topic_prefix = topic_prefix
        self.left_hand = left_hand
        self.right_hand = right_hand

        self.create_timer(1.0 / frequency, self.loop)  # 100Hz控制循环
    
    def loop(self):
        position_list = [0, 0, 0, 0, 0, 0]
        velocity_list = [1000, 1000, 1000, 1000, 1000, 1000]
        current_list = [500, 500, 500, 500, 500, 500]
        response = self.left_hand.set_motors(RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT,position=position_list,velocity=velocity_list,current=current_list)
        logger.debug("YES!!")
        return
