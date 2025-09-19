import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .controller import RuiyanHandController

class RuiyanHandNode(Node):
    def __init__(self, controller:RuiyanHandController, topic_name: str):
        self.topic_name = topic_name
        self.controller = controller
    
    def loop(self, frequency:int):
        return
