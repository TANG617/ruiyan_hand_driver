from typing import Optional, List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from .interface import RuiyanHandStatusMessage, SerialInterface
from .controller import RuiyanHandController, RuiyanHandInstructionType
import logging


logger = logging.getLogger(__name__)
class RuiyanHandNode(Node):
    def __init__(self, left_hand:Optional[RuiyanHandController], right_hand: Optional[RuiyanHandController], topic_prefix:str="ruiyan", frequency:int=50):
        rclpy.init()
        super().__init__("ruiyan_hand_node")
        self.topic_prefix = topic_prefix
        self.left_hand = left_hand
        self.right_hand = right_hand
        if left_hand:
            self.left_hand.position_list = [0, 0, 0, 0, 0, 0]
            self.left_hand.velocity_list = [1000, 1000, 1000, 1000, 1000, 1000]
            self.left_hand.current_list = [500, 500, 500, 500, 500, 500]
        if right_hand:
            self.right_hand.position_list = [0, 0, 0, 0, 0, 0]
            self.right_hand.velocity_list = [1000, 1000, 1000, 1000, 1000, 1000]
            self.right_hand.current_list = [500, 500, 500, 500, 500, 500]

        self.left_hand_joint_states_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/left/joint_states',
            10
        )
        self.left_hand_set_angles_subscription = self.create_subscription(
            JointState,
            '/ruiyan_hand/left/set_angles',
            lambda msg: self.set(msg, self.left_hand),
            10
        )

        self.right_hand_joint_states_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/right/joint_states',
            10
        )
        self.right_hand_set_angles_subscription = self.create_subscription(
            JointState,
            '/ruiyan_hand/right/set_angles',
            lambda msg: self.set(msg, self.right_hand),
            10
        )

        self.create_timer(1.0 / frequency, self.loop)

    def _ruiyan_status_message_list_to_joint_state(self, status_list: List[RuiyanHandStatusMessage], hand_type: str) -> JointState:
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = f"{hand_type}_hand_base_link"
        
        joint_names = []
        positions = []
        velocities = []
        efforts = []
        
        sorted_status = sorted(status_list, key=lambda x: x.motor_id)
        
        for status in sorted_status:
            joint_names.append(f"{hand_type}_hand_joint_{status.motor_id}")
            positions.append(float(status.position or 0))
            velocities.append(float(status.velocity or 0))
            efforts.append(float(status.current or 0))
        
        joint_state.name = joint_names
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        
        return joint_state


    
    def loop(self):
        if self.left_hand:
            left_hand_status_list = self.left_hand.loop()
            left_joint_state = self._ruiyan_status_message_list_to_joint_state(left_hand_status_list, "left")
            self.left_hand_joint_states_publisher.publish(left_joint_state)

        if self.right_hand:
            right_hand_status_list = self.right_hand.loop()
            right_joint_state = self._ruiyan_status_message_list_to_joint_state(right_hand_status_list, "right")
            self.right_hand_joint_states_publisher.publish(right_joint_state)

    def set(self, msg: JointState, controller:RuiyanHandController):
        controller.position_list = [int(p) for p in msg.position]
        controller.velocity_list = [int(v) for v in msg.velocity]
        controller.current_list = [int(e) for e in msg.effort]


def main():
    left_hand_interface = SerialInterface(
        port="/dev/ttyACM0", 
        baudrate=115200, 
        mock=False, 
        auto_connect=True)
    left_hand_controller = RuiyanHandController(
        left_hand_interface,motors_id=[1,2,3,4,5,6],
        instruction=RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT)
    right_hand_interface = SerialInterface(
        port="/dev/ttyACM1", 
        baudrate=115200, 
        mock=False, 
        auto_connect=True)
    right_hand_controller = RuiyanHandController(
        right_hand_interface,
        motors_id=[1,2,3,4,5,6], 
        instruction=RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT)
    node = RuiyanHandNode(
        left_hand=left_hand_controller,
        right_hand = right_hand_controller)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("节点已手动停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

