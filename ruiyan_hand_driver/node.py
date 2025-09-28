# Copyright (c) 2025 PSI Robot Team
# Licensed under the Apache License, Version 2.0

import json
import logging
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from .controller import RuiyanHandController, RuiyanHandInstructionType
from .interface import RuiyanHandStatusMessage, SerialInterface

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

logger = logging.getLogger(__name__)


class RuiyanHandNode(Node):
    def __init__(
        self,
        left_hand: Optional[RuiyanHandController] = None,
        right_hand: Optional[RuiyanHandController] = None,
        topic_prefix: str = "ruiyan",
        frequency: int = 100,
    ):
        super().__init__("ruiyan_hand_node")

        self.declare_parameter("topic_prefix", "ruiyan_hand")
        self.declare_parameter("frequency", 100)
        self.declare_parameter("left_port", "/dev/ttyACM0")
        self.declare_parameter("right_port", "/dev/ttyACM1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("auto_connect", True)
        self.declare_parameter("left_motors_id", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("right_motors_id", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("instruction_type", "0xAA")
        self.declare_parameter("enable_left_hand", True)
        self.declare_parameter("enable_right_hand", False)

        self.topic_prefix = (
            self.get_parameter("topic_prefix")
            .get_parameter_value()
            .string_value
        )
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().integer_value
        )
        left_port = (
            self.get_parameter("left_port").get_parameter_value().string_value
        )
        right_port = (
            self.get_parameter("right_port").get_parameter_value().string_value
        )
        baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        auto_connect = (
            self.get_parameter("auto_connect").get_parameter_value().bool_value
        )
        left_motors_id = (
            self.get_parameter("left_motors_id")
            .get_parameter_value()
            .integer_array_value
        )
        right_motors_id = (
            self.get_parameter("right_motors_id")
            .get_parameter_value()
            .integer_array_value
        )
        instruction_type_str = (
            self.get_parameter("instruction_type")
            .get_parameter_value()
            .string_value
        )
        enable_left_hand = (
            self.get_parameter("enable_left_hand")
            .get_parameter_value()
            .bool_value
        )
        enable_right_hand = (
            self.get_parameter("enable_right_hand")
            .get_parameter_value()
            .bool_value
        )

        # left_motors_id 和 right_motors_id 已经是整数数组，无需JSON解析

        instruction_type = int(instruction_type_str, 16)

        if left_hand is None and right_hand is None:
            if enable_left_hand:
                left_hand_interface = SerialInterface(
                    port=left_port,
                    baudrate=baudrate,
                    mock=False,
                    auto_connect=auto_connect,
                )
                self.left_hand = RuiyanHandController(
                    left_hand_interface,
                    motors_id=left_motors_id,
                    instruction=RuiyanHandInstructionType(instruction_type),
                )
            else:
                self.left_hand = None

            if enable_right_hand:
                right_hand_interface = SerialInterface(
                    port=right_port,
                    baudrate=baudrate,
                    mock=False,
                    auto_connect=auto_connect,
                )
                self.right_hand = RuiyanHandController(
                    right_hand_interface,
                    motors_id=right_motors_id,
                    instruction=RuiyanHandInstructionType(instruction_type),
                )
            else:
                self.right_hand = None
        else:
            self.left_hand = left_hand
            self.right_hand = right_hand

        if self.left_hand:
            self.left_hand.position_list = [0, 0, 0, 0, 0, 0]
            self.left_hand.velocity_list = [1000, 1000, 1000, 1000, 1000, 1000]
            self.left_hand.current_list = [500, 500, 500, 500, 500, 500]
        if self.right_hand:
            self.right_hand.position_list = [0, 0, 0, 0, 0, 0]
            self.right_hand.velocity_list = [
                1000,
                1000,
                1000,
                1000,
                1000,
                1000,
            ]
            self.right_hand.current_list = [500, 500, 500, 500, 500, 500]

        if self.left_hand:
            self.left_hand_joint_states_publisher = self.create_publisher(
                JointState, f"/{self.topic_prefix}/left/joint_states", 10
            )
            self.left_hand_set_angles_subscription = self.create_subscription(
                JointState,
                f"/{self.topic_prefix}/left/set_angles",
                lambda msg: self.set(msg, self.left_hand),
                10,
            )

        if self.right_hand:
            self.right_hand_joint_states_publisher = self.create_publisher(
                JointState, f"/{self.topic_prefix}/right/joint_states", 10
            )
            self.right_hand_set_angles_subscription = self.create_subscription(
                JointState,
                f"/{self.topic_prefix}/right/set_angles",
                lambda msg: self.set(msg, self.right_hand),
                10,
            )

        if frequency != 100:
            self.frequency = frequency
        else:
            self.frequency = self.frequency

        self.create_timer(1.0 / self.frequency, self.loop)

    def _ruiyan_status_message_list_to_joint_state(
        self, status_list: List[RuiyanHandStatusMessage], hand_type: str
    ) -> JointState:
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = f"{hand_type}_hand_base_link"

        joint_names = []
        positions = []
        velocities = []
        efforts = []

        # 过滤掉None值并排序
        valid_status_list = [status for status in status_list if status is not None]
        sorted_status = sorted(valid_status_list, key=lambda x: x.motor_id)

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
            left_joint_state = self._ruiyan_status_message_list_to_joint_state(
                left_hand_status_list, "left"
            )
            self.left_hand_joint_states_publisher.publish(left_joint_state)

        if self.right_hand:
            right_hand_status_list = self.right_hand.loop()
            right_joint_state = (
                self._ruiyan_status_message_list_to_joint_state(
                    right_hand_status_list, "right"
                )
            )
            self.right_hand_joint_states_publisher.publish(right_joint_state)

    def set(self, msg: JointState, controller: RuiyanHandController):
        controller.position_list = [int(p) for p in msg.position]
        controller.velocity_list = [int(v) for v in msg.velocity]
        controller.current_list = [int(e) for e in msg.effort]


def main(args=None):
    
    rclpy.init(args=args)
    node = RuiyanHandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Node manually stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
