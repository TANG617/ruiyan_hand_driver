# Copyright (c) 2025 PSI Robot Team
# Licensed under the Apache License, Version 2.0

"""Test module for RuiyanHandNode."""

import logging
import os
import sys
import unittest

import rclpy
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from ruiyan_hand_driver.controller import RuiyanHandController
from ruiyan_hand_driver.interface import (
    RuiyanHandInstructionType,
    SerialInterface,
)
from ruiyan_hand_driver.node import RuiyanHandNode


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


class TestRuiyanHandNode(unittest.TestCase):
    """Test class for RuiyanHandNode."""

    @classmethod
    def setUpClass(cls):
        """Set up class fixtures."""
        # Initialize ROS only once for the entire test class
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Set up test fixtures."""
        # Use mock mode for testing
        self.interface = SerialInterface(
            port="/dev/ttyACM0",
            baudrate=115200,
            mock=False,
            auto_connect=True,  # Use mock mode
        )
        self.controller = RuiyanHandController(
            communication_interface=self.interface,
            motors_id=[1, 2, 3, 4, 5, 6],
            instruction=RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT,  # noqa
        )
        self.node = RuiyanHandNode(left_hand=self.controller, right_hand=None)

    def test_node_initialization(self):
        """Test node initialization."""
        self.assertIsNotNone(self.node)
        self.assertIsNotNone(self.node.left_hand)
        self.assertIsNone(self.node.right_hand)

    def test_node_creation(self):
        """Test node creation."""
        # Test if node is created correctly
        self.assertEqual(self.node.get_name(), "ruiyan_hand_node")
        
    def test_node_spin(self):
        """Test node spinning for a short time."""
        # 创建一个简单的定时器来测试节点运行
        import threading
        import time
        
        def stop_after_delay():
            time.sleep(10)  # 运行100毫秒
            rclpy.shutdown()
        
        # 在后台线程中启动停止定时器
        stop_thread = threading.Thread(target=stop_after_delay)
        stop_thread.daemon = True
        stop_thread.start()
        
        # 运行节点
        try:
            rclpy.spin(self.node)
        except Exception as e:
            # 预期的异常，因为我们在后台线程中调用了shutdown
            pass

    def tearDown(self):
        """Clean up after tests."""
        if hasattr(self, "node"):
            self.node.destroy_node()
        if hasattr(self, "interface"):
            self.interface.disconnect()

    @classmethod
    def tearDownClass(cls):
        """Clean up after all tests."""
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
