# Copyright (c) 2025 PSI Robot Team
# Licensed under the Apache License, Version 2.0

"""Test module for RuiyanHandController."""

import logging
import os
import sys
import unittest

from ruiyan_hand_driver.controller import RuiyanHandController
from ruiyan_hand_driver.interface import (
    RuiyanHandInstructionType,
    SerialInterface,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))


class TestRuiyanHandController(unittest.TestCase):
    """Test class for RuiyanHandController."""

    def setUp(self):
        """Set up test fixtures."""
        # Use mock mode for testing to avoid requiring real hardware connection
        self.interface = SerialInterface(
            port="/dev/ttyACM0",
            baudrate=115200,
            mock=False,
            auto_connect=True,  # Use mock mode
        )
        self.controller = RuiyanHandController(
            self.interface,
            motors_id=[1, 2, 3, 4, 5, 6],
            instruction=RuiyanHandInstructionType.CTRL_MOTOR_POSITION_VELOCITY_CURRENT,  # noqa
        )

    def test_controller_initialization(self):
        """Test controller initialization."""
        self.assertIsNotNone(self.controller)
        self.assertEqual(len(self.controller.motor_ids), 6)
        self.assertEqual(self.controller.motor_ids, [1, 2, 3, 4, 5, 6])

    def test_set_motor_positions(self):
        """Test setting motor positions."""
        position_list = [0, 0, 3840, 3840, 3840, 3840]
        velocity_list = [1000, 1000, 1000, 1000, 1000, 1000]
        current_list = [500, 500, 500, 500, 500, 500]

        # Test setting positions
        success = self.controller.set(
            position_list=position_list,
            velocity_list=velocity_list,
            current_list=current_list,
        )
        self.assertTrue(success)

    def test_controller_loop(self):
        """Test controller loop."""
        # First set some positions
        position_list = [0, 0, 3840, 3840, 3840, 3840]
        velocity_list = [1000, 1000, 1000, 1000, 1000, 1000]
        current_list = [500, 500, 500, 500, 500, 500]

        self.controller.set(
            position_list=position_list,
            velocity_list=velocity_list,
            current_list=current_list,
        )

        # Test loop
        responses = self.controller.loop()
        self.assertIsInstance(responses, list)

    def tearDown(self):
        """Clean up after tests."""
        if hasattr(self, "interface"):
            self.interface.disconnect()


if __name__ == "__main__":
    unittest.main()
