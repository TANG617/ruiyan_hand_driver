#!/usr/bin/env python3
# Copyright (c) 2025 PSI Robot Team
# Licensed under the Apache License, Version 2.0

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    topic_prefix_arg = DeclareLaunchArgument(
        "topic_prefix",
        default_value="ruiyan_hand",
        description="Topic name prefix",
    )

    frequency_arg = DeclareLaunchArgument(
        "frequency",
        default_value="100",
        description="Node running frequency (Hz)",
    )

    left_port_arg = DeclareLaunchArgument(
        "left_port",
        default_value="/dev/ttyACM0",
        description="Left hand serial device path",
    )

    right_port_arg = DeclareLaunchArgument(
        "right_port",
        default_value="/dev/ttyACM1",
        description="Right hand serial device path",
    )

    baudrate_arg = DeclareLaunchArgument(
        "baudrate", default_value="115200", description="Serial baudrate"
    )

    auto_connect_arg = DeclareLaunchArgument(
        "auto_connect",
        default_value="true",
        description="Whether to auto-connect serial port",
    )

    left_motors_id_arg = DeclareLaunchArgument(
        "left_motors_id",
        default_value="[1,2,3,4,5,6]",
        description="Left hand motor ID list (JSON format)",
    )

    right_motors_id_arg = DeclareLaunchArgument(
        "right_motors_id",
        default_value="[1,2,3,4,5,6]",
        description="Right hand motor ID list (JSON format)",
    )

    instruction_type_arg = DeclareLaunchArgument(
        "instruction_type",
        default_value="0xAA",
        description="Control instruction type \
            (0xA0=read info, \
            0xAA=position velocity current control, \
            0xA5=clear error)",
    )

    enable_left_hand_arg = DeclareLaunchArgument(
        "enable_left_hand",
        default_value="true",
        description="Whether to enable left hand",
    )

    enable_right_hand_arg = DeclareLaunchArgument(
        "enable_right_hand",
        default_value="true",
        description="Whether to enable right hand",
    )

    ruiyan_hand_node = Node(
        package="ruiyan_hand_driver",
        executable="ruiyan_hand_node",
        name="ruiyan_hand_node",
        output="screen",
        parameters=[
            {
                "topic_prefix": LaunchConfiguration("topic_prefix"),
                "frequency": LaunchConfiguration("frequency"),
                "left_port": LaunchConfiguration("left_port"),
                "right_port": LaunchConfiguration("right_port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "auto_connect": LaunchConfiguration("auto_connect"),
                "left_motors_id": LaunchConfiguration("left_motors_id"),
                "right_motors_id": LaunchConfiguration("right_motors_id"),
                "instruction_type": LaunchConfiguration("instruction_type"),
                "enable_left_hand": LaunchConfiguration("enable_left_hand"),
                "enable_right_hand": LaunchConfiguration("enable_right_hand"),
            }
        ],
        remappings=[],
    )

    launch_info = LogInfo(
        msg=[
            "Starting Ruiyan dexterous hand driver node\n",
            "Topic prefix: ",
            LaunchConfiguration("topic_prefix"),
            "\n",
            "Running frequency: ",
            LaunchConfiguration("frequency"),
            " Hz\n",
            "Left hand serial port: ",
            LaunchConfiguration("left_port"),
            "\n",
            "Right hand serial port: ",
            LaunchConfiguration("right_port"),
            "\n",
            "Baudrate: ",
            LaunchConfiguration("baudrate"),
            "\n",
            "Left hand motor IDs: ",
            LaunchConfiguration("left_motors_id"),
            "\n",
            "Right hand motor IDs: ",
            LaunchConfiguration("right_motors_id"),
            "\n",
            "Control instruction type: ",
            LaunchConfiguration("instruction_type"),
            "\n",
            "Enable left hand: ",
            LaunchConfiguration("enable_left_hand"),
            "\n",
            "Enable right hand: ",
            LaunchConfiguration("enable_right_hand"),
        ]
    )

    return LaunchDescription(
        [
            topic_prefix_arg,
            frequency_arg,
            left_port_arg,
            right_port_arg,
            baudrate_arg,
            auto_connect_arg,
            left_motors_id_arg,
            right_motors_id_arg,
            instruction_type_arg,
            enable_left_hand_arg,
            enable_right_hand_arg,
            launch_info,
            ruiyan_hand_node,
        ]
    )
