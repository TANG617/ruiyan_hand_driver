#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import logging
import time
from typing import Dict, List, Optional
from .ruiyan_hand_controller import DexhandController
from .communication_interface import CommunicationConfig, CommunicationType

class RuiyanHandNode(Node):
    """
    瑞眼灵巧手控制节点
    
    功能:
    - 使用DexhandController控制瑞眼灵巧手
    - 仅支持RS485通信接口
    - 提供ROS2话题接口进行手部控制
    
    Topics:
    - /ruiyan_hand/set_angles (订阅) - 设置关节角度
    - /ruiyan_hand/joint_states (发布) - 当前关节状态
    """
    
    def __init__(self):
        super().__init__('ruiyan_hand_node')
        
        # 配置日志
        self.logger = self.get_logger()
        
        # 声明参数
        self.declare_parameter('rs485_port', '/dev/ttyUSB0')
        self.declare_parameter('rs485_baudrate', 115200)
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('control_rate', 100.0)  # 控制频率100Hz
        
        # 获取参数
        rs485_port = self.get_parameter('rs485_port').get_parameter_value().string_value
        rs485_baudrate = self.get_parameter('rs485_baudrate').get_parameter_value().integer_value
        motor_ids = self.get_parameter('motor_ids').get_parameter_value().integer_array_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        
        # 创建RS485通信配置
        self.config = CommunicationConfig.create_rs485_config(
            motor_ids=list(motor_ids),
            port=rs485_port,
            baudrate=rs485_baudrate
        )
        
        # 创建控制器
        try:
            self.controller = DexhandController(self.config)
            if not self.controller.connect():
                raise RuntimeError("无法连接到手部设备")
            self.logger.info("手部控制器初始化成功 (RS485)")
        except Exception as e:
            self.logger.error(f"初始化手部控制器失败: {e}")
            raise
        
        # 初始化缓冲区
        self.set_angles_buffer = [0, 0, 0, 0, 0, 0]  # 目标角度缓冲区
        self.joint_states_buffer = [0, 0, 0, 0, 0, 0]  # 当前状态缓冲区
        
        # 创建ROS2话题
        self._create_topics()
        
        # 创建定时器
        self.create_timer(1.0 / control_rate, self.control_loop)  # 100Hz控制循环
        self.create_timer(1.0, self.check_connection_status)  # 1Hz连接检查
        
        # 初始化关节状态消息
        self.current_joint_states = JointState()
        self.current_joint_states.name = [f'joint_{i}' for i in motor_ids]
        self.current_joint_states.position = [0.0] * len(motor_ids)
        self.current_joint_states.velocity = [0.0] * len(motor_ids)
        self.current_joint_states.effort = [0.0] * len(motor_ids)
        
        self.logger.info("瑞眼灵巧手控制节点初始化完成")
    
    def _create_topics(self):
        """创建ROS2话题"""
        # 订阅话题 - 设置关节角度
        self.set_angles_subscription = self.create_subscription(
            JointState,
            '/ruiyan_hand/set_angles',
            self.set_angles_callback,
            10
        )
        
        # 发布话题 - 当前关节状态
        self.joint_states_publisher = self.create_publisher(
            JointState,
            '/ruiyan_hand/joint_states',
            10
        )
        
        self.logger.info("ROS2话题创建完成")
    
    def set_angles_callback(self, msg: JointState):
        """处理设置关节角度的回调 - 更新到缓冲区"""
        try:
            if len(msg.position) != len(self.config.motor_ids):
                self.logger.warn(f"接收到的位置数量({len(msg.position)})与电机数量({len(self.config.motor_ids)})不匹配")
                return
            
            # 更新目标角度缓冲区
            for i, angle in enumerate(msg.position):
                if i < len(self.set_angles_buffer):
                    self.set_angles_buffer[i] = angle
            
            self.logger.debug(f"更新目标角度缓冲区: {self.set_angles_buffer}")
                
        except Exception as e:
            self.logger.error(f"设置关节角度时发生错误: {e}")
    
    def control_loop(self):
        """100Hz控制循环 - 发送move_motors命令并更新状态"""
        try:
            if not self.controller.is_connected():
                return
            
            # 将角度转换为电机位置值 (假设角度范围0-180度对应位置0-4095)
            positions = []
            speeds = []
            current_limits = []
            
            for angle in self.set_angles_buffer:
                # 角度转位置 (0-180度 -> 0-4095)
                position = int(angle * 4095 / 180.0)
                position = max(0, min(4095, position))  # 限制范围
                positions.append(position)
                
                # 设置速度 (默认中等速度)
                speeds.append(100)
                
                # 设置电流限制 (默认1000mA)
                current_limits.append(1000)
            
            # 发送控制命令
            results = self.controller.move_motors(positions, speeds, current_limits)
            
            # 更新joint_states_buffer和发布状态
            self.current_joint_states.header.stamp = self.get_clock().now().to_msg()
            
            for i, motor_id in enumerate(self.config.motor_ids):
                if motor_id in results and not results[motor_id].get('error'):
                    info = results[motor_id]
                    # 位置值转角度 (0-4095 -> 0-180度)
                    angle = info.get('position_normalized', 0.0) * 180.0
                    self.joint_states_buffer[i] = angle
                    self.current_joint_states.position[i] = angle
                    # 速度 (保持原始值)
                    self.current_joint_states.velocity[i] = info.get('velocity_physical', 0.0)
                    # 电流作为effort
                    self.current_joint_states.effort[i] = info.get('current_ma', 0.0)
                else:
                    # 如果有错误，保持之前的值
                    self.current_joint_states.position[i] = self.joint_states_buffer[i]
            
            # 发布关节状态
            self.joint_states_publisher.publish(self.current_joint_states)
            
        except Exception as e:
            self.logger.error(f"控制循环中发生错误: {e}")
    
    def check_connection_status(self):
        """检查连接状态"""
        try:
            is_connected = self.controller.is_connected()
            if not is_connected:
                self.logger.warn("手部设备连接丢失，尝试重连...")
                if self.controller.connect():
                    self.logger.info("手部设备重连成功")
                else:
                    self.logger.error("手部设备重连失败")
            else:
                self.logger.debug("手部设备连接正常")
        except Exception as e:
            self.logger.error(f"检查连接状态时发生错误: {e}")
    
    def shutdown(self):
        """关闭节点"""
        self.logger.info("正在关闭瑞眼灵巧手控制节点...")
        
        try:
            if hasattr(self, 'controller'):
                self.controller.disconnect()
                self.logger.info("手部控制器已关闭")
        except Exception as e:
            self.logger.error(f"关闭过程中发生错误: {e}")
        finally:
            self.destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # 创建瑞眼灵巧手控制节点
        ruiyan_node = RuiyanHandNode()
        
        print("\n瑞眼灵巧手控制节点已启动")
        print("Topics:")
        print("  订阅: /ruiyan_hand/set_angles (JointState) - 设置关节角度")
        print("  发布: /ruiyan_hand/joint_states (JointState) - 当前关节状态")
        print("\n参数配置:")
        print(f"  通信类型: RS485")
        print(f"  RS485端口: {ruiyan_node.get_parameter('rs485_port').get_parameter_value().string_value}")
        print(f"  RS485波特率: {ruiyan_node.get_parameter('rs485_baudrate').get_parameter_value().integer_value}")
        print(f"  电机ID: {ruiyan_node.get_parameter('motor_ids').get_parameter_value().integer_array_value}")
        print(f"  控制频率: {ruiyan_node.get_parameter('control_rate').get_parameter_value().double_value} Hz")
        print("\n按 Ctrl+C 退出")
        
        # 运行节点
        rclpy.spin(ruiyan_node)
        
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭...")
    except Exception as e:
        print(f"运行时错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'ruiyan_node' in locals():
            ruiyan_node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
