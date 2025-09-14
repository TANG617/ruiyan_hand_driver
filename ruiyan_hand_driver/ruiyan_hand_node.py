#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import logging
import time
from typing import Dict, List, Optional
from .ruiyan_hand_controller import DexhandController
from .communication_interface import CommunicationConfig, CommunicationType

class RuiyanHandController:
    """单个瑞眼灵巧手控制器"""
    
    def __init__(self, hand_type: str, rs485_port: str, rs485_baudrate: int, 
                 motor_ids: List[int], control_rate: float, logger):
        self.hand_type = hand_type
        self.logger = logger
        
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
                raise RuntimeError(f"无法连接到{hand_type}手部设备")
            self.logger.info(f"{hand_type}手部控制器初始化成功 (RS485: {rs485_port})")
        except Exception as e:
            self.logger.error(f"初始化{hand_type}手部控制器失败: {e}")
            raise
        
        # 初始化缓冲区
        self.set_angles_buffer = [0, 0, 0, 0, 0, 0]  # 目标角度缓冲区
        self.joint_states_buffer = [0, 0, 0, 0, 0, 0]  # 当前状态缓冲区
        
        # 频率监控
        self.control_loop_count = 0
        self.status_loop_count = 0
        self.last_freq_check_time = time.time()
        
        # 初始化关节状态消息
        self.current_joint_states = JointState()
        self.current_joint_states.name = [f'{hand_type}_joint_{i}' for i in motor_ids]
        self.current_joint_states.position = [0.0] * len(motor_ids)
        self.current_joint_states.velocity = [0.0] * len(motor_ids)
        self.current_joint_states.effort = [0.0] * len(motor_ids)
        
        self.control_rate = control_rate
        self.motor_ids = motor_ids


class RuiyanHandNode(Node):
    """
    瑞眼灵巧手控制节点 - 支持双手控制
    
    功能:
    - 使用DexhandController控制两个瑞眼灵巧手
    - 仅支持RS485通信接口
    - 提供ROS2话题接口进行双手控制
    
    Topics:
    - /ruiyan_hand/left/set_angles (订阅) - 设置左手关节角度
    - /ruiyan_hand/right/set_angles (订阅) - 设置右手关节角度
    - /ruiyan_hand/left/joint_states (发布) - 左手当前关节状态
    - /ruiyan_hand/right/joint_states (发布) - 右手当前关节状态
    """
    
    def __init__(self):
        super().__init__('ruiyan_hand_node')
        
        # 配置日志
        self.logger = self.get_logger()
        # 设置日志级别为DEBUG
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # 声明参数
        # 左手参数
        self.declare_parameter('left_rs485_port', '/dev/ttyACM0')
        self.declare_parameter('left_rs485_baudrate', 115200)
        self.declare_parameter('left_motor_ids', [1, 2, 3, 4, 5, 6])
        
        # 右手参数
        self.declare_parameter('right_rs485_port', '/dev/ttyACM1')
        self.declare_parameter('right_rs485_baudrate', 115200)
        self.declare_parameter('right_motor_ids', [1, 2, 3, 4, 5, 6])
        
        # 通用参数
        self.declare_parameter('control_rate', 100.0)  # 控制频率100Hz
        
        # 获取参数
        left_rs485_port = self.get_parameter('left_rs485_port').get_parameter_value().string_value
        left_rs485_baudrate = self.get_parameter('left_rs485_baudrate').get_parameter_value().integer_value
        left_motor_ids = self.get_parameter('left_motor_ids').get_parameter_value().integer_array_value
        
        right_rs485_port = self.get_parameter('right_rs485_port').get_parameter_value().string_value
        right_rs485_baudrate = self.get_parameter('right_rs485_baudrate').get_parameter_value().integer_value
        right_motor_ids = self.get_parameter('right_motor_ids').get_parameter_value().integer_array_value
        
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        
        # 创建左手控制器
        try:
            self.left_hand = RuiyanHandController(
                hand_type="left",
                rs485_port=left_rs485_port,
                rs485_baudrate=left_rs485_baudrate,
                motor_ids=list(left_motor_ids),
                control_rate=control_rate,
                logger=self.logger
            )
        except Exception as e:
            self.logger.error(f"左手控制器初始化失败: {e}")
            self.left_hand = None
        
        # 创建右手控制器
        try:
            self.right_hand = RuiyanHandController(
                hand_type="right",
                rs485_port=right_rs485_port,
                rs485_baudrate=right_rs485_baudrate,
                motor_ids=list(right_motor_ids),
                control_rate=control_rate,
                logger=self.logger
            )
        except Exception as e:
            self.logger.error(f"右手控制器初始化失败: {e}")
            self.right_hand = None
        
        # 创建ROS2话题
        self._create_topics()
        
        # 初始化频率监控变量
        self.last_freq_check_time = time.time()
        
        # 创建定时器
        self.create_timer(1.0 / control_rate, self.control_loop)  # 100Hz控制循环
        self.create_timer(1.0 / 5.0, self.status_loop)  # 5Hz状态读取循环（降低频率）
        self.create_timer(1.0, self.check_connection_status)  # 1Hz连接检查
        self.create_timer(5.0, self.print_frequency_stats)  # 5Hz频率统计
        
        self.logger.info("瑞眼灵巧手控制节点初始化完成")
    
    def _create_topics(self):
        """创建ROS2话题"""
        # 左手话题
        if self.left_hand:
            # 订阅左手话题 - 设置关节角度
            self.left_set_angles_subscription = self.create_subscription(
                JointState,
                '/ruiyan_hand/left/set_angles',
                lambda msg: self.set_angles_callback(msg, self.left_hand),
                10
            )
            
            # 发布左手话题 - 当前关节状态
            self.left_joint_states_publisher = self.create_publisher(
                JointState,
                '/ruiyan_hand/left/joint_states',
                10
            )
            self.logger.info("左手ROS2话题创建完成")
        
        # 右手话题
        if self.right_hand:
            # 订阅右手话题 - 设置关节角度
            self.right_set_angles_subscription = self.create_subscription(
                JointState,
                '/ruiyan_hand/right/set_angles',
                lambda msg: self.set_angles_callback(msg, self.right_hand),
                10
            )
            
            # 发布右手话题 - 当前关节状态
            self.right_joint_states_publisher = self.create_publisher(
                JointState,
                '/ruiyan_hand/right/joint_states',
                10
            )
            self.logger.info("右手ROS2话题创建完成")
        
        self.logger.info("所有ROS2话题创建完成")
    
    def set_angles_callback(self, msg: JointState, hand_controller: RuiyanHandController):
        """处理设置关节角度的回调 - 更新到缓冲区"""
        try:
            if len(msg.position) != len(hand_controller.config.motor_ids):
                self.logger.warn(f"接收到的{hand_controller.hand_type}手位置数量({len(msg.position)})与电机数量({len(hand_controller.config.motor_ids)})不匹配")
                return
            
            # 更新目标角度缓冲区
            for i, angle in enumerate(msg.position):
                if i < len(hand_controller.set_angles_buffer):
                    hand_controller.set_angles_buffer[i] = angle
            
            self.logger.debug(f"更新{hand_controller.hand_type}手目标角度缓冲区: {hand_controller.set_angles_buffer}")
            self.logger.debug(f"当前{hand_controller.hand_type}手关节状态缓冲区: {hand_controller.joint_states_buffer}")
                
        except Exception as e:
            self.logger.error(f"设置{hand_controller.hand_type}手关节角度时发生错误: {e}")
    
    def control_loop(self):
        """100Hz控制循环 - 仅发送控制命令，不等待响应"""
        try:
            # 控制左手
            if self.left_hand:
                self._control_single_hand_fast(self.left_hand)
                self.left_hand.control_loop_count += 1
            
            # 控制右手
            if self.right_hand:
                self._control_single_hand_fast(self.right_hand)
                self.right_hand.control_loop_count += 1
            
        except Exception as e:
            self.logger.error(f"控制循环中发生错误: {e}")
    
    def status_loop(self):
        """10Hz状态读取循环 - 读取电机状态并发布"""
        try:
            # 读取左手状态
            if self.left_hand:
                self._read_hand_status(self.left_hand, self.left_joint_states_publisher)
                self.left_hand.status_loop_count += 1
            
            # 读取右手状态
            if self.right_hand:
                self._read_hand_status(self.right_hand, self.right_joint_states_publisher)
                self.right_hand.status_loop_count += 1
            
        except Exception as e:
            self.logger.error(f"状态读取循环中发生错误: {e}")
    
    def _control_single_hand_fast(self, hand_controller: RuiyanHandController):
        """快速控制单个手 - 仅发送命令，不等待响应"""
        try:
            if not hand_controller.controller.is_connected():
                return
            
            # 将角度转换为电机位置值 (假设角度范围0-180度对应位置0-4095)
            positions = []
            speeds = []
            current_limits = []
            
            for angle in hand_controller.set_angles_buffer:
                # 角度转位置 (0-180度 -> 0-4095)
                position = int(angle * 4095 / 180.0)
                position = max(0, min(4095, position))  # 限制范围
                positions.append(position)
                
                # 设置速度 (默认中等速度)
                speeds.append(100)
                
                # 设置电流限制 (默认1000mA)
                current_limits.append(1000)
            
            # 快速发送控制命令 - 不等待响应
            success = hand_controller.controller.move_motors_fast(positions, speeds, current_limits)
            if not success:
                self.logger.warn(f"{hand_controller.hand_type}手快速控制命令发送失败")
            
        except Exception as e:
            self.logger.error(f"{hand_controller.hand_type}手快速控制循环中发生错误: {e}")
    
    def _read_hand_status(self, hand_controller: RuiyanHandController, publisher):
        """读取手部状态并发布"""
        try:
            if not hand_controller.controller.is_connected():
                return
            
            # 读取电机信息 - 使用简单方法逐个读取
            results = hand_controller.controller.get_motors_info_simple(timeout=0.02)  # 20ms超时
            
            # 更新joint_states_buffer和发布状态
            hand_controller.current_joint_states.header.stamp = self.get_clock().now().to_msg()
            
            for i, motor_id in enumerate(hand_controller.config.motor_ids):
                if motor_id in results and not results[motor_id].get('error'):
                    info = results[motor_id]
                    # 位置值转角度 (0-4095 -> 0-180度)
                    angle = info.get('position_normalized', 0.0) * 180.0
                    hand_controller.joint_states_buffer[i] = angle
                    hand_controller.current_joint_states.position[i] = angle
                    # 速度 (保持原始值)
                    hand_controller.current_joint_states.velocity[i] = info.get('velocity_physical', 0.0)
                    # 电流作为effort
                    hand_controller.current_joint_states.effort[i] = info.get('current_ma', 0.0)
                    
                    # 打印每个电机的详细信息
                    self.logger.debug(f"{hand_controller.hand_type}手电机{motor_id}: 位置={info.get('current_position', 0)}, "
                                    f"角度={angle:.2f}°, 速度={info.get('current_speed', 0)}, "
                                    f"电流={info.get('current_ma', 0)}mA, 状态={info.get('status_description', 'unknown')}")
                else:
                    # 如果有错误，保持之前的值
                    hand_controller.current_joint_states.position[i] = hand_controller.joint_states_buffer[i]
                    self.logger.debug(f"{hand_controller.hand_type}手电机{motor_id}: 无响应或错误 - {results.get(motor_id, {}).get('error', 'unknown error')}")
            
            # 发布关节状态
            publisher.publish(hand_controller.current_joint_states)
            
        except Exception as e:
            self.logger.error(f"{hand_controller.hand_type}手状态读取中发生错误: {e}")
    
    def check_connection_status(self):
        """检查连接状态"""
        try:
            # 检查左手连接状态
            if self.left_hand:
                is_connected = self.left_hand.controller.is_connected()
                if not is_connected:
                    self.logger.warn("左手部设备连接丢失，尝试重连...")
                    if self.left_hand.controller.connect():
                        self.logger.info("左手部设备重连成功")
                    else:
                        self.logger.error("左手部设备重连失败")
                else:
                    self.logger.debug("左手部设备连接正常")
            
            # 检查右手连接状态
            if self.right_hand:
                is_connected = self.right_hand.controller.is_connected()
                if not is_connected:
                    self.logger.warn("右手部设备连接丢失，尝试重连...")
                    if self.right_hand.controller.connect():
                        self.logger.info("右手部设备重连成功")
                    else:
                        self.logger.error("右手部设备重连失败")
                else:
                    self.logger.debug("右手部设备连接正常")
                    
        except Exception as e:
            self.logger.error(f"检查连接状态时发生错误: {e}")
    
    def print_frequency_stats(self):
        """打印频率统计信息"""
        try:
            current_time = time.time()
            time_diff = current_time - self.last_freq_check_time
            
            if time_diff > 0:
                # 计算左手频率
                if self.left_hand:
                    left_control_freq = self.left_hand.control_loop_count / time_diff
                    left_status_freq = self.left_hand.status_loop_count / time_diff
                    self.logger.info(f"左手频率统计 - 控制循环: {left_control_freq:.2f}Hz, 状态读取: {left_status_freq:.2f}Hz")
                    # 重置计数器
                    self.left_hand.control_loop_count = 0
                    self.left_hand.status_loop_count = 0
                
                # 计算右手频率
                if self.right_hand:
                    right_control_freq = self.right_hand.control_loop_count / time_diff
                    right_status_freq = self.right_hand.status_loop_count / time_diff
                    self.logger.info(f"右手频率统计 - 控制循环: {right_control_freq:.2f}Hz, 状态读取: {right_status_freq:.2f}Hz")
                    # 重置计数器
                    self.right_hand.control_loop_count = 0
                    self.right_hand.status_loop_count = 0
            
            # 重置时间
            self.last_freq_check_time = current_time
            
        except Exception as e:
            self.logger.error(f"打印频率统计时发生错误: {e}")
    
    def shutdown(self):
        """关闭节点"""
        self.logger.info("正在关闭瑞眼灵巧手控制节点...")
        
        try:
            # 关闭左手控制器
            if hasattr(self, 'left_hand') and self.left_hand:
                self.left_hand.controller.disconnect()
                self.logger.info("左手部控制器已关闭")
            
            # 关闭右手控制器
            if hasattr(self, 'right_hand') and self.right_hand:
                self.right_hand.controller.disconnect()
                self.logger.info("右手部控制器已关闭")
                
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
        print("  订阅: /ruiyan_hand/left/set_angles (JointState) - 设置左手关节角度")
        print("  订阅: /ruiyan_hand/right/set_angles (JointState) - 设置右手关节角度")
        print("  发布: /ruiyan_hand/left/joint_states (JointState) - 左手当前关节状态")
        print("  发布: /ruiyan_hand/right/joint_states (JointState) - 右手当前关节状态")
        print("\n参数配置:")
        print(f"  通信类型: RS485")
        print(f"  左手RS485端口: {ruiyan_node.get_parameter('left_rs485_port').get_parameter_value().string_value}")
        print(f"  左手RS485波特率: {ruiyan_node.get_parameter('left_rs485_baudrate').get_parameter_value().integer_value}")
        print(f"  左手电机ID: {ruiyan_node.get_parameter('left_motor_ids').get_parameter_value().integer_array_value}")
        print(f"  右手RS485端口: {ruiyan_node.get_parameter('right_rs485_port').get_parameter_value().string_value}")
        print(f"  右手RS485波特率: {ruiyan_node.get_parameter('right_rs485_baudrate').get_parameter_value().integer_value}")
        print(f"  右手电机ID: {ruiyan_node.get_parameter('right_motor_ids').get_parameter_value().integer_array_value}")
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
