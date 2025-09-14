#!/bin/bash

# 瑞眼灵巧手测试启动脚本

echo "瑞眼灵巧手测试脚本"
echo "=================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置"
    echo "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 设置工作空间环境
# 使用相对路径查找setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"
echo "WORKSPACE_ROOT: $WORKSPACE_ROOT"
INSTALL_SETUP="$WORKSPACE_ROOT/install/setup.bash"

if [ -f "$INSTALL_SETUP" ]; then
    source "$INSTALL_SETUP"
    echo "工作空间环境已设置"
else
    echo "错误: 工作空间未构建"
    echo "请先运行: colcon build"
    exit 1
fi

# 检查ruiyan_hand_node是否在运行
if ! pgrep -f "ruiyan_hand_node" > /dev/null; then
    echo "警告: ruiyan_hand_node 未运行"
    echo "请先启动手部控制节点:"
    echo "  ros2 run ruiyan_hand_driver ruiyan_hand_node"
    echo "或者使用launch文件:"
    echo "  ros2 launch ruiyan_hand_driver ruiyan_hands.launch.py"
    echo ""
    read -p "是否继续运行测试? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 显示测试选项
echo ""
echo "可用测试:"
echo "1. 简单测试 (快速握拳/张开)"
echo "2. 完整测试 (所有功能测试)"
echo "3. 交互式测试 (手动输入角度)"
echo "4. 启动手部控制节点"
echo "5. 查看话题列表"
echo "6. 退出"
echo ""

while true; do
    read -p "请选择测试 (1-6): " choice
    
    case $choice in
        1)
            echo "运行简单测试..."
            python3 "$SCRIPT_DIR/simple_test.py"
            ;;
        2)
            echo "运行完整测试..."
            python3 "$SCRIPT_DIR/test_ruiyan_hand_control.py"
            ;;
        3)
            echo "运行交互式测试..."
            python3 "$SCRIPT_DIR/interactive_test.py"
            ;;
        4)
            echo "启动手部控制节点..."
            echo "使用默认参数启动..."
            ros2 run ruiyan_hand_driver ruiyan_hand_node &
            echo "节点已启动，PID: $!"
            echo "按任意键继续..."
            read -n 1
            ;;
        5)
            echo "当前ROS2话题:"
            ros2 topic list | grep ruiyan_hand
            echo ""
            echo "话题详细信息:"
            echo "左手设置角度:"
            ros2 topic info /ruiyan_hand/left/set_angles
            echo ""
            echo "右手设置角度:"
            ros2 topic info /ruiyan_hand/right/set_angles
            echo ""
            echo "左手关节状态:"
            ros2 topic info /ruiyan_hand/left/joint_states
            echo ""
            echo "右手关节状态:"
            ros2 topic info /ruiyan_hand/right/joint_states
            ;;
        6)
            echo "退出"
            exit 0
            ;;
        *)
            echo "无效选择，请输入 1-6"
            ;;
    esac
    
    echo ""
    echo "按任意键继续..."
    read -n 1
    echo ""
done
