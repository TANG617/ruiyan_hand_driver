#!/bin/bash

echo "瑞眼灵巧手完整系统测试"
echo "=========================="

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置"
    exit 1
fi

echo "ROS2版本: $ROS_DISTRO"

# 检查设备是否存在
if [ ! -e "/dev/ttyACM0" ]; then
    echo "错误: 设备 /dev/ttyACM0 不存在"
    echo "请检查设备连接"
    exit 1
fi

echo "设备 /dev/ttyACM0 存在"

# 检查权限
if [ ! -r "/dev/ttyACM0" ] || [ ! -w "/dev/ttyACM0" ]; then
    echo "警告: 设备权限不足，尝试添加权限..."
    sudo chmod 666 /dev/ttyACM0
fi

echo "设备权限检查完成"

# 设置工作目录 - 使用相对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

echo ""
echo "1. 测试修改后的协议..."
echo "========================"
python3 "$SCRIPT_DIR/test_modified_protocol.py"

echo ""
echo "2. 启动ruiyan_hand_node..."
echo "=========================="
echo "在另一个终端中运行以下命令:"
echo "cd $PROJECT_ROOT"
echo "python3 -m ruiyan_hand_driver.ruiyan_hand_node"
echo ""
echo "等待5秒后开始测试..."

sleep 5

echo ""
echo "3. 测试ROS2节点..."
echo "=================="
python3 "$SCRIPT_DIR/../ros/test_ros2_node.py"

echo ""
echo "测试完成！"
