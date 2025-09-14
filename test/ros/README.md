# ROS测试脚本

这个文件夹包含所有与ROS2相关的测试脚本，这些脚本通过ROS2话题和节点来测试瑞眼灵巧手的功能。

## 文件说明

### 主要测试脚本
- **`test_ruiyan_hand_control.py`** - 瑞眼灵巧手控制测试脚本
  - 向set_angles话题发送JointState消息来控制手部
  - 包含基本动作、单个手指、手势、平滑运动等测试
  - 支持双手控制

- **`test_ros2_node.py`** - ROS2节点测试脚本
  - 测试修改后的协议是否能正常控制灵巧手
  - 验证ROS2节点功能

- **`simple_test.py`** - 简单测试脚本
  - 快速测试手部控制功能
  - 包含张开、握拳等基本动作

- **`interactive_test.py`** - 交互式测试脚本
  - 允许用户手动输入角度值进行测试
  - 支持实时控制

### 诊断和配置脚本
- **`diagnose_ruiyan.py`** - 瑞眼灵巧手诊断脚本
  - 检查通信状态和协议问题
  - 诊断ROS2节点运行状态

- **`fix_ruiyan_config.py`** - 配置修复脚本
  - 重新配置节点使用正确的端口
  - 自动检测可用端口

- **`test_with_usb.py`** - USB端口测试脚本
  - 使用USB端口测试瑞眼灵巧手
  - 验证USB通信功能

### 运行脚本
- **`run_tests.sh`** - 运行所有ROS测试的脚本

## 使用方法

### 前提条件
1. 确保ROS2环境已设置
2. 确保ruiyan_hand_node正在运行
3. 确保设备已正确连接

### 运行单个测试
```bash
# 基本控制测试
python3 test_ruiyan_hand_control.py

# 简单测试
python3 simple_test.py

# 交互式测试
python3 interactive_test.py
```

### 运行所有测试
```bash
./run_tests.sh
```

## 注意事项
- 这些脚本需要ROS2环境
- 需要先启动ruiyan_hand_node
- 某些测试可能需要设备权限
- 所有脚本都使用相对路径，具有良好的兼容性