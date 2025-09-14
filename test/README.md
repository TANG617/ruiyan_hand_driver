# 瑞眼灵巧手测试脚本

本目录包含用于测试瑞眼灵巧手控制功能的Python脚本。

## 文件说明

### 测试脚本

1. **`test_ruiyan_hand_control.py`** - 完整功能测试脚本
   - 包含多种测试场景：基本动作、单个手指、手势、平滑运动等
   - 支持交互式选择测试项目
   - 适合全面测试手部功能

2. **`simple_test.py`** - 简单快速测试脚本
   - 执行基本的张开/握拳动作
   - 适合快速验证系统是否正常工作
   - 运行时间短，适合调试

3. **`interactive_test.py`** - 交互式测试脚本
   - 允许用户手动输入角度值
   - 支持预设手势命令
   - 适合精确控制和调试

4. **`run_tests.sh`** - 测试启动脚本
   - 提供统一的测试入口
   - 自动检查环境和依赖
   - 包含启动手部控制节点的功能

## 使用方法

### 前提条件

1. 确保ROS2环境已设置：
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. 构建工作空间：
   ```bash
   cd /root/workspace
   colcon build --packages-select ruiyan_hand_driver
   source install/setup.bash
   ```

3. 启动瑞眼灵巧手控制节点：
   ```bash
   ros2 run ruiyan_hand_driver ruiyan_hand_node
   ```
   或使用launch文件：
   ```bash
   ros2 launch ruiyan_hand_driver ruiyan_hands.launch.py
   ```

### 运行测试

#### 方法1：使用启动脚本（推荐）
```bash
cd /root/workspace/src/ruiyan_hand_driver/test
chmod +x run_tests.sh
./run_tests.sh
```

#### 方法2：直接运行Python脚本
```bash
# 简单测试
python3 simple_test.py

# 完整测试
python3 test_ruiyan_hand_control.py

# 交互式测试
python3 interactive_test.py
```

## 测试内容

### 基本动作测试
- 完全张开 (0度)
- 轻微弯曲 (30度)
- 中等弯曲 (60度)
- 握拳 (90度)

### 单个手指测试
- 逐个测试每个手指的独立控制
- 验证手指间的独立性

### 手势测试
- 和平手势 (V字手势)
- OK手势
- 指向手势

### 平滑运动测试
- 从张开到握拳的平滑过渡
- 验证运动连续性

### 单手控制测试
- 独立控制左手或右手
- 验证双手独立控制能力

## 交互式测试命令

在交互式测试中，可以使用以下命令：

```
l <角度1> <角度2> ... <角度6>  # 设置左手角度
r <角度1> <角度2> ... <角度6>  # 设置右手角度
b <角度1> <角度2> ... <角度6>  # 设置双手角度
open                            # 张开双手
fist                            # 握拳
peace                           # 和平手势
ok                              # OK手势
point                           # 指向手势
help                            # 显示帮助
quit                            # 退出
```

### 示例
```bash
# 左手握拳
l 90 90 90 90 90 90

# 右手张开
r 0 0 0 0 0 0

# 双手做和平手势
peace

# 设置自定义角度
b 45 30 60 90 15 75
```

## 话题说明

测试脚本使用以下ROS2话题：

### 发布话题
- `/ruiyan_hand/left/set_angles` (sensor_msgs/JointState) - 设置左手关节角度
- `/ruiyan_hand/right/set_angles` (sensor_msgs/JointState) - 设置右手关节角度

### 订阅话题
- `/ruiyan_hand/left/joint_states` (sensor_msgs/JointState) - 左手当前关节状态
- `/ruiyan_hand/right/joint_states` (sensor_msgs/JointState) - 右手当前关节状态

## 故障排除

### 常见问题

1. **"话题不存在"错误**
   - 确保ruiyan_hand_node正在运行
   - 检查话题名称是否正确

2. **"无法连接设备"错误**
   - 检查RS485串口连接
   - 验证串口权限和设备路径

3. **"角度值无效"错误**
   - 确保角度值在0-180度范围内
   - 检查输入格式是否正确

4. **"灵巧手没有运动"问题**
   - 检查设备是否连接到正确的端口
   - 验证协议是否匹配
   - 使用诊断脚本检查通信状态

### 设备连接问题诊断

如果灵巧手没有响应，请按以下步骤诊断：

1. **检查设备连接**
   ```bash
   # 查看可用串口设备
   ls -la /dev/ttyACM* /dev/ttyUSB*
   
   # 检查设备权限
   ls -la /dev/ttyACM0 /dev/ttyUSB0
   ```

2. **运行诊断脚本**
   ```bash
   python3 diagnose_ruiyan.py
   ```

3. **测试不同端口**
   ```bash
   python3 test_usb_ports.py
   ```

4. **使用USB端口配置**
   ```bash
   # 停止当前节点
   pkill -f ruiyan_hand_node
   
   # 使用USB端口启动
   ros2 launch ruiyan_hand_driver ruiyan_hands_usb.launch.py
   ```

### 调试建议

1. 使用`ros2 topic list`查看可用话题
2. 使用`ros2 topic echo`监听话题数据
3. 检查节点日志输出
4. 使用简单测试脚本验证基本功能
5. 使用诊断脚本检查通信状态
6. 尝试不同的串口端口和波特率

## 注意事项

1. 角度值范围：0-180度
2. 确保手部设备已正确连接
3. 测试前确保手部处于安全位置
4. 测试完成后手部会自动回到张开位置
5. 如遇异常，按Ctrl+C中断测试

## 扩展

可以根据需要修改测试脚本：
- 添加新的手势定义
- 修改测试序列
- 调整运动速度和时间
- 添加安全检查和限制
