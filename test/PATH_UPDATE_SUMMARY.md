# 测试脚本路径更新总结

## 概述
为了适应新的文件夹结构并提高兼容性，我们对所有测试脚本中的路径引用进行了修改，使用相对路径替代绝对路径。

## 文件夹结构
```
test/
├── ros/                    # ROS2相关测试脚本
│   ├── test_ruiyan_hand_control.py
│   ├── test_ros2_node.py
│   ├── simple_test.py
│   ├── interactive_test.py
│   ├── diagnose_ruiyan.py
│   ├── fix_ruiyan_config.py
│   ├── test_with_usb.py
│   ├── run_tests.sh
│   └── README.md
├── controller/             # 控制器和协议相关测试脚本
│   ├── test_rs485.py
│   ├── test_modified_protocol.py
│   ├── test_clear_motor_error.py
│   ├── protocol_debug.py
│   ├── test_usb_ports.py
│   ├── test_full_system.sh
│   ├── RS485_TEST_README.md
│   └── README.md
├── README.md
└── PROBLEM_ANALYSIS.md
```

## 主要修改内容

### 1. Python脚本路径修改

#### 项目路径引用
**修改前：**
```python
sys.path.insert(0, '/root/workspace/src/ruiyan_hand_driver')
```

**修改后：**
```python
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(current_dir, '..', '..')
sys.path.insert(0, project_root)
```

#### 安装路径引用
**修改前：**
```python
sys.path.append('/root/workspace/install/ruiyan_hand_driver/lib/python3.10/site-packages')
```

**修改后：**
```python
install_path = os.path.join(project_root, '..', '..', 'install', 'ruiyan_hand_driver', 'lib', 'python3.10', 'site-packages')
if os.path.exists(install_path):
    sys.path.append(install_path)
```

### 2. Shell脚本路径修改

#### 工作空间路径
**修改前：**
```bash
cd /root/workspace/src/ruiyan_hand_driver
```

**修改后：**
```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"
```

#### 脚本路径引用
**修改前：**
```bash
python3 /root/workspace/src/ruiyan_hand_driver/test/ros/simple_test.py
```

**修改后：**
```bash
python3 "$SCRIPT_DIR/simple_test.py"
```

### 3. 配置文件路径修改

#### Launch文件路径
**修改前：**
```python
launch_file = '/root/workspace/src/ruiyan_hand_driver/launch/ruiyan_hands_working.launch.py'
```

**修改后：**
```python
current_dir = os.path.dirname(os.path.abspath(__file__))
launch_file = os.path.join(current_dir, '..', '..', 'launch', 'ruiyan_hands_working.launch.py')
```

## 修改的文件列表

### Controller文件夹
- `test_modified_protocol.py` - 项目路径引用
- `test_usb_ports.py` - 项目路径和安装路径引用
- `protocol_debug.py` - 项目路径和安装路径引用
- `test_full_system.sh` - 工作目录和脚本路径引用

### ROS文件夹
- `diagnose_ruiyan.py` - 项目路径和安装路径引用
- `fix_ruiyan_config.py` - Launch文件路径引用
- `run_tests.sh` - 工作空间环境和脚本路径引用

### 文档文件
- `controller/RS485_TEST_README.md` - 使用示例路径
- `README.md` - 使用示例路径

## 兼容性改进

### 1. 相对路径使用
- 所有脚本现在使用相对路径，不依赖特定的工作空间位置
- 可以在任何位置运行，只要保持相对文件夹结构

### 2. 动态路径检测
- 自动检测当前脚本位置
- 动态计算项目根目录
- 可选地添加安装路径（如果存在）

### 3. 跨平台兼容性
- 使用`os.path.join()`而不是硬编码的路径分隔符
- 支持不同的操作系统路径格式

## 使用方法

### 从任何位置运行测试
```bash
# 从项目根目录运行
cd /path/to/workspace/src/ruiyan_hand_driver
python3 test/controller/test_rs485.py

# 从test目录运行
cd /path/to/workspace/src/ruiyan_hand_driver/test
python3 controller/test_rs485.py

# 从controller目录运行
cd /path/to/workspace/src/ruiyan_hand_driver/test/controller
python3 test_rs485.py
```

### 运行shell脚本
```bash
# 从任何位置运行
cd /path/to/workspace/src/ruiyan_hand_driver/test/ros
./run_tests.sh

cd /path/to/workspace/src/ruiyan_hand_driver/test/controller
./test_full_system.sh
```

## 验证
所有修改后的脚本都经过测试，确保：
1. 路径设置正确
2. 模块导入正常
3. 脚本可以正常运行
4. 相对路径计算准确

这些修改大大提高了测试脚本的可移植性和兼容性。
