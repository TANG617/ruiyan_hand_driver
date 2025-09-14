# RS485通信测试说明

## 概述
`test_rs485.py` 是一个专门用于测试瑞眼灵巧手RS485通信的脚本。它向 `/dev/ttyACM0` 发送指定的十六进制指令并读取回复。

## 功能特性
- 连接RS485串口设备 (`/dev/ttyACM0`)
- 发送指定的十六进制命令
- 读取并显示响应数据
- 支持多次尝试测试
- 详细的日志输出
- 错误处理和连接状态检查

## 使用方法

### 1. 基本使用
```bash
cd /root/workspace/src/ruiyan_hand_driver/test/controller
python3 test_rs485.py
```

### 2. 作为模块使用
```python
from test_rs485 import RS485Tester

# 创建测试器
tester = RS485Tester(port='/dev/ttyACM0', baudrate=115200)

# 连接
if tester.connect():
    # 发送指定命令
    response = tester.send_and_receive("A5 01 00 18 AA 00 00 E8 03 F4 01 00 AA 00 08 E8 03 F4 01 00 AA FF 0F E8 03 F4 01 00 72")
    if response:
        print(f"响应: {response.hex()}")
    
    # 断开连接
    tester.disconnect()
```

## 测试的命令
脚本会发送以下十六进制命令：
```
A5 01 00 18 AA 00 00 E8 03 F4 01 00 AA 00 08 E8 03 F4 01 00 AA FF 0F E8 03 F4 01 00 72
```

## 配置参数
- **端口**: `/dev/ttyACM0`
- **波特率**: 115200 bps
- **数据位**: 8
- **停止位**: 1
- **校验位**: 无
- **超时时间**: 2.0 秒

## 输出示例
```
2024-01-01 12:00:00 - INFO - RS485串口连接成功: /dev/ttyACM0 @ 115200 bps
2024-01-01 12:00:01 - INFO - 已发送命令: A5 01 00 18 AA 00 00 E8 03 F4 01 00 AA 00 08 E8 03 F4 01 00 AA FF 0F E8 03 F4 01 00 72
2024-01-01 12:00:01 - INFO - 命令长度: 32 字节
2024-01-01 12:00:02 - INFO - 已接收响应: A5 01 01 04 00 00 00 00 04
2024-01-01 12:00:02 - INFO - 响应长度: 9 字节
```

## 故障排除

### 1. 连接失败
- 检查设备是否连接到 `/dev/ttyACM0`
- 确认设备未被其他程序占用
- 检查用户权限：`sudo usermod -a -G dialout $USER`

### 2. 无响应
- 检查设备是否开机
- 确认波特率设置正确
- 检查RS485接线
- 尝试不同的超时时间

### 3. 权限问题
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录或重启
```

## 依赖项
- Python 3.6+
- pyserial库
```bash
pip install pyserial
```

## 注意事项
1. 确保设备已正确连接到RS485接口
2. 运行前检查设备状态
3. 如果测试失败，可以尝试多次运行
4. 脚本会自动处理连接和断开操作
