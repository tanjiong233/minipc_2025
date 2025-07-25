# minipc_2025 - RoboMaster 机器人通信工具

一个用于与RoboMaster机器人进行串口通信的Python工具包。

## 📁 项目文件

```
minipc_2025/
├── comm.py          # 主通信工具（实时数据监控）
├── hex_serial.py    # 串口原始数据记录工具
└── README.md        # 本文档
```

## 🚀 快速开始

### 环境要求

- Python 3.6+
- pyserial库

### 安装依赖

```bash
pip install pyserial
```

## 🔧 使用方法

### 1. 实时数据监控 (comm.py)

**功能**: 实时显示机器人的IMU、里程计、系统状态数据

#### 基本使用

```bash
# 自动检测串口设备
python3 comm.py

# 或者指定串口
# 需要修改comm.py中的设备路径
```

#### 显示效果

```
================================================================================
🤖 RoboMaster 简化版实时数据仪表盘
================================================================================
串口: /dev/ttyACM0 | 波特率: 115200
接收: 1234包/56789字节 | 发送: 10包/123字节
错误: CRC错误0次 | 解析错误0次
--------------------------------------------------------------------------------
🟢 IMU数据 (收到 245 包, 0.1s前)
  加速度: X=   0.123 Y=  -0.456 Z=   9.801 m/s²
  角速度: X=   0.001 Y=   0.002 Z=  -0.001 rad/s
  姿态角: 俯仰=   2.1° 横滚=  -1.5° 偏航= 180.0°
  温度:   45.2°C | 时间戳: 1234567ms

🟢 里程计数据 (收到 123 包, 0.2s前)
  编码器: 左=      1234 右=      5678
  轮速: 左=   1.234 右=   1.256 rad/s
  机器人: 线速度=   0.123m/s 角速度=  -0.012rad/s
  参数: 轮距=0.335m 轮半径=0.076m | 时间戳: 1234567ms

🟢 系统状态 (收到 62 包, 0.8s前)
  机器人模式: 遥控
  变形模式: 车载模式
  设备状态: SBUS=✓ IMU=✓ 左轮=✓ 右轮=✓ 变形=✗
  时间戳: 1234567ms
================================================================================
🕒 更新时间: 14:30:25 | 按 Ctrl+C 退出
```

#### 自定义使用

```python
from comm import RoboMasterCommunicator

# 创建通信器
comm = RoboMasterCommunicator("/dev/ttyACM0", 115200)

# 打开连接
if comm.open():
    comm.start()
    
    # 发送运动命令
    comm.send_motion_command(linear_vel=1.0, angular_vel=0.5)
    
    # 发送自检命令
    comm.send_selfcheck_command(mode=0, debug_int=123)  # PING测试
    
    # 获取数据
    imu_data = comm.get_imu_data()
    print(f"当前偏航角: {imu_data.yaw * 57.3:.1f}°")
    
    odom_data = comm.get_odometry_data()
    print(f"当前速度: {odom_data.linear_velocity:.2f} m/s")
    
    # 清理
    comm.close()
```

### 2. 原始数据记录 (hex_serial.py)

**功能**: 记录串口的原始十六进制数据，用于协议调试

#### 基本使用

```bash
# 使用默认设置
python3 hex_serial.py

# 指定参数
python3 hex_serial.py -p /dev/ttyACM0 -b 115200 -o robot_data.log
```

#### 命令行参数

```bash
python3 hex_serial.py [选项]

选项:
  -p, --port PORT       串口设备 (默认: /dev/ttyUSB0)
  -b, --baudrate RATE   波特率 (默认: 115200)
  -o, --output FILE     输出文件名 (默认: serial_hex.log)
  --no-console          不在控制台显示，只写文件
  --timestamp           添加时间戳
```

#### 输出示例

```bash
# 控制台输出
已连接到 /dev/ttyACM0，波特率 115200
数据将保存到: serial_hex.log
按 Ctrl+C 停止...

[14:30:25.123] 53 54 00 01 2c 05 cd cc 4c 3e 00 00 80 bf 9a 99 | ST...,.L>......
[14:30:25.125] 19 40 00 00 00 00 cd cc 4c be 9a 99 19 be 00 00 | .@......L.......
[14:30:25.127] 80 3f 9a 99 59 40 b7 45 45 44                   | .?..Y@.EED
```

#### 日志文件格式

```
# 串口数据记录 - 2025-01-25 14:30:25
# 端口: /dev/ttyACM0, 波特率: 115200
# 格式: [时间戳] 十六进制数据 | ASCII
--------------------------------------------------------------------------------
53 54 00 01 2c 05 cd cc 4c 3e 00 00 80 bf 9a 99 | ST...,.L>.......
19 40 00 00 00 00 cd cc 4c be 9a 99 19 be 00 00 | .@......L.......
80 3f 9a 99 59 40 b7 45 45 44                   | .?..Y@.EED
```

## 📊 支持的数据类型

### IMU数据 (0x05)

- 三轴加速度 (m/s²)
- 三轴角速度 (rad/s)
- 姿态角：俯仰、横滚、偏航 (弧度)
- 传感器温度 (°C)

### 里程计数据 (0x06)

- 左右轮编码器数值
- 左右轮转速 (rad/s)
- 机器人线速度、角速度
- 轮距、轮半径参数

### 系统状态 (0x07)

- 机器人控制模式
- 变形模式状态
- 各设备连接状态

### 运动指令 (0x08)

- 目标线速度 (m/s)
- 目标角速度 (rad/s)
- 紧急停止标志

### 自检命令 (0x03)

- PING测试
- 状态查询
- 错误计数

## 🔍 常见问题

### 1. 找不到串口设备

```bash
# Linux下查看可用串口
ls /dev/tty*

# 常见设备名
/dev/ttyACM0    # USB CDC设备
/dev/ttyUSB0    # USB转串口设备
```

### 2. 权限不足

```bash
# 添加用户到串口组
sudo usermod -a -G dialout $USER
# 重新登录后生效

# 或临时获取权限
sudo chmod 666 /dev/ttyACM0
```

### 3. 设备被占用

确保没有其他程序（如Arduino IDE、minicom等）正在使用串口。

### 4. 无数据显示

- 检查串口连接
- 确认波特率正确 (115200)
- 确认机器人端有数据发送

## 💡 使用技巧

### 1. 数据录制与分析

```bash
# 录制5分钟数据
timeout 300 python3 hex_serial.py --timestamp -o test_session.log

# 分析数据包统计
grep "53 54" test_session.log | wc -l  # 统计数据包数量
```

### 2. 实时监控特定数据

```python
# 只监控IMU偏航角变化
from comm import RoboMasterCommunicator
import time

comm = RoboMasterCommunicator()
comm.open()
comm.start()

while True:
    imu = comm.get_imu_data()
    print(f"偏航角: {imu.yaw * 57.3:7.1f}°", end='\r')
    time.sleep(0.1)
```

### 3. 命令行快速测试

```python
# 发送测试命令
python3 -c "
from comm import RoboMasterCommunicator
c = RoboMasterCommunicator()
c.open()
c.start()
c.send_selfcheck_command(0, 123)  # 发送PING
c.close()
"
```

## 📝 开发说明

### 协议格式

```
[ST][序列号2字节][长度1字节][命令ID1字节][数据N字节][CRC8校验1字节][ED]
```

### 添加新的数据类型

1. 在`CMD_TO_LEN`中添加新命令ID和长度
2. 在`_parse_packet_data`中添加解析逻辑
3. 创建对应的数据类

------

**使用愉快！** 🎉

如遇问题请检查串口连接和权限设置。