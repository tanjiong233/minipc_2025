#!/usr/bin/env python3
"""
RoboMaster 单片机通信测试程序
支持接收和解析所有数据类型，包括IMU、里程计、系统状态等
"""

import os
import sys
import serial
import time
import threading
import struct
from copy import deepcopy
from typing import Optional, Dict, Any, List
import argparse


class CRC8Calculator:
    """CRC8校验计算器"""
    
    # CRC8查找表 (与单片机代码一致)
    CRC8_TABLE = [
        0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
        0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
        0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
        0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
        0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
        0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
        0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
        0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
        0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
        0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
        0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
        0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
        0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
        0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
        0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
        0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
    ]

    @classmethod
    def calculate(cls, data: bytes, initial_crc: int = 0) -> int:
        """计算CRC8校验值"""
        crc = initial_crc
        for byte in data:
            index = crc ^ byte
            crc = cls.CRC8_TABLE[index]
        return crc

    @classmethod
    def verify(cls, data: bytes) -> bool:
        """验证CRC8校验"""
        if len(data) < 2:
            return False
        expected_crc = cls.calculate(data[:-1], 0)
        return expected_crc == data[-1]


class MiniPCProtocolConfig:
    """协议配置类"""
    
    # 包头包尾
    PACK_START = b'ST'
    PACK_END = b'ED'
    
    # 偏移量
    SEQNUM_OFFSET = 2
    DATA_LENGTH_OFFSET = 4  # SEQNUM_OFFSET + 2
    CMD_ID_OFFSET = 5       # DATA_LENGTH_OFFSET + 1
    DATA_OFFSET = 6         # CMD_ID_OFFSET + 1
    
    # 命令ID
    GIMBAL_CMD_ID = 0x00
    COLOR_CMD_ID = 0x01
    CHASSIS_CMD_ID = 0x02
    SELFCHECK_CMD_ID = 0x03
    ARM_CMD_ID = 0x04
    IMU_CMD_ID = 0x05
    ODOMETRY_CMD_ID = 0x06
    SYSTEM_STATUS_CMD_ID = 0x07
    MOTION_CMD_ID = 0x08
    
    # 数据长度映射 (不包括包头包尾和CRC)
    CMD_TO_LEN = {
        GIMBAL_CMD_ID: 10,        # 4+4+1+1
        COLOR_CMD_ID: 1,          # 1
        CHASSIS_CMD_ID: 12,       # 4+4+4
        SELFCHECK_CMD_ID: 2,      # 1+1
        ARM_CMD_ID: 24,           # 4*6
        IMU_CMD_ID: 44,           # 4*10+4
        ODOMETRY_CMD_ID: 36,      # 4*8+4
        SYSTEM_STATUS_CMD_ID: 8,  # 1+1+1+1+4
        MOTION_CMD_ID: 16,        # 4+4+1+3+4
    }
    
    # 包长度 (包头2 + 序列号2 + 数据长度1 + 命令ID1 + 数据N + CRC1 + 包尾2)
    HT_LEN = 9  # 不包括数据部分的长度
    
    MAX_PACKET_LENGTH = max(CMD_TO_LEN.values()) + HT_LEN  # 53
    MIN_PACKET_LENGTH = min(CMD_TO_LEN.values()) + HT_LEN  # 10


class MiniPCCommunicator:
    """Mini PC 通信器"""
    
    def __init__(self, device_path: Optional[str] = None, baudrate: int = 115200):
        self.config = MiniPCProtocolConfig()
        self.crc_calculator = CRC8Calculator()
        
        # 串口设置
        self.serial_port = None
        self.device_path = device_path or self._guess_serial_device()
        self.baudrate = baudrate
        
        # 缓冲区
        self.circular_buffer: List[int] = []
        self.buffer_size = self.config.MAX_PACKET_LENGTH * 100
        
        # 状态数据
        self.robot_state = {}
        self.state_lock = threading.Lock()
        
        # 统计信息
        self.parsed_packet_count = 0
        self.total_bytes_received = 0
        self.last_seq_num = 0
        
        # 线程控制
        self._running = False
        self._listen_thread = None

    def _guess_serial_device(self) -> Optional[str]:
        """自动检测串口设备"""
        possible_devices = []
        
        # 常见的串口设备路径
        prefixes = ['ttyACM', 'ttyUSB', 'cu.usbmodem', 'cu.usbserial']
        
        try:
            dev_list = os.listdir('/dev')
            for dev_name in dev_list:
                for prefix in prefixes:
                    if dev_name.startswith(prefix):
                        possible_devices.append(f'/dev/{dev_name}')
        except Exception as e:
            print(f"扫描设备时出错: {e}")
            return None
        
        # 尝试打开设备
        for device in possible_devices:
            try:
                test_port = serial.Serial(
                    port=device,
                    baudrate=self.baudrate,
                    timeout=0.1
                )
                test_port.close()
                print(f"找到可用设备: {device}")
                return device
            except Exception:
                continue
        
        return None

    def open(self) -> bool:
        """打开串口连接"""
        if not self.device_path:
            print("错误: 未找到可用的串口设备")
            return False
        
        try:
            self.serial_port = serial.Serial(
                port=self.device_path,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001
            )
            print(f"成功打开串口: {self.device_path}, 波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"打开串口失败: {e}")
            return False

    def close(self):
        """关闭串口连接"""
        self.stop_listening()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已关闭")

    def start_listening(self):
        """开始监听线程"""
        if self._listen_thread and self._listen_thread.is_alive():
            return
        
        self._running = True
        self._listen_thread = threading.Thread(target=self._listen_loop)
        self._listen_thread.daemon = True
        self._listen_thread.start()
        print("开始监听数据...")

    def stop_listening(self):
        """停止监听线程"""
        self._running = False
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join()
        print("停止监听")

    def _listen_loop(self):
        """监听循环"""
        while self._running:
            try:
                self._read_serial_data()
                self._parse_packets()
                time.sleep(0.001)  # 1ms间隔
            except Exception as e:
                print(f"监听循环出错: {e}")
                break

    def _read_serial_data(self) -> bool:
        """从串口读取数据到循环缓冲区"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        
        try:
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                data = self.serial_port.read(waiting)
                self.total_bytes_received += len(data)
                
                # 添加到循环缓冲区
                for byte in data:
                    if len(self.circular_buffer) >= self.buffer_size:
                        self.circular_buffer.pop(0)
                    self.circular_buffer.append(byte)
                return True
        except Exception as e:
            print(f"读取串口数据出错: {e}")
        
        return False

    def _parse_packets(self):
        """解析数据包"""
        start_idx = 0
        
        while start_idx <= len(self.circular_buffer) - self.config.MIN_PACKET_LENGTH:
            # 查找包头
            if (start_idx < len(self.circular_buffer) - 1 and
                self.circular_buffer[start_idx] == ord('S') and
                self.circular_buffer[start_idx + 1] == ord('T')):
                
                # 尝试解析数据包
                packet_data = self._try_parse_packet(start_idx)
                if packet_data:
                    self.parsed_packet_count += 1
                    self._update_robot_state(packet_data)
                    
                    # 移除已解析的数据
                    packet_len = self.config.CMD_TO_LEN[packet_data['cmd_id']] + self.config.HT_LEN
                    self.circular_buffer = self.circular_buffer[start_idx + packet_len:]
                    start_idx = 0
                    continue
            
            start_idx += 1

    def _try_parse_packet(self, start_idx: int) -> Optional[Dict[str, Any]]:
        """尝试解析单个数据包"""
        buffer = self.circular_buffer
        
        # 检查缓冲区长度
        if start_idx + self.config.MIN_PACKET_LENGTH > len(buffer):
            return None
        
        # 检查包头
        if (buffer[start_idx] != ord('S') or 
            buffer[start_idx + 1] != ord('T')):
            return None
        
        # 获取命令ID和数据长度
        if start_idx + self.config.CMD_ID_OFFSET >= len(buffer):
            return None
            
        cmd_id = buffer[start_idx + self.config.CMD_ID_OFFSET]
        
        # 检查命令ID有效性
        if cmd_id not in self.config.CMD_TO_LEN:
            return None
        
        data_len = self.config.CMD_TO_LEN[cmd_id]
        total_packet_len = data_len + self.config.HT_LEN
        
        # 检查完整包长度
        if start_idx + total_packet_len > len(buffer):
            return None
        
        # 检查包尾
        end_idx = start_idx + total_packet_len
        if (buffer[end_idx - 2] != ord('E') or 
            buffer[end_idx - 1] != ord('D')):
            return None
        
        # 提取完整包数据
        packet_bytes = bytes(buffer[start_idx:end_idx])
        
        # CRC校验 (不包括包尾的2个字节)
        if not self.crc_calculator.verify(packet_bytes[:-2]):
            print("CRC校验失败")
            return None
        
        # 解析数据
        seq_num = struct.unpack('<H', packet_bytes[self.config.SEQNUM_OFFSET:self.config.SEQNUM_OFFSET + 2])[0]
        data = self._parse_packet_data(packet_bytes, cmd_id)
        
        return {
            'cmd_id': cmd_id,
            'seq_num': seq_num,
            'data': data,
            'timestamp': time.time()
        }

    def _parse_packet_data(self, packet_bytes: bytes, cmd_id: int) -> Dict[str, Any]:
        """解析数据包的数据部分"""
        data_start = self.config.DATA_OFFSET
        data = {}
        
        if cmd_id == self.config.GIMBAL_CMD_ID:
            # 云台数据: rel_yaw(4) + rel_pitch(4) + mode(1) + debug_int(1)
            rel_yaw = struct.unpack('<f', packet_bytes[data_start:data_start + 4])[0]
            rel_pitch = struct.unpack('<f', packet_bytes[data_start + 4:data_start + 8])[0]
            mode = packet_bytes[data_start + 8]
            debug_int = packet_bytes[data_start + 9]
            
            data = {
                'rel_yaw': rel_yaw,
                'rel_pitch': rel_pitch,
                'mode': mode,
                'debug_int': debug_int
            }
            
        elif cmd_id == self.config.COLOR_CMD_ID:
            # 颜色数据: my_color(1)
            my_color = packet_bytes[data_start]
            data = {'my_color': 'red' if my_color == 0 else 'blue'}
            
        elif cmd_id == self.config.CHASSIS_CMD_ID:
            # 底盘数据: vx(4) + vy(4) + vw(4)
            vx = struct.unpack('<f', packet_bytes[data_start:data_start + 4])[0]
            vy = struct.unpack('<f', packet_bytes[data_start + 4:data_start + 8])[0]
            vw = struct.unpack('<f', packet_bytes[data_start + 8:data_start + 12])[0]
            
            data = {'vx': vx, 'vy': vy, 'vw': vw}
            
        elif cmd_id == self.config.SELFCHECK_CMD_ID:
            # 自检数据: mode(1) + debug_int(1)
            mode = packet_bytes[data_start]
            debug_int = packet_bytes[data_start + 1]
            
            data = {'mode': mode, 'debug_int': debug_int}
            
        elif cmd_id == self.config.ARM_CMD_ID:
            # 机械臂数据: floats[6]
            floats = []
            for i in range(6):
                float_val = struct.unpack('<f', packet_bytes[data_start + i * 4:data_start + (i + 1) * 4])[0]
                floats.append(float_val)
            
            data = {'floats': floats}
            
        elif cmd_id == self.config.IMU_CMD_ID:
            # IMU数据: accel_xyz(12) + gyro_xyz(12) + euler_xyz(12) + temp(4) + timestamp(4)
            idx = data_start
            accel_x = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            accel_y = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            accel_z = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            gyro_x = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            gyro_y = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            gyro_z = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            pitch = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            roll = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            yaw = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            temperature = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            timestamp = struct.unpack('<I', packet_bytes[idx:idx + 4])[0]
            
            data = {
                'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z,
                'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z,
                'pitch': pitch, 'roll': roll, 'yaw': yaw,
                'temperature': temperature, 'timestamp': timestamp
            }
            
        elif cmd_id == self.config.ODOMETRY_CMD_ID:
            # 里程计数据
            idx = data_start
            left_encoder = struct.unpack('<i', packet_bytes[idx:idx + 4])[0]; idx += 4
            right_encoder = struct.unpack('<i', packet_bytes[idx:idx + 4])[0]; idx += 4
            left_wheel_speed = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            right_wheel_speed = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            linear_velocity = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            angular_velocity = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            wheel_base = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            wheel_radius = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            timestamp = struct.unpack('<I', packet_bytes[idx:idx + 4])[0]
            
            data = {
                'left_encoder': left_encoder, 'right_encoder': right_encoder,
                'left_wheel_speed': left_wheel_speed, 'right_wheel_speed': right_wheel_speed,
                'linear_velocity': linear_velocity, 'angular_velocity': angular_velocity,
                'wheel_base': wheel_base, 'wheel_radius': wheel_radius,
                'timestamp': timestamp
            }
            
        elif cmd_id == self.config.SYSTEM_STATUS_CMD_ID:
            # 系统状态数据
            robot_mode = packet_bytes[data_start]
            transform_mode = packet_bytes[data_start + 1]
            device_status = packet_bytes[data_start + 2]
            timestamp = struct.unpack('<I', packet_bytes[data_start + 4:data_start + 8])[0]
            
            data = {
                'robot_mode': robot_mode,
                'transform_mode': transform_mode,
                'sbus_connected': bool(device_status & 0x01),
                'imu_connected': bool(device_status & 0x02),
                'vl_motor_online': bool(device_status & 0x04),
                'vr_motor_online': bool(device_status & 0x08),
                'tf_motor_online': bool(device_status & 0x10),
                'timestamp': timestamp
            }
            
        elif cmd_id == self.config.MOTION_CMD_ID:
            # 运动指令数据
            idx = data_start
            target_linear_vel = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            target_angular_vel = struct.unpack('<f', packet_bytes[idx:idx + 4])[0]; idx += 4
            emergency_stop = packet_bytes[idx]; idx += 4  # +3 for reserved bytes
            timestamp = struct.unpack('<I', packet_bytes[idx:idx + 4])[0]
            
            data = {
                'target_linear_vel': target_linear_vel,
                'target_angular_vel': target_angular_vel,
                'emergency_stop': bool(emergency_stop),
                'timestamp': timestamp
            }
        
        return data

    def _update_robot_state(self, packet_data: Dict[str, Any]):
        """更新机器人状态"""
        with self.state_lock:
            cmd_id = packet_data['cmd_id']
            self.robot_state[cmd_id] = packet_data
            self.last_seq_num = packet_data['seq_num']

    def get_robot_state(self) -> Dict[str, Any]:
        """获取当前机器人状态"""
        with self.state_lock:
            return deepcopy(self.robot_state)

    def get_statistics(self) -> Dict[str, int]:
        """获取统计信息"""
        return {
            'parsed_packets': self.parsed_packet_count,
            'total_bytes': self.total_bytes_received,
            'last_seq_num': self.last_seq_num,
            'buffer_size': len(self.circular_buffer)
        }

    def print_packet_data(self, packet_data: Dict[str, Any]):
        """打印数据包内容"""
        cmd_id = packet_data['cmd_id']
        seq_num = packet_data['seq_num']
        data = packet_data['data']
        
        print(f"\n=== 数据包 #{self.parsed_packet_count} (序列号: {seq_num}) ===")
        
        if cmd_id == self.config.GIMBAL_CMD_ID:
            print("【云台数据】")
            print(f"  相对偏航角: {data['rel_yaw']:.3f} rad ({data['rel_yaw'] * 57.296:.1f}°)")
            print(f"  相对俯仰角: {data['rel_pitch']:.3f} rad ({data['rel_pitch'] * 57.296:.1f}°)")
            print(f"  模式: {data['mode']} ({'搜索目标' if data['mode'] == 0 else '移动云台'})")
            print(f"  调试整数: {data['debug_int']}")
            
        elif cmd_id == self.config.COLOR_CMD_ID:
            print("【颜色数据】")
            print(f"  我方颜色: {data['my_color']}")
            
        elif cmd_id == self.config.CHASSIS_CMD_ID:
            print("【底盘数据】")
            print(f"  X方向速度: {data['vx']:.3f} m/s")
            print(f"  Y方向速度: {data['vy']:.3f} m/s")
            print(f"  角速度: {data['vw']:.3f} rad/s")
            
        elif cmd_id == self.config.SELFCHECK_CMD_ID:
            print("【自检数据】")
            mode_str = {0: 'FLUSH', 1: 'ECHO', 2: 'ID'}.get(data['mode'], '未知')
            print(f"  模式: {data['mode']} ({mode_str})")
            print(f"  调试整数: {data['debug_int']}")
            
        elif cmd_id == self.config.ARM_CMD_ID:
            print("【机械臂数据】")
            for i, val in enumerate(data['floats']):
                print(f"  关节{i + 1}: {val:.3f}")
                
        elif cmd_id == self.config.IMU_CMD_ID:
            print("【IMU数据】")
            print(f"  加速度: X={data['accel_x']:.3f}, Y={data['accel_y']:.3f}, Z={data['accel_z']:.3f} m/s²")
            print(f"  角速度: X={data['gyro_x']:.3f}, Y={data['gyro_y']:.3f}, Z={data['gyro_z']:.3f} rad/s")
            print(f"  姿态角: 俯仰={data['pitch'] * 57.296:.1f}°, 横滚={data['roll'] * 57.296:.1f}°, 偏航={data['yaw'] * 57.296:.1f}°")
            print(f"  温度: {data['temperature']:.1f}°C")
            print(f"  时间戳: {data['timestamp']} ms")
            
        elif cmd_id == self.config.ODOMETRY_CMD_ID:
            print("【里程计数据】")
            print(f"  编码器计数: 左={data['left_encoder']}, 右={data['right_encoder']}")
            print(f"  轮速: 左={data['left_wheel_speed']:.3f}, 右={data['right_wheel_speed']:.3f} rad/s")
            print(f"  机器人速度: 线速度={data['linear_velocity']:.3f} m/s, 角速度={data['angular_velocity']:.3f} rad/s")
            print(f"  参数: 轮距={data['wheel_base']:.3f}m, 轮半径={data['wheel_radius']:.3f}m")
            print(f"  时间戳: {data['timestamp']} ms")
            
        elif cmd_id == self.config.SYSTEM_STATUS_CMD_ID:
            print("【系统状态】")
            robot_modes = {0: '手动', 1: '自动导航', 2: '建图模式'}
            transform_modes = {0: '车载模式', 1: '飞行模式', 2: '变形中'}
            print(f"  机器人模式: {robot_modes.get(data['robot_mode'], '未知')}")
            print(f"  变形模式: {transform_modes.get(data['transform_mode'], '未知')}")
            print(f"  设备状态:")
            print(f"    SBUS: {'连接' if data['sbus_connected'] else '断开'}")
            print(f"    IMU: {'连接' if data['imu_connected'] else '断开'}")
            print(f"    左轮电机: {'在线' if data['vl_motor_online'] else '离线'}")
            print(f"    右轮电机: {'在线' if data['vr_motor_online'] else '离线'}")
            print(f"    变形电机: {'在线' if data['tf_motor_online'] else '离线'}")
            print(f"  时间戳: {data['timestamp']} ms")
            
        elif cmd_id == self.config.MOTION_CMD_ID:
            print("【运动指令】")
            print(f"  目标线速度: {data['target_linear_vel']:.3f} m/s")
            print(f"  目标角速度: {data['target_angular_vel']:.3f} rad/s")
            print(f"  紧急停止: {'是' if data['emergency_stop'] else '否'}")
            print(f"  时间戳: {data['timestamp']} ms")
        
        print("=" * 50)

    def send_motion_command(self, linear_vel: float, angular_vel: float, emergency_stop: bool = False):
        """发送运动指令 (示例功能)"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        
        # 构造运动指令包
        packet = bytearray()
        packet.extend(self.config.PACK_START)  # 'ST'
        packet.extend(struct.pack('<H', 0))    # 序列号 (简化为0)
        packet.append(self.config.CMD_TO_LEN[self.config.MOTION_CMD_ID])  # 数据长度
        packet.append(self.config.MOTION_CMD_ID)  # 命令ID
        
        # 数据部分
        packet.extend(struct.pack('<f', linear_vel))    # 目标线速度
        packet.extend(struct.pack('<f', angular_vel))   # 目标角速度
        packet.append(1 if emergency_stop else 0)      # 紧急停止
        packet.extend(b'\x00\x00\x00')                 # 保留字节
        packet.extend(struct.pack('<I', int(time.time() * 1000)))  # 时间戳
        
        # CRC校验
        crc = self.crc_calculator.calculate(packet)
        packet.append(crc)
        
        # 包尾
        packet.extend(self.config.PACK_END)  # 'ED'
        
        try:
            self.serial_port.write(packet)
            print(f"发送运动指令: 线速度={linear_vel}, 角速度={angular_vel}, 急停={emergency_stop}")
            return True
        except Exception as e:
            print(f"发送指令失败: {e}")
            return False


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='RoboMaster 单片机通信测试程序')
    parser.add_argument('-d', '--device', type=str, help='串口设备路径 (如: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='波特率 (默认: 115200)')
    parser.add_argument('-q', '--quiet', action='store_true', help='静默模式，减少输出')
    
    args = parser.parse_args()
    
    print("RoboMaster 上位机通信测试程序 (Python版)")
    print("=" * 50)
    
    # 创建通信器
    comm = MiniPCCommunicator(args.device, args.baudrate)
    
    # 打开连接
    if not comm.open():
        print("无法打开串口连接，请检查设备路径和权限")
        return
    
    try:
        # 开始监听
        comm.start_listening()
        
        last_stats_time = time.time()
        last_packet_count = 0
        
        print("开始接收数据... (按 Ctrl+C 退出)")
        print("如果需要发送运动指令，请修改代码中的示例")
        
        while True:
            time.sleep(0.1)
            
            # 检查是否有新数据包
            current_count = comm.parsed_packet_count
            if current_count > last_packet_count:
                # 获取最新的数据包并显示
                robot_state = comm.get_robot_state()
                if robot_state:
                    # 显示最新收到的数据包
                    latest_cmd_id = max(robot_state.keys(), key=lambda k: robot_state[k]['timestamp'])
                    latest_packet = robot_state[latest_cmd_id]
                    
                    if not args.quiet:
                        comm.print_packet_data(latest_packet)
                
                last_packet_count = current_count
            
            # 每5秒显示统计信息
            now = time.time()
            if now - last_stats_time >= 5:
                stats = comm.get_statistics()
                print(f"\n[统计] 数据包: {stats['parsed_packets']}, "
                      f"字节: {stats['total_bytes']}, "
                      f"缓冲区: {stats['buffer_size']}, "
                      f"最后序列号: {stats['last_seq_num']}")
                last_stats_time = now
            
            # 示例: 发送运动指令 (取消注释以启用)
            # if current_count > 0 and current_count % 100 == 0:
            #     comm.send_motion_command(0.5, 0.1)  # 0.5m/s线速度, 0.1rad/s角速度
    
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    
    except Exception as e:
        print(f"程序运行出错: {e}")
    
    finally:
        comm.close()
        print("程序结束")


if __name__ == '__main__':
    main()
