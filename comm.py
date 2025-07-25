#!/usr/bin/env python3
"""
RoboMaster 简化版单片机通信类
只处理里程计、系统状态、IMU、自检、运动指令五个包
"""

import os
import sys
import serial
import time
import threading
import struct
from copy import deepcopy
from typing import Optional, Dict, Any
from dataclasses import dataclass
from datetime import datetime
import queue


class CRC8Calculator:
    """CRC8校验计算器 - 使用与单片机一致的CRC8-Dallas/Maxim算法"""
    
    # 与单片机crc8.cc中完全一致的CRC8查找表
    CRC8_TABLE = [
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
    ]
    
    CRC8_INIT = 0x00

    @classmethod
    def calculate(cls, data: bytes, initial_crc: int = None) -> int:
        """计算CRC8校验值"""
        if initial_crc is None:
            initial_crc = cls.CRC8_INIT
        crc = initial_crc
        for byte in data:
            index = crc ^ byte
            crc = cls.CRC8_TABLE[index]
        return crc

    @classmethod
    def verify(cls, data: bytes) -> bool:
        """验证CRC8校验值"""
        if len(data) <= 2:
            return False
        expected_crc = cls.calculate(data[:-1], cls.CRC8_INIT)
        return expected_crc == data[-1]


@dataclass
class IMUData:
    """IMU数据结构"""
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    temperature: float = 0.0
    timestamp: int = 0


@dataclass
class OdometryData:
    """里程计数据结构"""
    left_encoder: int = 0
    right_encoder: int = 0
    left_wheel_speed: float = 0.0
    right_wheel_speed: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    wheel_base: float = 0.0
    wheel_radius: float = 0.0
    timestamp: int = 0


@dataclass
class SystemStatusData:
    """系统状态数据结构"""
    robot_mode: int = 0
    transform_mode: int = 0
    sbus_connected: bool = False
    imu_connected: bool = False
    vl_motor_online: bool = False
    vr_motor_online: bool = False
    tf_motor_online: bool = False
    timestamp: int = 0


@dataclass
class SelfcheckCommand:
    """自检命令结构"""
    mode: int = 0
    debug_int: int = 0


class RoboMasterCommunicator:
    """RoboMaster机器人简化通信类"""
    
    # 协议常量
    PACK_START = b'ST'
    PACK_END = b'ED'
    
    # 数据偏移量
    SEQNUM_OFFSET = 2
    DATA_LENGTH_OFFSET = 4
    CMD_ID_OFFSET = 5
    DATA_OFFSET = 6
    
    # 命令ID定义（只保留需要的5个）
    SELFCHECK_CMD_ID = 0x03
    IMU_CMD_ID = 0x05
    ODOMETRY_CMD_ID = 0x06
    SYSTEM_STATUS_CMD_ID = 0x07
    MOTION_CMD_ID = 0x08
    
    # 数据长度映射
    CMD_TO_LEN = {
        SELFCHECK_CMD_ID: 2,
        IMU_CMD_ID: 44,
        ODOMETRY_CMD_ID: 36,
        SYSTEM_STATUS_CMD_ID: 7,
        MOTION_CMD_ID: 16,
    }
    
    HT_LEN = 9  # 头部+尾部+CRC长度
    MAX_PACKET_LENGTH = max(CMD_TO_LEN.values()) + HT_LEN
    MIN_PACKET_LENGTH = min(CMD_TO_LEN.values()) + HT_LEN

    def __init__(self, device_path: Optional[str] = None, baudrate: int = 115200):
        """初始化通信器"""
        self.device_path = device_path or self._auto_detect_device()
        self.baudrate = baudrate
        self.serial_port = None
        self.crc_calculator = CRC8Calculator()
        
        # 接收缓冲区
        self.rx_buffer = []
        self.buffer_size = self.MAX_PACKET_LENGTH * 50
        
        # 数据存储
        self.imu_data = IMUData()
        self.odometry_data = OdometryData()
        self.system_status_data = SystemStatusData()
        
        # 线程控制
        self._running = False
        self._rx_thread = None
        self._data_lock = threading.Lock()
        
        # 发送相关
        self._seq_num = 0
        
        # 统计信息
        self.stats = {
            'rx_packets': 0,
            'tx_packets': 0,
            'rx_bytes': 0,
            'tx_bytes': 0,
            'crc_errors': 0,
            'parse_errors': 0,
            'packet_counts': {cmd_id: 0 for cmd_id in self.CMD_TO_LEN.keys()},
            'last_packet_time': {cmd_id: 0 for cmd_id in self.CMD_TO_LEN.keys()}
        }

    def _auto_detect_device(self) -> Optional[str]:
        """自动检测串口设备"""
        possible_devices = []
        prefixes = ['ttyACM', 'ttyUSB', 'cu.usbmodem', 'cu.usbserial']
        
        try:
            if os.name == 'posix':  # Linux/Mac
                dev_list = os.listdir('/dev')
                for dev_name in dev_list:
                    for prefix in prefixes:
                        if dev_name.startswith(prefix):
                            possible_devices.append(f'/dev/{dev_name}')
            elif os.name == 'nt':  # Windows
                import serial.tools.list_ports
                ports = serial.tools.list_ports.comports()
                for port in ports:
                    possible_devices.append(port.device)
        except Exception as e:
            print(f"设备检测出错: {e}")
            return None
        
        # 测试设备可用性
        for device in possible_devices:
            try:
                test_port = serial.Serial(port=device, baudrate=self.baudrate, timeout=0.1)
                test_port.close()
                print(f"检测到可用设备: {device}")
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
        self.stop()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("串口已关闭")

    def start(self):
        """开始接收线程"""
        if self._running:
            return
        
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_thread_func, daemon=True)
        self._rx_thread.start()
        print("开始监听数据...")

    def stop(self):
        """停止接收线程"""
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join()
        print("停止监听")

    def _rx_thread_func(self):
        """接收线程函数"""
        while self._running:
            try:
                self._read_serial_data()
                self._parse_packets()
                time.sleep(0.001)
            except Exception as e:
                print(f"接收线程出错: {e}")
                break

    def _read_serial_data(self):
        """从串口读取数据"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        try:
            waiting = self.serial_port.in_waiting
            if waiting > 0:
                data = self.serial_port.read(waiting)
                self.stats['rx_bytes'] += len(data)
                
                # 添加到循环缓冲区
                for byte in data:
                    if len(self.rx_buffer) >= self.buffer_size:
                        self.rx_buffer.pop(0)
                    self.rx_buffer.append(byte)
        except Exception as e:
            print(f"读取串口数据出错: {e}")

    def _parse_packets(self):
        """解析数据包"""
        start_idx = 0
        
        while start_idx <= len(self.rx_buffer) - self.MIN_PACKET_LENGTH:
            # 查找包头
            if (start_idx < len(self.rx_buffer) - 1 and
                self.rx_buffer[start_idx] == ord('S') and
                self.rx_buffer[start_idx + 1] == ord('T')):
                
                packet_data = self._try_parse_packet(start_idx)
                if packet_data:
                    self._process_received_packet(packet_data)
                    
                    # 移除已处理的数据
                    cmd_id = packet_data['cmd_id']
                    packet_len = self.CMD_TO_LEN[cmd_id] + self.HT_LEN
                    self.rx_buffer = self.rx_buffer[start_idx + packet_len:]
                    start_idx = 0
                    continue
            
            start_idx += 1

    def _try_parse_packet(self, start_idx: int) -> Optional[Dict[str, Any]]:
        """尝试解析单个数据包"""
        buffer = self.rx_buffer
        
        if start_idx + self.MIN_PACKET_LENGTH > len(buffer):
            return None
        
        # 检查包头
        if (buffer[start_idx] != ord('S') or buffer[start_idx + 1] != ord('T')):
            return None
        
        # 获取命令ID
        if start_idx + self.CMD_ID_OFFSET >= len(buffer):
            return None
            
        cmd_id = buffer[start_idx + self.CMD_ID_OFFSET]
        
        # 检查命令ID是否有效
        if cmd_id not in self.CMD_TO_LEN:
            self.stats['parse_errors'] += 1
            return None
        
        # 检查数据长度
        data_len = self.CMD_TO_LEN[cmd_id]
        total_packet_len = data_len + self.HT_LEN
        
        if start_idx + total_packet_len > len(buffer):
            return None
        
        # 检查包尾
        end_idx = start_idx + total_packet_len
        if (buffer[end_idx - 2] != ord('E') or buffer[end_idx - 1] != ord('D')):
            return None
        
        # 提取完整数据包
        packet_bytes = bytes(buffer[start_idx:end_idx])
        
        # CRC校验
        if not self.crc_calculator.verify(packet_bytes[:-2]):
            self.stats['crc_errors'] += 1
            return None
        
        # 解析序列号
        seq_num = struct.unpack('<H', packet_bytes[self.SEQNUM_OFFSET:self.SEQNUM_OFFSET + 2])[0]
        
        # 解析数据部分
        data = self._parse_packet_data(packet_bytes, cmd_id)
        
        return {
            'cmd_id': cmd_id,
            'seq_num': seq_num,
            'data': data,
            'timestamp': time.time()
        }

    def _parse_packet_data(self, packet_bytes: bytes, cmd_id: int) -> Dict[str, Any]:
        """解析数据包的数据部分"""
        data_start = self.DATA_OFFSET
        data = {}
            
        if cmd_id == self.IMU_CMD_ID:
            idx = data_start
            data = {
                'accel_x': struct.unpack('<f', packet_bytes[idx:idx + 4])[0],
                'accel_y': struct.unpack('<f', packet_bytes[idx + 4:idx + 8])[0],
                'accel_z': struct.unpack('<f', packet_bytes[idx + 8:idx + 12])[0],
                'gyro_x': struct.unpack('<f', packet_bytes[idx + 12:idx + 16])[0],
                'gyro_y': struct.unpack('<f', packet_bytes[idx + 16:idx + 20])[0],
                'gyro_z': struct.unpack('<f', packet_bytes[idx + 20:idx + 24])[0],
                'pitch': struct.unpack('<f', packet_bytes[idx + 24:idx + 28])[0],
                'roll': struct.unpack('<f', packet_bytes[idx + 28:idx + 32])[0],
                'yaw': struct.unpack('<f', packet_bytes[idx + 32:idx + 36])[0],
                'temperature': struct.unpack('<f', packet_bytes[idx + 36:idx + 40])[0],
                'timestamp': struct.unpack('<I', packet_bytes[idx + 40:idx + 44])[0]
            }
            
        elif cmd_id == self.ODOMETRY_CMD_ID:
            idx = data_start
            data = {
                'left_encoder': struct.unpack('<i', packet_bytes[idx:idx + 4])[0],
                'right_encoder': struct.unpack('<i', packet_bytes[idx + 4:idx + 8])[0],
                'left_wheel_speed': struct.unpack('<f', packet_bytes[idx + 8:idx + 12])[0],
                'right_wheel_speed': struct.unpack('<f', packet_bytes[idx + 12:idx + 16])[0],
                'linear_velocity': struct.unpack('<f', packet_bytes[idx + 16:idx + 20])[0],
                'angular_velocity': struct.unpack('<f', packet_bytes[idx + 20:idx + 24])[0],
                'wheel_base': struct.unpack('<f', packet_bytes[idx + 24:idx + 28])[0],
                'wheel_radius': struct.unpack('<f', packet_bytes[idx + 28:idx + 32])[0],
                'timestamp': struct.unpack('<I', packet_bytes[idx + 32:idx + 36])[0]
            }
            
        elif cmd_id == self.SYSTEM_STATUS_CMD_ID:
            robot_mode = packet_bytes[data_start]
            transform_mode = packet_bytes[data_start + 1]
            device_status = packet_bytes[data_start + 2]
            timestamp = struct.unpack('<I', packet_bytes[data_start + 3:data_start + 7])[0]
            
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
        
        elif cmd_id == self.SELFCHECK_CMD_ID:
            data = {
                'mode': packet_bytes[data_start],
                'debug_int': packet_bytes[data_start + 1]
            }
        
        return data

    def _process_received_packet(self, packet_data: Dict[str, Any]):
        """处理接收到的数据包"""
        cmd_id = packet_data['cmd_id']
        data = packet_data['data']
        
        with self._data_lock:
            self.stats['rx_packets'] += 1
            self.stats['packet_counts'][cmd_id] += 1
            self.stats['last_packet_time'][cmd_id] = time.time()
            
            # 更新对应的数据结构
            if cmd_id == self.IMU_CMD_ID:
                self.imu_data = IMUData(**data)
            elif cmd_id == self.ODOMETRY_CMD_ID:
                self.odometry_data = OdometryData(**data)
            elif cmd_id == self.SYSTEM_STATUS_CMD_ID:
                self.system_status_data = SystemStatusData(**data)

    def _pack_packet(self, cmd_id: int, data: bytes) -> bytes:
        """打包数据包"""
        # 计算包长度
        data_len = len(data)
        total_len = data_len + self.HT_LEN
        
        # 创建数据包
        packet = bytearray(total_len)
        
        # 包头
        packet[0:2] = self.PACK_START
        
        # 序列号
        struct.pack_into('<H', packet, self.SEQNUM_OFFSET, self._seq_num)
        self._seq_num = (self._seq_num + 1) % 65536
        
        # 数据长度
        packet[self.DATA_LENGTH_OFFSET] = data_len
        
        # 命令ID
        packet[self.CMD_ID_OFFSET] = cmd_id
        
        # 数据
        packet[self.DATA_OFFSET:self.DATA_OFFSET + data_len] = data
        
        # CRC8校验
        crc_offset = self.DATA_OFFSET + data_len
        crc_value = self.crc_calculator.calculate(packet[:crc_offset])
        packet[crc_offset] = crc_value
        
        # 包尾
        packet[-2:] = self.PACK_END
        
        return bytes(packet)

    def send_motion_command(self, linear_vel: float, angular_vel: float, emergency_stop: bool = False) -> bool:
        """发送运动控制命令"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        
        try:
            # 打包数据
            timestamp = int(time.time() * 1000) % (2**32)  # 转换为毫秒时间戳
            data = struct.pack('<ffBBBBI', 
                             linear_vel, angular_vel, 
                             1 if emergency_stop else 0, 
                             0, 0, 0,  # 保留字节
                             timestamp)
            
            # 创建数据包
            packet = self._pack_packet(self.MOTION_CMD_ID, data)
            
            # 发送数据包
            self.serial_port.write(packet)
            self.stats['tx_packets'] += 1
            self.stats['tx_bytes'] += len(packet)
            
            return True
        except Exception as e:
            print(f"发送运动命令失败: {e}")
            return False

    def send_selfcheck_command(self, mode: int, debug_int: int = 0) -> bool:
        """发送自检命令"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        
        try:
            # 打包数据
            data = struct.pack('<BB', mode, debug_int)
            
            # 创建数据包
            packet = self._pack_packet(self.SELFCHECK_CMD_ID, data)
            
            # 发送数据包
            self.serial_port.write(packet)
            self.stats['tx_packets'] += 1
            self.stats['tx_bytes'] += len(packet)
            
            return True
        except Exception as e:
            print(f"发送自检命令失败: {e}")
            return False

    # 数据获取方法
    def get_imu_data(self) -> IMUData:
        """获取IMU数据"""
        with self._data_lock:
            return deepcopy(self.imu_data)

    def get_odometry_data(self) -> OdometryData:
        """获取里程计数据"""
        with self._data_lock:
            return deepcopy(self.odometry_data)

    def get_system_status(self) -> SystemStatusData:
        """获取系统状态数据"""
        with self._data_lock:
            return deepcopy(self.system_status_data)

    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self._data_lock:
            return deepcopy(self.stats)

    def print_dashboard(self):
        """打印数据仪表盘"""
        os.system('clear' if os.name == 'posix' else 'cls')

        current_time = time.time()
        imu = self.get_imu_data()
        odom = self.get_odometry_data()
        sys_status = self.get_system_status()
        stats = self.get_statistics()

        print("=" * 80)
        print("🤖 RoboMaster 简化版实时数据仪表盘")
        print("=" * 80)
        print(f"串口: {self.device_path} | 波特率: {self.baudrate}")
        print(
            f"接收: {stats['rx_packets']}包/{stats['rx_bytes']}字节 | 发送: {stats['tx_packets']}包/{stats['tx_bytes']}字节")
        print(f"错误: CRC错误{stats['crc_errors']}次 | 解析错误{stats['parse_errors']}次")
        print("-" * 80)

        # IMU数据
        imu_age = current_time - stats['last_packet_time'][self.IMU_CMD_ID] if stats['last_packet_time'][
                                                                                   self.IMU_CMD_ID] > 0 else float(
            'inf')
        imu_status = "🟢" if imu_age < 1.0 else "🟡" if imu_age < 3.0 else "🔴"
        print(f"{imu_status} IMU数据 (收到 {stats['packet_counts'][self.IMU_CMD_ID]} 包, {imu_age:.1f}s前)")
        print(f"  加速度: X={imu.accel_x:8.3f} Y={imu.accel_y:8.3f} Z={imu.accel_z:8.3f} m/s²")
        print(f"  角速度: X={imu.gyro_x:8.3f} Y={imu.gyro_y:8.3f} Z={imu.gyro_z:8.3f} rad/s")
        print(f"  姿态角: 俯仰={imu.pitch * 57.3:7.1f}° 横滚={imu.roll * 57.3:7.1f}° 偏航={imu.yaw * 57.3:7.1f}°")
        print(f"  温度: {imu.temperature:6.1f}°C | 时间戳: {imu.timestamp}ms")
        print()

        # 里程计数据
        odom_age = current_time - stats['last_packet_time'][self.ODOMETRY_CMD_ID] if stats['last_packet_time'][
                                                                                         self.ODOMETRY_CMD_ID] > 0 else float(
            'inf')
        odom_status = "🟢" if odom_age < 1.0 else "🟡" if odom_age < 3.0 else "🔴"
        print(
            f"{odom_status} 里程计数据 (收到 {stats['packet_counts'][self.ODOMETRY_CMD_ID]} 包, {odom_age:.1f}s前)")
        print(f"  编码器: 左={odom.left_encoder:10d} 右={odom.right_encoder:10d}")
        print(f"  轮速: 左={odom.left_wheel_speed:8.3f} 右={odom.right_wheel_speed:8.3f} rad/s")
        print(f"  机器人: 线速度={odom.linear_velocity:8.3f}m/s 角速度={odom.angular_velocity:8.3f}rad/s")
        print(f"  参数: 轮距={odom.wheel_base:.3f}m 轮半径={odom.wheel_radius:.3f}m | 时间戳: {odom.timestamp}ms")
        print()

        # 系统状态数据
        status_age = current_time - stats['last_packet_time'][self.SYSTEM_STATUS_CMD_ID] if \
        stats['last_packet_time'][self.SYSTEM_STATUS_CMD_ID] > 0 else float('inf')
        status_status = "🟢" if status_age < 1.0 else "🟡" if status_age < 3.0 else "🔴"

        # 机器人模式映射
        robot_modes = {0: "待机", 1: "遥控", 2: "自动", 3: "紧急停止"}
        transform_modes = {0: "车载模式", 1: "全地形模式", 2: "步兵模式"}

        robot_mode_str = robot_modes.get(sys_status.robot_mode, f"未知({sys_status.robot_mode})")
        transform_mode_str = transform_modes.get(sys_status.transform_mode, f"未知({sys_status.transform_mode})")

        print(
            f"{status_status} 系统状态 (收到 {stats['packet_counts'][self.SYSTEM_STATUS_CMD_ID]} 包, {status_age:.1f}s前)")
        print(f"  机器人模式: {robot_mode_str}")
        print(f"  变形模式: {transform_mode_str}")

        # 设备状态
        sbus_icon = "✓" if sys_status.sbus_connected else "✗"
        imu_icon = "✓" if sys_status.imu_connected else "✗"
        vl_icon = "✓" if sys_status.vl_motor_online else "✗"
        vr_icon = "✓" if sys_status.vr_motor_online else "✗"
        tf_icon = "✓" if sys_status.tf_motor_online else "✗"

        print(f"  设备状态: SBUS={sbus_icon} IMU={imu_icon} 左轮={vl_icon} 右轮={vr_icon} 变形={tf_icon}")
        print(f"  时间戳: {sys_status.timestamp}ms")
        print()

        # 自检数据（如果有的话）
        selfcheck_age = current_time - stats['last_packet_time'][self.SELFCHECK_CMD_ID] if \
        stats['last_packet_time'][self.SELFCHECK_CMD_ID] > 0 else float('inf')
        if selfcheck_age < 10.0:  # 只显示最近10秒内的自检数据
            selfcheck_status = "🟢" if selfcheck_age < 1.0 else "🟡" if selfcheck_age < 3.0 else "🔴"
            print(
                f"{selfcheck_status} 自检数据 (收到 {stats['packet_counts'][self.SELFCHECK_CMD_ID]} 包, {selfcheck_age:.1f}s前)")
            print()

        print("=" * 80)
        current_time_str = datetime.now().strftime("%H:%M:%S")
        print(f"🕒 更新时间: {current_time_str} | 按 Ctrl+C 退出")

def test_communication():
    """测试通信功能"""
    print("🚀 启动RoboMaster简化版通信测试...")

    # 创建通信器
    comm = RoboMasterCommunicator("/dev/ttyACM0", 115200)

    try:
        # 打开串口
        if not comm.open():
            print("❌ 无法打开串口连接")
            return

        # 开始监听
        comm.start()

        print("✅ 通信已建立，开始测试...")
        time.sleep(1)

        print("📤 发送自检命令...")
        comm.send_selfcheck_command(0, 123)  # PING测试，序列号123

        # 实时显示数据仪表盘
        try:
            while True:
                comm.send_selfcheck_command(0, 123)
                comm.print_dashboard()
                time.sleep(0.5)  # 每0.5秒刷新一次
        except KeyboardInterrupt:
            print("\n👋 用户中断，正在退出...")

    except Exception as e:
        print(f"❌ 测试过程中出错: {e}")

    finally:
        # 清理资源
        comm.close()
        print("🔌 连接已关闭")


if __name__ == '__main__':
    print("RoboMaster 通信测试程序")
    try:
        test_communication()
    except KeyboardInterrupt:
        print("\n程序退出")
    except Exception as e:
        print(f"程序异常: {e}")