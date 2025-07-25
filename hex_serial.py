#!/usr/bin/env python3
import serial
import sys
import datetime
import argparse


def main():
    # 命令行参数
    parser = argparse.ArgumentParser(description='串口十六进制数据记录器')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='串口设备 (默认: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='波特率 (默认: 115200)')
    parser.add_argument('-o', '--output', default='serial_hex.log', help='输出文件名 (默认: serial_hex.log)')
    parser.add_argument('--no-console', action='store_true', help='不在控制台显示，只写文件')
    parser.add_argument('--timestamp', action='store_true', help='添加时间戳')

    args = parser.parse_args()

    port = args.port
    baudrate = args.baudrate
    output_file = args.output

    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"已连接到 {port}，波特率 {baudrate}")
        print(f"数据将保存到: {output_file}")
        print("按 Ctrl+C 停止...")

        # 打开输出文件
        with open(output_file, 'w', encoding='utf-8') as f:
            # 写入文件头
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"# 串口数据记录 - {timestamp}\n")
            f.write(f"# 端口: {port}, 波特率: {baudrate}\n")
            f.write(f"# 格式: [时间戳] 十六进制数据 | ASCII\n")
            f.write("-" * 80 + "\n")
            f.flush()

            while True:
                data = ser.read(16)  # 一次读16字节
                if data:
                    # 格式化十六进制
                    hex_str = ' '.join([f'{b:02x}' for b in data])
                    # 格式化ASCII（不可打印字符显示为.)
                    ascii_str = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in data])

                    # 准备输出行
                    if args.timestamp:
                        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        line = f"[{timestamp}] {hex_str:<48} | {ascii_str}\n"
                    else:
                        line = f"{hex_str:<48} | {ascii_str}\n"

                    # 写入文件
                    f.write(line)
                    f.flush()  # 立即刷新到文件

                    # 控制台显示（如果需要）
                    if not args.no_console:
                        print(line.rstrip())

    except KeyboardInterrupt:
        print("\n正在停止...")
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("请检查:")
        print("1. 设备是否已连接")
        print("2. 串口路径是否正确")
        print("3. 是否有权限访问串口")
        print("4. 串口是否被其他程序占用")
    except FileNotFoundError:
        print(f"无法创建输出文件: {output_file}")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")


if __name__ == "__main__":
    main()