#!/usr/bin/env python3
"""
简易平衡车姿态串口监视器
用法:
    python tools/serial_monitor.py COM3
    python tools/serial_monitor.py COM6 115200
"""
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 缺少 pyserial 库。请先安装: pip install pyserial")
    sys.exit(1)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    if not port:
        ports = list(serial.tools.list_ports.comports())
        print("未指定串口，当前可用串口列表:")
        for p in ports:
            print(f"  - {p.device}: {p.description}")
        if not ports:
            print("未检测到任何串口设备！请先连接 USB 转串口或蓝牙。")
            return
        port = ports[0].device
        print(f"\n默认使用第一个串口: {port}")

    print(f"正在打开 {port} (波特率 {baud})... (按 Ctrl+C 退出)")
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        print(f"打开串口失败: {e}")
        return

    try:
        while True:
            line = ser.readline()
            if line:
                try:
                    text = line.decode('utf-8', errors='replace').strip()
                    if text:
                        print(text)
                except Exception:
                    pass
    except KeyboardInterrupt:
        print("\n已退出监视。")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
