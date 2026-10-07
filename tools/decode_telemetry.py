"""
平衡车串口日志特征字节统计器。
仅统计 0x84、0x85、0x86，不解析或验证 LwPKT 帧。
"""
import json
from input_path import require_input_path

def parse_record_file(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        records = json.load(f)

    raw = bytearray()
    for r in records:
        if 'data' in r:
            parts = [int(x.strip(), 16) for x in r['data'].split(',') if x.strip()]
            raw.extend(parts)

    print(f"=== 成功加载串口数据: 共 {len(raw)} 字节 ===")

    # Count literal bytes, including payload bytes; these are not frame counts.
    count_84 = 0
    count_85 = 0
    count_86 = 0

    for cmd in raw:
        if cmd == 0x86:
            count_86 += 1
        elif cmd == 0x84:
            count_84 += 1
        elif cmd == 0x85:
            count_85 += 1

    print("特征字节出现次数（不是有效帧数量）:")
    print(f"  - 0x86: {count_86}")
    print(f"  - 0x84: {count_84}")
    print(f"  - 0x85: {count_85}")
    print("\n本工具没有校验帧头、长度、CRC 或时间间隔，不能据此判断传输链路健康。")

if __name__ == '__main__':
    path = require_input_path("Count telemetry marker bytes in serial-record JSON; does not validate frames or link health.")
    parse_record_file(path)
