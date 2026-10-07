#!/usr/bin/env python3
"""Binary LwPKT parameter client. SET changes RAM; only SAVE writes EEPROM.

The frame uses this project's single-byte command/length, reflected CRC8 and
AA/55 delimiters. No text, motor command, ARM or DTR/RTS operation is emitted.
Live access imports pyserial lazily; the codec and tests need only Python.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import secrets
import struct
import sys
import time
from typing import Callable

from encoder_monitor import Frame, StreamParser, crc8

COMMAND = 0x04
RESPONSE_COMMAND = 0x83
VERSION = 1
REQUEST = struct.Struct("<BBBBi")
RESPONSE = struct.Struct("<BBBBBBHiIII")
OPERATIONS = {"get": 1, "set": 2, "save": 3, "load": 4, "defaults": 5, "status": 6}
PARAMETERS = {
    "balance.kp": 1, "balance.kd": 2,
    "velocity.kp": 3, "velocity.ki": 4, "velocity.integral_limit": 5,
    "velocity.lpf_alpha": 6, "velocity.max_output": 7,
    "turn.kp": 8, "turn.kd": 9, "turn.max_output": 10,
    "mechanical_zero": 11, "deadband_left": 12, "deadband_right": 13,
    "max_pwm": 14,
}
ALIASES = {
    "mechanical_zero_pitch": "mechanical_zero", "zero": "mechanical_zero",
    "left.deadband": "deadband_left", "right.deadband": "deadband_right",
}
STATUS_NAMES = {
    0: "OK", 1: "BUSY", 2: "INVALID", 3: "UNSUPPORTED", 4: "UNSAFE",
    5: "STORAGE_ERROR", 6: "NO_SAVED", 7: "NOT_READY",
}
STATE_NAMES = {0: "DISARMED", 1: "CALIBRATING", 2: "ARMED", 3: "FALLEN"}
FAULT_NAMES = {
    1 << 0: "FALL_PITCH", 1 << 1: "FALL_ROLL", 1 << 2: "CMD_TIMEOUT",
    1 << 3: "SENSOR_INVALID", 1 << 4: "CALIBRATION_FAIL", 1 << 5: "EMERGENCY_STOP",
}
STORAGE_NAMES = {
    1 << 0: "CONFIGURED", 1 << 1: "READY", 1 << 2: "VALID",
    1 << 3: "DIRTY", 1 << 4: "BUSY", 1 << 5: "ERROR",
}
STORAGE_ERROR_NAMES = {
    0: "NONE", 1: "DISABLED", 2: "NOT_READY", 3: "ARGUMENT", 4: "INIT",
    5: "READ", 6: "WRITE", 7: "CONTEXT",
    0x103: "RECORD_INVALID", 0x104: "RECORD_IO", 0x105: "RECORD_VERIFY",
    0x106: "PROFILE_INVALID", 0x107: "RECORD_ARGUMENT",
}


def parameter_key(name: str) -> int:
    normalized = name.strip().lower()
    normalized = ALIASES.get(normalized, normalized)
    if normalized in PARAMETERS:
        return PARAMETERS[normalized]
    if normalized.isdecimal() and 1 <= int(normalized) <= len(PARAMETERS):
        return int(normalized)
    raise argparse.ArgumentTypeError("未知参数；可用名称：" + ", ".join(PARAMETERS))


def q16_encode(value: float) -> int:
    if not math.isfinite(value) or not -32768.0 <= value <= 32767.9999847412109375:
        raise ValueError("参数必须有限且可表示为 signed Q16.16 [-32768, 32767.9999847412]")
    scaled = value * 65536.0
    # Same nearest/half-away-from-zero rule as parameter_profile.c lroundf.
    encoded = math.floor(scaled + 0.5) if scaled >= 0 else math.ceil(scaled - 0.5)
    if not -(1 << 31) <= encoded < (1 << 31):
        raise ValueError("参数超出 signed Q16.16 范围")
    return encoded


def parameter_value(text: str) -> float:
    try:
        value = float(text)
        q16_encode(value)
        return value
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def positive_seconds(text: str) -> float:
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("超时应为正数秒") from exc
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("超时应为有限正数秒")
    return value


def build_frame(command: int, payload: bytes) -> bytes:
    if not 0 <= command <= 255 or len(payload) > 64:
        raise ValueError("当前 LwPKT 配置使用 8-bit CMD 和最多 64-byte payload")
    covered = bytes((command, len(payload))) + payload
    return b"\xaa" + covered + bytes((crc8(covered), 0x55))


def build_request(operation: str, key: int = 0, value: float = 0.0,
                  sequence: int | None = None) -> tuple[int, bytes]:
    if operation not in OPERATIONS:
        raise ValueError("未知参数操作")
    if operation in ("get", "set"):
        if not 1 <= key <= len(PARAMETERS):
            raise ValueError("GET/SET 参数键必须在 1..14")
    elif key != 0:
        raise ValueError("SAVE/LOAD/DEFAULTS/STATUS 不携带参数键")
    if sequence is None:
        sequence = secrets.randbelow(256)
    if not 0 <= sequence <= 255:
        raise ValueError("sequence 必须在 0..255")
    encoded = q16_encode(value) if operation == "set" else 0
    payload = REQUEST.pack(VERSION, sequence, OPERATIONS[operation], key, encoded)
    return sequence, build_frame(COMMAND, payload)


@dataclass(frozen=True)
class Reply:
    version: int
    sequence: int
    request_command: int
    status: int
    key: int
    controller_state: int
    storage_flags: int
    value_q16_16: int
    fault_flags: int
    config_revision: int
    timestamp_ms: int

    @classmethod
    def decode(cls, payload: bytes) -> Reply:
        if len(payload) != RESPONSE.size:
            raise ValueError("回执载荷必须为 24 字节")
        result = cls(*RESPONSE.unpack(payload))
        if result.version != VERSION:
            raise ValueError("不支持的回执协议版本")
        return result


@dataclass
class ExchangeStats:
    ignored_frames: int = 0
    malformed_replies: int = 0


def wait_reply(port, sequence: int, key: int, timeout: float,
               clock: Callable[[], float] = time.monotonic) -> tuple[Reply, StreamParser, ExchangeStats]:
    parser = StreamParser()
    stats = ExchangeStats()
    deadline = clock() + timeout
    last_data = clock()

    def matching(frames: list[Frame]) -> Reply | None:
        for frame in frames:
            if frame.command != RESPONSE_COMMAND:
                stats.ignored_frames += 1
                continue
            try:
                reply = Reply.decode(frame.payload)
            except ValueError:
                stats.malformed_replies += 1
                continue
            if reply.sequence == sequence and reply.request_command == COMMAND and reply.key == key:
                return reply
            stats.ignored_frames += 1
        return None

    while clock() < deadline:
        data = port.read(256)
        if data:
            last_data = clock()
            reply = matching(parser.feed(data))
        elif clock() - last_data >= 0.2:
            # Recover an ACK behind a truncated candidate, once the line is idle.
            reply = matching(parser.flush_incomplete())
        else:
            reply = None
        if reply is not None:
            return reply, parser, stats
    reply = matching(parser.flush_incomplete())
    if reply is not None:
        return reply, parser, stats
    raise TimeoutError(
        f"未收到匹配回执 seq={sequence}；RX={parser.stats.bytes_received} bytes，"
        f"CRC错误={parser.stats.crc_errors}，忽略帧={stats.ignored_frames}。"
        "操作是否完成尚未确认；先查询 status/get，不自动重发 SAVE。"
    )


def exchange(port, operation: str, key: int = 0, value: float = 0.0,
             timeout: float = 3.0, sequence: int | None = None,
             clock: Callable[[], float] = time.monotonic) -> tuple[Reply, StreamParser, ExchangeStats]:
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout 必须是有限正数")
    sequence, request = build_request(operation, key, value, sequence)
    written = port.write(request)
    if written != len(request):
        raise OSError(f"串口只发送 {written}/{len(request)} 字节；未自动重发")
    return wait_reply(port, sequence, key, timeout, clock)


def describe_bits(value: int, names: dict[int, str]) -> str:
    values = [name for bit, name in names.items() if value & bit]
    known = sum(names)
    if value & ~known:
        values.append(f"UNKNOWN:0x{value & ~known:X}")
    return "|".join(values) if values else "NONE"


def format_reply(operation: str, reply: Reply, parser: StreamParser,
                 stats: ExchangeStats) -> str:
    status = STATUS_NAMES.get(reply.status, f"UNKNOWN({reply.status})")
    state = STATE_NAMES.get(reply.controller_state, f"UNKNOWN({reply.controller_state})")
    key_name = next((name for name, key in PARAMETERS.items() if key == reply.key), str(reply.key))
    lines = [
        f"result={status} seq={reply.sequence} operation={operation} key={key_name}",
        f"state={state} faults=0x{reply.fault_flags:08X} ({describe_bits(reply.fault_flags, FAULT_NAMES)})",
        f"storage_flags=0x{reply.storage_flags:04X} ({describe_bits(reply.storage_flags, STORAGE_NAMES)}) "
        f"revision={reply.config_revision} "
        f"timestamp_ms={reply.timestamp_ms}",
        f"value={reply.value_q16_16 / 65536.0:.6f} raw_q16={reply.value_q16_16}",
        f"rx_frames={parser.stats.valid_frames} crc_errors={parser.stats.crc_errors} "
        f"ignored_frames={stats.ignored_frames} malformed_replies={stats.malformed_replies}",
    ]
    if operation == "status":
        value = reply.value_q16_16 / 65536.0
        error_name = STORAGE_ERROR_NAMES.get(value, "UNKNOWN")
        if value >= 0x200 and value <= 0x207 and value == int(value):
            error_name = "BOOT_APPLY_" + STATUS_NAMES.get(int(value) - 0x200, "UNKNOWN")
        lines.append(f"storage_error={value:g} ({error_name})；revision 是 RAM 配置版本")
    if reply.status == 0 and operation == "set":
        lines.append("RAM 参数已应用；尚未写入 EEPROM。需要持久化时显式执行 save。")
    if reply.status == 0 and operation == "defaults":
        lines.append("已恢复 RAM 编译默认值；该操作不会擦除 EEPROM。")
    if reply.status == 0 and operation == "save":
        lines.append("保存已完成或存储内容未变化；设备不会因此使能电机。")
    return "\n".join(lines)


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="串口，如 COM6")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200 8N1")
    parser.add_argument("--timeout", type=positive_seconds, default=3.0, help="回执等待秒数，默认 3")
    sub = parser.add_subparsers(dest="operation", required=True)
    for operation in OPERATIONS:
        command = sub.add_parser(operation)
        if operation in ("get", "set"):
            command.add_argument("key", type=parameter_key, help="参数名或数字键 1..14")
        if operation == "set":
            command.add_argument("value", type=parameter_value)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = argument_parser()
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("波特率必须为正数")
    try:
        import serial
    except ImportError:
        print("需要 pyserial：python -m pip install pyserial", file=sys.stderr)
        return 2
    try:
        with serial.Serial(port=args.port, baudrate=args.baud, bytesize=8, parity="N", stopbits=1,
                           timeout=min(0.1, args.timeout), write_timeout=args.timeout,
                           xonxoff=False, rtscts=False, dsrdtr=False) as port:
            reply, stream, stats = exchange(port, args.operation, getattr(args, "key", 0),
                                             getattr(args, "value", 0.0), args.timeout)
            print(format_reply(args.operation, reply, stream, stats))
            return 0 if reply.status == 0 else 1
    except (OSError, TimeoutError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    raise SystemExit(main())
