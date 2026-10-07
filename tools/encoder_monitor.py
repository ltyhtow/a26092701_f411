#!/usr/bin/env python3
"""Read-only encoder telemetry capture/replay for this project's LwPKT configuration.

Wire evidence: middleware/serial/include/lwpkt_opts.h and vendored
lwpkt.c/lwpkt_opt.h: AA, single-byte CMD, varint LEN (max 64: one byte),
payload, reflected CRC8(poly 0x8C, init 0, no xorout), 55. CRC covers CMD
through payload, not either delimiter. No address/flags/extended-command fields.
Only live capture imports pyserial. This program never transmits commands.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
from dataclasses import asdict, dataclass, field
import math
from pathlib import Path
import struct
import sys
import time
from typing import Iterable

ENCODER_COMMAND = 0x87
PAYLOAD = struct.Struct("<BBHIIIIiiIqqII")
MAX_PAYLOAD = 64
FLAG_VALID = 1 << 0
FLAG_INITIALIZED = 1 << 1
FLAG_MOTOR_LATCHED = 1 << 2
FLAG_INVERT_LEFT = 1 << 3
FLAG_INVERT_RIGHT = 1 << 4


def crc8(data: bytes | bytearray) -> int:
    """The actual vendored LwPKT CRC8, including command and encoded length."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8C if crc & 1 else 0)
    return crc


@dataclass(frozen=True)
class Frame:
    command: int
    payload: bytes


@dataclass
class ParserStats:
    bytes_received: int = 0
    valid_frames: int = 0
    crc_errors: int = 0
    stop_errors: int = 0
    length_errors: int = 0
    discarded_bytes: int = 0
    incomplete_candidates: int = 0


class StreamParser:
    """Incremental framing; delimiters inside a valid length-delimited payload are data.

    On a corrupt candidate, discard only its initial AA and scan the remaining
    bytes again. A stalled/truncated candidate is rescanned by flush_incomplete(),
    used at EOF or after a live idle timeout. CRC8 cannot rule out all noise that
    coincidentally forms a valid frame; this has the wire protocol's integrity.
    """

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.stats = ParserStats()

    def feed(self, data: bytes) -> list[Frame]:
        self.stats.bytes_received += len(data)
        self.buffer.extend(data)
        return self._drain(final=False)

    def flush_incomplete(self) -> list[Frame]:
        return self._drain(final=True)

    def _discard(self, count: int) -> None:
        del self.buffer[:count]
        self.stats.discarded_bytes += count

    def _drain(self, final: bool) -> list[Frame]:
        frames = []
        while self.buffer:
            start = self.buffer.find(b"\xaa")
            if start < 0:
                self._discard(len(self.buffer))
                break
            if start:
                self._discard(start)
            if len(self.buffer) < 3:
                if final:
                    self.stats.incomplete_candidates += 1
                    self._discard(len(self.buffer))
                break
            # LwPKT limits encoded LEN to one byte when MAX_DATA_LEN is 64.
            length = self.buffer[2]
            if length > MAX_PAYLOAD:
                self.stats.length_errors += 1
                self._discard(1)
                continue
            frame_size = length + 5
            if len(self.buffer) < frame_size:
                if not final:
                    break
                self.stats.incomplete_candidates += 1
                self._discard(1)
                continue
            if self.buffer[frame_size - 1] != 0x55:
                self.stats.stop_errors += 1
                self._discard(1)
                continue
            if crc8(self.buffer[1:frame_size - 2]) != self.buffer[frame_size - 2]:
                self.stats.crc_errors += 1
                self._discard(1)
                continue
            frames.append(Frame(self.buffer[1], bytes(self.buffer[3:frame_size - 2])))
            del self.buffer[:frame_size]
            self.stats.valid_frames += 1
        return frames


@dataclass(frozen=True)
class EncoderSample:
    version: int
    sequence: int
    flags: int
    timestamp_ms: int
    sample_period_ms: int
    raw_left: int
    raw_right: int
    delta_left: int
    delta_right: int
    gpio_levels: int
    total_left: int
    total_right: int
    tx_dropped: int
    sample_errors: int

    @classmethod
    def decode(cls, payload: bytes) -> EncoderSample:
        if len(payload) != PAYLOAD.size:
            raise ValueError(f"encoder payload must be {PAYLOAD.size} bytes")
        sample = cls(*PAYLOAD.unpack(payload))
        if sample.version != 1:
            raise ValueError(f"unsupported encoder version {sample.version}")
        if sample.flags & ~0x1F:
            raise ValueError(f"unknown version-1 flags 0x{sample.flags:04x}")
        if sample.gpio_levels & ~0xF:
            raise ValueError("GPIO field contains unknown bits")
        if sample.flags & FLAG_VALID and sample.sample_period_ms == 0:
            raise ValueError("valid delta must have a nonzero sample period")
        return sample

    def csv_row(self) -> dict[str, int]:
        row = asdict(self)
        row.update(
            delta_valid=int(bool(self.flags & FLAG_VALID)),
            encoder_initialized=int(bool(self.flags & FLAG_INITIALIZED)),
            motor_stop_latched=int(bool(self.flags & FLAG_MOTOR_LATCHED)),
            invert_left=int(bool(self.flags & FLAG_INVERT_LEFT)),
            invert_right=int(bool(self.flags & FLAG_INVERT_RIGHT)),
            left_a=(self.gpio_levels >> 0) & 1,
            left_b=(self.gpio_levels >> 1) & 1,
            right_a=(self.gpio_levels >> 2) & 1,
            right_b=(self.gpio_levels >> 3) & 1,
        )
        return row


@dataclass
class WheelStats:
    positive: int = 0
    negative: int = 0
    zero: int = 0
    observed_delta_sum: int = 0
    peak_abs_delta: int = 0

    def observe(self, delta: int) -> None:
        self.positive += delta > 0
        self.negative += delta < 0
        self.zero += delta == 0
        self.observed_delta_sum += delta
        self.peak_abs_delta = max(self.peak_abs_delta, abs(delta))


@dataclass
class CaptureStats:
    encoder_frames: int = 0
    unknown_commands: int = 0
    payload_errors: int = 0
    valid_samples: int = 0
    invalid_samples: int = 0
    initialized_frames: int = 0
    motor_latched_frames: int = 0
    sequence_gaps: int = 0
    duplicates: int = 0
    timestamp_discontinuities: int = 0
    ambiguous_sequence_wraps: int = 0
    left: WheelStats = field(default_factory=WheelStats)
    right: WheelStats = field(default_factory=WheelStats)
    last: EncoderSample | None = None

    def consume(self, frames: Iterable[Frame]) -> list[EncoderSample]:
        samples = []
        for frame in frames:
            if frame.command != ENCODER_COMMAND:
                self.unknown_commands += 1
                continue
            try:
                sample = EncoderSample.decode(frame.payload)
            except ValueError:
                self.payload_errors += 1
                continue
            self.encoder_frames += 1
            self.initialized_frames += bool(sample.flags & FLAG_INITIALIZED)
            self.motor_latched_frames += bool(sample.flags & FLAG_MOTOR_LATCHED)
            duplicate = False
            if self.last is not None:
                elapsed = (sample.timestamp_ms - self.last.timestamp_ms) & 0xFFFFFFFF
                step = (sample.sequence - self.last.sequence) & 0xFF
                if elapsed > 0x7FFFFFFF:
                    # Reboot or out-of-order data: do not invent a packet loss count.
                    self.timestamp_discontinuities += 1
                elif elapsed == 0 and step == 0:
                    self.duplicates += 1
                    duplicate = True
                elif step == 0:
                    self.ambiguous_sequence_wraps += 1
                else:
                    self.sequence_gaps += step - 1
            self.last = sample
            samples.append(sample)
            if not duplicate:
                if sample.flags & FLAG_VALID and sample.flags & FLAG_INITIALIZED:
                    self.valid_samples += 1
                    self.left.observe(sample.delta_left)
                    self.right.observe(sample.delta_right)
                else:
                    self.invalid_samples += 1
        return samples

    def summary(self, parser: ParserStats) -> str:
        lines = [
            f"接收 {parser.bytes_received} 字节；CRC/帧尾校验通过 {parser.valid_frames} 帧。",
            f"0x87 有效载荷帧 {self.encoder_frames}；未知命令 {self.unknown_commands}；载荷/版本错误 {self.payload_errors}。",
            f"CRC 错误 {parser.crc_errors}；帧尾错误 {parser.stop_errors}；长度错误 {parser.length_errors}；"
            f"丢弃字节 {parser.discarded_bytes}；未完成候选帧 {parser.incomplete_candidates}。",
            f"序列缺口下界 {self.sequence_gaps}（模 256）；重复 {self.duplicates}；"
            f"时间戳重启/乱序 {self.timestamp_discontinuities}；整圈序号歧义 {self.ambiguous_sequence_wraps}。",
            f"去重采样：有效 {self.valid_samples}，无效 {self.invalid_samples}。",
            f"固件标志：编码器初始化成功 {self.initialized_frames}/{self.encoder_frames} 帧；"
            f"电机停机锁存 {self.motor_latched_frames}/{self.encoder_frames} 帧。",
        ]
        for name, wheel in (("左轮", self.left), ("右轮", self.right)):
            motion = "观察到计数变化" if wheel.positive or wheel.negative else "未观察到计数变化"
            lines.append(
                f"{name}：{motion}；正/负/零 delta 帧 {wheel.positive}/{wheel.negative}/{wheel.zero}；"
                f"收到的有效 delta 合计 {wheel.observed_delta_sum:+d}；最大 |delta| {wheel.peak_abs_delta}。"
            )
        if self.last is not None:
            sample = self.last
            lines.append(
                f"末帧总计 L={sample.total_left:+d}, R={sample.total_right:+d}；"
                f"设备累计 tx_dropped={sample.tx_dropped}, sample_errors={sample.sample_errors}。"
            )
        lines.append("这些是接收和计数观察结果；无脉冲不能自动判定硬件损坏，也不代表方向或精度已验收。")
        return "\n".join(lines)


def format_sample(sample: EncoderSample) -> str:
    pins = "".join(str((sample.gpio_levels >> bit) & 1) for bit in range(4))
    return (
        f"t={sample.timestamp_ms}ms seq={sample.sequence:3d} dt={sample.sample_period_ms}ms "
        f"raw L={sample.raw_left:08X} R={sample.raw_right:08X} "
        f"delta L={sample.delta_left:+d} R={sample.delta_right:+d} "
        f"total L={sample.total_left:+d} R={sample.total_right:+d} "
        f"AB L={pins[:2]} R={pins[2:]} flags=0x{sample.flags:04X}"
    )


def positive_seconds(value: str) -> float:
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise argparse.ArgumentTypeError("seconds must be finite and greater than zero")
    return seconds


def arguments(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="明确指定串口，例如 COM7；不自动选择")
    source.add_argument("--input", type=Path, help="离线回放原始二进制文件，不需要 pyserial")
    parser.add_argument("--usb-cdc", action="store_true",
                        help="板载 USB CDC：置 DTR 打开接收会话；不发送应用数据")
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率，默认 115200")
    parser.add_argument("--seconds", type=positive_seconds, default=60.0,
                        help="实时采集时长，默认 60 秒；离线回放读取整个文件")
    parser.add_argument("--csv", type=Path, help="新建 UTF-8 CSV，不覆盖已有文件")
    parser.add_argument("--raw-output", type=Path, help="新建原始字节文件，保留错误帧和噪声")
    result = parser.parse_args(argv)
    if result.baud <= 0:
        parser.error("--baud must be positive")
    return result


def main(argv: list[str] | None = None) -> int:
    args = arguments(argv)
    parser = StreamParser()
    capture = CaptureStats()
    interrupted = False
    last_display = -math.inf
    try:
        serial_module = None
        if args.port:
            try:
                import serial as serial_module  # type: ignore[no-redef]
            except ImportError:
                print("实时采集需要 pyserial：python -m pip install pyserial", file=sys.stderr)
                return 1
        with ExitStack() as stack:
            input_file = stack.enter_context(args.input.open("rb")) if args.input else None
            raw_file = None
            writer = None
            if args.raw_output:
                args.raw_output.parent.mkdir(parents=True, exist_ok=True)
                raw_file = stack.enter_context(args.raw_output.open("xb"))
            if args.csv:
                args.csv.parent.mkdir(parents=True, exist_ok=True)
                csv_file = stack.enter_context(args.csv.open("x", newline="", encoding="utf-8"))
                empty_sample = EncoderSample(*([0] * 14))
                writer = csv.DictWriter(csv_file, fieldnames=list(empty_sample.csv_row()))
                writer.writeheader()

            def accept(frames: list[Frame]) -> None:
                nonlocal last_display
                samples = capture.consume(frames)
                if writer is not None:
                    for sample in samples:
                        writer.writerow(sample.csv_row())
                if samples and time.monotonic() - last_display >= 1.0:
                    print(format_sample(samples[-1]), flush=True)
                    last_display = time.monotonic()

            port = None
            if args.port:
                # Configure modem-control lines before open; no application writes.
                # Some USB adapters may still toggle lines at driver level on open.
                port = serial_module.Serial(port=None, baudrate=args.baud, timeout=0.1,
                                            rtscts=False, dsrdtr=False, xonxoff=False)
                stack.callback(port.close)
                port.dtr = args.usb_cdc
                port.rts = False
                port.port = args.port
                port.open()
                print(f"只接收 {args.port}，{args.baud} baud，{args.seconds:g} 秒；Ctrl+C 提前结束。")
            started = last_received = time.monotonic()
            try:
                while True:
                    if input_file is not None:
                        chunk = input_file.read(4096)
                        if not chunk:
                            break
                    else:
                        if time.monotonic() - started >= args.seconds:
                            break
                        chunk = port.read(min(max(port.in_waiting, 1), 4096))
                        if not chunk:
                            if parser.buffer and time.monotonic() - last_received >= 0.5:
                                accept(parser.flush_incomplete())
                            continue
                    if raw_file is not None:
                        raw_file.write(chunk)
                    last_received = time.monotonic()
                    accept(parser.feed(chunk))
            except KeyboardInterrupt:
                interrupted = True
            accept(parser.flush_incomplete())
    except (OSError, ValueError) as error:
        print(f"采集/回放失败：{error}", file=sys.stderr)
        print(capture.summary(parser.stats))
        return 1
    print("已提前结束。" if interrupted else "采集/回放结束。")
    print(capture.summary(parser.stats))
    return 0 if capture.encoder_frames else 2


if __name__ == "__main__":
    # PowerShell 7 pipelines use UTF-8; Windows Python's redirected default may not.
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    raise SystemExit(main())
