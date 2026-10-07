#!/usr/bin/env python3
"""One-shot, heartbeat-supervised WheelTest client; binary LwPKT only.

This program commands motors. Use only the dedicated WheelTest firmware with
the wheels suspended. No serial device is accessed by importing this module.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
from dataclasses import asdict, dataclass
from pathlib import Path
import struct
import sys
import time

from encoder_monitor import ENCODER_COMMAND, EncoderSample, StreamParser
from tuner import build_frame, positive_seconds

COMMAND = 0x06
STATUS_COMMAND = 0x88
REQUEST = struct.Struct("<BBBB")
STATUS = struct.Struct("<BBBBIIhhIHHII")
START, STOP, HEARTBEAT = 1, 2, 3
WAIT, COUNTDOWN, DONE, ABORTED = 0, 1, 9, 10
READY, LATCHED, SAMPLE_VALID, HEARTBEAT_SEEN = 1, 2, 4, 8
PHASES = ("WAIT", "COUNTDOWN", "LEFT_FORWARD", "REST_LEFT", "RIGHT_FORWARD",
          "REST_RIGHT", "BOTH_FORWARD", "REST_FORWARD", "BOTH_REVERSE", "DONE", "ABORTED")
REASONS = ("NONE", "USER_STOP", "LINK_TIMEOUT", "SENSOR_ERROR", "DRIVER_FAULT", "DEADLINE")
HEARTBEAT_SECONDS = 0.2
STATUS_TIMEOUT = 1.0
DURATION_MS = 21000
# Indexes 0..7 are the start of phases COUNTDOWN..BOTH_REVERSE.
PHASE_START_MS = (0, 5000, 8000, 9000, 12000, 13000, 16000, 18000)
PHASE_END_MS = (*PHASE_START_MS[1:], DURATION_MS)


def command_frame(action: int, sequence: int) -> bytes:
    if action not in (START, STOP, HEARTBEAT) or not 0 <= sequence <= 255:
        raise ValueError("invalid WheelTest action or sequence")
    return build_frame(COMMAND, REQUEST.pack(1, sequence, action, 0))


@dataclass(frozen=True)
class WheelStatus:
    version: int
    sequence: int
    phase: int
    reason: int
    timestamp_ms: int
    phase_elapsed_ms: int
    left_pwm: int
    right_pwm: int
    heartbeat_age_ms: int
    flags: int
    reserved: int
    remaining_ms: int
    test_elapsed_ms: int

    @classmethod
    def decode(cls, payload: bytes) -> WheelStatus:
        if len(payload) != STATUS.size:
            raise ValueError("WheelTest status must contain 32 bytes")
        value = cls(*STATUS.unpack(payload))
        if value.version != 1 or value.phase >= len(PHASES) or value.reason >= len(REASONS):
            raise ValueError("unknown WheelTest status version, phase or reason")
        if value.reserved or value.flags & ~0xF:
            raise ValueError("unknown WheelTest status flags/reserved bits")
        if abs(value.left_pwm) > 1500 or abs(value.right_pwm) > 1500:
            raise ValueError("WheelTest PWM exceeds its fixed limit")
        return value


class TestFailure(RuntimeError):
    """A test that must terminate and attempt STOP."""


def validate_schedule(status):
    """Check device timing, independent of host buffering/receive speed."""
    if max(status.test_elapsed_ms, status.remaining_ms, status.phase_elapsed_ms) > DURATION_MS:
        raise TestFailure("状态时间字段超过 21000ms")
    if status.phase == WAIT:
        expected = status.test_elapsed_ms == status.phase_elapsed_ms == 0 and status.remaining_ms == DURATION_MS
    elif COUNTDOWN <= status.phase < DONE:
        start = PHASE_START_MS[status.phase - 1]
        end = PHASE_END_MS[status.phase - 1]
        expected = (start <= status.test_elapsed_ms < end
                    and status.phase_elapsed_ms == status.test_elapsed_ms - start
                    and status.remaining_ms == DURATION_MS - status.test_elapsed_ms)
    elif status.phase == DONE:
        expected = (status.test_elapsed_ms == DURATION_MS
                    and status.remaining_ms == status.phase_elapsed_ms == 0)
    else:  # ABORTED freezes elapsed at the failure and has no pending run.
        expected = status.remaining_ms == status.phase_elapsed_ms == 0
    if not expected:
        raise TestFailure(f"{PHASES[status.phase]} 设备计时与 21 秒测试计划不一致")


class Session:
    def __init__(self, port, *, timeout=45.0, clock=time.monotonic,
                 encoder_writer=None, status_writer=None, raw_file=None, emit=print):
        self.port = port
        self.timeout = timeout
        self.clock = clock
        self.encoder_writer = encoder_writer
        self.status_writer = status_writer
        self.raw_file = raw_file
        self.emit = emit
        self.parser = StreamParser()
        self.latest = None
        self.latest_received = None
        self.sequence = 0
        self.started = False
        self.started_at = None
        self.acknowledged = False
        self.phase = WAIT
        self.done_at = None
        self.encoder_frames = 0
        self.status_frames = 0
        self.normal_telemetry_frames = 0
        self.bad_payloads = 0
        self.stale_statuses = 0
        self.stop_written = False
        self.last_display = float("-inf")

    def send(self, action):
        frame = command_frame(action, self.sequence)
        self.sequence = (self.sequence + 1) & 255
        if self.port.write(frame) != len(frame):
            raise TestFailure("串口只写入了部分指令")

    def accept_status(self, status, now):
        if self.latest is not None:
            elapsed = (status.timestamp_ms - self.latest.timestamp_ms) & 0xFFFFFFFF
            if elapsed == 0 or elapsed >= 0x80000000:
                self.stale_statuses += 1
                return  # Replayed/old packets cannot extend the status watchdog.
        validate_schedule(status)
        if self.latest is not None and status.test_elapsed_ms < self.latest.test_elapsed_ms:
            raise TestFailure("设备测试计时倒退，不能确认单次完整流程")
        self.latest = status
        self.latest_received = now
        self.status_frames += 1
        if self.status_writer:
            self.status_writer.writerow({"host_elapsed_s": now - self.opened_at, **asdict(status),
                                         "phase_name": PHASES[status.phase], "reason_name": REASONS[status.reason]})
        if status.phase == ABORTED:
            raise TestFailure("固件已中止：" + REASONS[status.reason] + "；检查后复位再试")
        if status.reason:
            raise TestFailure("状态包含故障：" + REASONS[status.reason])
        if not self.started:
            if status.phase != WAIT:
                raise TestFailure("固件不是 WAIT；不会重复启动，请确认停机后复位 WheelTest 固件")
            if status.left_pwm or status.right_pwm or status.flags & LATCHED:
                raise TestFailure("WAIT 输出/锁存异常；请检查并复位固件")
            if status.flags & (READY | SAMPLE_VALID) != READY | SAMPLE_VALID:
                return
            # Only one START is sent, even if the acknowledgement is lost.
            self.started = True
            self.started_at = now
            self.send(START)
            self.send(HEARTBEAT)
            self.next_heartbeat = now + HEARTBEAT_SECONDS
            self.emit("START 已写入；等待 COUNTDOWN 确认，5 秒后轮子开始转动。")
            return
        if status.phase == WAIT:
            if self.acknowledged:
                raise TestFailure("运行过程中回到 WAIT，疑似复位；不会自动重启")
            return
        if status.phase < DONE:
            if status.flags & LATCHED or status.flags & (READY | SAMPLE_VALID) != READY | SAMPLE_VALID:
                raise TestFailure("运行时编码器无效或电机已锁存")
        if status.phase != self.phase:
            if status.phase != self.phase + 1:
                raise TestFailure("状态阶段缺失或乱序，无法确认完整单次测试")
            self.phase = status.phase
            self.emit(f"阶段 {PHASES[status.phase]}；PWM L={status.left_pwm} R={status.right_pwm}")
        if status.phase == COUNTDOWN:
            self.acknowledged = True
        if status.phase == DONE:
            if status.left_pwm or status.right_pwm or not status.flags & LATCHED:
                raise TestFailure("DONE 未确认零 PWM 和停机锁存")
            if self.done_at is None:
                self.done_at = now
                self.emit("DONE：继续记录 1 秒停机数据。")

    def accept_encoder(self, sample, now):
        self.encoder_frames += 1
        if self.encoder_writer:
            row = {"host_elapsed_s": now - self.opened_at, **sample.csv_row()}
            row.update(latest_status_timestamp_ms="", latest_status_phase="", latest_status_left_pwm="",
                       latest_status_right_pwm="", status_receive_age_ms="", encoder_minus_status_ms="")
            if self.latest is not None:
                # Receive-order association only, NOT a synchronized measurement.
                delta = (sample.timestamp_ms - self.latest.timestamp_ms) & 0xFFFFFFFF
                if delta >= 0x80000000:
                    delta -= 0x100000000
                row.update(latest_status_timestamp_ms=self.latest.timestamp_ms,
                           latest_status_phase=PHASES[self.latest.phase],
                           latest_status_left_pwm=self.latest.left_pwm,
                           latest_status_right_pwm=self.latest.right_pwm,
                           status_receive_age_ms=(now - self.latest_received) * 1000,
                           encoder_minus_status_ms=delta)
            self.encoder_writer.writerow(row)
        if now - self.last_display >= 0.5:
            phase = PHASES[self.latest.phase] if self.latest else "NO_STATUS"
            self.emit(f"{phase}: delta L={sample.delta_left:+d} R={sample.delta_right:+d}; "
                      f"total L={sample.total_left} R={sample.total_right}; flags=0x{sample.flags:02x}")
            self.last_display = now

    def run(self):
        self.opened_at = self.clock()
        self.next_heartbeat = float("inf")
        completed = False
        try:
            while True:
                now = self.clock()
                last_status = self.latest_received if self.latest_received is not None else self.opened_at
                if now - last_status >= STATUS_TIMEOUT:
                    if self.status_frames == 0 and self.normal_telemetry_frames:
                        raise TestFailure("收到普通固件的 0x81 遥测，但没有 0x88 转轮状态；"
                                          "请烧录 WheelTest（UART）或 UsbWheelTest（板载 USB）固件")
                    raise TestFailure("超过 1 秒没有新的有效 WheelTest 状态帧；编码器帧不能代替状态心跳")
                if now - self.opened_at >= self.timeout:
                    raise TestFailure("达到主机总超时")
                if self.started and not self.acknowledged and now - self.started_at >= STATUS_TIMEOUT:
                    raise TestFailure("START 未获 COUNTDOWN 确认；不重复发送 START")
                if self.done_at is not None and now - self.done_at >= 1.0:
                    if not self.encoder_frames:
                        raise TestFailure("流程结束但没有有效编码器载荷")
                    completed = True
                    return
                if self.started and now >= self.next_heartbeat:
                    self.send(HEARTBEAT)
                    self.next_heartbeat = now + HEARTBEAT_SECONDS
                chunk = self.port.read(min(max(self.port.in_waiting, 1), 4096))
                now = self.clock()
                if chunk:
                    if self.raw_file:
                        self.raw_file.write(chunk)
                    for frame in self.parser.feed(chunk):
                        try:
                            if frame.command == STATUS_COMMAND:
                                value = WheelStatus.decode(frame.payload)
                            elif frame.command == ENCODER_COMMAND:
                                value = EncoderSample.decode(frame.payload)
                            elif frame.command == 0x81 and len(frame.payload) == 32 and frame.payload[0] == 1:
                                self.normal_telemetry_frames += 1
                                continue
                            else:
                                continue
                        except ValueError:
                            self.bad_payloads += 1
                            continue
                        if frame.command == STATUS_COMMAND:
                            self.accept_status(value, now)
                        else:
                            self.accept_encoder(value, now)
        finally:
            try:
                self.send(STOP)
                self.stop_written = True
                self.emit("STOP 帧已写入串口；这不等于已收到硬件停机确认。")
            except Exception as exc:
                self.emit(f"STOP 写入失败：{exc}；未确认停机，固件失联保护应在 750ms 后锁停。")
                if completed:
                    raise TestFailure("DONE 已记录，但结束时 STOP 写入失败") from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="显式指定 USB 串口，例如 COM7")
    parser.add_argument("--usb-cdc", action="store_true",
                        help="板载 USB CDC：置 DTR 打开数据会话；仅用于 UsbWheelTest 固件")
    parser.add_argument("--timeout", type=positive_seconds, default=45.0, help="总超时秒数，默认 45")
    parser.add_argument("--csv", type=Path, help="编码器 CSV；同时生成同名 .status.csv")
    parser.add_argument("--raw-output", type=Path, help="完整接收字节流，二进制 .bin")
    args = parser.parse_args(argv)
    try:
        import serial
        with ExitStack() as stack:
            raw_file = None
            encoder_writer = status_writer = None
            outputs = [p for p in (args.csv, args.raw_output) if p is not None]
            status_path = args.csv.with_suffix(".status.csv") if args.csv else None
            if status_path:
                outputs.append(status_path)
            resolved = [str(p.resolve()).casefold() for p in outputs]
            if len(resolved) != len(set(resolved)):
                raise ValueError("输出路径不能相同")
            if any(p.exists() for p in outputs):
                raise ValueError("输出文件已存在，请换文件名；不会覆盖记录")
            for p in outputs:
                p.parent.mkdir(parents=True, exist_ok=True)
            if args.raw_output:
                raw_file = stack.enter_context(args.raw_output.open("xb"))
            if args.csv:
                encoder_file = stack.enter_context(args.csv.open("x", newline="", encoding="utf-8"))
                status_file = stack.enter_context(status_path.open("x", newline="", encoding="utf-8"))
                encoder_keys = ["host_elapsed_s", *EncoderSample(*([0] * 14)).csv_row(),
                                "latest_status_timestamp_ms", "latest_status_phase", "latest_status_left_pwm",
                                "latest_status_right_pwm", "status_receive_age_ms", "encoder_minus_status_ms"]
                status_keys = ["host_elapsed_s", *WheelStatus.__dataclass_fields__, "phase_name", "reason_name"]
                encoder_writer = csv.DictWriter(encoder_file, fieldnames=encoder_keys)
                status_writer = csv.DictWriter(status_file, fieldnames=status_keys)
                encoder_writer.writeheader()
                status_writer.writeheader()
            port = serial.Serial(port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                                 timeout=0.05, write_timeout=0.1, rtscts=False, dsrdtr=False, xonxoff=False)
            stack.callback(port.close)
            port.dtr = args.usb_cdc
            port.rts = False
            port.port = args.port
            port.open()
            transport = "板载 USB CDC" if args.usb_cdc else "UART 115200 8N1"
            print(f"连接 {args.port}，{transport}。等待 WheelTest WAIT；确认后自动执行一次转轮测试。")
            session = Session(port, timeout=args.timeout, encoder_writer=encoder_writer,
                              status_writer=status_writer, raw_file=raw_file)
            session.run()
            print(f"完整流程 DONE；编码器 {session.encoder_frames} 帧，状态 {session.status_frames} 帧，"
                  f"CRC 错误 {session.parser.stats.crc_errors}，旧状态 {session.stale_statuses}。")
            print("流程完成不代表编码器验收通过：仍需检查轮子实际转向、计数符号和左右响应。")
            return 0
    except KeyboardInterrupt:
        print("用户中断；未确认完整测试通过。", file=sys.stderr)
        return 130
    except (OSError, ValueError, TestFailure, ImportError) as exc:
        print(f"转轮测试未通过：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    raise SystemExit(main())
