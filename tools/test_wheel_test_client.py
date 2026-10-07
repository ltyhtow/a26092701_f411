"""WheelTest host regression tests. All serial access is mocked."""
import csv
import io
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import encoder_monitor as encoder
import wheel_test as wheel


def status_frame(timestamp, phase=wheel.WAIT, *, flags=5, reason=0, version=1, seq=0,
                 test_elapsed=None, remaining=None, phase_elapsed=None):
    pwm = 1200 if phase in (2, 4, 6) else -1500 if phase == 8 else 0
    if test_elapsed is None:
        test_elapsed = wheel.PHASE_START_MS[phase - 1] if 1 <= phase < 9 else 21000 if phase == 9 else 0
    if remaining is None:
        remaining = 21000 - test_elapsed if phase < wheel.DONE else 0
    if phase_elapsed is None:
        phase_elapsed = test_elapsed - wheel.PHASE_START_MS[phase - 1] if 1 <= phase < 9 else 0
    payload = wheel.STATUS.pack(version, seq, phase, reason, timestamp, phase_elapsed, pwm, pwm,
                                100, flags, 0, remaining, test_elapsed)
    return wheel.build_frame(wheel.STATUS_COMMAND, payload)


def encoder_frame(timestamp):
    return wheel.build_frame(encoder.ENCODER_COMMAND,
                             encoder.PAYLOAD.pack(1, 0, 3, timestamp, 50, 123, 456,
                                                  5, -7, 10, 1000, -2000, 0, 0))


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeSerial:
    def __init__(self, clock, chunks=(), read_error=None, write_error=None, short=False):
        self.clock = clock
        self.chunks = iter(chunks)
        self.read_error = read_error
        self.write_error = write_error
        self.short = short
        self.writes = []
        self.closed = False
        self.opened = False
        self.expected_dtr = False
        self.in_waiting = 4096

    def read(self, count):
        self.clock.now += 0.1
        if self.read_error:
            raise self.read_error
        return next(self.chunks, b"")

    def write(self, data):
        self.writes.append(data)
        if self.write_error:
            raise self.write_error
        return len(data) - int(self.short)

    def open(self):
        assert self.dtr is self.expected_dtr and self.rts is False
        self.opened = True

    def close(self):
        self.closed = True

    def actions(self):
        parser = encoder.StreamParser()
        frames = parser.feed(b"".join(self.writes))
        assert parser.stats.crc_errors == 0
        return [wheel.REQUEST.unpack(frame.payload)[2] for frame in frames]


def complete_chunks():
    # One WAIT, then real device elapsed times for 21s plus trailing DONE data.
    for i in range(235):
        elapsed = max(0, min((i - 1) * 100, 21000))
        phase = 0 if i == 0 else 1 + sum(elapsed >= boundary for boundary in wheel.PHASE_END_MS)
        yield status_frame((i + 1) * 100, phase, flags=7 if phase == 9 else 5,
                           seq=i & 255, test_elapsed=elapsed) + encoder_frame(i * 100)


class WireTests(unittest.TestCase):
    def test_exact_binary_command_wire_bytes(self):
        self.assertEqual(wheel.command_frame(wheel.START, 0).hex(), "aa060401000100e655")
        for action in (wheel.START, wheel.STOP, wheel.HEARTBEAT):
            frame = encoder.StreamParser().feed(wheel.command_frame(action, 255))[0]
            self.assertEqual(frame.command, 6)
            self.assertEqual(frame.payload, bytes((1, 255, action, 0)))
        with self.assertRaises(ValueError):
            wheel.command_frame(4, 0)

    def test_status_schema_and_invalid_version(self):
        frame = encoder.StreamParser().feed(status_frame(0xFFFFFFF0, 8))[0]
        value = wheel.WheelStatus.decode(frame.payload)
        self.assertEqual((value.left_pwm, value.right_pwm), (-1500, -1500))
        self.assertEqual(wheel.STATUS.size, 32)
        with self.assertRaises(ValueError):
            wheel.WheelStatus.decode(frame.payload[:-1])
        with self.assertRaises(ValueError):
            wheel.WheelStatus.decode(bytes((2,)) + frame.payload[1:])


class SessionTests(unittest.TestCase):
    def make(self, chunks=(), **port_kwargs):
        clock = Clock()
        port = FakeSerial(clock, chunks, **port_kwargs)
        messages = []
        session = wheel.Session(port, clock=clock, emit=messages.append)
        return session, port, messages

    def test_complete_single_run_with_heartbeats_and_stop_capture(self):
        session, port, _ = self.make(complete_chunks())
        session.raw_file = io.BytesIO()
        session.run()
        actions = port.actions()
        self.assertEqual(actions.count(wheel.START), 1)
        self.assertGreater(actions.count(wheel.HEARTBEAT), 4)
        self.assertEqual(actions[-1], wheel.STOP)
        self.assertGreaterEqual(session.clock() - session.done_at, 1.0)
        self.assertGreaterEqual(session.encoder_frames, 19)
        self.assertTrue(session.raw_file.getvalue().startswith(b"\xaa\x88\x20"))

    def test_encoder_only_and_bad_crc_do_not_start(self):
        good = status_frame(100)
        bad = good[:-2] + bytes((good[-2] ^ 1,)) + good[-1:]
        for chunks in ([encoder_frame(10)] * 20, [bad] * 20,
                       [status_frame(i * 100, version=2) for i in range(20)]):
            with self.subTest(chunks=chunks[:1]):
                session, port, _ = self.make(chunks)
                with self.assertRaises(wheel.TestFailure):
                    session.run()
                self.assertEqual(port.actions(), [wheel.STOP])

    def test_non_wait_and_latched_wait_are_never_started(self):
        for phase, flags in ((1, 5), (8, 5), (9, 7), (10, 7), (0, 7)):
            session, port, _ = self.make([status_frame(10, phase, flags=flags)])
            with self.assertRaises(wheel.TestFailure):
                session.run()
            self.assertEqual(port.actions(), [wheel.STOP])

    def test_normal_firmware_reports_mismatch_without_start(self):
        ordinary = wheel.build_frame(0x81, bytes((1,)) + bytes(31))
        session, port, _ = self.make([ordinary] * 20)
        with self.assertRaisesRegex(wheel.TestFailure, "0x81.*UsbWheelTest"):
            session.run()
        self.assertGreater(session.normal_telemetry_frames, 0)
        self.assertEqual(port.actions(), [wheel.STOP])

    def test_wait_not_ready_does_not_start_then_becomes_ready(self):
        chunks = [status_frame(1, flags=1), status_frame(2, flags=5)]
        session, port, _ = self.make(chunks)
        with self.assertRaises(wheel.TestFailure):
            session.run()
        self.assertEqual(port.actions().count(wheel.START), 1)
        self.assertEqual(port.actions()[-1], wheel.STOP)

    def test_missing_ack_never_retransmits_start(self):
        session, port, _ = self.make([status_frame(i + 1) for i in range(30)])
        with self.assertRaisesRegex(wheel.TestFailure, "未获 COUNTDOWN"):
            session.run()
        self.assertEqual(port.actions().count(wheel.START), 1)

    def test_old_status_and_encoder_do_not_extend_watchdog(self):
        chunks = [status_frame(200), status_frame(300, 1)]
        chunks += [status_frame(300, 1) + status_frame(100, 1) + encoder_frame(999)] * 20
        session, port, _ = self.make(chunks)
        with self.assertRaisesRegex(wheel.TestFailure, "超过 1 秒"):
            session.run()
        self.assertGreater(session.stale_statuses, 0)
        self.assertLess(session.clock(), 1.5)
        self.assertEqual(port.actions()[-1], wheel.STOP)

    def test_abort_and_incomplete_phase_progression_fail(self):
        for final in (status_frame(3, 10, flags=7, reason=2), status_frame(3, 9, flags=7),
                      status_frame(3, 2, flags=1)):
            session, port, _ = self.make([status_frame(1), status_frame(2, 1), final])
            with self.assertRaises(wheel.TestFailure):
                session.run()
            self.assertEqual(port.actions()[-1], wheel.STOP)

    def test_disconnect_keyboard_interrupt_and_partial_write_attempt_stop(self):
        for error in (OSError("unplugged"), KeyboardInterrupt()):
            session, port, _ = self.make(read_error=error)
            with self.assertRaises(type(error)):
                session.run()
            self.assertEqual(port.actions(), [wheel.STOP])
        session, port, messages = self.make([status_frame(1)], short=True)
        with self.assertRaises(wheel.TestFailure):
            session.run()
        self.assertEqual(port.actions(), [wheel.START, wheel.STOP])
        self.assertFalse(session.stop_written)
        self.assertTrue(any("STOP 写入失败" in m for m in messages))

    def test_timestamp_wrap_is_forward_progress(self):
        session, _, _ = self.make([status_frame(0xFFFFFFF0), status_frame(5, 1)])
        with self.assertRaises(wheel.TestFailure):
            session.run()
        self.assertEqual(session.status_frames, 2)
        self.assertEqual(session.stale_statuses, 0)

    def test_done_without_stop_latch_is_not_success(self):
        chunks = [status_frame(i + 1, i) for i in range(10)]
        session, port, _ = self.make(chunks)
        with self.assertRaisesRegex(wheel.TestFailure, "停机锁存"):
            session.run()
        self.assertEqual(port.actions()[-1], wheel.STOP)

    def test_final_stop_write_error_is_not_silently_successful(self):
        session, port, messages = self.make(complete_chunks())
        write = port.write
        def fail_stop(frame):
            if frame[5] == wheel.STOP:
                raise OSError("unplugged after DONE")
            return write(frame)
        port.write = fail_stop
        with self.assertRaisesRegex(wheel.TestFailure, "结束时 STOP 写入失败"):
            session.run()
        self.assertFalse(session.stop_written)
        self.assertTrue(any("STOP 写入失败" in m for m in messages))

    def test_early_done_and_inconsistent_device_timing_fail(self):
        for bad in (status_frame(10, wheel.DONE, flags=7, test_elapsed=1000),
                    status_frame(10, wheel.DONE, flags=7, remaining=1),
                    status_frame(10, 8, test_elapsed=22000, remaining=0),
                    status_frame(10, 8, remaining=21001),
                    status_frame(10, 8, test_elapsed=19000, phase_elapsed=900),
                    status_frame(10, 8, test_elapsed=19000, remaining=1999)):
            session, port, _ = self.make([status_frame(i + 1, i) for i in range(8)] + [bad])
            with self.assertRaisesRegex(wheel.TestFailure, "计时|时间字段"):
                session.run()
            self.assertEqual(port.actions()[-1], wheel.STOP)

    def test_device_phase_boundaries_and_repeated_phase_samples(self):
        for phase, (start, end) in enumerate(zip(wheel.PHASE_START_MS, wheel.PHASE_END_MS), 1):
            for elapsed in (start, start + 1, end - 1):
                frame = encoder.StreamParser().feed(status_frame(elapsed, phase, test_elapsed=elapsed))[0]
                wheel.validate_schedule(wheel.WheelStatus.decode(frame.payload))
            frame = encoder.StreamParser().feed(status_frame(end, phase, test_elapsed=end))[0]
            with self.assertRaises(wheel.TestFailure):
                wheel.validate_schedule(wheel.WheelStatus.decode(frame.payload))

    def test_total_timeout_stops_even_with_live_wait(self):
        session, port, _ = self.make([status_frame(i + 1, flags=1) for i in range(30)])
        session.timeout = 0.5
        with self.assertRaisesRegex(wheel.TestFailure, "总超时"):
            session.run()
        self.assertEqual(port.actions(), [wheel.STOP])


class CliTests(unittest.TestCase):
    def test_usb_cdc_asserts_dtr_and_preserves_binary_stop(self):
        port = FakeSerial(Clock(), read_error=KeyboardInterrupt())
        port.expected_dtr = True
        with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=lambda **kw: port)}), patch("builtins.print"):
            self.assertEqual(wheel.main(["--port", "COM99", "--usb-cdc"]), 130)
        self.assertTrue(port.opened and port.closed and port.dtr)
        self.assertEqual(port.actions(), [wheel.STOP])

    def test_real_main_mock_port_csv_raw_and_close(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = Clock()
            port = FakeSerial(clock, complete_chunks())
            real_session = wheel.Session
            def session_factory(*args, **kwargs):
                return real_session(*args, clock=clock, emit=lambda message: None, **kwargs)
            csv_path = Path(directory) / "run.csv"
            raw_path = Path(directory) / "run.bin"
            with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=lambda **kw: port)}), \
                    patch.object(wheel, "Session", side_effect=session_factory), patch("builtins.print"):
                result = wheel.main(["--port", "COM99", "--csv", str(csv_path), "--raw-output", str(raw_path)])
            self.assertEqual(result, 0)
            self.assertTrue(port.opened and port.closed)
            with csv_path.open(encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertGreater(len(rows), 10)
            self.assertEqual(rows[0]["delta_right"], "-7")
            self.assertEqual(rows[0]["encoder_minus_status_ms"], "-100")
            self.assertEqual(rows[0]["latest_status_phase"], "WAIT")
            self.assertTrue(csv_path.with_suffix(".status.csv").exists())
            self.assertGreater(raw_path.stat().st_size, 100)

    def test_existing_output_refuses_before_port_open(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "existing.csv"
            output.write_text("keep", encoding="utf-8")
            constructor = unittest.mock.Mock()
            with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=constructor)}), patch("builtins.print"):
                self.assertEqual(wheel.main(["--port", "COM99", "--csv", str(output)]), 1)
            constructor.assert_not_called()
            self.assertEqual(output.read_text(encoding="utf-8"), "keep")

    def test_ctrl_c_main_closes_port_and_returns_130(self):
        port = FakeSerial(Clock(), read_error=KeyboardInterrupt())
        with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=lambda **kw: port)}), patch("builtins.print"):
            self.assertEqual(wheel.main(["--port", "COM99"]), 130)
        self.assertTrue(port.closed)
        self.assertEqual(port.actions(), [wheel.STOP])


class RealCFixtureTests(unittest.TestCase):
    @unittest.skipUnless(os.environ.get("WHEEL_PROTOCOL_FIXTURE"), "real C fixture not requested")
    def test_real_codec_lwpkt_lwrb_golden_fixture(self):
        data = Path(os.environ["WHEEL_PROTOCOL_FIXTURE"]).read_bytes()
        self.assertEqual(len(data), 101)
        parser = encoder.StreamParser()
        frames = parser.feed(data)
        self.assertEqual([frame.command for frame in frames], [6, 6, 6, 0x88, 0x88])
        self.assertEqual(parser.stats.crc_errors, 0)
        self.assertEqual(data[:27], b"".join(wheel.command_frame(action, action + 6)
                                            for action in (wheel.START, wheel.STOP, wheel.HEARTBEAT)))
        reverse = wheel.WheelStatus.decode(frames[3].payload)
        self.assertEqual(reverse, wheel.WheelStatus(1, 10, 8, 0, 0x88776655, 2000,
                                                   -1500, -1200, 200, 13, 0, 1000, 20000))
        done = wheel.WheelStatus.decode(frames[4].payload)
        self.assertEqual(done, wheel.WheelStatus(1, 11, 9, 0, 0x88776A3D, 0,
                                                0, 0, 200, 15, 0, 0, 21000))
        wheel.validate_schedule(reverse)
        wheel.validate_schedule(done)
        for split in range(len(data) + 1):
            parser = encoder.StreamParser()
            self.assertEqual(parser.feed(data[:split]) + parser.feed(data[split:]), frames)


if __name__ == "__main__":
    unittest.main()
