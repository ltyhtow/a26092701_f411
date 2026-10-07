"""Offline tests; all serial I/O is mocked and no real ports are accessed."""

from collections import deque
from contextlib import redirect_stderr, redirect_stdout
import io
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import tuner

# Generated with this repo's serial_codec_encode -> real lwpkt_write -> LwRB,
# then accepted by its C lwpkt_process. Inputs also match test_serial_codec.c:
# parameter {1,7,SET,14,INT32_MIN}; response {1,7,4,UNSAFE,14,FALLEN,0x1234,
# INT32_MIN,0x04030201,0x08070605,0x88776655}.
C_REQUEST = bytes.fromhex("AA 04 08 01 07 02 0E 00 00 00 80 FE 55")
C_RESPONSE = bytes.fromhex(
    "AA 83 18 01 07 04 04 0E 03 34 12 00 00 00 80 "
    "01 02 03 04 05 06 07 08 55 66 77 88 57 55"
)


def response(**changes):
    fields = dict(version=1, sequence=7, request_command=4, status=0,
                  key=14, controller_state=0, storage_flags=0, value_q16_16=4500 * 65536,
                  fault_flags=0, config_revision=12, timestamp_ms=1000)
    fields.update(changes)
    return tuner.build_frame(0x83, tuner.RESPONSE.pack(*fields.values()))


class FakeClock:
    def __init__(self):
        self.current = 0.0

    def __call__(self):
        self.current += 0.001
        return self.current


class FakePort:
    def __init__(self, chunks=(), short_write=False):
        self.chunks = deque(chunks)
        self.writes = []
        self.short_write = short_write
        self.closed = False

    def __setattr__(self, name, value):
        if name in ("dtr", "rts"):
            raise AssertionError("The tool must not manipulate DTR/RTS")
        object.__setattr__(self, name, value)

    def read(self, count):
        return self.chunks.popleft() if self.chunks else b""

    def write(self, data):
        if not isinstance(data, bytes):
            raise AssertionError("Only framed binary bytes may reach serial.write")
        self.writes.append(data)
        return len(data) - 1 if self.short_write else len(data)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True


class CodecTests(unittest.TestCase):
    def test_actual_c_lwpkt_request_golden(self):
        sequence, wire = tuner.build_request("set", 14, -32768.0, 7)
        self.assertEqual(sequence, 7)
        self.assertEqual(wire, C_REQUEST)
        self.assertEqual(tuner.REQUEST.size, 8)
        parser = tuner.StreamParser()
        frames = parser.feed(wire)
        self.assertEqual(frames[0].command, 4)
        self.assertEqual(tuner.REQUEST.unpack(frames[0].payload), (1, 7, 2, 14, -(1 << 31)))

    def test_actual_c_lwpkt_response_golden(self):
        parser = tuner.StreamParser()
        frames = parser.feed(C_RESPONSE)
        self.assertEqual(len(frames), 1)
        reply = tuner.Reply.decode(frames[0].payload)
        self.assertEqual(tuner.RESPONSE.size, 24)
        self.assertEqual(reply, tuner.Reply(1, 7, 4, 4, 14, 3, 0x1234, -(1 << 31),
                                           0x04030201, 0x08070605, 0x88776655))

    def test_all_operations_emit_only_parameter_frames(self):
        for operation, code in tuner.OPERATIONS.items():
            key = 1 if operation in ("get", "set") else 0
            _, wire = tuner.build_request(operation, key, 1.25, 255)
            frame = tuner.StreamParser().feed(wire)[0]
            self.assertEqual(frame.command, 4)
            self.assertEqual(tuner.REQUEST.unpack(frame.payload),
                             (1, 255, code, key, 81920 if operation == "set" else 0))

    def test_finite_fixed_point_validation(self):
        for value in (math.nan, math.inf, -math.inf, -32768.1, 32768.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                tuner.q16_encode(value)
        self.assertEqual(tuner.q16_encode(-32768.0), -(1 << 31))
        self.assertEqual(tuner.q16_encode(32767.9999847412109375), (1 << 31) - 1)
        self.assertEqual(tuner.q16_encode(-1.5), -98304)
        self.assertEqual(tuner.q16_encode(0.5 / 65536), 1)
        self.assertEqual(tuner.q16_encode(-0.5 / 65536), -1)

    def test_parameter_names_and_random_sequence(self):
        self.assertEqual([tuner.parameter_key(name) for name in tuner.PARAMETERS], list(range(1, 15)))
        self.assertEqual(tuner.parameter_key("BALANCE.KP"), 1)
        self.assertEqual(tuner.parameter_key("mechanical_zero_pitch"), 11)
        self.assertEqual(tuner.parameter_key("14"), 14)
        for key in ("0", "15", "typo"):
            with self.assertRaises(tuner.argparse.ArgumentTypeError):
                tuner.parameter_key(key)
        with patch.object(tuner.secrets, "randbelow", return_value=93) as random:
            sequence, wire = tuner.build_request("status")
        self.assertEqual(sequence, 93)
        self.assertEqual(wire[4], 93)
        random.assert_called_once_with(256)

    def test_malformed_request_rejected_before_serial(self):
        for args in (("arm",), ("get", 0), ("set", 15), ("save", 1)):
            with self.assertRaises(ValueError):
                tuner.build_request(*args)
        with self.assertRaises(ValueError):
            tuner.build_request("status", sequence=256)
        with self.assertRaises(ValueError):
            tuner.Reply.decode(b"short")


class ExchangeTests(unittest.TestCase):
    def test_fragmentation_at_every_boundary(self):
        for split in range(len(C_RESPONSE) + 1):
            port = FakePort([C_RESPONSE[:split], C_RESPONSE[split:]])
            reply, _, _ = tuner.exchange(port, "set", 14, -32768, sequence=7, clock=FakeClock())
            self.assertEqual(reply.status, 4)
            self.assertEqual(port.writes, [C_REQUEST])

    def test_crc_noise_telemetry_wrong_correlation_and_version_ignored(self):
        bad_crc = bytearray(response())
        bad_crc[-2] ^= 1
        data = (
            b"noise" + tuner.build_frame(0x81, bytes(32)) + bytes(bad_crc) +
            response(sequence=8) + response(request_command=2) + response(key=13) +
            response(version=2) + tuner.build_frame(0x83, b"short") +
            tuner.build_frame(0x82, C_RESPONSE[3:-2]) + response()
        )
        port = FakePort(bytes((byte,)) for byte in data)
        reply, parser, stats = tuner.exchange(port, "get", 14, sequence=7, clock=FakeClock())
        self.assertEqual(reply.status, 0)
        self.assertEqual(parser.stats.crc_errors, 1)
        self.assertEqual(stats.malformed_replies, 2)
        self.assertEqual(stats.ignored_frames, 5)
        self.assertEqual(len(port.writes), 1)

    def test_stalled_bad_length_candidate_recovers_on_idle(self):
        port = FakePort([b"\xaa\x90\x40" + response()])
        reply, parser, _ = tuner.exchange(port, "get", 14, sequence=7, clock=FakeClock())
        self.assertEqual(reply.status, 0)
        self.assertGreater(parser.stats.incomplete_candidates, 0)

    def test_timeout_does_not_retransmit_save(self):
        port = FakePort([response(sequence=123, key=0)])
        with self.assertRaisesRegex(TimeoutError, "不自动重发 SAVE"):
            tuner.exchange(port, "save", timeout=0.05, sequence=7, clock=FakeClock())
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(tuner.REQUEST.unpack(port.writes[0][3:-2]), (1, 7, 3, 0, 0))

    def test_short_write_not_retried(self):
        port = FakePort(short_write=True)
        with self.assertRaisesRegex(OSError, "未自动重发"):
            tuner.exchange(port, "save", sequence=7)
        self.assertEqual(len(port.writes), 1)

    def test_read_error_propagates_without_retransmission(self):
        port = FakePort()
        with patch.object(port, "read", side_effect=OSError("disconnected")):
            with self.assertRaises(OSError):
                tuner.exchange(port, "save", sequence=7)
        self.assertEqual(len(port.writes), 1)


class CliTests(unittest.TestCase):
    def test_status_explains_storage_error_and_ram_revision(self):
        port = FakePort([response(key=0, storage_flags=0x29, value_q16_16=0x105 * 65536,
                                  fault_flags=0x80000000)])
        reply, parser, stats = tuner.exchange(port, "status", sequence=7, clock=FakeClock())
        text = tuner.format_reply("status", reply, parser, stats)
        self.assertIn("CONFIGURED|DIRTY|ERROR", text)
        self.assertIn("RECORD_VERIFY", text)
        self.assertIn("UNKNOWN:0x80000000", text)
        self.assertIn("revision 是 RAM 配置版本", text)

    def test_mock_serial_receives_binary_and_never_arm_or_line_control(self):
        port = FakePort([response(key=11, value_q16_16=-98304)])
        settings = {}

        def constructor(**kwargs):
            settings.update(kwargs)
            return port

        stdout = io.StringIO()
        with patch.dict("sys.modules", {"serial": SimpleNamespace(Serial=constructor)}), \
                patch.object(tuner.secrets, "randbelow", return_value=7), redirect_stdout(stdout):
            code = tuner.main(["--port", "MOCK", "set", "mechanical_zero", "-1.5"])
        self.assertEqual(code, 0)
        self.assertEqual(settings["baudrate"], 115200)
        self.assertEqual((settings["bytesize"], settings["parity"], settings["stopbits"]), (8, "N", 1))
        self.assertFalse(settings["xonxoff"] or settings["rtscts"] or settings["dsrdtr"])
        self.assertTrue(port.closed)
        self.assertEqual(len(port.writes), 1)
        self.assertEqual(tuner.REQUEST.unpack(port.writes[0][3:-2]), (1, 7, 2, 11, -98304))
        for text in ("result=OK", "state=DISARMED", "faults=0x00000000", "storage_flags=0x0000",
                     "revision=12", "value=-1.500000", "尚未写入 EEPROM"):
            self.assertIn(text, stdout.getvalue())

    def test_device_rejection_is_nonzero_exit_and_printed(self):
        port = FakePort([response(status=4, controller_state=2, fault_flags=1 << 5)])
        output = io.StringIO()
        with patch.dict("sys.modules", {"serial": SimpleNamespace(Serial=lambda **kwargs: port)}), \
                patch.object(tuner.secrets, "randbelow", return_value=7), redirect_stdout(output):
            code = tuner.main(["--port", "MOCK", "get", "max_pwm"])
        self.assertEqual(code, 1)
        self.assertIn("result=UNSAFE", output.getvalue())
        self.assertIn("state=ARMED", output.getvalue())
        self.assertIn("EMERGENCY_STOP", output.getvalue())

    def test_invalid_cli_values_fail_before_open(self):
        for args in (("set", "balance.kp", "nan"), ("set", "max_pwm", "32768"),
                     ("get", "unknown"), ("arm",), ("--timeout", "nan", "status")):
            with self.subTest(args=args), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    tuner.main(["--port", "MOCK", *args])
                self.assertEqual(result.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
