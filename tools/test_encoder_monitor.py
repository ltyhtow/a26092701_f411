"""Offline tests only; no serial port is enumerated, opened or written."""
import contextlib
import csv
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import encoder_monitor as monitor

# Frozen from tools/test_encoder_lwpkt.c using the vendored C lwpkt_write,
# and validated there with the real lwpkt_process receiver (not this CRC code).
C_GOLDEN_FRAME = bytes.fromhex(
    "AA 87 38 01 07 17 00 3A E2 01 00 32 00 00 00 AA "
    "55 AA 55 40 E2 01 00 F4 FF FF FF 22 00 00 00 09 "
    "00 00 00 35 FB 04 8E E0 FE FF FF F2 8F 5B 25 22 "
    "02 00 00 02 00 00 00 03 00 00 00 CF 55"
)
C_GOLDEN_SAMPLE = monitor.EncoderSample(
    1, 7, 0x17, 123450, 50, 0x55AA55AA, 123456, -12, 34, 9,
    -1234567890123, 2345678901234, 2, 3,
)


def payload(sequence=1, timestamp=50, flags=0x17, **changes):
    values = dict(
        version=1, sequence=sequence, flags=flags, timestamp_ms=timestamp,
        sample_period_ms=50, raw_left=0xAA55AA55, raw_right=0x55AA55AA,
        delta_left=11, delta_right=-22, gpio_levels=0xA,
        total_left=(1 << 40) + 11, total_right=-(1 << 40) - 22,
        tx_dropped=3, sample_errors=4,
    )
    values.update(changes)
    return monitor.PAYLOAD.pack(*values.values())


def frame(data=None, command=monitor.ENCODER_COMMAND):
    if data is None:
        data = payload()
    body = bytes([command, len(data)]) + data
    return b"\xaa" + body + bytes([monitor.crc8(body), 0x55])


class FramingTests(unittest.TestCase):
    def test_crc_known_check_value_and_single_byte_command(self):
        self.assertEqual(monitor.crc8(b"123456789"), 0xA1)
        wire = frame()
        self.assertEqual(wire[:3], b"\xaa\x87\x38")
        self.assertEqual(len(wire), 61)
        self.assertNotEqual(monitor.crc8(wire[:-2]), wire[-2])

    def test_every_two_chunk_split_and_bytewise_input(self):
        wire = frame()
        for cut in range(len(wire) + 1):
            with self.subTest(cut=cut):
                parser = monitor.StreamParser()
                result = parser.feed(wire[:cut]) + parser.feed(wire[cut:])
                self.assertEqual(result, [monitor.Frame(0x87, payload())])
                self.assertEqual(parser.stats.valid_frames, 1)
                self.assertFalse(parser.buffer)
        parser = monitor.StreamParser()
        result = []
        for byte in wire:
            result.extend(parser.feed(bytes([byte])))
        self.assertEqual(result, [monitor.Frame(0x87, payload())])

    def test_concatenation_noise_and_embedded_delimiters(self):
        parser = monitor.StreamParser()
        self.assertIn(b"\xaa\x55", payload())
        result = parser.feed(b"garbage\x00\xff" + frame() + frame(payload(2, 100)))
        self.assertEqual(len(result), 2)
        self.assertEqual(parser.stats.discarded_bytes, 9)
        self.assertEqual(parser.stats.crc_errors, 0)

    def test_crc_corruption_resynchronizes(self):
        damaged = bytearray(frame())
        damaged[-2] ^= 1
        parser = monitor.StreamParser()
        result = parser.feed(bytes(damaged) + frame(payload(2, 100)))
        result += parser.flush_incomplete()
        self.assertEqual(result, [monitor.Frame(0x87, payload(2, 100))])
        self.assertGreaterEqual(parser.stats.crc_errors, 1)

    def test_bad_stop_and_length_resynchronize(self):
        for prefix in (frame()[:-1] + b"\x00", b"\xaa\x87\x80", b"\xaa\x87\x41"):
            with self.subTest(prefix=prefix.hex()):
                parser = monitor.StreamParser()
                result = parser.feed(prefix + frame()) + parser.flush_incomplete()
                self.assertEqual(result, [monitor.Frame(0x87, payload())])
                self.assertGreater(parser.stats.stop_errors + parser.stats.length_errors, 0)

    def test_truncated_candidate_at_eof_recovers_complete_frame(self):
        parser = monitor.StreamParser()
        self.assertEqual(parser.feed(b"\xaa\x99\x40" + frame()), [])
        self.assertEqual(parser.flush_incomplete(), [monitor.Frame(0x87, payload())])
        self.assertGreaterEqual(parser.stats.incomplete_candidates, 1)
        self.assertFalse(parser.buffer)
        parser.feed(frame()[:20])
        self.assertEqual(parser.flush_incomplete(), [])
        self.assertFalse(parser.buffer)

    def test_unknown_commands_are_validated_and_skipped(self):
        parser = monitor.StreamParser()
        capture = monitor.CaptureStats()
        result = capture.consume(parser.feed(frame(b"\xaa\x55", 0x81) + frame()))
        self.assertEqual(len(result), 1)
        self.assertEqual(capture.unknown_commands, 1)
        self.assertEqual(parser.stats.valid_frames, 2)


class SampleTests(unittest.TestCase):
    def test_signed_64bit_fields_gpio_flags(self):
        sample = monitor.EncoderSample.decode(payload())
        self.assertEqual(sample.total_left, (1 << 40) + 11)
        self.assertEqual(sample.total_right, -(1 << 40) - 22)
        self.assertEqual((sample.delta_left, sample.delta_right), (11, -22))
        row = sample.csv_row()
        self.assertEqual([row[name] for name in ("left_a", "left_b", "right_a", "right_b")], [0, 1, 0, 1])
        self.assertEqual(row["invert_left"], 0)
        self.assertEqual(row["invert_right"], 1)
        self.assertEqual(row["motor_stop_latched"], 1)

    def test_bad_payload_length_version_and_fields(self):
        parser = monitor.StreamParser()
        capture = monitor.CaptureStats()
        malformed = [payload()[:-1], payload(version=2), payload(flags=0x80),
                     payload(gpio_levels=16), payload(sample_period_ms=0)]
        samples = capture.consume(parser.feed(b"".join(frame(data) for data in malformed) + frame()))
        self.assertEqual(len(samples), 1)
        self.assertEqual(capture.payload_errors, len(malformed))

    def test_sequence_wrap_gap_duplicates_and_reboot(self):
        parser = monitor.StreamParser()
        capture = monitor.CaptureStats()
        data = [payload(254, 0xFFFFFFCD), payload(255, 0xFFFFFFFF), payload(0, 49),
                payload(3, 199), payload(3, 199), payload(0, 0), payload(0, 12800)]
        capture.consume(parser.feed(b"".join(frame(item) for item in data)))
        self.assertEqual(capture.sequence_gaps, 2)
        self.assertEqual(capture.duplicates, 1)
        self.assertEqual(capture.timestamp_discontinuities, 1)
        self.assertEqual(capture.ambiguous_sequence_wraps, 1)
        self.assertEqual(capture.valid_samples, 6)

    def test_invalid_samples_do_not_claim_motion(self):
        parser = monitor.StreamParser()
        capture = monitor.CaptureStats()
        capture.consume(parser.feed(frame(payload(flags=0x16, sample_period_ms=0))))
        self.assertEqual(capture.invalid_samples, 1)
        self.assertEqual(capture.left.observed_delta_sum, 0)
        self.assertIn("未观察到计数变化", capture.summary(parser.stats))
        self.assertIn("不能自动判定硬件损坏", capture.summary(parser.stats))

    def test_direction_counts_are_observations(self):
        parser = monitor.StreamParser()
        capture = monitor.CaptureStats()
        packets = [payload(1, 50), payload(2, 100, delta_left=-3, delta_right=4),
                   payload(3, 150, delta_left=0, delta_right=0)]
        capture.consume(parser.feed(b"".join(frame(item) for item in packets)))
        self.assertEqual((capture.left.positive, capture.left.negative, capture.left.zero), (1, 1, 1))
        self.assertEqual(capture.left.observed_delta_sum, 8)
        self.assertEqual(capture.right.observed_delta_sum, -18)
        self.assertIn("不代表方向或精度已验收", capture.summary(parser.stats))


class CommandLineTests(unittest.TestCase):
    def test_live_path_is_receive_only_with_mocked_serial(self):
        instances = []

        class FakeSerial:
            def __init__(self, **settings):
                self.settings = settings
                self.port = None
                self.dtr = self.rts = True
                self.remaining = C_GOLDEN_FRAME
                self.closed = self.opened = False
                instances.append(self)

            @property
            def in_waiting(self):
                return len(self.remaining)

            def open(self):
                self.opened = True
                assert self.port == "COM_NOT_A_REAL_PORT"
                assert self.dtr == usb and not self.rts

            def read(self, count):
                data, self.remaining = self.remaining[:count], self.remaining[count:]
                return data

            def write(self, _data):
                raise AssertionError("encoder monitor must never transmit bytes")

            def close(self):
                self.closed = True

        for usb in (False, True):
            instances.clear()
            args = ["--port", "COM_NOT_A_REAL_PORT", "--seconds", "0.01"]
            if usb:
                args.append("--usb-cdc")
            with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=FakeSerial)}):
                with contextlib.redirect_stdout(io.StringIO()):
                    result = monitor.main(args)
            self.assertEqual(result, 0)
            self.assertEqual(len(instances), 1)
            self.assertTrue(instances[0].opened and instances[0].closed)
            self.assertFalse(instances[0].remaining)
            self.assertEqual(instances[0].settings["baudrate"], 115200)

    def test_offline_csv_and_raw_copy_use_only_standard_library(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "capture.bin"
            source.write_bytes(b"noise" + frame() + frame(payload(2, 100)))
            output_csv = root / "counts.csv"
            output_raw = root / "raw.bin"
            # -S excludes installed packages, proving offline replay needs no pyserial.
            result = subprocess.run(
                [sys.executable, "-S", str(Path(monitor.__file__)), "--input", str(source),
                 "--csv", str(output_csv), "--raw-output", str(output_raw)],
                capture_output=True, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(output_raw.read_bytes(), source.read_bytes())
            with output_csv.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual(int(rows[0]["total_right"]), -(1 << 40) - 22)
            self.assertEqual(rows[0]["left_b"], "1")

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.bin"
            source.write_bytes(frame())
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                result = monitor.main(["--input", str(source), "--raw-output", str(source)])
            self.assertEqual(result, 1)
            self.assertEqual(source.read_bytes(), frame())

    def test_no_port_default_and_empty_replay(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            monitor.arguments([])
        self.assertEqual(error.exception.code, 2)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "empty.bin"
            source.write_bytes(b"")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(monitor.main(["--input", str(source)]), 2)


class RealCFixtureTests(unittest.TestCase):
    def test_frozen_real_c_writer_frame_and_every_field(self):
        parser = monitor.StreamParser()
        capture = monitor.CaptureStats()
        samples = capture.consume(parser.feed(C_GOLDEN_FRAME))
        self.assertEqual(samples, [C_GOLDEN_SAMPLE])
        self.assertEqual(monitor.crc8(C_GOLDEN_FRAME[1:-2]), 0xCF)
        self.assertEqual(parser.stats.valid_frames, 1)
        self.assertEqual(parser.stats.discarded_bytes, 0)

    def test_vendored_lwpkt_fixture_when_requested(self):
        fixture = os.environ.get("ENCODER_LWPKT_FIXTURE")
        if fixture is None:
            self.skipTest("set ENCODER_LWPKT_FIXTURE to the real C writer output for interoperability")
        wire = Path(fixture).read_bytes()
        self.assertEqual(wire, C_GOLDEN_FRAME, "C writer output changed; review wire contract and fixture")
        parser = monitor.StreamParser()
        capture = monitor.CaptureStats()
        samples = []
        for offset in range(0, len(wire), 7):
            samples.extend(capture.consume(parser.feed(wire[offset:offset + 7])))
        samples.extend(capture.consume(parser.flush_incomplete()))
        self.assertEqual(samples, [C_GOLDEN_SAMPLE])
        self.assertEqual(parser.stats.crc_errors, 0)
        self.assertEqual(parser.stats.stop_errors, 0)
        self.assertEqual(parser.stats.length_errors, 0)
        self.assertEqual(capture.payload_errors, 0)
        self.assertFalse(parser.buffer)


if __name__ == "__main__":
    unittest.main()
