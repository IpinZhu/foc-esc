import contextlib
import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


_MODULE_PATH = Path(__file__).with_name("verify_usb.py")
_SPEC = importlib.util.spec_from_file_location("verify_usb", _MODULE_PATH)
verify_usb = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(verify_usb)


class TelemetryVerifierTests(unittest.TestCase):
    def setUp(self):
        self.verifier = verify_usb.TelemetryVerifier(2)
        self.first = b"seq=0 state=1 faults=00000000 ia=1.25 ib=-0.50 ic=-0.75 id=0.00 iq=1.50 vbus=24.00 vel=12.50 temp=35.0\r\n"
        self.second = self.first.replace(b"seq=0 ", b"seq=100 ")

    def start(self, verifier):
        verifier.feed(b"opening residue\r\n")

    def test_valid_stream_at_every_split_and_coalesced(self):
        stream = b"opening residue\r\n" + self.first + self.second
        for split in range(len(stream) + 1):
            verifier = verify_usb.TelemetryVerifier(2)
            self.start(verifier)
            verifier.feed(stream[:split])
            verifier.feed(stream[split:])
            self.assertEqual(verifier.valid_count, 2, split)
        verifier = verify_usb.TelemetryVerifier(2)
        self.start(verifier)
        self.assertEqual(verifier.feed(stream), 2)
        self.assertEqual(verifier.last_sequence, 100)

    def test_initial_partial_line_and_startup_noise(self):
        verifier = verify_usb.TelemetryVerifier(1)
        verifier.feed(b"partial ROM residue\r\nbooting\r\n" + self.first)
        self.assertEqual(verifier.valid_count, 1)

    def test_duplicate_and_out_of_order_fail(self):
        for second_sequence in (0, 0xFFFFFF9C):
            verifier = verify_usb.TelemetryVerifier(3)
            self.start(verifier)
            verifier.feed(self.first)
            line = self.first.replace(b"seq=0 ", f"seq={second_sequence} ".encode())
            with self.assertRaises(verify_usb.VerificationError):
                verifier.feed(line)

    def test_missing_sample_fails_by_default(self):
        verifier = verify_usb.TelemetryVerifier(2)
        self.start(verifier)
        verifier.feed(self.first)
        with self.assertRaisesRegex(verify_usb.VerificationError, "gap"):
            verifier.feed(self.first.replace(b"seq=0 ", b"seq=200 "))

    def test_allow_gaps_counts_missing_samples(self):
        verifier = verify_usb.TelemetryVerifier(2, allow_gaps=True)
        self.start(verifier)
        verifier.feed(self.first)
        verifier.feed(self.first.replace(b"seq=0 ", b"seq=300 "))
        self.assertEqual(verifier.gaps, 2)
        self.assertEqual(verifier.valid_count, 2)

    def test_uint32_wraparound(self):
        verifier = verify_usb.TelemetryVerifier(2)
        self.start(verifier)
        verifier.feed(self.first.replace(b"seq=0 ", b"seq=4294967196 "))
        verifier.feed(self.first)
        self.assertEqual(verifier.valid_count, 2)
        self.assertEqual(verifier.last_sequence, 0)

    def test_missing_fields_and_wrong_values_fail_after_sync(self):
        verifier = verify_usb.TelemetryVerifier(2)
        verifier.feed(self.first)
        for bad_line in (
            self.second.replace(b" faults=00000000", b""),
            self.second.replace(b"ia=1.25", b"ia=nan"),
            self.second.replace(b"state=1", b"state=2"),
            self.second[:-2] + b"\n",
        ):
            with self.subTest(line=bad_line):
                current = verify_usb.TelemetryVerifier(2)
                self.start(current)
                current.feed(self.first)
                with self.assertRaises(verify_usb.VerificationError):
                    current.feed(bad_line)

    def test_overlong_line_fails(self):
        verifier = verify_usb.TelemetryVerifier(1)
        self.start(verifier)
        with self.assertRaisesRegex(verify_usb.VerificationError, "maximum length"):
            verifier.feed(b"x" * (verify_usb.MAX_LINE + 1))

    def test_timeout_before_expected_samples(self):
        verifier = verify_usb.TelemetryVerifier(1)
        with self.assertRaisesRegex(verify_usb.VerificationError, "timed out"):
            verifier.check_timeout(10.0, 10.0)
        self.start(verifier)
        verifier.feed(self.first)
        verifier.check_timeout(10.0, 10.0)

    def test_startup_noise_has_a_limit(self):
        verifier = verify_usb.TelemetryVerifier(1)
        self.start(verifier)
        for _ in range(verify_usb.MAX_STARTUP_LINES):
            verifier.feed(b"noise\r\n")
        with self.assertRaisesRegex(verify_usb.VerificationError, "startup noise"):
            verifier.feed(b"noise\r\n")

    def test_finish_requires_complete_line(self):
        self.start(self.verifier)
        self.verifier.feed(self.first + self.second[:50])
        self.assertFalse(self.verifier.at_line_boundary)
        with self.assertRaises(verify_usb.VerificationError):
            self.verifier.finish()
        self.verifier.feed(self.second[50:])
        self.assertTrue(self.verifier.at_line_boundary)
        self.verifier.finish()

    def test_nonfinite_timeout_rejected(self):
        for value in ("nan", "inf", "-inf"):
            with contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    verify_usb.main(["--port", "COM_TEST", f"--timeout={value}"])
            self.assertEqual(result.exception.code, 2)

    def test_port_loop_waits_for_partial_line(self):
        clock = [0.0]
        chunks = iter([b"initial residue\r\n", self.first, self.second[:50], b"", self.second[50:]])
        test = self

        class FakeSerial:
            def __init__(self, **kwargs):
                test.assertIsNone(kwargs["port"])
                test.assertFalse(kwargs["rtscts"])
                test.assertFalse(kwargs["dsrdtr"])

            def __enter__(self):
                return self

            def __exit__(self, *args):
                pass

            def open(self):
                test.assertFalse(self.dtr)
                test.assertFalse(self.rts)

            def read_until(self, *args, **kwargs):
                clock[0] += 0.2
                return next(chunks, b"")

        with patch.dict("sys.modules", {"serial": SimpleNamespace(Serial=FakeSerial)}):
            with patch.object(verify_usb.time, "monotonic", lambda: clock[0]):
                result = verify_usb.verify_port("COM_TEST", 1, 2.0, False)
        self.assertEqual(result.valid_count, 2)


if __name__ == "__main__":
    unittest.main()
