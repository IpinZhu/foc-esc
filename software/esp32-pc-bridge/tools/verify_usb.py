import argparse
import math
import re
import sys
import time

U32 = 1 << 32
MAX_LINE = 224
MAX_STARTUP_LINES = 32
OBSERVE_SECONDS = 0.25


class VerificationError(Exception):
    pass


class TelemetryVerifier:
    def __init__(self, samples, allow_gaps=False):
        if samples < 1:
            raise ValueError("samples must be positive")
        self.samples = samples
        self.allow_gaps = allow_gaps
        self.valid_count = 0
        self.gaps = 0
        self.last_sequence = None
        self._buffer = bytearray()
        self._discard_initial = True
        self._started = False
        self._startup_lines = 0

    def feed(self, data):
        accepted = 0
        for byte in data:
            if self._discard_initial:
                if byte == 10:
                    self._discard_initial = False
                continue
            if byte == 10:
                line = bytes(self._buffer)
                self._buffer.clear()
                if not line.endswith(b"\r"):
                    accepted += self._invalid_line()
                    continue
                accepted += self._line(line[:-1])
                continue
            self._buffer.append(byte)
            if len(self._buffer) >= MAX_LINE:
                raise VerificationError("line exceeds maximum length")
        return accepted

    def _line(self, line):
        expected_prefix = re.fullmatch(rb"seq=(0|[1-9][0-9]*) state=1 faults=00000000 ia=1\.25 ib=-0\.50 ic=-0\.75 id=0\.00 iq=1\.50 vbus=24\.00 vel=12\.50 temp=35\.0", line)
        if expected_prefix is None:
            return self._invalid_line()
        sequence = int(expected_prefix.group(1))
        if sequence >= U32:
            return self._invalid_line()
        if not self._started:
            self._started = True
        else:
            delta = (sequence - self.last_sequence) % U32
            if delta == 0 or delta >= (1 << 31) or delta % 100:
                raise VerificationError(f"duplicate or out-of-order ADC sequence {sequence}")
            if delta != 100:
                if not self.allow_gaps:
                    raise VerificationError(f"ADC sequence gap: expected {(self.last_sequence + 100) % U32}, got {sequence}")
                self.gaps += delta // 100 - 1
        self.last_sequence = sequence
        self.valid_count += 1
        return 1

    def _invalid_line(self):
        if self._started:
            raise VerificationError("invalid telemetry line after synchronization")
        self._startup_lines += 1
        if self._startup_lines > MAX_STARTUP_LINES:
            raise VerificationError("too many startup noise lines")
        return 0

    def check_timeout(self, now, deadline):
        if self.valid_count < self.samples and now >= deadline:
            raise VerificationError(f"timed out after {self.valid_count}/{self.samples} samples")

    @property
    def at_line_boundary(self):
        return not self._buffer

    def finish(self):
        if self._buffer:
            raise VerificationError("stream ended with an incomplete line")


def _list_ports():
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise VerificationError("pyserial is required for --list-ports") from exc
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found")
    for port in ports:
        print(f"{port.device}\t{port.description}")


def verify_port(port, samples, timeout, allow_gaps):
    try:
        import serial
    except ImportError as exc:
        raise VerificationError("pyserial is required to read a serial port") from exc

    verifier = TelemetryVerifier(samples, allow_gaps)
    deadline = time.monotonic() + timeout
    observe_until = None
    with serial.Serial(port=None, baudrate=115200, timeout=0.1,
                       rtscts=False, dsrdtr=False) as connection:
        connection.dtr = False
        connection.rts = False
        connection.port = port
        connection.open()
        while True:
            now = time.monotonic()
            verifier.check_timeout(now, deadline)
            if verifier.valid_count >= samples:
                if observe_until is None:
                    observe_until = min(deadline, now + OBSERVE_SECONDS)
                if now >= observe_until and verifier.at_line_boundary:
                    break
            remaining = deadline - now
            if remaining <= 0:
                verifier.check_timeout(now, deadline)
                verifier.finish()
                break
            connection.timeout = min(0.1, remaining)
            chunk = connection.read_until(b"\n", size=MAX_LINE + 1)
            if chunk:
                verifier.feed(chunk)
    return verifier


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate raw FOC USB telemetry")
    parser.add_argument("--port")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--allow-gaps", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args(argv)
    if args.list_ports:
        try:
            _list_ports()
            return 0
        except VerificationError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 1
    if not args.port:
        parser.error("--port is required unless --list-ports is used")
    if args.samples < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--samples must be positive and --timeout must be finite and positive")
    try:
        result = verify_port(args.port, args.samples, args.timeout, args.allow_gaps)
    except (VerificationError, OSError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"PASS: {result.valid_count} valid samples, {result.gaps} missing samples")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
