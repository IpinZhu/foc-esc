"""Build/test STM32 firmware without hardware access; flash is explicit and probe-selected."""
import argparse
import os
from pathlib import Path
import struct
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TARGET = 'thumbv7em-none-eabihf'
FEATURES = {'encoder': 'sensor-encoder', 'hall': 'sensor-hall', 'sensorless': 'sensorless'}
FLASH_START = 0x08000000
PARAMETER_START = 0x0803C000
FLASH_END = 0x08040000


class BuildError(RuntimeError):
    pass


def run(args, cwd, env, capture=False):
    try:
        result = subprocess.run([str(x) for x in args], cwd=str(cwd), env=env,
                                text=True, encoding='utf-8', errors='replace',
                                stdout=subprocess.PIPE if capture else None,
                                stderr=subprocess.PIPE if capture else None)
    except OSError as exc:
        raise BuildError(f'Cannot run {args[0]}: {exc}') from exc
    if result.returncode:
        details = (result.stdout or '') + (result.stderr or '') if capture else ''
        raise BuildError(f'{args[0]} failed with exit code {result.returncode}\n{details}')
    return result.stdout or ''


def check_elf(path):
    """Reject invalid images or ELF load data that could program reserved parameter pages."""
    data = Path(path).read_bytes()
    if len(data) < 52 or data[:7] != b'\x7fELF\x01\x01\x01':
        raise BuildError('Firmware is not a little-endian ELF32 image')
    if struct.unpack_from('<H', data, 18)[0] != 40:
        raise BuildError('Firmware is not an ARM ELF')
    offset = struct.unpack_from('<I', data, 28)[0]
    entry_size, count = struct.unpack_from('<HH', data, 42)
    if entry_size < 32 or offset + entry_size * count > len(data):
        raise BuildError('Invalid ELF program headers')
    flash_data = False
    for i in range(count):
        kind, file_offset, _, physical, size, _, _, _ = struct.unpack_from('<8I', data, offset + i * entry_size)
        if kind != 1 or not size:
            continue
        if file_offset + size > len(data):
            raise BuildError('Truncated ELF load segment')
        end = physical + size
        if physical < FLASH_START or end > PARAMETER_START:
            raise BuildError('ELF load data is outside application Flash or overlaps reserved parameter pages')
        flash_data = True
    if not flash_data:
        raise BuildError('ELF has no application Flash load data; check firmware Cargo linker configuration')


class Firmware:
    def __init__(self, root=ROOT, sensor='encoder'):
        self.root = Path(root).resolve()
        self.sensor = sensor
        self.feature = FEATURES[sensor]
        self.target_dir = self.root / 'target' / sensor
        self.elf = self.target_dir / TARGET / 'release/foc-firmware'
        self.env = os.environ.copy()
        # Do not let inherited probe presets or erase/unlock overrides affect explicit flashing.
        for key in list(self.env):
            if key.startswith('PROBE_RS_'):
                self.env.pop(key)

    def cargo(self, action, host=False):
        target = TARGET
        if host:
            info = run(['rustc', '-vV'], self.root, self.env, True)
            target = next((line[6:].strip() for line in info.splitlines() if line.startswith('host: ')), '')
            if not target:
                raise BuildError('Could not determine native target from rustc -vV')
        directory = self.root / 'target/host' / self.sensor if host else self.target_dir
        args = ['cargo', action, '--locked', '--target', target, '--target-dir', directory,
                '--no-default-features', '--features', self.feature]
        if not host:
            args += ['--release', '--bin', 'foc-firmware']
        if action == 'clippy':
            if host:
                args += ['--lib', '--tests']
            args += ['--', '-D', 'warnings']
        return args

    def execute(self, action, probe=None, speed=None, under_reset=False):
        if action == 'flash' and (not probe or not probe.strip()):
            raise BuildError('Flashing requires an explicit --probe (-Probe). No device was changed.')
        if action == 'fmt':
            run(['cargo', 'fmt', '--', '--check'], self.root, self.env)
        elif action in ('test', 'host-clippy'):
            run(self.cargo('test' if action == 'test' else 'clippy', host=True), self.root, self.env)
        elif action == 'clippy':
            run(self.cargo('clippy'), self.root, self.env)
        else:
            # cwd is essential: --manifest-path alone does not load .cargo/config.toml.
            run(self.cargo('build'), self.root, self.env)
            if not self.elf.is_file():
                raise BuildError(f'Build produced no ELF: {self.elf}')
            check_elf(self.elf)
            print(f'Firmware ({self.sensor}): {self.elf}')
            if action == 'flash':
                command = ['probe-rs', 'download', '--chip', 'STM32G473RC', '--protocol', 'swd',
                           '--non-interactive', '--probe', probe, '--verify']
                if speed:
                    command += ['--speed', str(speed)]
                if under_reset:
                    command += ['--connect-under-reset']
                run(command + [self.elf], self.root, self.env)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=('build', 'test', 'clippy', 'host-clippy', 'fmt', 'flash'), default='build')
    parser.add_argument('--sensor', choices=FEATURES, default='encoder')
    parser.add_argument('--probe', help='Explicit VID:PID[:SERIAL] from probe-rs list')
    parser.add_argument('--speed', type=int, help='SWD clock in kHz')
    parser.add_argument('--connect-under-reset', action='store_true', help='Requires NRST wiring')
    args = parser.parse_args(argv)
    if args.speed is not None and args.speed <= 0:
        parser.error('--speed must be positive')
    try:
        Firmware(sensor=args.sensor).execute(args.action, args.probe, args.speed, args.connect_under_reset)
        return 0
    except (BuildError, OSError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
