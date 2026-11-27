import contextlib
import importlib.util
import io
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('firmware', SOURCE / 'tools/firmware.py')
driver = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(driver)


def elf(address=driver.FLASH_START, length=16):
    data = bytearray(84 + length)
    data[:7] = b'\x7fELF\x01\x01\x01'
    struct.pack_into('<H', data, 18, 40)
    struct.pack_into('<I', data, 28, 52)
    struct.pack_into('<HH', data, 42, 32, 1)
    struct.pack_into('<8I', data, 52, 1, 84, address, address, length, length, 5, 4)
    return data


class FirmwareTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='foc-firmware-')
        self.addCleanup(temporary.cleanup)
        self.root = (Path(temporary.name) / "firmware fixture's").resolve()
        self.root.mkdir()
        self.calls = []
        self.mock = patch.object(driver, 'run', side_effect=self.run_command)
        self.mock.start()
        self.addCleanup(self.mock.stop)
        self.firmware = driver.Firmware(self.root, 'sensorless')

    def run_command(self, args, cwd, env, capture=False):
        self.calls.append(([str(a) for a in args], Path(cwd), env.copy()))
        if args[:2] == ['rustc', '-vV']:
            return 'rustc 1.93.1\nhost: aarch64-apple-darwin\n'
        if args[:2] == ['cargo', 'build']:
            target = Path(args[args.index('--target-dir') + 1])
            output = target / driver.TARGET / 'release/foc-firmware'
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_bytes(elf())
        return ''

    def test_build_features_are_exclusive_and_outputs_are_isolated(self):
        for sensor, feature in driver.FEATURES.items():
            firmware = driver.Firmware(self.root, sensor)
            firmware.execute('build')
            args, cwd, _ = self.calls[-1]
            self.assertEqual(cwd, self.root)
            self.assertEqual(args[:2], ['cargo', 'build'])
            self.assertIn('--locked', args)
            self.assertIn('--release', args)
            self.assertIn('--no-default-features', args)
            self.assertEqual(args[args.index('--features') + 1], feature)
            self.assertEqual(args[args.index('--target-dir') + 1], str(self.root / 'target' / sensor))
            self.assertTrue(firmware.elf.is_file())
        self.assertFalse(any(args[0] == 'probe-rs' for args, _, _ in self.calls))

    def test_host_tests_override_arm_target(self):
        self.firmware.execute('test')
        args, _, _ = self.calls[-1]
        self.assertEqual(args[:2], ['cargo', 'test'])
        self.assertEqual(args[args.index('--target') + 1], 'aarch64-apple-darwin')
        self.assertNotIn('--release', args)

    def test_clippy_and_format_are_not_flash_actions(self):
        for action in ('clippy', 'host-clippy', 'fmt'):
            self.firmware.execute(action)
        self.assertTrue(all(args[0] != 'probe-rs' for args, _, _ in self.calls))
        self.assertEqual(self.calls[-1][0], ['cargo', 'fmt', '--', '--check'])
        self.assertTrue(any(args[-3:] == ['--', '-D', 'warnings'] for args, _, _ in self.calls))

    def test_flash_without_probe_does_not_build_or_touch_hardware(self):
        with self.assertRaisesRegex(driver.BuildError, 'explicit'):
            self.firmware.execute('flash')
        self.assertEqual(self.calls, [])

    def test_flash_rebuilds_selected_sensor_and_never_mass_erases(self):
        self.firmware.execute('flash', '1234:5678:serial', 100, True)
        self.assertEqual([args[0] for args, _, _ in self.calls], ['cargo', 'probe-rs'])
        args, _, _ = self.calls[-1]
        self.assertEqual(args[:2], ['probe-rs', 'download'])
        self.assertEqual(args[-1], str(self.firmware.elf))
        self.assertIn('--non-interactive', args)
        self.assertIn('--verify', args)
        self.assertIn('--connect-under-reset', args)
        self.assertEqual(args[args.index('--speed') + 1], '100')
        self.assertNotIn('--chip-erase', args)
        self.assertNotIn('--allow-erase-all', args)

    def test_probe_environment_cannot_enable_erase_or_presets(self):
        with patch.dict(os.environ, {'PROBE_RS_ALLOW_ERASE_ALL': 'true', 'PROBE_RS_CONFIG_PRESET': 'unsafe'}):
            before = os.environ.copy()
            firmware = driver.Firmware(self.root)
            self.assertEqual(os.environ, before)
        self.assertFalse(any(k.startswith('PROBE_RS_') for k in firmware.env))

    def test_failed_build_prevents_flash_even_if_stale_artifact_exists(self):
        self.firmware.elf.parent.mkdir(parents=True)
        self.firmware.elf.write_bytes(elf())
        with patch.object(driver, 'run', side_effect=driver.BuildError('exit code 7')):
            with self.assertRaisesRegex(driver.BuildError, 'exit code 7'):
                self.firmware.execute('flash', '1234:5678')
        self.assertEqual(self.calls, [])

    def test_elf_parameter_pages_and_bad_linking_are_rejected(self):
        image = self.root / 'image.elf'
        image.write_bytes(elf())
        driver.check_elf(image)
        for data in (b'\x7fE', elf(driver.PARAMETER_START), elf(driver.PARAMETER_START - 8, 16),
                     elf(0x20000000), elf(driver.FLASH_END), elf()[:60]):
            image.write_bytes(data)
            with self.assertRaises(driver.BuildError):
                driver.check_elf(image)

    def test_missing_elf_prevents_flash(self):
        with patch.object(driver, 'run', return_value=''):
            with self.assertRaisesRegex(driver.BuildError, 'no ELF'):
                self.firmware.execute('flash', '1234:5678')

    def test_invalid_sensor_and_speed_rejected(self):
        for args in (['--sensor', 'unknown'], ['--speed', '0']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                driver.main(args)

    def test_shell_wrapper_help_and_gate(self):
        bash = shutil.which('bash')
        if not bash:
            self.skipTest('Bash not installed')
        env = os.environ.copy()
        env['PYTHON'] = sys.executable
        for args, code in [(['--help'], 0), (['--action', 'flash'], 1)]:
            result = subprocess.run([bash, str(SOURCE / 'build.sh')] + args, cwd=tempfile.gettempdir(),
                                    env=env, text=True, capture_output=True)
            self.assertEqual(result.returncode, code, result.stdout + result.stderr)
            if code:
                self.assertIn('explicit', result.stderr)

    def test_powershell_wrapper_help_and_gate(self):
        shell = shutil.which('pwsh') or shutil.which('powershell')
        if not shell:
            self.skipTest('PowerShell not installed')
        for args, code in [(['-Help'], 0), (['-Action', 'flash'], 1)]:
            result = subprocess.run([shell, '-NoProfile', '-File', str(SOURCE / 'build.ps1')] + args,
                                    cwd=tempfile.gettempdir(), text=True, capture_output=True)
            self.assertEqual(result.returncode, code, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
