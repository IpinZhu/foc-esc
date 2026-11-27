import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('idf_driver', SOURCE / 'tools/idf_driver.py')
driver = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(driver)


class BuildScriptTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='foc-test-')
        self.addCleanup(temporary.cleanup)
        self.root = (Path(temporary.name) / "bridge fixture's").resolve()
        self.sdk = self.root / 'SDK with spaces'
        (self.sdk / 'tools').mkdir(parents=True)
        for name in ('idf.py', 'idf_tools.py'):
            (self.sdk / 'tools' / name).touch()
        self.tools = self.root / '.tools/espressif'
        self.environment = self.tools / 'python_env/idf5.5_py3.12_env'
        self.python = driver.python_in(self.environment)
        self.python.parent.mkdir(parents=True)
        self.python.touch()
        self.bridge = driver.Bridge(self.root, self.sdk, self.tools)
        self.calls = []
        self.generate_binary = True
        self.mock = patch.object(driver, 'run', side_effect=self.run_command)
        self.mock.start()
        self.addCleanup(self.mock.stop)

    def run_command(self, args, cwd, env, capture=False):
        args = [str(a) for a in args]
        self.calls.append((args, Path(cwd), env.copy()))
        if 'export' in args:
            return 'PATH=' + str(self.root / 'tools-bin') + os.pathsep + '$PATH\nTOKEN=a=b\n'
        if '-I' in args:
            return json.dumps(sys.executable)
        if args[:1] == ['git']:
            if 'origin' in args:
                return driver.ORIGIN + '\n'
            if 'describe' in args:
                return driver.VERSION + '\n'
        if args[-1] == 'build' and self.generate_binary:
            build = Path(args[args.index('-B') + 1])
            build.mkdir(parents=True, exist_ok=True)
            (build / 'foc_pc_bridge.bin').write_bytes(b'TEST ONLY')
        return ''

    def idf_calls(self):
        return [args for args, _, _ in self.calls if any(a.endswith('/idf.py') or a.endswith('\\idf.py') for a in args)]

    def test_each_variant_has_separate_config_and_build_without_flash(self):
        for variant in driver.VARIANTS:
            with self.subTest(variant=variant):
                self.bridge.execute('build', variant)
                args = self.idf_calls()[-1]
                self.assertEqual(args[0], str(self.python))
                self.assertNotIn('-I', args)  # SDK scripts import adjacent modules.
                self.assertEqual(args[-1], 'build')
                self.assertIn(str(self.root / f'build-{variant}'), args)
                self.assertIn(f'-DSDKCONFIG={self.root / ("sdkconfig." + variant)}', args)
                expected = str(self.root / 'sdkconfig.defaults')
                if variant != 'receiver':
                    expected += ';' + str(self.root / f'sdkconfig.{variant}.defaults')
                self.assertIn('-DSDKCONFIG_DEFAULTS=' + expected, args)
        self.assertFalse(any('flash' in args or 'monitor' in args for args in self.idf_calls()))

    def test_missing_port_rejected_before_any_sdk_call(self):
        with self.assertRaisesRegex(driver.BuildError, 'explicit'):
            self.bridge.execute('flash', 'motor')
        self.assertEqual(self.calls, [])
        with patch.object(driver, 'Bridge') as bridge, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(driver.main(['--action', 'flash']), 1)
            bridge.assert_not_called()

    def test_flash_builds_first_and_forwards_exact_port(self):
        self.bridge.execute('flash', 'motor', '/dev/cu.usbmodem-test')
        self.assertEqual([x[-1] for x in self.idf_calls()], ['build', 'flash'])
        self.assertEqual(self.idf_calls()[-1][-3:], ['-p', '/dev/cu.usbmodem-test', 'flash'])

    def test_missing_artifact_prevents_flash(self):
        self.generate_binary = False
        with self.assertRaisesRegex(driver.BuildError, 'not generated'):
            self.bridge.execute('flash', port='COM9')
        self.assertEqual([x[-1] for x in self.idf_calls()], ['build'])

    def test_failed_sdk_command_prevents_flash(self):
        with patch.object(driver, 'run', side_effect=driver.BuildError('exit code 7')):
            with self.assertRaisesRegex(driver.BuildError, 'exit code 7'):
                self.bridge.execute('flash', port='COM9')
        self.assertEqual(self.idf_calls(), [])

    def test_menuconfig_and_version_do_not_build(self):
        self.bridge.execute('menuconfig', 'pc')
        self.bridge.execute('version')
        self.assertEqual([x[-1] for x in self.idf_calls()], ['menuconfig', '--version'])

    def test_activation_reuses_python_despite_foreign_path_and_env(self):
        with patch.dict(os.environ, {'PYTHONPATH': '/foreign', 'PYTHONHOME': '/foreign',
                                    'IDF_PYTHON_ENV_PATH': '/foreign',
                                    'PIP_TARGET': '/foreign', 'PIP_CONFIG_FILE': '/foreign'}):
            before = os.environ.copy()
            env = self.bridge.activated()
            self.assertEqual(os.environ, before)
        self.assertNotIn('PYTHONPATH', env)
        self.assertNotIn('PYTHONHOME', env)
        self.assertNotIn('PIP_TARGET', env)
        self.assertEqual(env['PIP_CONFIG_FILE'], os.devnull)
        self.assertEqual(env['PIP_REQUIRE_VIRTUALENV'], '1')
        self.assertEqual(env['IDF_PYTHON_ENV_PATH'], str(self.environment))
        self.assertEqual(env['TOKEN'], 'a=b')
        self.assertNotIn('$PATH', env['PATH'])
        for key in ('TMP', 'TEMP', 'TMPDIR', 'PIP_CACHE_DIR', 'CCACHE_DIR', 'XDG_CACHE_HOME'):
            self.assertTrue(Path(env[key]).is_relative_to(self.root / '.tools'))
        self.assertTrue(all(args[0] == str(self.python) for args, _, _ in self.calls))

    def test_no_shell_evaluation_of_environment(self):
        env = driver.exported_environment('PATH=/local:$PATH\nVALUE=$(do-not-run)\n', {'PATH': '/old'})
        self.assertEqual(env['VALUE'], '$(do-not-run)')
        self.assertEqual(env['PATH'], '/local:/old')
        with self.assertRaises(driver.BuildError):
            driver.exported_environment('not an assignment', {})

    def test_state_reuses_environment_not_named_for_current_python(self):
        self.bridge.save_state()
        other = driver.Bridge(self.root, self.sdk, self.tools)
        self.assertEqual(other.environment, self.environment)
        self.assertEqual(other.python(), self.python)

    def test_sdk_metadata_reuses_same_file_spelling(self):
        alias = self.root / 'SDK alias'
        try:
            alias.symlink_to(self.sdk, target_is_directory=True)
        except OSError:
            self.skipTest('Symlinks unavailable to this account')
        (self.tools / 'idf-env.json').write_text(json.dumps({'idfInstalled': {
            'unrelated': {'path': str(self.root / 'absent')},
            'installed': {'path': str(self.sdk)}}}))
        self.assertEqual(driver.sdk_spelling(alias, self.tools), self.sdk)

    def test_windows_and_unix_python_locations(self):
        self.assertEqual(driver.python_in(self.root, True), self.root / 'Scripts/python.exe')
        self.assertEqual(driver.python_in(self.root, False), self.root / 'bin/python')

    def test_multiple_environments_require_explicit_selection(self):
        extra = driver.python_in(self.tools / 'python_env/idf5.5_py3.11_env')
        extra.parent.mkdir(parents=True)
        extra.touch()
        with self.assertRaisesRegex(driver.BuildError, 'Multiple'):
            driver.Bridge(self.root, self.sdk, self.tools)
        self.assertEqual(driver.Bridge(self.root, self.sdk, self.tools, self.environment).python(), self.python)

    def test_setup_reuses_environment_and_installs_local_cmake_ninja(self):
        with patch.object(self.bridge, 'select_python', return_value='/base/python'):
            self.bridge.setup()
            self.bridge.setup()
        self.assertFalse(any('-m' in args and 'venv' in args for args, _, _ in self.calls))
        self.assertTrue(any(args[-2:] == ['cmake', 'ninja'] for args, _, _ in self.calls))
        self.assertFalse(any('--user' in args or '--reinstall' in args for args, _, _ in self.calls))
        self.assertTrue(self.bridge.state_file.is_file())

    def test_missing_venv_support_fails_before_git_or_install(self):
        with patch.object(driver, 'run', side_effect=driver.BuildError('missing ensurepip')):
            with self.assertRaisesRegex(driver.BuildError, 'venv/ensurepip'):
                self.bridge.setup('/unsupported/python')
        self.assertEqual(self.calls, [])

    def test_setup_refuses_wrong_sdk_without_overwriting(self):
        original = self.run_command
        def wrong(args, *rest, **kwargs):
            if 'origin' in args:
                return 'https://invalid.example/sdk.git'
            return original(args, *rest, **kwargs)
        with patch.object(self.bridge, 'select_python', return_value='/base/python'), patch.object(driver, 'run', side_effect=wrong):
            with self.assertRaisesRegex(driver.BuildError, 'Refusing'):
                self.bridge.setup()
        self.assertFalse(any('install' in args for args, _, _ in self.calls))

    def test_raw_compatibility_checks_role_and_forbids_erase(self):
        args = ['-B', 'build-motor', '-DIDF_TARGET=esp32c3',
                f'-DSDKCONFIG={self.root / "sdkconfig.motor"}', 'menuconfig']
        self.assertEqual(driver.raw_arguments(args, self.bridge), ('menuconfig', 'motor', None))
        self.assertEqual(driver.raw_arguments(['--version'], self.bridge), ('version', 'receiver', None))
        for args in (['erase-flash'], ['-B', '/outside', 'build'], ['-DOTHER=x', 'build']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                driver.raw_arguments(args, self.bridge)

    def test_windows_host_generator_does_not_assume_ninja_initializes_msvc(self):
        with patch.object(driver.shutil, 'which', side_effect=lambda name, **kw: 'ninja' if name == 'ninja' else None):
            self.assertEqual(driver.host_generator({}, windows=True), [])
            self.assertEqual(driver.host_generator({'CXX': 'cl'}, windows=True), ['-G', 'Ninja'])
            self.assertEqual(driver.host_generator({}, windows=False), ['-G', 'Ninja'])
            self.assertEqual(driver.host_generator({}, cached=True), [])

    def test_multiconfig_host_tests_use_release(self):
        with patch.object(driver.shutil, 'which', side_effect=lambda name, **kw: name):
            self.bridge.host_tests()
        self.assertTrue(any(args[1:3] == ['--build', str(self.root / 'build-host-script')]
                            and args[-2:] == ['--config', 'Release'] for args, _, _ in self.calls))
        self.assertTrue(any('--output-on-failure' in args and '-C' in args and 'Release' in args for args, _, _ in self.calls))


class EntryPointTests(unittest.TestCase):
    def test_shell_help_and_flash_gate_from_other_directory(self):
        bash = shutil.which('bash')
        if not bash:
            self.skipTest('Bash not installed')
        env = os.environ.copy()
        env['PYTHON'] = sys.executable
        for script, args, status in [('build.sh', ['--help'], 0), ('setup.sh', ['--help'], 0),
                                     ('build.sh', ['--action', 'flash'], 1)]:
            result = subprocess.run([bash, str(SOURCE / script)] + args, cwd=tempfile.gettempdir(),
                                    env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, status, result.stdout + result.stderr)
            if status:
                self.assertIn('explicit', result.stderr)

    def test_powershell_help_and_flash_gate(self):
        shell = shutil.which('pwsh') or shutil.which('powershell')
        if not shell:
            self.skipTest('PowerShell not installed')
        for args, status in [(['-Help'], 0), (['-Action', 'flash'], 1)]:
            result = subprocess.run([shell, '-NoProfile', '-File', str(SOURCE / 'build.ps1')] + args,
                                    cwd=tempfile.gettempdir(), capture_output=True, text=True)
            self.assertEqual(result.returncode, status, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
