"""Project-local ESP-IDF orchestration. Requires only Python's standard library."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

VERSION = 'v5.5.1'
ORIGIN = 'https://github.com/espressif/esp-idf.git'
VARIANTS = ('receiver', 'uart', 'mock', 'pc', 'motor')
PROJECT = Path(__file__).resolve().parents[1]


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
        raise BuildError(f'{Path(args[0]).name} failed with exit code {result.returncode}\n{details}')
    return result.stdout or ''


def physical(path):
    path = Path(path).expanduser().absolute()
    if sys.platform == 'darwin' and path.is_dir():
        # Bash pwd and Path.resolve need not correct APFS case aliases.
        return Path(subprocess.check_output(['/bin/pwd', '-P'], cwd=str(path), text=True).strip())
    return path.resolve()


def same_path(left, right):
    try:
        return os.path.samefile(left, right)
    except OSError:
        return physical(left) == physical(right)


def python_in(environment, windows=None):
    if windows is None:
        windows = os.name == 'nt'
    return Path(environment) / ('Scripts/python.exe' if windows else 'bin/python')


def sdk_spelling(sdk, tools):
    metadata = Path(tools) / 'idf-env.json'
    if metadata.is_file():
        try:
            records = json.loads(metadata.read_text(encoding='utf-8')).get('idfInstalled', {})
            for record in records.values():
                saved = record.get('path')
                if saved and same_path(saved, sdk):
                    return Path(saved)
        except (ValueError, AttributeError) as exc:
            raise BuildError(f'Invalid SDK metadata: {metadata}') from exc
    return physical(sdk)


def local_env(project, sdk, tools, python_env=None):
    env = os.environ.copy()
    for key in ('PYTHONHOME', 'PYTHONPATH', 'IDF_PYTHON_ENV_PATH', 'IDF_PATH', 'IDF_TOOLS_PATH',
                'IDF_DEACTIVATE_FILE_PATH', 'IDF_TOOLS_EXPORT_CMD', 'IDF_TOOLS_INSTALL_CMD',
                'PIP_TARGET', 'PIP_PREFIX', 'PIP_USER', 'PYTHONUSERBASE'):
        env.pop(key, None)
    local = project / '.tools'
    for name, directory in (('PIP_CACHE_DIR', 'cache/pip'), ('XDG_CACHE_HOME', 'cache'),
                            ('CCACHE_DIR', 'cache/ccache'), ('TMPDIR', 'tmp'),
                            ('TEMP', 'tmp'), ('TMP', 'tmp')):
        location = local / directory
        location.mkdir(parents=True, exist_ok=True)
        env[name] = str(location)
    env.update(IDF_PATH=str(sdk), IDF_TOOLS_PATH=str(tools), PYTHONNOUSERSITE='1',
               PIP_DISABLE_PIP_VERSION_CHECK='1', PIP_CONFIG_FILE=os.devnull,
               PIP_REQUIRE_VIRTUALENV='1', PYTHONUTF8='1', PYTHONIOENCODING='utf-8')
    if python_env:
        env['IDF_PYTHON_ENV_PATH'] = str(python_env)
    return env


def exported_environment(output, env):
    result = env.copy()
    for line in output.splitlines():
        if not line.strip():
            continue
        key, separator, value = line.partition('=')
        if not separator or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', key):
            raise BuildError('Invalid key-value output from ESP-IDF environment export')
        if key == 'PATH':
            for placeholder in ('$PATH', '${PATH}', '%PATH%'):
                value = value.replace(placeholder, env.get('PATH', ''))
        result[key] = value
    return result


def host_generator(env, cached=False, windows=None):
    if cached:
        return []
    if windows is None:
        windows = os.name == 'nt'
    lookup = lambda name: shutil.which(name, path=env.get('PATH'))
    # Ninja alone does not initialize MSVC. Otherwise let CMake discover Visual Studio.
    if lookup('ninja') and (not windows or env.get('CXX') or lookup('cl')):
        return ['-G', 'Ninja']
    if windows and lookup('mingw32-make') and lookup('g++'):
        return ['-G', 'MinGW Makefiles', '-DCMAKE_CXX_COMPILER=' + lookup('g++')]
    return []


class Bridge:
    def __init__(self, project=PROJECT, idf_path=None, tools_path=None, python_env=None):
        self.project = physical(project)
        self.tools = physical(tools_path or self.project / '.tools/espressif')
        self.sdk = sdk_spelling(idf_path or self.project / f'.tools/esp-idf-{VERSION}', self.tools)
        self.state_file = self.project / '.tools/idf-state.json'
        self.environment = Path(python_env).expanduser().absolute() if python_env else None
        if not self.environment and self.state_file.is_file():
            try:
                state = json.loads(self.state_file.read_text(encoding='utf-8'))
                if same_path(state['sdk'], self.sdk) and same_path(state['tools'], self.tools):
                    self.environment = Path(state['python_env'])
            except (ValueError, KeyError, TypeError) as exc:
                raise BuildError('Invalid local IDF state; repair .tools/idf-state.json') from exc
        if not self.environment:
            candidates = sorted((self.tools / 'python_env').glob('idf5.5*_env'))
            candidates = [p for p in candidates if python_in(p).is_file()]
            if len(candidates) > 1:
                raise BuildError('Multiple local Python environments; select --python-env explicitly')
            if candidates:
                self.environment = candidates[0]

    def env(self):
        return local_env(self.project, self.sdk, self.tools, self.environment)

    def sdk_check(self):
        if not all((self.sdk / 'tools' / name).is_file() for name in ('idf.py', 'idf_tools.py')):
            raise BuildError('ESP-IDF SDK not found. Run setup.sh/setup.ps1 first.')

    def python(self):
        if not self.environment or not python_in(self.environment).is_file():
            raise BuildError('Local IDF Python environment missing. Run setup.sh/setup.ps1 first.')
        return python_in(self.environment)

    def save_state(self):
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        # No SDK configs or keys are read or copied into orchestration state.
        data = dict(sdk=str(self.sdk), tools=str(self.tools), python_env=str(self.environment))
        temporary = self.state_file.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
        temporary.replace(self.state_file)

    def select_python(self, explicit=None):
        commands = [[explicit]] if explicit else []
        if not explicit:
            if self.environment:
                base = run([self.python(), '-I', '-c', 'import sys; print(sys._base_executable)'],
                           self.project, self.env(), capture=True).strip()
                commands.append([base])
            commands += [[sys.executable], ['/usr/bin/python3']]
            commands += [[f'python3.{minor}'] for minor in range(13, 8, -1)]
            commands += [['python3'], ['python']]
            if os.name == 'nt':
                commands += [['py', f'-3.{minor}'] for minor in range(13, 8, -1)]
        probe = ('import sys, json, venv, ensurepip; '
                 'assert sys.prefix == sys.base_prefix; '
                 'assert (3, 9) <= sys.version_info[:2] < (3, 14); '
                 'print(json.dumps(sys.executable))')
        for command in commands:
            try:
                value = run(command + ['-I', '-c', probe], self.project, self.env(), capture=True)
                return json.loads(value)
            except (BuildError, ValueError):
                continue
        raise BuildError('Need an existing base Python 3.9–3.13 with venv/ensurepip. '
                         'Use --python /path/to/python (PowerShell: -Python). '
                         'No global/user packages were installed.')

    def setup(self, explicit_python=None):
        base = self.select_python(explicit_python)
        env = self.env()
        if self.sdk.exists():
            origin = run(['git', '-C', self.sdk, 'remote', 'get-url', 'origin'], self.project, env, True).strip()
            tag = run(['git', '-C', self.sdk, 'describe', '--tags', '--exact-match', 'HEAD'], self.project, env, True).strip()
            if origin.rstrip('/') != ORIGIN or tag != VERSION:
                raise BuildError(f'Refusing to modify existing SDK: expected official {VERSION}')
            run(['git', '-C', self.sdk, 'submodule', 'update', '--init', '--recursive', '--depth', '1'],
                self.project, env)
        else:
            self.sdk.parent.mkdir(parents=True, exist_ok=True)
            run(['git', 'clone', '--depth', '1', '--branch', VERSION, '--recurse-submodules',
                 '--shallow-submodules', ORIGIN, self.sdk], self.project, env)
        self.sdk_check()
        if not self.environment:
            self.environment = self.tools / 'python_env/foc-idf5.5'
        env = self.env()
        if not python_in(self.environment).is_file():
            if self.environment.exists():
                raise BuildError(f'Incomplete Python environment: {self.environment}; not overwritten')
            # Create it ourselves so SDK cannot fall back to pip install --user virtualenv.
            run([base, '-m', 'venv', self.environment], self.project, env)
        run([self.python(), '-m', 'pip', '--version'], self.project, env)
        if os.name != 'nt':
            run([self.python(), '-c', 'import curses'], self.project, env)
        tool = self.sdk / 'tools/idf_tools.py'
        run([base, tool, 'install', '--targets=esp32c3'], self.project, env)
        run([base, tool, 'install', '--targets=esp32c3', 'cmake', 'ninja'], self.project, env)
        run([base, tool, 'install-python-env'], self.project, env)
        self.save_state()
        self.activated()
        print('Project-local ESP-IDF ready; no shell profile or global packages changed.')

    def activated(self):
        self.sdk_check()
        python = self.python()
        tool = self.sdk / 'tools/idf_tools.py'
        env = self.env()
        run([python, tool, 'check-python-dependencies'], self.project, env)
        output = run([python, tool, 'export', '--format', 'key-value'], self.project, env, True)
        return exported_environment(output, env)

    def arguments(self, variant):
        defaults = [self.project / 'sdkconfig.defaults']
        if variant != 'receiver':
            defaults.append(self.project / f'sdkconfig.{variant}.defaults')
        return ['-C', str(self.project), '-B', str(self.project / f'build-{variant}'),
                '-DIDF_TARGET=esp32c3', f'-DSDKCONFIG={self.project / ("sdkconfig." + variant)}',
                '-DSDKCONFIG_DEFAULTS=' + ';'.join(str(p) for p in defaults)]

    def execute(self, action, variant='receiver', port=None):
        if action == 'flash' and (not port or not port.strip()):
            raise BuildError('Flashing requires an explicit --port (-Port). No device was changed.')
        env = self.activated()
        command = [self.python(), self.sdk / 'tools/idf.py']
        if action == 'version':
            run(command + ['--version'], self.project, env)
            return
        args = self.arguments(variant)
        if action == 'menuconfig':
            run(command + args + ['menuconfig'], self.project, env)
            return
        run(command + args + ['build'], self.project, env)
        binary = self.project / f'build-{variant}/foc_pc_bridge.bin'
        if not binary.is_file():
            raise BuildError(f'Build returned success but firmware was not generated: {binary}')
        print(f'Firmware ({variant}): {binary}')
        if action == 'flash':
            run(command + args + ['-p', port, 'flash'], self.project, env)

    def host_tests(self):
        env = self.env()
        # SDK is optional. If installed, use its local CMake/Ninja.
        if self.environment and (self.sdk / 'tools/idf_tools.py').is_file():
            env = self.activated()
        cmake = shutil.which('cmake', path=env.get('PATH'))
        ctest = shutil.which('ctest', path=env.get('PATH'))
        if not cmake or not ctest:
            raise BuildError('Host tests require CMake/CTest and a C++17 compiler; setup installs local CMake.')
        build = self.project / 'build-host-script'
        args = [cmake, '-S', self.project / 'test', '-B', build]
        args += host_generator(env, (build / 'CMakeCache.txt').exists())
        run(args, self.project, env)
        run([cmake, '--build', build, '--config', 'Release'], self.project, env)
        run([ctest, '--test-dir', build, '-C', 'Release', '--output-on-failure'], self.project, env)
        for script in ('test_verify_usb.py', 'test_build_scripts.py'):
            run([sys.executable, '-I', self.project / 'tools' / script], self.project, env)


def raw_arguments(argv, bridge):
    parser = argparse.ArgumentParser(description='Restricted idf-local.sh compatibility interface')
    parser.add_argument('--version', action='store_true')
    parser.add_argument('-B', default='build-receiver')
    parser.add_argument('-D', action='append', default=[])
    parser.add_argument('-p', '--port')
    parser.add_argument('action', nargs='?', choices=('build', 'menuconfig', 'flash'))
    args = parser.parse_args(argv)
    if args.version:
        if args.action or args.D or args.port or args.B != 'build-receiver':
            parser.error('--version must be used alone')
        return 'version', 'receiver', None
    variant = Path(args.B).name.removeprefix('build-')
    if variant not in VARIANTS or not same_path(bridge.project / args.B, bridge.project / f'build-{variant}'):
        parser.error('-B must select an existing role name: build-pc, build-motor, etc.')
    expected = dict(x[2:].split('=', 1) for x in bridge.arguments(variant) if x.startswith('-D'))
    for definition in args.D:
        key, separator, value = definition.partition('=')
        if not separator or key not in expected:
            parser.error('Only IDF_TARGET, SDKCONFIG and SDKCONFIG_DEFAULTS are supported')
        paths = value.split(';')
        wanted = expected[key].split(';')
        matches = value == expected[key] if key == 'IDF_TARGET' else (
            len(paths) == len(wanted) and all(same_path(bridge.project / a, b) for a, b in zip(paths, wanted)))
        if not matches:
            parser.error('Configuration must match the selected role; use build.sh for custom SDK paths')
    if not args.action:
        parser.error('Choose build, menuconfig or flash')
    return args.action, variant, args.port


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=('setup', 'build', 'menuconfig', 'flash', 'test', 'version'), default='build')
    parser.add_argument('--variant', choices=VARIANTS, default='receiver')
    parser.add_argument('--port')
    parser.add_argument('--python')
    parser.add_argument('--idf-path')
    parser.add_argument('--idf-tools-path')
    parser.add_argument('--python-env')
    parser.add_argument('--project', type=Path, default=PROJECT, help=argparse.SUPPRESS)
    try:
        if argv and argv[0] == 'raw':
            bridge = Bridge()
            action, variant, port = raw_arguments(argv[1:], bridge)
            bridge.execute(action, variant, port)
            return 0
        args = parser.parse_args(argv)
        if args.action == 'flash' and (not args.port or not args.port.strip()):
            raise BuildError('Flashing requires an explicit --port (-Port). No device was changed.')
        bridge = Bridge(args.project, args.idf_path, args.idf_tools_path, args.python_env)
        if args.action == 'setup':
            bridge.setup(args.python)
        elif args.action == 'test':
            bridge.host_tests()
        else:
            bridge.execute(args.action, args.variant, args.port)
        return 0
    except (BuildError, OSError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
