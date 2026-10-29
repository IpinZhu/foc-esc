import json
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest


SOURCE = pathlib.Path(__file__).resolve().parents[1]
SHELL = shutil.which("pwsh") or shutil.which("powershell")


@unittest.skipUnless(SHELL, "PowerShell is required")
class BuildScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="foc-build-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = pathlib.Path(self.temp.name) / "bridge fixture's"
        (self.root / "tools").mkdir(parents=True)
        self.sdk = self.root / "SDK fixture's"
        (self.sdk / "tools").mkdir(parents=True)
        shutil.copy2(SOURCE / "build.ps1", self.root)
        shutil.copy2(SOURCE / "tools/invoke_idf.ps1", self.root / "tools")
        python_env = sys.prefix.replace("'", "''")
        (self.sdk / "export.ps1").write_text(
            f"$env:IDF_PYTHON_ENV_PATH = '{python_env}'\n$global:LASTEXITCODE = 0\n",
            encoding="utf-8",
        )
        (self.sdk / "tools/idf.py").write_text(
            "import json, pathlib, sys\n"
            "args = sys.argv[1:]\n"
            "project = pathlib.Path(args[args.index('-C') + 1])\n"
            "(project / 'arguments.json').write_text(json.dumps(args))\n"
            "if (project / 'fail').exists(): sys.exit(7)\n"
            "if args[-1] == 'build':\n"
            "    build = pathlib.Path(args[args.index('-B') + 1])\n"
            "    build.mkdir(parents=True, exist_ok=True)\n"
            "    (build / 'foc_pc_bridge.bin').write_bytes(b'TEST FIXTURE, NOT FIRMWARE')\n",
            encoding="utf-8",
        )

    def run_build(self, *args):
        return subprocess.run(
            [SHELL, "-NoProfile", "-File", str(self.root / "build.ps1"),
             "-IdfPath", str(self.sdk), *args],
            capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=30,
        )

    def arguments(self):
        return json.loads((self.root / "arguments.json").read_text())

    def test_variant_paths_and_defaults(self):
        for variant in ("receiver", "mock", "uart", "pc", "motor"):
            with self.subTest(variant=variant):
                result = self.run_build("-Variant", variant)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                args = self.arguments()
                self.assertEqual(args[-1], "build")
                self.assertIn("-DIDF_TARGET=esp32c3", args)
                self.assertEqual(args[args.index("-B") + 1], str(self.root / f"build-{variant}"))
                self.assertIn(f"-DSDKCONFIG={self.root / f'sdkconfig.{variant}'}", args)
                defaults = str(self.root / "sdkconfig.defaults")
                if variant != "receiver":
                    defaults += ";" + str(self.root / f"sdkconfig.{variant}.defaults")
                self.assertIn(f"-DSDKCONFIG_DEFAULTS={defaults}", args)
                self.assertNotIn("flash", args)

    def test_missing_flash_port_fails_before_sdk(self):
        result = self.run_build("-Action", "flash")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / "arguments.json").exists())
        self.assertIn("explicit -Port", result.stdout + result.stderr)

    def test_idf_failure_is_not_success(self):
        (self.root / "fail").touch()
        result = self.run_build()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("exit code 7", result.stdout + result.stderr)

    def test_menuconfig_action(self):
        result = self.run_build("-Variant", "uart", "-Action", "menuconfig")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.arguments()[-1], "menuconfig")

    def test_invalid_sdk_path(self):
        (self.sdk / "export.ps1").unlink()
        result = self.run_build()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SDK not found", result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
