# STM32 固件构建与烧录

目标为 STM32G473RC，Rust 2024 + Embassy。使用 stable Rust；当前验证工具链为 1.93.1。不需要 nightly。电机控制配置保持 100 kHz PWM、200 ns 死区、纯软件保护，尚不等同于实板验收。

## 前置条件

Windows、Linux、macOS 都需要已有的 Rust/Cargo、Python 3.9+。ARM 构建需要 `thumbv7em-none-eabihf` 标准库；lint 需要 rustfmt/clippy。烧录额外需要 probe-rs、对应调试器驱动/USB 访问权限。脚本不会自动安装或更改全局工具。

如这些 Rust 组件尚未安装，可由使用者明确执行：

```text
rustup target add thumbv7em-none-eabihf
rustup component add rustfmt clippy
```

## 构建、测试

下列命令在本目录执行；入口也支持从其他工作目录使用完整路径调用。

| 操作 | Linux/macOS（Bash，也可从 Zsh/Fish 调用） | Windows PowerShell |
|---|---|---|
| 编码器 release 构建 | `bash ./build.sh --sensor encoder` | `.\build.ps1 -Sensor encoder` |
| Hall release 构建 | `bash ./build.sh --sensor hall` | `.\build.ps1 -Sensor hall` |
| 无感 release 构建 | `bash ./build.sh --sensor sensorless` | `.\build.ps1 -Sensor sensorless` |
| 本机测试 | `bash ./build.sh --sensor sensorless --action test` | `.\build.ps1 -Sensor sensorless -Action test` |
| ARM Clippy | `bash ./build.sh --sensor sensorless --action clippy` | `.\build.ps1 -Sensor sensorless -Action clippy` |
| 本机 Clippy | `bash ./build.sh --action host-clippy` | `.\build.ps1 -Action host-clippy` |
| 格式检查 | `bash ./build.sh --action fmt` | `.\build.ps1 -Action fmt` |

也可以直接使用三平台共用入口，例如：

```text
python3 -I tools/firmware.py --sensor sensorless --action build
```

Windows 可使用 `python` 或 `py -3` 替代 `python3`。Shell 入口支持 `PYTHON` 环境变量选择运行驱动的解释器，不会向该解释器安装包。

每次明确启用且仅启用一个 sensor feature，使用 `--locked`；脚本从固件目录运行 Cargo，确保加载 `.cargo/config.toml` 中的 `link.x`/`defmt.x`。仅从仓库根目录传 `--manifest-path` 不足以加载此配置。本机测试目标自动读取 `rustc -vV`，不会硬编码 Windows target 或误用 ARM target。

产物按传感器隔离：

```text
target/encoder/thumbv7em-none-eabihf/release/foc-firmware
target/hall/thumbv7em-none-eabihf/release/foc-firmware
target/sensorless/thumbv7em-none-eabihf/release/foc-firmware
```

脚本显式传 `--target-dir`，不使用外部 `CARGO_TARGET_DIR`，以保证烧录选择与产物路径一致。

## 显式烧录

先用 `probe-rs list` 查询调试器标识。烧录必须同时指定 `flash` 和具体 `VID:PID[:SERIAL]`，例如将下面的占位值替换成自己的调试器：

```bash
bash ./build.sh --sensor sensorless --action flash --probe VID:PID:SERIAL --speed 100
```

```powershell
.\build.ps1 -Sensor sensorless -Action flash -Probe VID:PID:SERIAL -Speed 100
```

需要复位下连接时再显式添加 `--connect-under-reset` / `-ConnectUnderReset`，并连接 NRST。烧录动作先重新构建所选传感器版本，成功并检查 ELF 后才调用 `probe-rs download --verify`，不自动启动 RTT/monitor。不要将调试器的电压参考脚当作供电输出。

**保留参数：** `memory.x` 将程序限制在 `0x08000000..0x0803C000`，末尾四页 `0x0803C000..0x08040000` 用于电机记录和通用参数。脚本检查 ELF load data，不提供整片擦除或解锁选项，并清除继承的 probe-rs 环境预设。只使用正常应用区编程；不能把这当作已经完成了实际探针/Flash 算法的保留参数验收。

`cargo run` 仍遵循既有 runner，会烧录并运行，**不是普通构建命令**。上述构建/测试脚本不会调用它。

## 无感与首次上电

无感是实验性功能，目前需要先通过编码器/Hall 对同一台电机整定，保存电机记录，再保留记录切换无感固件；缺少或不兼容记录时拒绝使能。首次检查须断开电机、妥善隔离功率级，使用正确供电及限流手段。软件停用不是硬件急停。

SWD 的 `Target device did not respond` 表示连接未建立，可能是供电、接线、复位、速度或硬件问题，不代表应该擦除芯片。UCC27211 冒烟属于硬件故障，未经测量不能归因于 10 V、电源能力或焊接；构建脚本修复不代表驱动板已修复。
