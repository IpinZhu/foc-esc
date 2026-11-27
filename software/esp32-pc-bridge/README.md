# ESP32-C3 PC / 电机桥接

使用原生 ESP-IDF 5.5.1 的 C++ 工程，不使用 PlatformIO/Arduino。Windows、Linux、macOS 共用 `tools/idf_driver.py`，Shell/PowerShell 入口仅负责传参；不需要预先进入 ESP-IDF 终端或定义会话函数。

## 固件角色

| Variant | 用途 |
|---|---|
| `receiver`（默认） | 旧 v1 接收器，通过原生 USB Serial/JTAG 输出 |
| `uart` | 旧 v1 接收器，通过 UART0 输出 |
| `mock` | 旧 v1 明文假遥测发送器，仅用于测试 |
| `pc` | v2 PC 双向桥接，用于当前上位机 |
| `motor` | v2 STM32 UART / ESP-NOW 双向桥接 |

当前真实链路需要两块 ESP32-C3：电脑 USB → `pc` → 加密 ESP-NOW → `motor` → STM32 UART。两块使用同一工程、不同角色的构建结果。v1 mock 数据不是真实测量值，也不能代替 v2 控制验证。

## 本地安装，不污染系统环境

前提：已有 Git，以及带 `venv/ensurepip` 的基础 Python 3.9–3.13；Linux 如缺少 Python venv 支持，需要使用者自行准备完整解释器。macOS 需要 Apple 命令行开发工具，Windows 原生 C++ host 测试需要 C++17 工具链，Linux 需要可用的 C++17 编译器。脚本不会运行 Homebrew、系统包管理器、全局 pip 或 `pip --user`，不会修改 Shell profile 或父进程 PATH。

从本目录执行：

```bash
# Linux/macOS；也能从 Fish/Zsh 直接调用，不需要 source
bash ./setup.sh
# 可明确指定现有基础 Python，不需要使用 PATH 中的默认 Python
bash ./setup.sh --python /usr/bin/python3
```

```powershell
# Windows PowerShell
.\setup.ps1
# 可选：明确指定已有 Python
.\setup.ps1 -Python C:\Python312\python.exe
```

SDK、编译器、CMake/Ninja、Python 虚拟环境、缓存和临时文件默认均放在 `.tools/`。后端本身可由较新的 Python 运行，但安装器会查找受支持的基础 Python；例如 PATH 中是 3.14 时，不会据此创建 IDF 3.14 环境。找不到合适解释器会报错，不自动改系统。

setup 固定官方 SDK 版本，检查已存在 checkout 的来源/tag 并复用；不会覆盖不匹配 SDK。虚拟环境由基础 Python 显式创建，避免 SDK 的 `pip --user virtualenv` 回退。Unix 下也显式安装项目内 CMake/Ninja。`.tools/idf-state.json` 记录 SDK、工具和实际虚拟环境路径，后续构建直接复用该环境的绝对解释器。

已有本项目安装会复用；存在多个环境时用共用入口的 `--python-env`，或 PowerShell `-PythonEnv` 指定。可显式复用外部 SDK，但不自动搜索全局安装：

```bash
bash ./build.sh --variant pc --idf-path /path/to/esp-idf --idf-tools-path /path/to/tools --python-env /path/to/venv
```

PowerShell 对应 `-IdfPath`、`-IdfToolsPath`、`-PythonEnv`。移动工程可能使 venv 和 CMake 缓存中的绝对路径失效；此时应按报错处理，不能假定环境可直接搬迁。

## 配置和构建

| 操作 | Linux/macOS | Windows PowerShell |
|---|---|---|
| 版本检查 | `bash ./build.sh --action version` | `.\build.ps1 -Action version` |
| PC 配置 | `bash ./build.sh --variant pc --action menuconfig` | `.\build.ps1 -Variant pc -Action menuconfig` |
| 电机端配置 | `bash ./build.sh --variant motor --action menuconfig` | `.\build.ps1 -Variant motor -Action menuconfig` |
| PC 构建 | `bash ./build.sh --variant pc` | `.\build.ps1 -Variant pc` |
| 电机端构建 | `bash ./build.sh --variant motor` | `.\build.ps1 -Variant motor` |
| host 测试 | `bash ./build.sh --action test` | `.\build.ps1 -Action test` |

共用入口也能直接运行：`python3 -I tools/idf_driver.py --variant pc`（Windows 使用 `python` 或 `py -3`）。驱动只依赖标准库。Shell 入口可用 `PYTHON` 选择运行驱动的解释器；安装所用基础解释器由 setup `--python` 单独指定。

**v2 没有默认控制密钥。** 在 `menuconfig → FOC PC bridge` 分别核对角色、对端 **station MAC**、相同频道和两端匹配的 16 字节 ASCII PMK/LMK。PMK 与 LMK 可以不同，但两端对应项必须一致。已有 sdkconfig 会保留，不会因重跑 build 而强制重设角色；在菜单中确认它与目标板用途一致。

配置为空时可编译，但固件拒绝初始化真实链路；CI 无密钥编译不是可运行无线配置。不要把真实密钥写入已跟踪的 `sdkconfig.*.defaults`。

输出：

```text
sdkconfig.pc                      # PC 本地配置，可能包含密钥
sdkconfig.motor                   # 电机端本地配置，可能包含密钥
build-pc/foc_pc_bridge.bin         # PC 应用固件
build-motor/foc_pc_bridge.bin      # 电机端应用固件
```

**两端应用文件同名但内容不同。** 编译生成的配置、头文件、日志和二进制可能含密钥，不应公开上传。`.tools/`、`.venv/`、`build*/` 和本地配置均被 Git 忽略。

## 显式烧录

普通 build/test/menuconfig 不会烧录或打开串口监视器。烧录必须同时指定 flash 和端口；先构建并确认应用产物，再由 IDF 使用对应 bootloader、分区表和应用烧录：

```bash
# macOS 示例；替换为实际端口
bash ./build.sh --variant pc --action flash --port /dev/cu.usbmodemXXXX
# Linux 示例
bash ./build.sh --variant motor --action flash --port /dev/ttyUSB0
```

```powershell
.\build.ps1 -Variant pc -Action flash -Port COM5
.\build.ps1 -Variant motor -Action flash -Port COM6
```

不自动选择端口，不提供整片擦除/解锁操作。烧录通常会复位并启动 ESP32。首次上电请先将 ESP32 与损坏或尚未验证的功率板分离，单独验证两端固件。

电机 UART1 默认 GPIO4 TX → STM32 PB11 RX，GPIO5 RX ← STM32 PB10 TX，115200 8N1、3.3 V 逻辑并共地。原生 USB 使用 GPIO18/19；以实际板卡接线为准。

## 旧 `idf-local.sh` 命令兼容

保留版本查询及受控的 build/menuconfig/带端口 flash，例如：

```bash
bash ./idf-local.sh --version
bash ./idf-local.sh -B build-motor menuconfig
bash ./idf-local.sh -B build-motor build
```

原来的角色对应 `-DIDF_TARGET`、`-DSDKCONFIG`、`-DSDKCONFIG_DEFAULTS` 参数仍可使用，但必须与选择的角色匹配。此入口不再任意透传 IDF 动作，`erase-flash` 等会被拒绝。优先使用 `build.sh`/`build.ps1`，不要再依赖终端里旧的 `idf_local` 函数。

## 测试及此前问题

`--action test` 运行 C++ 接口测试、USB 验证器单元测试和编排回归测试，不需要设备；无 SDK 时可使用现有 CMake/CTest 和 C++17 编译器。有 SDK 时优先使用其本地工具。Windows host 测试可在已配置 MSVC 的 Developer PowerShell 中执行；多配置生成器统一使用 Release。

此前问题及修复：

| 现象 | 原因与处理 |
|---|---|
| 安装用 3.9，构建寻找 `py3.14_env` | `export.sh`/`export.ps1` 从 PATH 重新选 Python；现在持久化并复用虚拟环境，不再自动重选 |
| `No module named python_version_checker` | 直接给 SDK 脚本加 `-I` 会屏蔽同目录导入；仅项目自身标准库驱动和测试使用隔离模式，SDK 入口正常执行并清除外部 PYTHONPATH/PYTHONHOME |
| `Desktop`/`desktop` 后提示缺少其他芯片工具链 | IDF 按路径字符串标识安装；现在按文件身份复用安装记录中的路径拼写 |
| `Checking python3 ... 3.14` 仍出现 | 终端残留旧函数；直接运行文件入口，不依赖当前会话状态 |
| Unix 找不到 Ninja/CMake | 它们在 IDF 中是按需工具；setup 现在明确安装到本工程 |

三系统 CI 配置了共用驱动/平台入口测试、STM32 三传感器构建和 ESP 双角色编译；CI 不烧录硬件、不上传可能含密钥的产物。脚本支持不等于三系统实板验收。

当前未完成 PC—ESP32—STM32 整机板级验收；加密转发和协议单元测试也不等同于完整安全审计、无线覆盖或断线故障响应验证。
