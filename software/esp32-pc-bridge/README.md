# ESP32-C3 PC / 电机桥接

使用原生 ESP-IDF 的 C++ 工程，不使用 PlatformIO 或 Arduino。提供旧 v1 遥测测试链路，以及 v2 PC/电机端串口与加密 ESP-NOW 双向桥接。

## 固件角色

| Variant | 用途 |
|---|---|
| `receiver`（默认） | 旧 v1 接收器，通过原生 USB Serial/JTAG 输出 |
| `uart` | 旧 v1 接收器，通过 UART0 输出 |
| `mock` | 旧 v1 明文假遥测发送器，仅用于测试 |
| `pc` | v2 PC 双向桥接，用于当前上位机 |
| `motor` | v2 STM32 UART / ESP-NOW 双向桥接 |

v1 mock 数据不是真实电机测量值，也不能代替 v2 控制链路验证。

## 环境与构建

以下命令在本目录的普通 PowerShell 中执行。脚本支持现有 ESP-IDF 安装；本地安装脚本使用 ESP-IDF 5.5.1。

```powershell
.\setup.ps1
.\build.ps1 -Variant pc
.\build.ps1 -Variant motor
```

`setup.ps1` 会安装本地工具链；已有 SDK 时可传入 `-IdfPath`，必要时指定 `-IdfToolsPath`。`build` 只构建，不烧录、不连接电机。

v2 没有默认控制密钥。使用 `menuconfig` 分别配置两端的对端 station MAC、相同频道以及匹配的 16 字节 ASCII PMK/LMK：

```powershell
.\build.ps1 -Variant pc -Action menuconfig
.\build.ps1 -Variant motor -Action menuconfig
```

配置为空时，v2 固件拒绝初始化真实链路。`sdkconfig.pc`、`sdkconfig.motor` 等本地配置被 Git 忽略，不得把真实密钥写入已跟踪的 `sdkconfig.*.defaults`。编译生成的配置和二进制也可能包含密钥，不应公开分发。

默认电机 UART1 为 GPIO4 TX、GPIO5 RX，115200 8N1；连接 STM32 的 PB11 RX、PB10 TX，使用 3.3 V 电平并共地。原生 USB 使用 GPIO18/19。以上是代码配置，不替代实际板卡引脚核对。

烧录是单独动作，须显式指定端口；本仓库的构建与测试不自动执行烧录。

## 主机测试

需要 CMake、CTest、C++17 编译器和 Python。脚本会使用本目录 `.venv`，不存在时创建；主机测试不需要 ESP-IDF SDK：

```powershell
.\build.ps1 -Action test
```

其中 C++ 接口测试也可单独运行：

```powershell
cmake -S test -B build-host
cmake --build build-host --config Release
ctest --test-dir build-host -C Release --output-on-failure
```

多配置生成器需要选择对应配置；Windows 可按本机编译器选择 CMake generator。

## 验证边界

- `main/wire_v2.hpp` 是共享 Rust 协议 `../foc-wire` 的 C++ 对应实现。
- 桥接包含帧检查、双向转发、加密单播和 HMAC 校验，但这些实现不等同于完成安全审计。
- 主机协议测试、畸形包测试和固件编译不能证明无线覆盖、断线恢复或电机故障响应已经通过实机验证。
- `.tools/`、`.venv/`、`build*/` 和含本地设置的 `sdkconfig.*` 不纳入版本控制。
- 当前未完成 PC—ESP32—STM32 整机板级验收。
