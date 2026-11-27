# STM32G473 三相 FOC 驱动器

基于 **STM32G473RC** 的实验性三相 PMSM/BLDC 磁场定向控制（FOC）平台。仓库涵盖主驱动板与扩展板 KiCad 硬件设计、Rust 2024 + Embassy 实时固件、物理闭环仿真器、ESP32-C3 加密无线桥接以及 Tauri 桌面调试上位机。

> **安全提示：** 本项目为大电流电力电子工程原型，固件与 PCB **尚未完成板级电气、热性能、EMI 及故障关断鉴定**。在通过限流电源与示波器验证栅极波形、死区时间、采样极性、ADC 时序和保护动作前，请勿连接电机或电池。当前固件仅启用**纯软件保护**（未启用 `COMP → DAC → TIM1-BRK` 硬件过流链），软件停用不能替代硬件急停。

---

## 模块总览与当前状态

| 模块 | 路径 | 职责 | 当前验证状态 |
|---|---|---|---|
| **STM32 固件** | [`software/firmware`](software/firmware) | STM32G473RC 无堆实时 FOC、Flash 参数双槽持久化、Auto-Tune、UART/CAN 通信 | 主机单测/集成测试与三类后端 ARM 编译通过；**未板级实测** |
| **物理仿真器** | [`software/foc-sim`](software/foc-sim/README.md) | 独立 `f64` PMSM 电气/机械闭环模型，直接驱动固件 `FocController` 与 `AutoTuneController` | M1 有感基线、物理耦合自整定与实验性无感闭环回归通过；M2 非理想矩阵待完善 |
| **桌面上位机** | [`software/host-app`](software/host-app/README.md) | Vue 3 + Tauri 2 调试工作台：遥测监控、43 项参数管理、控制权租约、录制导出与自动运行日志 | 前端构建与 Rust 工作区测试通过；**未整机实板联调** |
| **无线桥接** | [`software/esp32-pc-bridge`](software/esp32-pc-bridge/README.md) | 原生 ESP-IDF C++ 工程：PC 端与电机端串口 ↔ 加密 ESP-NOW 双向桥接 | 主机协议测试与固件构建脚本就绪；**未整机实板联调** |
| **共享线协议** | [`software/foc-wire`](software/foc-wire) | 固件与上位机共用的 `no_std` v2 帧编解码、CRC 与控制权租约定义 | 随固件与上位机测试一并验证 |
| **驱动板硬件** | [`hardware/foc-drive`](hardware/foc-drive) / [`hardware/foc-expansion`](hardware/foc-expansion) | 主功率驱动板与扩展板 KiCad 原理图和 PCB（另含 [`foc/foc.ioc`](foc/foc.ioc) CubeMX 参考） | 工程设计阶段；待实板测量验证 |

---

## 系统架构

### 1. 无线调试与控制链路

```text
  PC 上位机 (FOC Studio)
      │ USB 串口 (foc-wire v2)
      ▼
  ESP32-C3 (pc 桥接)  ◀══ 加密单播 ESP-NOW (PMK/LMK + HMAC) ══▶  ESP32-C3 (motor 桥接)
                                                                        │ UART (115200 8N1)
                                                                        ▼
                                                                 STM32G473RC (USART3)
```

- 上位机真实连接要求 PC 桥接握手并确认 `secure=1`，随后验证 STM32 设备身份与能力。
- 连接不等于取得控制权：必须显式申请控制权租约并维持心跳，链路超时自动触发停用请求。
- 上位机内置的浏览器/桌面模拟器仅用于界面与会话验证，**不是**物理电机仿真（物理仿真请使用 `software/foc-sim`）。

### 2. STM32 板级控制闭环

```text
  Encoder / Hall / 无感估角 ──▶ ┌───────────────────────┐ ──▶ TIM1 互补 PWM (100 kHz, 200 ns*)
  三相电流 (INA240A2 ×3) ─────▶ │     STM32G473RC       │ ──▶ UCC27211 ×3 → 070N10NS 三相桥
  VBUS / TMP235 ──────────────▶ │   Rust 2024 + Embassy │
  USART3 (PB10/PB11) ─────────▶ │  100 kHz 电流 / 1 kHz 速度 │
  FDCAN1 (PA11/PA12) ─────────▶ └───────────────────────┘
```

- 高速控制路径与通信任务解耦：TIM1 TRGO2 触发注入 ADC 采样唤醒电机控制任务；UART 与 CAN 任务通过有界 Embassy 通道交换命令和遥测，不直接操作 PWM。
- 关键默认参数：100 kHz 中心对齐互补 PWM、200 ns 死区（170 MHz 定时器时钟）、5%–95% 占空比限幅、100 kHz 电流环、1 kHz 速度环。
- 纯软件保护：过流阈值与去抖、相电流残差、母线欠压/过压、热模型与 TMP235 过温、堵转、传感器有效性与控制超时故障锁存。
- Flash 双槽持久化：带版本号与 CRC，支持 UART `param ...` 与 CAN 参数帧读写/保存/恢复默认；`autotune start` 可执行 Rs/L/磁链辨识、电流环 PI 阶跃验证及观测器参数生成并落盘。

---

## 角度反馈模式（编译期三选一）

通过 Cargo feature 在编译期互斥选择：

| Feature | 反馈方式 | 硬件引脚 | 定位与状态 |
|---|---|---|---|
| `sensor-encoder`（默认） | TIM3 正交编码器（QEI） | PB4 / PB5 | 主力有感后端；支持全套模式与自整定 |
| `sensor-hall` | TIM3 Hall 输入捕获 + 速度插值 | PC6 / PC7 / PC8 | 可选有感后端；支持全套模式与自整定 |
| `sensorless`（实验性） | SPMSM 反电动势观测器 + PLL | 无（不使用 TIM3） | 实验性无感后端；**仅主机物理仿真验证** |

Windows、Linux、macOS 共用标准库 Python 构建后端，提供 PowerShell 和 Bash 入口。ESP-IDF、编译工具、Python 依赖和缓存默认放在工程 `.tools/`，不修改全局环境。完整步骤见 [STM32 工具说明](software/firmware/README.md) 和 [ESP32 工具说明](software/esp32-pc-bridge/README.md)。

以下分别在对应固件目录执行：

| 操作 | Linux / macOS | Windows PowerShell |
|---|---|---|
| STM32 编码器构建 | `bash ./build.sh --sensor encoder` | `.\build.ps1 -Sensor encoder` |
| STM32 Hall 构建 | `bash ./build.sh --sensor hall` | `.\build.ps1 -Sensor hall` |
| STM32 无感构建 | `bash ./build.sh --sensor sensorless` | `.\build.ps1 -Sensor sensorless` |
| STM32 本机测试 | `bash ./build.sh --action test` | `.\build.ps1 -Action test` |
| ESP32 本地 SDK 安装 | `bash ./setup.sh` | `.\setup.ps1` |
| ESP32 PC 角色构建 | `bash ./build.sh --variant pc` | `.\build.ps1 -Variant pc` |
| ESP32 电机角色构建 | `bash ./build.sh --variant motor` | `.\build.ps1 -Variant motor` |

普通构建不接触硬件。烧录是单独显式操作，STM32 必须指定 probe，ESP32 必须指定串口；具体命令见各工具说明。STM32 各传感器产物独立，脚本检查 ELF 应用区并禁止整片擦除路径，保留切换无感所需的参数页。

脚本从 STM32 固件目录加载 Cargo 配置，本机测试目标自动检测，不再硬编码 Windows target。保留原生 Cargo 操作时，从仓库根目录构建必须明确加载 `software/firmware/.cargo/config.toml`，只传 `--manifest-path` 不足以保证加载链接脚本。

详见 [`software/foc-sim/README.md`](software/foc-sim/README.md) 与 [`docs/sensorless-simulation-plan.md`](docs/sensorless-simulation-plan.md)。

### 3. 启动上位机（`software/host-app`）

在 `software/host-app` 目录执行：

```powershell
npm ci
npm run dev              # 浏览器预览（仅内置模拟器）
npm run tauri -- dev     # 桌面开发模式（支持串口扫描、C3 加密桥接与自动日志）
npm run build            # 前端类型检查与生产构建
cargo test --locked --workspace
```

详见 [`software/host-app/README.md`](software/host-app/README.md)。

### 4. 构建 ESP32-C3 无线桥接（`software/esp32-pc-bridge`）

在 `software/esp32-pc-bridge` 目录执行（原生 ESP-IDF，不使用 PlatformIO/Arduino）：

```powershell
.\setup.ps1                              # 安装本地 ESP-IDF 工具链（已有 SDK 可传 -IdfPath）
.\build.ps1 -Variant pc -Action menuconfig
.\build.ps1 -Variant motor -Action menuconfig
.\build.ps1 -Variant pc
.\build.ps1 -Variant motor
.\build.ps1 -Action test                 # 主机 C++/Python 协议测试（无需 ESP-IDF）
```

详见 [`software/esp32-pc-bridge/README.md`](software/esp32-pc-bridge/README.md)。

---

## 硬件概要

主驱动板（[`hardware/foc-drive/FOC2.kicad_pro`](hardware/foc-drive/FOC2.kicad_pro)）与扩展板（[`hardware/foc-expansion/foc-ex.kicad_pro`](hardware/foc-expansion/foc-ex.kicad_pro)）关键器件与设计目标如下：

| 单元 | 器件 / 规格目标 |
|---|---|
| 主控 MCU | STM32G473RC（170 MHz Cortex-M4F） |
| 栅极驱动 | UCC27211 半桥驱动 ×3 |
| 功率级 | 100 V 级 N-MOSFET（070N10NS 级）×6 组成三相桥 |
| 电流采样 | INA240A2 双向电流放大器 ×3 + 0.5 mΩ Kelvin 分流电阻 ×3 |
| 母线与传感 | 2S–6S 电池目标（约 5.5–25.2 V）、VBUS 分压采样、TMP235 模拟温度传感器 |
| 通信与接口 | USART3（PB10/PB11）、FDCAN1（PA11/PA12）、SWD、编码器（PB4/PB5）、Hall（PC6–PC8）、扩展接口 |

器件额定值、热限制、自举尺寸与开关损耗在实板测量前均为工程目标；[`foc/foc.ioc`](foc/foc.ioc) 仅作 CubeMX 引脚参考，运行时以 Rust 固件源码为准。本地详细计算文档有意不纳入仓库。

---

## 上板前验证清单

软件测试和主机仿真无法证明功率级安全。首次上电与带电机运行前须逐项完成：

1. **装联与绝缘检查**：核对器件极性、焊接、爬电间距、接地及分流电阻 Kelvin 走线。
2. **无功率级门极检查**：功率母线不上电，示波器确认六路互补 PWM 极性、空闲状态、MOE 关断与实测死区（约 200 ns）。
3. **低压限流标定**：接入低压限流电源，校验 ADC 偏置、相电流极性与比例、VBUS 比例、温度换算、转子方向与电角度零点。
4. **软件保护注入**：受控条件下逐项触发并确认过流、残差、欠压/过压、过温、堵转与传感器故障锁存。
5. **实时性测量（WCET）**：使用 DWT 测量控制中断与主控制环最坏执行时间，确认满足 10 µs 周期（100 kHz）。
6. **温升与损耗评估**：逐步加载，实测 MOSFET、栅极驱动、分流电阻与板面温度；若 WCET 或热损耗超标，应将集中配置中的 PWM 频率降至 40 kHz 或 20 kHz 并重新回归全部验证。
7. **通信联调**：在目标物理接线下验收 UART、FDCAN 及 ESP32-C3 加密无线链路。

---

## 文档索引

- 固件软件设计基线：[`docs/software-design.md`](docs/software-design.md)
- 无感 FOC 软件仿真方案：[`docs/sensorless-simulation-plan.md`](docs/sensorless-simulation-plan.md)
- 主机物理仿真说明：[`software/foc-sim/README.md`](software/foc-sim/README.md)
- 桌面上位机说明：[`software/host-app/README.md`](software/host-app/README.md)
- ESP32-C3 无线桥接说明：[`software/esp32-pc-bridge/README.md`](software/esp32-pc-bridge/README.md)

## 许可证

仓库根目录采用 MIT 许可证（见 [`LICENSE`](LICENSE)），固件与协议 crate 声明为 `MIT OR Apache-2.0`。本项目处于工程开发阶段，投入实际使用前应完成硬件复核与实板验证。
