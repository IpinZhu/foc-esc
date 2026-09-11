# STM32G473 三相 FOC 驱动器

这是一个基于 STM32G473RC 的实验性三相 PMSM/BLDC 电机控制平台。仓库包含驱动板设计，以及使用 Rust 2024 + Embassy 实现的电流控制型磁场定向控制（FOC）固件。

> **安全提示：** 这是一个大电流电力电子原型。固件和 PCB 尚未完成生产级电气鉴定或验证。在使用合适的实验设备和限流措施检查栅极波形、死区时间、电流采样极性、ADC 时序、故障响应、绝缘/间距和热性能之前，不要连接电机或电池。

## 当前实现

当前固件位于 [`software/firmware`](software/firmware)，目标芯片为 STM32G473RC：

- TIM1 中心对齐互补 PWM，输出引脚为 PA8/PA9/PA10 和 PB13/PB14/PB15。
- 配置的 PWM 频率：**100 kHz**。
- 配置的死区时间：在 170 MHz 定时器时钟下为 **200 ns**。
- 占空比范围：5%–95%。
- 通过 INA240A2 放大器和 0.5 mΩ 分流电阻进行三相电流反馈。
- 由定时器触发 ADC，对相电流、VBUS 和 TMP235 温度传感器进行采样。
- 支持电流 FOC、速度 FOC、开环控制、启动电流偏置校准、SVPWM、遥测和软件故障处理。
- 带版本和 CRC 的双槽 Flash 参数持久化：上电自动加载，UART `param ...` 命令和 CAN 参数帧可读取、设置、保存或恢复出厂参数。
- 编译期选择转子传感器：
  - `sensor-encoder`（默认）：TIM3 正交编码器，连接 PB4/PB5。
  - `sensor-hall`：TIM3 Hall 输入捕获，连接 PC6/PC7/PC8。
- USART3 ASCII 调试命令，连接 PB10/PB11。
- FDCAN1 classic CAN 控制和遥测，连接 PA11/PA12。

当前固件采用**纯软件保护**。本轮没有启用 COMP→DAC→TIM1-BRK 硬件过流链。软件保护包括电流阈值/去抖、相电流残差、母线欠压/过压、模型温度和传感器温度、堵转、传感器有效性以及控制超时处理。软件保护不能替代经过验证的硬件关断路径。

## 系统概览

```text
                    +-----------------------------+
  编码器 / Hall --->|                             |--> TIM1 互补 PWM
  相电流 ---------->|       STM32G473RC           |       (100 kHz, 200 ns)
  VBUS / TMP235 --->|       Rust + Embassy        |--> UCC27211 x3
  UART ------------>|       FOC 控制器            |--> 070N10NS 级三相桥
  FDCAN ----------->|                             |<-- INA240A2 x3 + 分流电阻
                    +-----------------------------+
```

固件将高速控制路径与通信路径分离。ADC 帧唤醒电机控制任务；UART 和 CAN 任务通过有界的 Embassy 同步原语交换命令和遥测数据。

## 仓库结构

| 路径 | 用途 |
|---|---|
| [`software/firmware`](software/firmware) | Rust/Embassy STM32G473RC 固件 |
| [`hardware/foc-drive/FOC2.kicad_pro`](hardware/foc-drive/FOC2.kicad_pro) | 主驱动板 KiCad 工程 |
| [`hardware/foc-drive/FOC2.kicad_sch`](hardware/foc-drive/FOC2.kicad_sch) | 主驱动板原理图 |
| [`hardware/foc-drive/FOC2.kicad_pcb`](hardware/foc-drive/FOC2.kicad_pcb) | 主驱动板 PCB 布局 |
| [`hardware/foc-expansion/foc-ex.kicad_pro`](hardware/foc-expansion/foc-ex.kicad_pro) | 扩展板 KiCad 工程 |
| [`hardware/foc-expansion/foc-ex.kicad_sch`](hardware/foc-expansion/foc-ex.kicad_sch) | 扩展板原理图 |
| [`hardware/foc-expansion/foc-ex.kicad_pcb`](hardware/foc-expansion/foc-ex.kicad_pcb) | 扩展板 PCB 布局 |
| [`foc/foc.ioc`](foc/foc.ioc) | STM32G473 CubeMX 参考配置；不是运行时固件的唯一依据 |

生成的生产文件、编辑器状态、备份文件和固件二进制文件不属于设计源文件。

## 构建和测试

以下命令均从 `software/firmware` 目录执行。由于固件的默认 Cargo 目标是 `thumbv7em-none-eabihf`，主机测试必须显式指定主机目标。

### 主机测试和 lint

```bash
cargo fmt -- --check
cargo test --target x86_64-pc-windows-msvc
cargo clippy --target x86_64-pc-windows-msvc --test fault_protection --test fault_recovery_and_diagnostics -- -D warnings
```

故障诊断集成测试位于 `software/firmware/tests/`，会使用主机仿真的 ADC 帧和转子采样，不会驱动真实 PWM 或功率级。

### Encoder 固件

```bash
cargo build --bin foc-firmware
```

### Hall 固件

```bash
cargo build --release --bin foc-firmware \
  --no-default-features --features sensor-hall
```

固件 `.cargo/config.toml` 配置了 `thumbv7em-none-eabihf`、`link.x`、`defmt.x` 以及针对 STM32G473RC 的 `probe-rs` 烧录运行器。烧录需要正确连接且获得授权的调试探针；本文档不会执行烧录操作。

## 硬件概要

主驱动设计基于以下器件和目标：

- STM32G473RC，170 MHz Cortex-M4F 电机控制 MCU。
- 三个 UCC27211 半桥栅极驱动器。
- 六个 100 V 级 N 沟道 MOSFET 位置，组成三相桥。
- 三个 INA240A2 双向电流采样放大器。
- 三个 0.5 mΩ、带 Kelvin 引线的分流电阻。
- 面向 2S–6S 电池的母线设计目标，约 5.5–25.2 V。
- VBUS 分压采样和 TMP235 模拟温度传感器。
- USART3、FDCAN1、SWD、编码器、Hall 和扩展接口。

器件额定值、热限制、自举电路尺寸、开关损耗和电流能力在实际装配电路板完成测量前都只是工程目标。详细计算和待解决问题见开发机上的本地设计说明；该文件有意不纳入共享仓库。

## 验证状态和限制

软件构建和主机测试覆盖了当前实现，但不能证明功率级运行安全。在加电连接电机之前：

1. 检查已装配电路板，确认器件安装、极性、间距、接地和 Kelvin 电流路径正确。
2. 在功率级未上电时，检查全部六路 PWM 波形、互补极性、空闲行为、MOE 关断和实测死区时间。
3. 使用低电压限流电源，验证 ADC 偏置、电流极性/比例、VBUS 比例、温度换算、转子方向和电角度零点对齐。
4. 通过受控故障注入逐项执行软件保护测试。
5. 根据 10 µs PWM 周期测量控制环最坏执行时间。
6. 在逐步增加负载的过程中测量栅极驱动器、MOSFET、分流电阻和电路板温度。
7. 在目标物理接线下确认 UART 和 CAN 行为。

因此，100 kHz 只是配置的运行目标，并不代表已证明装配后的硬件可以长期连续运行在该频率。如果时序、开关损耗、EMI、自举刷新或热测试结果不可接受，应降低集中配置中的 PWM 频率并重新进行全部验证。

## 许可证和贡献状态

固件 crate 声明采用 `MIT OR Apache-2.0` 许可证。仓库级许可证文件尚未添加。本项目目前仍处于积极的工程开发阶段；在将改动视为可发布版本之前，应结合硬件设计进行审查，并在目标电路板上完成验证。
