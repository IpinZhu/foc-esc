# ESP32-C3 有线桥（方案验证 v0）

需求依据：`../requirements.md` §2.2（TR-B5）、里程碑 M5a。
固件形态：C/C++（Arduino-ESP32 框架，PlatformIO 构建）。

## 功能

PC 与 FOC 之间的**双向纯透传**：

```text
PC --USB-CDC(USB-Serial-JTAG)--> ESP32-C3 --UART1--> FOC USART3
```

- 对协议字节零修改、零注入（后续上位机 M1 可把它当成一根普通串口线）。
- 无线化（M5b）时成帧/ACK 只加在两片 ESP32 的 2.4G 链路上，本固件不变。

## 接线（三根线，务必共地）

| ESP32-C3 | FOC 板 | 说明 |
|---|---|---|
| GPIO4 (UART1 TX) | PB11 (USART3 RX) | 3.3 V 电平 |
| GPIO5 (UART1 RX) | PB10 (USART3 TX) | 3.3 V 电平 |
| GND | GND | 必须共地 |

避开引脚：GPIO18/19（USB D-/D+）、GPIO2/8/9（strapping）。板载 LED（`LED_BUILTIN`）作心跳/活动指示。

## 先判别：你的板子是哪种 USB？

单 USB 口的 C3 板有两种形态，决定用哪个编译环境：

| 形态 | 特征 | 编译环境 |
|---|---|---|
| **USB 直连模组**（多数单口 C3 板，如 super mini） | USB 口旁没有独立的 USB 转串口芯片；USB 走 GPIO18/19 | `esp32c3_usb`（默认），烧录与桥的 CDC 口是**同一个 USB 口** |
| **板载 USB 转串口芯片** | USB 口旁有 CH340/CP210x 丝印的小芯片 | `esp32c3_uart0`，PC 侧透传走 UART0 |

## 构建与烧录（PlatformIO）

```bash
pio run -t upload                    # 按 default_envs（JTAG 型）编译并烧录
pio run -t upload -e esp32c3_uart0   # CH340/UART0 型板子用这个
pio device monitor                   # 观察串口
```

首次编译会下载 ESP32 工具链，耐心等待。烧录失败（JTAG 型）时的兜底：按住 BOOT 键 → 短按 RESET → 松开 RESET → 松开 BOOT → 再烧录。

Arduino IDE 等价设置（JTAG 型）：开发板 `ESP32C3 Dev Module`，`USB Mode: Hardware JTAG`，`USB CDC On Boot: Enabled`。

**注意（UART0 型）**：上位机/终端每次打开串口，DTR/RTS 自动进 Boot 电路可能令 C3 复位一次，属正常现象；复位横幅（乱码）之后即是干净的透传数据。

## 验证步骤

1. FOC 板保持自身供电并完成上电校准（进入 `Idle`）。
2. 按上表接三根线。
3. 电脑插上 C3，设备管理器出现 CDC 串口（芯片复位时 USB 会重枚举，重新打开串口即可）。
4. 用任意串口终端（115200）连 C3 的 CDC 口，发送 `status\r\n`：
   - 期望收到 `seq=... state=... faults=... ia=...` 格式的遥测行；
   - 发送 `param show` 应逐行吐出全部 43 个参数。
5. 上位机 M1 完成后，把通道类型选为串口即直接经此桥工作。

## 故障排查：串口里反复出现 rst:0x3 / ROM 横幅（复位循环）

USB-Serial-JTAG 会把主机 DTR/RTS 的特定翻转序列解析为"复位/进入下载模式"。
串口终端打开端口或自动重连时翻转控制线 → 芯片复位 → USB 重枚举 → 终端再重连 →
循环，表现为 ROM 横幅刷屏（`rst:0x3 (RTC_SW_SYS_RST)`）。这不是固件崩溃。

对策（platformio.ini 已配置）：

```ini
monitor_rts = 0
monitor_dtr = 0
```

用 `python -m platformio device monitor` 观察串口；用其它终端工具时关闭
DTR/RTS 自动控制（或选择不带流控的连接方式）。上位机的串口层必须遵守同一
约束（需求 TR-B7）。

## 已知边界

- USB-Serial-JTAG 在芯片复位时重枚举，串口会短暂消失（上位机需容忍，见需求 TR-B5）。
- v0 不含统计/流控；透传缓存依赖 Arduino 核心的串口驱动缓冲，100 Hz 遥测 + 115200 波特下余量充足。
- 隧道成帧（CRC/SEQ/ACK）属于无线段（M5b）引入，本固件到时不变。
