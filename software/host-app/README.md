# FOC Studio

Vue 3 + TypeScript + Tauri 2 上位机。Rust 后端复用 `../foc-wire` 的 v2 帧协议，与 STM32 固件交换控制命令、参数和遥测。

## 当前范围

- 运行状态、电流、母线电压、速度和温度监控及曲线。
- 43 项参数读写、JSON 导入导出及参数持久化命令。
- 控制权申请/释放、电流/速度/开环控制、停用与故障清除。
- 自动整定命令入口和状态显示；真实辨识由 STM32 执行。
- 遥测录制与 CSV 导出。
- 设备身份握手、心跳、遥测过期判断和失联停用请求。

浏览器模式只支持界面模拟器，不访问真实串口，也不执行自动整定算法。桌面模拟器的整定进度同样不是真实电机参数辨识。两者都不是无感控制的物理仿真平台。

## 开发与验证

以下命令在本目录执行。需要 Node.js/npm；桌面模式还需要 Rust MSVC 工具链、Windows C++ 构建工具和 WebView2。

```powershell
npm ci
npm run dev
```

上述命令启动浏览器模式。桌面开发模式：

```powershell
npm run tauri -- dev
```

类型检查、前端构建和 Rust 工作区测试：

```powershell
npm run build
cargo test --locked --workspace
```

前端构建不等于桌面安装包构建，也不验证真实设备连接。

## 真实设备连接

当前 `device-foc` 会话支持 `simulator` 和 `serial`。串口路径需要 [ESP32 v2 PC 桥接](../esp32-pc-bridge/README.md) 的握手，并要求桥接报告 `secure=1`，随后验证 STM32 身份与能力、读取参数。

`host-transport` 中的 TCP、UDP、HID、BLE 等通用适配器不代表这些链路已接入当前 FOC 会话。不要把旧版明文 v1 接收器或直接 UART ASCII 终端当成 v2 PC 桥接。

连接不等于获得控制权。使用真实硬件前须先完成限流供电、PWM、采样、故障和热验证；界面上的“停用”是软件请求，不是硬件急停。

## 模块

- `src/`：界面、图表和浏览器模拟器。
- `src-tauri/`：桌面命令、串口扫描、事件与文件导出。
- `crates/device-foc/`：设备模型、参数描述、会话与桌面模拟器。
- `crates/host-transport/`：传输适配器。
- `crates/host-core/`：通用参数与模拟器基础。

`node_modules/`、`target/`、`dist/` 和本地环境文件不纳入版本控制。目前未完成整机实板联调验收。
