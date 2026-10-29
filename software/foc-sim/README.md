# FOC 主机仿真（M1）

当前实现有感 FOC 基线，不是无感控制。直接调用 `foc-firmware` 的 `FocController`；独立 `f64` PMSM 模型包含 dq 电流、机械转矩、惯量、摩擦及外负载。固定 10 µs 控制周期、默认 1 µs RK4 物理步长，使用模拟 ADC 帧和理想转子反馈。平均逆变器仅根据最终三相占空比及桥使能输出电压，不读取控制器请求的 dq 电压；当前一周期 PWM 延迟是明示的仿真假设，并非实测 TIM1 时序。

从仓库根目录运行：

```powershell
cargo test --locked --manifest-path software/foc-sim/Cargo.toml --target x86_64-pc-windows-msvc
cargo clippy --locked --manifest-path software/foc-sim/Cargo.toml --all-targets --target x86_64-pc-windows-msvc -- -D warnings
cargo run --release --locked --manifest-path software/foc-sim/Cargo.toml --target x86_64-pc-windows-msvc -- current
cargo run --release --locked --manifest-path software/foc-sim/Cargo.toml --target x86_64-pc-windows-msvc -- speed --csv software/foc-sim/runs/speed.csv --json software/foc-sim/runs/speed.json
cargo run --release --locked --manifest-path software/foc-sim/Cargo.toml --target x86_64-pc-windows-msvc -- open-loop
```

运行 CSV/JSON 示例前先创建 `software/foc-sim/runs/` 目录；该目录已被 Git 忽略。`--csv`、`--json` 可以单独使用或任意顺序组合。未请求输出文件时仅打印摘要；场景验收失败返回非零退出码。数据以预设合成电机参数和单场景控制参数运行；每个测试用例的单位和阈值以代码为准。JSON 含 M1 使用的关键物理和控制参数、时序假设、场景验收指标；非适用指标为 null。

## 自整定参数验证（主机仿真）

复用固件的 `AutoTuneController` 快环、1 kHz 慢环和 `FocController`。在 Rs/L 辨识与电流 PI 阶跃阶段，仿真以明确的锁轴夹具固定转子；进入开环试转前释放夹具，随后由电磁转矩、惯量和负载决定转速，**从不将转速赋为控制指令**。所有注入与委托控制都通过最终 PWM 占空比、桥使能及一周期延迟驱动物理模型。

```powershell
cargo run --release --locked --manifest-path software/foc-sim/Cargo.toml --target x86_64-pc-windows-msvc -- autotune --csv software/foc-sim/runs/autotune.csv --json software/foc-sim/runs/autotune.json
cargo run --release --locked --manifest-path software/foc-sim/Cargo.toml --target x86_64-pc-windows-msvc -- autotune --angle-electrical 0.5
```

CSV 记录阶段转换、转速与锁轴状态；JSON 分别记录电机真值、辨识结果、相对误差、PI 阶跃指标、实时应用的电流/速度 PI、设定与结果。对照组 `--fixture locked` 故意不释放转子，预期报告 `open-loop-failed` 并返回非零状态。参数验收门槛：Rs/磁链相对误差 <5%，Ld/Lq <15%，电流 PI 必须经过阶跃验证并应用到控制器；速度 PI 只是继承，不是自动调节。`Ld=Lq` 仍是同一测量值，不代表单独辨识。

为使 5 A 限流的**合成场景**里 12 位 ADC 能分辨阶跃，仿真自整定使用 `pi_step_ratio=0.7`（整定内部电流上限 2.5 A、目标阶跃 1.75 A）。**固件默认仍是 0.08**：在同一低限流仿真场景下，0.2 A 阶跃会因量化触发 `PiUnstable`，有专门的失败回归测试。不能据此断言实板在不同限流配置下会通过或失败；更不能把仿真较大阶跃直接当作第一次上电建议。

保存阶段只模拟“有效整定记录请求”的确认；JSON 明确标记 `flash_persisted=false`，本平台尚未仿真 Flash 写入、断电恢复或实际功率级。观测器、PLL 仅生成参数，不进行无感估角/接管；实板整定需要验证传感器角度、电流采样、限流与故障停用。

现有测试覆盖 RL 解析解、电磁与机械能量收支、转矩方向、物理步长收敛、采样与 PWM 延迟因果关系、禁用清空排队使能、电流和速度闭环、真实负载扰动及开环未强制同步。关桥电流按理想化开路衰减近似处理，不用于判断回生母线、电流峰值或停机距离；M1 未模拟器件开关、死区误差、ADC 噪声、TIM3 外设、无感观测器或目标板的计算耗时。M2 及后续范围见 [设计方案](../../docs/sensorless-simulation-plan.md)。
