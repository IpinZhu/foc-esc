# FOC Firmware Handoff

## Project and user requirements

This repository contains a three-phase motor-control project. The active firmware work is under `software/firmware` and must continue using the existing Rust 2024 + Embassy framework for the STM32G473RC.

User-selected requirements:

- Implement the FOC firmware in `software/firmware` using the hardware mapping in the root `README.md` and the FOC drive board files.
- Support both rotor-sensor backends, selected at compile time:
  - Default: `sensor-encoder` using TIM3 QEI on PB4/PB5.
  - Alternative: `sensor-hall` using TIM3 input capture on PC6/PC7/PC8.
- Use TIM1 center-aligned complementary PWM on PA8/PA9/PA10 and PB13/PB14/PB15.
- Default PWM frequency: 100 kHz.
- Default dead time: 200 ns (`DEAD_TIME_TICKS = 34` at a 170 MHz timer clock).
- Limit duty cycles to 5%–95%.
- Run a 100 kHz current loop and a 1 kHz speed loop.
- Implement software protections only for this iteration. Do not add or enable the COMP -> DAC -> TIM1 BRK hardware over-current chain unless the user explicitly requests it.
- Implement both UART debug commands and classic CAN control/telemetry.
- Do not create commits or push changes unless explicitly requested.

## Current implementation status

The firmware implementation is present and currently compiles for both sensor features.

### Core modules

- `src/foc_math.rs`
  - Clarke/Park/inverse-Park transforms.
  - Fixed polynomial sine/cosine approximation.
  - Min/max zero-sequence SVPWM.
  - Bus-voltage and duty-range validation.
  - Non-finite alpha/beta voltage inputs return `PhaseDuty::DISABLED`.
- `src/pid.rs`
  - Fixed-period PID with output limits, anti-windup, slew limiting, reset, and non-finite intermediate protection.
- `src/interfaces.rs`
  - Motor states, control modes, commands, raw ADC frames, rotor samples, faults, telemetry, and control output types.
- `src/hardware.rs`
  - `PwmBridge` and `RotorSensor` traits.
- `src/foc_core.rs`
  - Calibration state, current and velocity loops, SVPWM output, bus-current reconstruction, input/output power estimates, thermal model, NTC/TMP235 conversion, current residual protection, software over-current protection, UV/OV, stall, sensor-invalid, control-overrun, and fault latching.
  - Calibration faults can be cleared only into a fresh calibration cycle when calibration was incomplete.
  - Runtime velocity and open-loop electrical velocity are bounded by `FocConfig`.
  - Open-loop q-axis voltage is clamped to half the measured bus voltage.

### STM32G473 BSP

`src/bsp_hardware.rs` contains the Embassy hardware implementation:

- TIM1 complementary PWM at 100 kHz, center-aligned, 200 ns dead time, MOE-controlled enable/disable.
- TIM1 TRGO2 update trigger for injected ADC conversions.
- ADC1 phase-A/VBUS/TMP235 sequence; ADC2 phase-B; ADC3 phase-C.
- Static ADC handles protected by Embassy critical-section mutexes.
- ADC ISR copies raw samples and publishes `RawAdcFrame` through a one-slot `Signal`.
- ADC overrun detection begins only after the first frame has been consumed, preventing startup backlog from immediately becoming a fault.
- Encoder QEI implementation and Hall input-capture implementation are feature-gated.
- Hall angle is interpolated between valid Hall transitions using measured electrical velocity.
- Invalid Hall states and non-adjacent transitions are rejected.
- PA6 is configured as a pulled-down input placeholder only; it is not connected to TIM1 BKIN.

The encoder backend currently reports a valid QEI count whenever the count is in the configured timer range. A stationary cable disconnection cannot be reliably diagnosed from QEI count alone without additional hardware/status information. Do not claim complete unplug detection based solely on the current implementation.

### Communications

`src/comm_task.rs` contains:

- UART ASCII parser and Embassy UART task on USART3:
  - PB11 RX, PB10 TX, 115200 baud.
  - Commands include enable/mode, disable, clear-fault, Iq, velocity, open-loop, cell count, electrical zero, PID configuration, and status.
  - Non-finite floats are rejected.
  - An overlong line is discarded until newline; its suffix cannot be interpreted as a separate command.
- Classic CAN protocol on FDCAN1:
  - PA11 RX, PA12 TX, 500 kbit/s.
  - Fixed command IDs `0x100`–`0x105`.
  - Fixed telemetry IDs `0x180`–`0x182`.
  - Explicit little-endian encoding.
  - Receive path accepts only standard classic data frames with exact command DLC.
- UART/CAN tasks communicate with the motor-control task through Embassy channels/watch state; they do not directly access PWM or the controller.

### Entry point

`src/main.rs` contains:

- Compile-time mutual exclusion checks for `sensor-encoder` and `sensor-hall`.
- 170 MHz clock configuration:
  - HSI / 4 * 85 / 2 = 170 MHz SYSCLK.
  - PLL1Q / 8 for FDCAN kernel clock.
  - ADC12 and ADC345 use SYSCLK.
- Initialization of PWM, ADC resources, selected rotor sensor, UART, and FDCAN.
- Embassy tasks for motor control, UART, CAN receive, and CAN telemetry.
- ADC interrupt is enabled at the start of `motor_control_task`, after resources are initialized.
- ADC interrupt priority is higher than TIM3 and communications.

## Verification already completed

Run commands from the repository root. When building the ARM binary from the root, explicitly load the firmware Cargo configuration; otherwise Cargo may omit `link.x` and `defmt.x` and produce an invalid two-byte ELF:

```bash
cargo fmt --manifest-path "software/firmware/Cargo.toml" -- --check
cargo test --manifest-path "software/firmware/Cargo.toml" --lib --target x86_64-pc-windows-msvc
cargo clippy --manifest-path "software/firmware/Cargo.toml" --lib --target x86_64-pc-windows-msvc -- -D warnings
```

Current host result: **21 tests passed**.

Encoder checks:

```bash
cargo --config "software/firmware/.cargo/config.toml" clippy \
  --manifest-path "software/firmware/Cargo.toml" \
  --target thumbv7em-none-eabihf --release --bin foc-firmware -- -D warnings

cargo --config "software/firmware/.cargo/config.toml" build \
  --manifest-path "software/firmware/Cargo.toml" \
  --target thumbv7em-none-eabihf --release --bin foc-firmware
```

Hall checks:

```bash
cargo --config "software/firmware/.cargo/config.toml" clippy \
  --manifest-path "software/firmware/Cargo.toml" \
  --target thumbv7em-none-eabihf --release --bin foc-firmware \
  --no-default-features --features sensor-hall -- -D warnings

cargo --config "software/firmware/.cargo/config.toml" build \
  --manifest-path "software/firmware/Cargo.toml" \
  --target thumbv7em-none-eabihf --release --bin foc-firmware \
  --no-default-features --features sensor-hall
```

Most recent linked sizes:

- Encoder: approximately 103,136 bytes text, 344 bytes data, 3,920 bytes BSS.
- Hall: approximately 104,052 bytes text, 344 bytes data, 3,960 bytes BSS.

No heap allocation symbols were found in the release ELF. No firmware commit has been created.

## Important repository state

The working tree contains existing unrelated hardware/project modifications. Do not reset, restore, clean, or delete them. Review `git status` before any operation that could discard changes. The firmware files are modified but intentionally uncommitted.

The root README currently documents some earlier 20/40 kHz and hardware-primary-break assumptions that do not exactly match this user-selected firmware configuration. Do not silently change the hardware protection choice. If documentation is updated later, clearly state that this firmware iteration uses software protection only and that 100 kHz requires board validation.

## Recommended next steps

1. Re-read the current firmware files before making additional changes; the code may have changed after this handoff.
2. If further software work is needed, add focused host tests for Hall interpolation and command edge cases without adding heap usage.
3. Decide explicitly whether to improve sensor-disconnect diagnostics; QEI/Hall digital inputs alone cannot distinguish a stationary rotor from all forms of cable failure.
4. Perform board validation before claiming 100 kHz is production-safe:
   - Observe all six complementary gate outputs with no motor connected.
   - Verify measured dead time is approximately 200 ns and idle/MOE shutdown behavior is safe.
   - Validate ADC trigger alignment, conversion completion, current offsets, and ADC noise.
   - Start with a low-voltage current-limited supply and verify rotor angle direction/zero.
   - Exercise software over-current, UV/OV, residual, sensor, stall, and over-temperature protections.
   - Measure worst-case control execution time using DWT and check that it fits the 10 us period.
   - Measure MOSFET, gate-driver, shunt, and board temperatures under load.
5. If the measured WCET or thermal loss is too high, reduce the single PWM-frequency configuration to 40 kHz or 20 kHz and re-run all tests/builds.
6. Only create a Git commit when the user explicitly asks for one.
