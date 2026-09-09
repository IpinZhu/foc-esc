# STM32G473 Three-Phase FOC Drive

An experimental three-phase PMSM/BLDC motor-control platform built around the STM32G473RC. The repository contains the drive-board design and a Rust 2024 + Embassy firmware implementation for current-controlled field-oriented control (FOC).

> **Safety notice:** This is a high-current power-electronics prototype. The firmware and PCB have not been electrically qualified or validated for production use. Do not connect a motor or battery until the gate waveforms, dead time, current-sense polarity, ADC timing, fault response, insulation/clearance, and thermal behavior have been checked with appropriate laboratory equipment and current limiting.

## Current implementation

The active firmware is in [`software/firmware`](software/firmware) and targets the STM32G473RC:

- TIM1 center-aligned complementary PWM on PA8/PA9/PA10 and PB13/PB14/PB15.
- Configured PWM frequency: **100 kHz**.
- Configured dead time: **200 ns** at a 170 MHz timer clock.
- Duty-cycle range: 5%–95%.
- Three-phase current feedback through INA240A2 amplifiers and 0.5 mΩ shunts.
- Timer-triggered ADC sampling of phase currents, VBUS, and the TMP235 temperature sensor.
- Current and velocity FOC modes, open-loop mode, startup current-offset calibration, SVPWM, telemetry, and software fault handling.
- Compile-time rotor-sensor selection:
  - `sensor-encoder` (default): TIM3 quadrature encoder on PB4/PB5.
  - `sensor-hall`: TIM3 Hall input capture on PC6/PC7/PC8.
- USART3 ASCII debug commands on PB10/PB11.
- FDCAN1 classic CAN control and telemetry on PA11/PA12.

The active firmware uses **software-only protection for this iteration**. It does not enable the COMP-to-DAC-to-TIM1-BRK hardware over-current chain. Software protection includes current threshold/debounce, phase-current residual, bus under/over-voltage, modeled and sensor temperature, stall, sensor validity, and control-overrun handling. Software protection is not a substitute for a validated hardware shutdown path.

## System overview

```text
                    +-----------------------------+
  Encoder / Hall -->|                             |--> TIM1 complementary PWM
  Phase currents -->|       STM32G473RC           |       (100 kHz, 200 ns)
  VBUS / TMP235 --->|       Rust + Embassy        |--> UCC27211 x3
  UART ------------>|       FOC controller        |--> 070N10NS-class bridge
  FDCAN ----------->|                             |<-- INA240A2 x3 + shunts
                    +-----------------------------+
```

The firmware separates the high-rate control path from communications. ADC frames wake the motor-control task; UART and CAN tasks exchange commands and telemetry through bounded Embassy synchronization primitives.

## Repository layout

| Path | Purpose |
|---|---|
| [`software/firmware`](software/firmware) | Rust/Embassy STM32G473RC firmware |
| [`hardware/foc-drive/FOC2.kicad_pro`](hardware/foc-drive/FOC2.kicad_pro) | Main drive-board KiCad project |
| [`hardware/foc-drive/FOC2.kicad_sch`](hardware/foc-drive/FOC2.kicad_sch) | Main drive-board schematic |
| [`hardware/foc-drive/FOC2.kicad_pcb`](hardware/foc-drive/FOC2.kicad_pcb) | Main drive-board PCB layout |
| [`hardware/foc-expansion/foc-ex.kicad_pro`](hardware/foc-expansion/foc-ex.kicad_pro) | Expansion-board KiCad project |
| [`hardware/foc-expansion/foc-ex.kicad_sch`](hardware/foc-expansion/foc-ex.kicad_sch) | Expansion-board schematic |
| [`hardware/foc-expansion/foc-ex.kicad_pcb`](hardware/foc-expansion/foc-ex.kicad_pcb) | Expansion-board PCB layout |
| [`foc/foc.ioc`](foc/foc.ioc) | STM32G473 CubeMX reference configuration; not the runtime firmware source of truth |

Generated manufacturing output, editor state, backups, and firmware binaries are not treated as source-of-truth design files.

## Build and test

Run these commands from the repository root. The firmware has its own Cargo configuration for linker scripts and the `probe-rs` runner. When invoking Cargo from the root, pass that configuration explicitly.

### Host tests and lint

```bash
cargo fmt --manifest-path "software/firmware/Cargo.toml" -- --check
cargo test --manifest-path "software/firmware/Cargo.toml" --lib --target x86_64-pc-windows-msvc
cargo clippy --manifest-path "software/firmware/Cargo.toml" --lib --target x86_64-pc-windows-msvc -- -D warnings
```

### Encoder firmware

```bash
cargo --config "software/firmware/.cargo/config.toml" build \
  --manifest-path "software/firmware/Cargo.toml" \
  --target thumbv7em-none-eabihf --release --bin foc-firmware
```

### Hall firmware

```bash
cargo --config "software/firmware/.cargo/config.toml" build \
  --manifest-path "software/firmware/Cargo.toml" \
  --target thumbv7em-none-eabihf --release --bin foc-firmware \
  --no-default-features --features sensor-hall
```

The firmware `.cargo/config.toml` configures `thumbv7em-none-eabihf`, `link.x`, `defmt.x`, and a `probe-rs` runner for `STM32G473RC`. Flashing requires an appropriately connected and authorized debug probe; no flashing is performed by this documentation.

## Hardware summary

The main drive design is based on:

- STM32G473RC, 170 MHz Cortex-M4F motor-control MCU.
- Three UCC27211 half-bridge gate drivers.
- Six 100 V-class N-channel MOSFET positions forming a three-phase bridge.
- Three INA240A2 bidirectional current-sense amplifiers.
- Three 0.5 mΩ shunts with Kelvin sensing.
- A 2S–6S battery-oriented bus design target, approximately 5.5–25.2 V.
- VBUS divider sensing and a TMP235 analog temperature sensor.
- USART3, FDCAN1, SWD, encoder, Hall, and expansion interfaces.

Component ratings, thermal limits, bootstrap sizing, switching loss, and current capability remain engineering targets until the populated board is measured. See the local design note on the developer machine for detailed calculations and open issues; it is intentionally not part of the shared repository.

## Validation status and limitations

Software builds and host tests cover the current implementation, but they do not prove safe power-stage operation. Before applying motor power:

1. Inspect the assembled board and verify component population, polarity, clearances, grounding, and Kelvin-current paths.
2. With the power stage unpowered, verify all six PWM waveforms, complementary polarity, idle behavior, MOE shutdown, and measured dead time.
3. With a low-voltage current-limited supply, verify ADC offsets, current polarity/scaling, VBUS scaling, temperature conversion, rotor direction, and electrical-zero alignment.
4. Exercise each software protection path with controlled fault injection.
5. Measure control-loop worst-case execution time against the 10 µs PWM period.
6. Measure gate-driver, MOSFET, shunt, and board temperatures under progressively increasing load.
7. Confirm UART and CAN behavior on the intended physical wiring.

The 100 kHz setting is therefore a configured operating target, not a claim that the assembled hardware can run continuously at that rate. If timing, switching loss, EMI, bootstrap refresh, or thermal results are unacceptable, reduce the centralized PWM-frequency configuration and repeat validation.

## License and contribution status

The firmware crate declares `MIT OR Apache-2.0` in its package metadata. A repository-wide license file has not been added. This project is currently an active engineering workspace; changes should be reviewed against the hardware design and validated on the target board before being treated as release-ready.
