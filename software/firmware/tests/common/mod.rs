use foc_firmware::control::foc_core::{FocConfig, FocController};
use foc_firmware::control::pid::PidConfig;
use foc_firmware::interfaces::{
    Command, ControlMode, ControlOutput, FaultFlags, MotorState, RawAdcFrame,
    RotorSample,
};

pub const NORMAL_BUS_VOLTAGE: f32 = 7.0;

pub fn test_config() -> FocConfig {
    let mut config = FocConfig::default();
    config.pwm_frequency_hz = 1_000;
    config.speed_loop_frequency_hz = 1_000;
    config.calibration_samples = 2;
    config.battery_cells = 2;
    config.over_current_threshold = 5.0;
    config.over_current_debounce = 3;
    config.residual_threshold = 3.0;
    config.residual_debounce = 3;
    config.sensor_fault_time = 0.003;
    config.stall_target_velocity = 1.0;
    config.stall_max_velocity = 0.1;
    config.stall_min_iq = 0.5;
    config.stall_time = 0.003;
    config.velocity_pid =
        PidConfig::new(1.0, 0.0, 0.0, config.current_limit, f32::INFINITY);
    config
}

pub fn rotor(valid: bool, mechanical_velocity: f32) -> RotorSample {
    RotorSample {
        mechanical_angle: 0.0,
        mechanical_velocity,
        valid,
    }
}

pub fn frame(
    config: FocConfig,
    sequence: u32,
    currents: [f32; 3],
    bus_voltage: f32,
    temperature: f32,
    overrun: bool,
) -> RawAdcFrame {
    let current_scale = config.adc_reference_voltage
        / config.adc_full_scale
        / (config.current_sense_gain * config.shunt_resistance);
    let bus_scale = config.adc_reference_voltage / config.adc_full_scale
        * (config.bus_divider_high + config.bus_divider_low)
        / config.bus_divider_low;
    let ntc_voltage = config.temperature_sensor_zero_voltage
        + temperature * config.temperature_sensor_volts_per_degree;

    RawAdcFrame {
        phase_a: adc_count(2_048.0 + currents[0] / current_scale),
        phase_b: adc_count(2_048.0 + currents[1] / current_scale),
        phase_c: adc_count(2_048.0 + currents[2] / current_scale),
        bus_voltage: adc_count(bus_voltage / bus_scale),
        ntc: adc_count(
            ntc_voltage / config.adc_reference_voltage * config.adc_full_scale,
        ),
        sequence,
        overrun,
    }
}

fn adc_count(value: f32) -> u16 {
    value.round().clamp(0.0, 4_095.0) as u16
}

pub fn calibrated_controller() -> (FocController, FocConfig, u32) {
    let config = test_config();
    let mut controller = FocController::new(config);
    for sequence in 1..=config.calibration_samples {
        controller.step(
            frame(
                config,
                sequence,
                [0.0; 3],
                NORMAL_BUS_VOLTAGE,
                config.ambient_temperature,
                false,
            ),
            rotor(true, 0.0),
        );
    }
    assert_eq!(controller.state(), MotorState::Idle);
    (controller, config, config.calibration_samples + 1)
}

#[allow(clippy::too_many_arguments)]
pub fn step(
    controller: &mut FocController,
    config: FocConfig,
    sequence: u32,
    currents: [f32; 3],
    bus_voltage: f32,
    temperature: f32,
    rotor_sample: RotorSample,
    overrun: bool,
) -> ControlOutput {
    controller.step(
        frame(
            config,
            sequence,
            currents,
            bus_voltage,
            temperature,
            overrun,
        ),
        rotor_sample,
    )
}

pub fn enable(controller: &mut FocController, mode: ControlMode) {
    controller.handle_command(Command::Enable(mode));
    assert_eq!(controller.state(), MotorState::Running(mode));
}

pub fn assert_fault(
    controller: &FocController,
    output: ControlOutput,
    expected: FaultFlags,
) {
    assert!(controller.faults().contains(expected));
    assert_eq!(controller.state(), MotorState::Fault);
    assert!(!output.bridge_enabled);
    assert!(output.telemetry.faults.contains(expected));
}
