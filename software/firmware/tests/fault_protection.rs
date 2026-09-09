mod common;

use common::{
    NORMAL_BUS_VOLTAGE, assert_fault, calibrated_controller, enable, rotor,
    step,
};
use foc_firmware::interfaces::{Command, ControlMode, FaultFlags, MotorState};

#[test]
fn over_current_latches_after_debounce_and_disables_bridge() {
    let (mut controller, config, mut sequence) = calibrated_controller();
    controller.handle_command(Command::SetIq(0.0));
    enable(&mut controller, ControlMode::Current);

    for _ in 0..config.over_current_debounce - 1 {
        let output = step(
            &mut controller,
            config,
            sequence,
            [6.0, -3.0, -3.0],
            NORMAL_BUS_VOLTAGE,
            config.ambient_temperature,
            rotor(true, 0.0),
            false,
        );
        sequence += 1;
        assert!(!controller.faults().contains(FaultFlags::OVER_CURRENT));
        assert!(output.bridge_enabled);
    }

    let output = step(
        &mut controller,
        config,
        sequence,
        [6.0, -3.0, -3.0],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(true, 0.0),
        false,
    );
    assert_fault(&controller, output, FaultFlags::OVER_CURRENT);
}

#[test]
fn over_current_counter_resets_after_safe_sample() {
    let (mut controller, config, mut sequence) = calibrated_controller();
    enable(&mut controller, ControlMode::OpenLoop);

    for _ in 0..config.over_current_debounce - 1 {
        step(
            &mut controller,
            config,
            sequence,
            [6.0, -3.0, -3.0],
            NORMAL_BUS_VOLTAGE,
            config.ambient_temperature,
            rotor(false, 0.0),
            false,
        );
        sequence += 1;
    }
    step(
        &mut controller,
        config,
        sequence,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    sequence += 1;

    let output = step(
        &mut controller,
        config,
        sequence,
        [6.0, -3.0, -3.0],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert!(!controller.faults().contains(FaultFlags::OVER_CURRENT));
    assert!(output.bridge_enabled);
}

#[test]
fn current_residual_latches_without_over_current() {
    let (mut controller, config, mut sequence) = calibrated_controller();
    enable(&mut controller, ControlMode::OpenLoop);

    for _ in 0..config.residual_debounce - 1 {
        let output = step(
            &mut controller,
            config,
            sequence,
            [2.0, 2.0, 2.0],
            NORMAL_BUS_VOLTAGE,
            config.ambient_temperature,
            rotor(false, 0.0),
            false,
        );
        sequence += 1;
        assert!(!controller.faults().contains(FaultFlags::CURRENT_RESIDUAL));
        assert!(output.bridge_enabled);
    }

    let output = step(
        &mut controller,
        config,
        sequence,
        [2.0, 2.0, 2.0],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert_fault(&controller, output, FaultFlags::CURRENT_RESIDUAL);
}

#[test]
fn bus_voltage_faults_require_an_active_bridge() {
    let (mut idle, config, sequence) = calibrated_controller();
    let output = step(
        &mut idle,
        config,
        sequence,
        [0.0; 3],
        5.0,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert!(idle.faults().is_empty());
    assert!(!output.bridge_enabled);

    let (mut under_voltage, config, sequence) = calibrated_controller();
    enable(&mut under_voltage, ControlMode::OpenLoop);
    let output = step(
        &mut under_voltage,
        config,
        sequence,
        [0.0; 3],
        5.0,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert_fault(&under_voltage, output, FaultFlags::UNDER_VOLTAGE);

    let (mut over_voltage, config, sequence) = calibrated_controller();
    enable(&mut over_voltage, ControlMode::OpenLoop);
    let output = step(
        &mut over_voltage,
        config,
        sequence,
        [0.0; 3],
        9.0,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert_fault(&over_voltage, output, FaultFlags::OVER_VOLTAGE);
}

#[test]
fn invalid_sensor_latches_in_current_mode_but_not_open_loop() {
    let (mut current, config, mut sequence) = calibrated_controller();
    enable(&mut current, ControlMode::Current);
    let samples_to_fault =
        (config.sensor_fault_time * config.pwm_frequency_hz as f32) as u32;

    for index in 0..samples_to_fault {
        let output = step(
            &mut current,
            config,
            sequence,
            [0.0; 3],
            NORMAL_BUS_VOLTAGE,
            config.ambient_temperature,
            rotor(false, 0.0),
            false,
        );
        sequence += 1;
        if index + 1 < samples_to_fault {
            assert!(!current.faults().contains(FaultFlags::SENSOR));
            assert!(output.bridge_enabled);
        } else {
            assert_fault(&current, output, FaultFlags::SENSOR);
        }
    }

    let (mut open_loop, config, sequence) = calibrated_controller();
    enable(&mut open_loop, ControlMode::OpenLoop);
    let output = step(
        &mut open_loop,
        config,
        sequence,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert!(!open_loop.faults().contains(FaultFlags::SENSOR));
    assert!(output.bridge_enabled);
}

#[test]
fn velocity_stall_latches_after_configured_time() {
    let (mut controller, config, mut sequence) = calibrated_controller();
    controller.handle_command(Command::SetVelocity(2.0));
    enable(&mut controller, ControlMode::Velocity);
    let samples_to_fault =
        (config.stall_time * config.speed_loop_frequency_hz as f32) as u32;

    for index in 0..samples_to_fault {
        let output = step(
            &mut controller,
            config,
            sequence,
            [0.0; 3],
            NORMAL_BUS_VOLTAGE,
            config.ambient_temperature,
            rotor(true, 0.0),
            false,
        );
        sequence += 1;
        if index + 1 < samples_to_fault {
            assert!(!controller.faults().contains(FaultFlags::STALL));
            assert!(output.bridge_enabled);
        } else {
            assert_fault(&controller, output, FaultFlags::STALL);
        }
    }
}

#[test]
fn over_temperature_from_sensor_sample_disables_bridge() {
    let (mut controller, config, sequence) = calibrated_controller();
    enable(&mut controller, ControlMode::OpenLoop);
    let output = step(
        &mut controller,
        config,
        sequence,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.shutdown_temperature + 5.0,
        rotor(false, 0.0),
        false,
    );
    assert_fault(&controller, output, FaultFlags::OVER_TEMPERATURE);
}

#[test]
fn control_overrun_is_reported_from_all_software_inputs() {
    let (mut adc_overrun, config, sequence) = calibrated_controller();
    enable(&mut adc_overrun, ControlMode::OpenLoop);
    let output = step(
        &mut adc_overrun,
        config,
        sequence,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        true,
    );
    assert_fault(&adc_overrun, output, FaultFlags::CONTROL_OVERRUN);

    let (mut skipped_frame, config, sequence) = calibrated_controller();
    enable(&mut skipped_frame, ControlMode::OpenLoop);
    let output = step(
        &mut skipped_frame,
        config,
        sequence + 1,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert_fault(&skipped_frame, output, FaultFlags::CONTROL_OVERRUN);

    let (mut command_overrun, config, sequence) = calibrated_controller();
    enable(&mut command_overrun, ControlMode::OpenLoop);
    command_overrun.handle_command(Command::ReportControlOverrun);
    let output = step(
        &mut command_overrun,
        config,
        sequence,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert_fault(&command_overrun, output, FaultFlags::CONTROL_OVERRUN);

    assert_eq!(command_overrun.state(), MotorState::Fault);
}
