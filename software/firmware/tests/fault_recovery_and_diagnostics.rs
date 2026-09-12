mod common;

use common::{
    NORMAL_BUS_VOLTAGE, assert_fault, calibrated_controller, enable, rotor,
    step,
};
use foc_firmware::comm::comm_task::{
    CAN_CONTROL_ID, CAN_STATUS_ID, UartRequest, decode_can_command,
    encode_can_status, parse_uart_request,
};
use foc_firmware::control::foc_core::{FocConfig, FocController};
use foc_firmware::interfaces::{
    Command, ControlMode, FaultFlags, MotorState, RawAdcFrame, RotorSample,
    Telemetry,
};

#[test]
fn default_vbus_divider_matches_board_resistors() {
    let mut config = FocConfig::default();
    assert_eq!(config.bus_divider_high, 100_000.0);
    assert_eq!(config.bus_divider_low, 10_000.0);
    config.calibration_samples = 2;
    let mut controller = FocController::new(config);

    for sequence in 1..=config.calibration_samples {
        controller.step(
            RawAdcFrame {
                phase_a: 2_048,
                phase_b: 2_048,
                phase_c: 2_048,
                bus_voltage: 2_707,
                ntc: 930,
                sequence,
                overrun: false,
            },
            RotorSample {
                mechanical_angle: 0.0,
                mechanical_velocity: 0.0,
                valid: true,
            },
        );
    }

    assert!((controller.telemetry().bus_voltage - 24.0).abs() < 0.02);
}

#[test]
fn fault_stays_latched_until_a_safe_clear() {
    let (mut controller, config, mut sequence) = calibrated_controller();
    enable(&mut controller, ControlMode::OpenLoop);

    for _ in 0..config.over_current_debounce {
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
        sequence += 1;
        if controller.state() == MotorState::Fault {
            assert_fault(&controller, output, FaultFlags::OVER_CURRENT);
        }
    }
    assert!(controller.faults().contains(FaultFlags::OVER_CURRENT));

    controller.handle_command(Command::ClearFault);
    assert_eq!(controller.state(), MotorState::Fault);

    let output = step(
        &mut controller,
        config,
        sequence,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert!(!output.bridge_enabled);
    sequence += 1;

    controller.handle_command(Command::ClearFault);
    assert_eq!(controller.state(), MotorState::Idle);
    assert!(controller.faults().is_empty());

    enable(&mut controller, ControlMode::OpenLoop);
    let output = step(
        &mut controller,
        config,
        sequence,
        [0.0; 3],
        NORMAL_BUS_VOLTAGE,
        config.ambient_temperature,
        rotor(false, 0.0),
        false,
    );
    assert!(output.bridge_enabled);
}

#[test]
fn can_status_exposes_fault_state_and_bitmap() {
    let mut faults = FaultFlags::empty();
    faults.insert(FaultFlags::OVER_CURRENT);
    faults.insert(FaultFlags::SENSOR);
    let telemetry = Telemetry {
        state: MotorState::Fault,
        faults,
        sensor_valid: false,
        sequence: 0x1234,
        ..Telemetry::default()
    };

    let data = encode_can_status(&telemetry);
    let bitmap = u32::from_le_bytes(data[1..5].try_into().unwrap());

    assert_eq!(data.len(), 8);
    assert_eq!(data[0], 5);
    assert_eq!(bitmap, telemetry.faults.bits());
    assert_eq!(data[5], 0);
    assert_eq!(u16::from_le_bytes([data[6], data[7]]), 0x1234);
    assert_eq!(CAN_STATUS_ID, 0x180);
}

#[test]
fn uart_and_can_clear_fault_commands_are_decodable() {
    assert_eq!(
        parse_uart_request("clear-fault"),
        Ok(UartRequest::Command(Command::ClearFault))
    );
    assert_eq!(parse_uart_request("status"), Ok(UartRequest::Status));
    assert_eq!(
        decode_can_command(CAN_CONTROL_ID, &[1]),
        Some(Command::ClearFault)
    );
}

#[test]
fn rejected_commands_do_not_claim_an_unimplemented_fault() {
    let (mut controller, _, _) = calibrated_controller();
    let initial_state = controller.state();

    assert_eq!(
        parse_uart_request("enable unknown"),
        Err(foc_firmware::comm::comm_task::ParseError::InvalidArgument)
    );
    assert_eq!(decode_can_command(CAN_CONTROL_ID, &[9]), None);

    controller.handle_command(Command::SetCellCount(1));
    assert_eq!(controller.state(), initial_state);
    assert!(!controller.faults().contains(FaultFlags::INVALID_COMMAND));
}
