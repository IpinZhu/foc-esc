use foc_firmware::interfaces::{Command, ControlMode, MotorState};
use foc_sim::plant::PlantParams;
use foc_sim::runner::{CONTROL_PERIOD_NS, Runner, controller_config};

#[test]
fn sample_precedes_output_and_output_has_one_tick_delay() {
    let mut runner = Runner::new(PlantParams::default(), controller_config()).unwrap();
    runner.calibrate().unwrap();
    assert!(runner.controller.handle_command(Command::SetIq(2.0)));
    assert!(
        runner
            .controller
            .handle_command(Command::Enable(ControlMode::Current))
    );
    let first = runner.step().unwrap();
    assert_eq!(
        first.time_ns,
        (u64::from(controller_config().calibration_samples) + 1) * CONTROL_PERIOD_NS
    );
    assert!(first.output.bridge_enabled);
    assert!(!first.effective.bridge_enabled);
    assert_eq!(runner.plant.state.iq, 0.0);
    let next = runner.step().unwrap();
    assert!(next.effective.bridge_enabled);
    assert_eq!(next.truth.iq, 0.0);
    assert!(runner.plant.state.iq > 0.0);
}

#[test]
fn disable_clears_pending_enable() {
    let mut runner = Runner::new(PlantParams::default(), controller_config()).unwrap();
    runner.calibrate().unwrap();
    runner.controller.handle_command(Command::SetIq(2.0));
    runner
        .controller
        .handle_command(Command::Enable(ControlMode::Current));
    runner.step().unwrap();
    runner.controller.handle_command(Command::Disable);
    let disabled = runner.step().unwrap();
    assert_eq!(disabled.state, MotorState::Idle);
    assert!(!disabled.effective.bridge_enabled);
    assert!(!runner.step().unwrap().effective.bridge_enabled);
}

#[test]
fn calibration_does_not_drive_the_plant() {
    let mut runner = Runner::new(PlantParams::default(), controller_config()).unwrap();
    runner.calibrate().unwrap();
    assert_eq!(runner.controller.state(), MotorState::Idle);
    assert_eq!(runner.plant.state.mechanical_speed, 0.0);
}

#[test]
fn invalid_substep_is_rejected() {
    let mut runner = Runner::new(PlantParams::default(), controller_config()).unwrap();
    runner.physics_step_ns = 3_000;
    assert!(runner.step().is_err());
}
