use foc_firmware::interfaces::{Command, ControlMode, MotorState};
use foc_sim::plant::{Load, PlantParams};
use foc_sim::runner::{Runner, Scenario, controller_config, run};

#[test]
fn current_loop_tracks_before_back_emf_saturation() {
    let summary = run(Scenario::Current, |_| {}).unwrap();
    assert!(!summary.faulted);
    assert!(summary.mean_iq_error < 0.1, "{summary:?}");
    assert!(summary.peak_phase_current < 5.0);
}

#[test]
fn speed_loop_tracks_with_opposing_load() {
    let summary = run(Scenario::Speed, |_| {}).unwrap();
    assert!(!summary.faulted);
    assert!(summary.mean_speed_error < 2.5, "{summary:?}");
    assert!((summary.final_speed - 50.0).abs() < 2.5);
}

#[test]
fn open_loop_command_does_not_force_rotor_speed() {
    let summary = run(Scenario::OpenLoop, |_| {}).unwrap();
    assert!(!summary.faulted);
    assert!(
        (summary.final_speed - 120.0 / 4.0).abs() > 5.0,
        "{summary:?}"
    );
}

#[test]
fn speed_feedback_responds_to_load_step() {
    let mut runner = Runner::new(PlantParams::default(), controller_config()).unwrap();
    runner.calibrate().unwrap();
    assert!(runner.controller.handle_command(Command::SetVelocity(50.0)));
    assert!(
        runner
            .controller
            .handle_command(Command::Enable(ControlMode::Velocity))
    );
    for _ in 0..20_000 {
        runner.step().unwrap();
    }
    let speed_before = runner.plant.state.mechanical_speed;
    runner.plant.load = Load::Opposing(0.03);
    for _ in 0..100 {
        runner.step().unwrap();
    }
    assert!(runner.plant.state.mechanical_speed < speed_before - 1.0);
    for _ in 0..20_000 {
        runner.step().unwrap();
    }
    assert_eq!(
        runner.controller.state(),
        MotorState::Running(ControlMode::Velocity)
    );
    assert!((runner.plant.state.mechanical_speed - 50.0).abs() < 2.5);
}
