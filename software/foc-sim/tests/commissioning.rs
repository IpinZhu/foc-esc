use foc_firmware::autotune::{AutoTuneError, AutoTuneState};
use foc_sim::commissioning::{CommissioningConfig, Fixture, run_commissioning};

#[test]
fn physical_plant_commissioning_identifies_parameters() {
    let result = run_commissioning(CommissioningConfig::baseline(), |stage| {
        eprintln!(
            "stage={} time={:.3} speed={:.2}",
            stage.state.name(),
            stage.time_ns as f64 * 1e-9,
            stage.mechanical_speed
        );
    })
    .unwrap();
    assert!(result.passed(), "{result:?}");
}

#[test]
fn locked_rotor_cannot_fake_open_loop_synchronism() {
    let mut config = CommissioningConfig::baseline();
    config.fixture = Fixture::AlwaysLocked;
    let result = run_commissioning(config, |_| {}).unwrap();
    assert_eq!(result.state, AutoTuneState::Error, "{result:?}");
    assert_eq!(result.error, AutoTuneError::OpenLoopFailed);
    assert!(!result.passed());
}

#[test]
fn measured_current_pi_replaces_bad_initial_gains_but_speed_pi_is_inherited() {
    let mut config = CommissioningConfig::baseline();
    config.controller.current_pid.kp = 0.001;
    config.controller.current_pid.ki = 1.0;
    config.controller.velocity_pid.kp = 0.12;
    config.controller.velocity_pid.ki = 2.4;
    let result = run_commissioning(config, |_| {}).unwrap();
    assert!(result.passed(), "{result:?}");
    let omega = std::f32::consts::TAU * result.verified_bandwidth_hz;
    let motor = result.motor;
    assert!((motor.id_kp - motor.ld * omega).abs() < 1.0e-5);
    assert!((motor.iq_kp - motor.lq * omega).abs() < 1.0e-5);
    assert!((motor.id_ki - motor.rs * omega).abs() < 1.0e-3);
    assert!((motor.iq_ki - motor.rs * omega).abs() < 1.0e-3);
    assert_eq!(motor.iq_kp, result.live_current_kp);
    assert_eq!(motor.iq_ki, result.live_current_ki);
    assert_eq!(motor.speed_kp, 0.12);
    assert_eq!(motor.speed_ki, 2.4);
    assert_eq!(motor.hfi_voltage, 0.0);
    assert_eq!(motor.hfi_freq, 0.0);
}

#[test]
fn multiple_starting_angles_keep_identification_within_bounds() {
    for electrical_angle in [0.0, 0.5, 1.0] {
        let mut config = CommissioningConfig::baseline();
        config.initial_mechanical_angle = electrical_angle / f64::from(config.motor.pole_pairs);
        let result = run_commissioning(config, |_| {}).unwrap();
        assert!(result.passed(), "angle={electrical_angle} {result:?}");
    }
}

#[test]
fn completion_without_actual_save_is_not_claimed() {
    let mut config = CommissioningConfig::baseline();
    config.max_frames = 1_000;
    let result = run_commissioning(config, |_| {}).unwrap();
    assert_ne!(result.state, AutoTuneState::Done);
    assert!(!result.save_requested);
    assert!(!result.passed());
}

#[test]
fn parameter_fit_uses_physical_motor_not_initial_guess() {
    let mut config = CommissioningConfig::baseline();
    config.motor.resistance = 0.65;
    config.motor.ld = 0.000_25;
    config.motor.lq = 0.000_25;
    config.motor.flux = 0.012;
    config.initial_mechanical_angle = 0.1;
    let result = run_commissioning(config, |_| {}).unwrap();
    assert!(result.passed(), "{result:?}");
    assert!((result.motor.rs - 0.65).abs() < 0.05);
    assert!((result.motor.flux - 0.012).abs() < 0.001);
}

#[test]
fn original_small_pi_step_exposes_adc_resolution_limit() {
    let mut config = CommissioningConfig::baseline();
    config.tune.pi_step_ratio = 0.08;
    let result = run_commissioning(config, |_| {}).unwrap();
    assert_eq!(result.state, AutoTuneState::Error, "{result:?}");
    assert_eq!(result.error, AutoTuneError::PiUnstable);
    assert!(!result.passed());
}
