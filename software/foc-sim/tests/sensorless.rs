use foc_firmware::control::foc_core::FocController;
use foc_firmware::control::sensorless::Stage;
use foc_firmware::interfaces::{Command, ControlMode, FaultFlags, MotorState, RotorSample};
use foc_firmware::params::motor_param::MotorParam;
use foc_sim::plant::{Load, PlantParams};
use foc_sim::runner::{Runner, controller_config};

fn fixture(p: PlantParams) -> (Runner, MotorParam) {
    let mut cfg = controller_config();
    cfg.pole_pairs = p.pole_pairs;
    // Outer loop bandwidth well below the 314 rad/s estimator bandwidth.
    cfg.velocity_pid.kp = 0.018;
    cfg.velocity_pid.ki = 0.25;
    let mut m = MotorParam::from_foc_config(&cfg);
    m.rs = p.resistance as f32 * 1.03;
    m.ld = p.ld as f32 * 0.97;
    m.lq = m.ld;
    m.flux = p.flux as f32 * 1.02;
    m.observer_bandwidth = 314.0;
    m.pll_kp = 628.0;
    m.pll_ki = 314.0 * 314.0;
    m.observer_switch_speed = 40.0;
    m.mark_commissioned();
    let mut r = Runner::new(p, cfg).unwrap();
    r.controller = FocController::new_sensorless(cfg, Some(&m));
    r.calibrate().unwrap();
    (r, m)
}
fn tick(r: &mut Runner) -> foc_firmware::interfaces::ControlOutput {
    let (adc, _) = r.capture();
    let out = r.controller.step(adc, RotorSample::default());
    r.drive(out).unwrap();
    out
}
fn start(r: &mut Runner, speed: f32) {
    assert!(r.controller.handle_command(Command::SetVelocity(speed)));
    assert!(
        r.controller
            .handle_command(Command::Enable(ControlMode::Velocity))
    );
}
#[test]
fn two_motor_profiles_start_in_both_directions_without_rotor_feedback() {
    for p in [
        PlantParams::default(),
        PlantParams {
            resistance: 0.72,
            ld: 0.00032,
            lq: 0.00032,
            flux: 0.013,
            pole_pairs: 5,
            inertia: 1.7e-5,
            ..PlantParams::default()
        },
    ] {
        for sign in [-1.0, 1.0] {
            let (mut r, _) = fixture(p);
            r.plant.state.mechanical_angle = 0.29;
            r.plant.load = Load::Opposing(0.0005);
            start(&mut r, sign * 25.0);
            let mut seen_blend = false;
            for n in 0..140_000 {
                let out = tick(&mut r);
                assert!(
                    out.bridge_enabled,
                    "tick {n} {:?} {:?}",
                    r.plant.state,
                    r.controller.faults()
                );
                seen_blend |= r.controller.sensorless_stage() == Some(Stage::Blend);
                assert!(
                    r.plant
                        .state
                        .phase_currents(p)
                        .iter()
                        .all(|i| i.abs() < 5.0)
                );
                if n >= 120_000 {
                    assert!(out.telemetry.sensor_valid);
                    assert!((r.plant.state.mechanical_speed - f64::from(sign) * 25.0).abs() < 1.0);
                    let delta = (out.telemetry.mechanical_angle as f64
                        - r.plant.state.mechanical_angle)
                        * f64::from(p.pole_pairs);
                    assert!(delta.cos() > 0.98, "angle error {delta}");
                }
            }
            assert!(seen_blend);
        }
    }
}
#[test]
fn locked_shaft_cannot_qualify_and_timeout_latches() {
    let (mut r, _) = fixture(PlantParams::default());
    r.plant.locked_rotor = true;
    start(&mut r, 25.0);
    for _ in 0..150_000 {
        let out = tick(&mut r);
        assert!(!out.telemetry.sensor_valid);
        if !out.bridge_enabled {
            break;
        }
    }
    assert_eq!(r.controller.state(), MotorState::Fault);
    assert!(r.controller.faults().contains(FaultFlags::SENSOR));
    assert!(!tick(&mut r).bridge_enabled);
}

#[test]
fn current_mode_tracks_then_latches_loss_of_emf() {
    let p = PlantParams {
        friction: 0.0012,
        ..PlantParams::default()
    };
    let (mut r, _) = fixture(p);
    assert!(r.controller.handle_command(Command::SetIq(0.5)));
    assert!(
        r.controller
            .handle_command(Command::Enable(ControlMode::Current))
    );
    for n in 0..110_000 {
        let out = tick(&mut r);
        assert!(out.bridge_enabled, "{n} {:?}", r.plant.state);
        if n >= 105_000 {
            assert!(out.telemetry.sensor_valid);
            assert!((r.plant.state.iq - 0.5).abs() < 0.05);
        }
    }
    // External fixture stops the shaft, not a forged rotor sample.
    r.plant.locked_rotor = true;
    r.plant.state.mechanical_speed = 0.0;
    for _ in 0..4000 {
        if !tick(&mut r).bridge_enabled {
            break;
        }
    }
    assert!(r.controller.faults().contains(FaultFlags::SENSOR));
    assert!(
        !r.controller
            .handle_command(Command::Enable(ControlMode::Current))
    );
    assert!(!tick(&mut r).bridge_enabled);
}
#[test]
fn missing_or_incompatible_record_cannot_energize_even_with_valid_encoder() {
    let p = PlantParams::default();
    let (_, m) = fixture(p);
    let mut saliency = m;
    saliency.lq = 1.8 * m.ld;
    let mut mismatch = m;
    mismatch.pole_pairs += 1;
    let mut unstable = m;
    unstable.pll_kp = 1e8;
    for record in [
        None,
        Some(MotorParam::default()),
        Some(saliency),
        Some(mismatch),
        Some(unstable),
    ] {
        let cfg = controller_config();
        let mut r = Runner::new(p, cfg).unwrap();
        r.controller = FocController::new_sensorless(cfg, record.as_ref());
        r.calibrate().unwrap();
        r.controller.handle_command(Command::SetVelocity(25.0));
        r.controller.handle_command(Command::SetIq(0.5));
        for mode in [
            ControlMode::OpenLoop,
            ControlMode::Velocity,
            ControlMode::Current,
        ] {
            assert!(!r.controller.handle_command(Command::Enable(mode)));
            assert!(!r.step().unwrap().output.bridge_enabled);
        }
    }
}
#[test]
fn zero_stops_and_reenable_requires_fresh_target() {
    let (mut r, _) = fixture(PlantParams::default());
    start(&mut r, 25.0);
    assert!(tick(&mut r).bridge_enabled);
    assert!(!r.controller.handle_command(Command::SetVelocity(-25.0)));
    assert!(r.controller.handle_command(Command::SetVelocity(0.0)));
    assert!(!tick(&mut r).bridge_enabled);
    assert!(
        !r.controller
            .handle_command(Command::Enable(ControlMode::Velocity))
    );
}

/// Longer deterministic light-load acceptance grid; run explicitly in release.
#[test]
#[ignore = "72 physical starts; run with --release -- --ignored"]
fn nominal_start_grid() {
    let p = PlantParams::default();
    for sector in 0..12 {
        for sign in [-1.0, 1.0] {
            for load in [0.0, 0.0005, 0.002] {
                let (mut r, _) = fixture(p);
                r.plant.state.mechanical_angle =
                    std::f64::consts::TAU * sector as f64 / (12.0 * f64::from(p.pole_pairs));
                r.plant.load = Load::Opposing(load);
                start(&mut r, sign * 25.0);
                let mut seen_blend = false;
                let mut sum_sq = 0.0;
                let mut peak_error = 0.0_f64;
                for tick_id in 0..140_000 {
                    let truth = r.plant.state;
                    let out = tick(&mut r);
                    assert!(
                        out.bridge_enabled,
                        "sector={sector} sign={sign} load={load} tick={tick_id} fault={:?}",
                        r.controller.faults()
                    );
                    assert!(truth.phase_currents(p).iter().all(|i| i.abs() < 5.0));
                    seen_blend |= r.controller.sensorless_stage() == Some(Stage::Blend);
                    if tick_id >= 120_000 {
                        assert!(out.telemetry.sensor_valid);
                        assert!((truth.mechanical_speed - f64::from(sign) * 25.0).abs() < 1.25);
                        assert!(
                            (truth.mechanical_speed - f64::from(out.telemetry.mechanical_velocity))
                                .abs()
                                < 1.25
                        );
                        let delta = f64::from(p.pole_pairs)
                            * (truth.mechanical_angle - f64::from(out.telemetry.mechanical_angle));
                        let error = delta.sin().atan2(delta.cos()).to_degrees();
                        sum_sq += error * error;
                        peak_error = peak_error.max(error.abs());
                    }
                }
                assert!(
                    seen_blend && (sum_sq / 20_000.0).sqrt() <= 5.0 && peak_error <= 15.0,
                    "sector={sector} sign={sign} load={load}"
                );
            }
        }
    }
}
