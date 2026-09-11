//! Full auto-tune procedure against a simulated PMSM.
//!
//! The plant is a dq-frame R-L model with back-EMF. The rotor is locked
//! for the resistance, inductance, and current-loop phases (a standard
//! commissioning condition) and follows the imposed field synchronously
//! during the open-loop flux phase. The simulation mirrors the firmware
//! task structure: one fast step per ADC frame, one slow tick per millisecond.

use foc_firmware::autotune::{
    AutoTuneConfig, AutoTuneController, AutoTuneState, ControlRequest,
    FastAction,
};
use foc_firmware::foc_core::{FocConfig, FocController};
use foc_firmware::foc_math::{
    AlphaBeta, Dq, SQRT_3_OVER_2, inverse_park, normalize_angle, park,
};
use foc_firmware::interfaces::{
    Command, ControlMode, MotorState, RawAdcFrame, RotorSample, Telemetry,
};
use foc_firmware::motor_param::MotorParam;

const FREQ: f32 = 100_000.0;
const DT: f32 = 1.0 / FREQ;
const BUS: f32 = 24.0;
/// Electrical angle of the locked rotor.
const LOCKED_THETA: f32 = 0.0;

struct Plant {
    rs: f32,
    ls: f32,
    flux: f32,
    pole_pairs: f32,
    theta_e: f32,
    omega_e: f32,
    id: f32,
    iq: f32,
}

struct Sim {
    controller: FocController,
    autotune: AutoTuneController,
    plant: Plant,
    sequence: u32,
    frame_count: u32,
    status_updates: u32,
}

impl Sim {
    fn new() -> Self {
        let config = FocConfig::default();
        let mut autotune_config = AutoTuneConfig::from_foc_config(&config);
        autotune_config.offset_frames = 256;
        autotune_config.rs_settle_frames = 4_000;
        autotune_config.rs_sample_frames = 2_000;
        Self {
            controller: FocController::new(config),
            autotune: AutoTuneController::new(autotune_config),
            plant: Plant {
                rs: 0.5,
                ls: 0.000_2,
                flux: 0.01,
                pole_pairs: 4.0,
                theta_e: LOCKED_THETA,
                omega_e: 0.0,
                id: 0.0,
                iq: 0.0,
            },
            sequence: 0,
            frame_count: 0,
            status_updates: 0,
        }
    }

    fn amperes_per_count(&self) -> f32 {
        let config = self.autotune.config();
        config.adc_reference_voltage
            / config.adc_full_scale
            / (config.current_sense_gain * config.shunt_resistance)
    }

    fn frame(&mut self) -> RawAdcFrame {
        let stationary = inverse_park(
            Dq {
                d: self.plant.id,
                q: self.plant.iq,
            },
            self.plant.theta_e,
        );
        let ia = stationary.alpha;
        let ib = -0.5 * stationary.alpha + SQRT_3_OVER_2 * stationary.beta;
        let ic = -0.5 * stationary.alpha - SQRT_3_OVER_2 * stationary.beta;
        let apc = self.amperes_per_count();
        self.sequence += 1;
        RawAdcFrame {
            phase_a: (2048.0 + ia / apc) as u16,
            phase_b: (2048.0 + ib / apc) as u16,
            phase_c: (2048.0 + ic / apc) as u16,
            bus_voltage: (BUS
                / (3.3 / 4095.0 * (100_000.0 + 10_000.0) / 10_000.0))
                as u16,
            ntc: 930,
            sequence: self.sequence,
            overrun: false,
        }
    }

    fn rotor(&self) -> RotorSample {
        RotorSample {
            mechanical_angle: self.plant.theta_e / self.plant.pole_pairs,
            mechanical_velocity: self.plant.omega_e / self.plant.pole_pairs,
            valid: true,
        }
    }

    fn reconstruct_alpha(
        &self,
        duty: &foc_firmware::foc_math::PhaseDuty,
    ) -> f32 {
        let va = (duty.a - 0.5) * BUS;
        let vb = (duty.b - 0.5) * BUS;
        let vc = (duty.c - 0.5) * BUS;
        (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc)
    }

    /// Applies one control request to the simulated controller, mirroring
    /// the motor task.
    fn apply(&mut self, request: ControlRequest, param: &MotorParam) {
        match request {
            | ControlRequest::DisableAndResync => {
                self.controller.handle_command(Command::Disable);
                self.controller.resynchronize_control_input();
            },
            | ControlRequest::EnableCurrent => {
                self.controller
                    .handle_command(Command::Enable(ControlMode::Current));
            },
            | ControlRequest::EnableOpenLoop => {
                self.controller
                    .handle_command(Command::Enable(ControlMode::OpenLoop));
            },
            | ControlRequest::SetIq(iq) => {
                self.controller.handle_command(Command::SetIq(iq));
            },
            | ControlRequest::SetOpenLoop {
                electrical_velocity,
                q_voltage,
            } => {
                self.controller.handle_command(Command::SetOpenLoop {
                    electrical_velocity,
                    q_voltage,
                });
                // Perfect field synchronization: the rotor tracks the
                // imposed electrical speed without slip.
                self.plant.omega_e = electrical_velocity;
            },
            | ControlRequest::ApplyMotorParam => {
                assert!(
                    self.controller.apply_motor_param(param),
                    "apply_motor_param rejected during commissioning"
                );
            },
        }
    }

    /// Runs one ADC frame: fast step, plant update, and (every 100 frames)
    /// the slow task tick including request application and save handling.
    fn step(&mut self) {
        let frame = self.frame();
        let mut v_alpha = 0.0;
        let mut v_beta = 0.0;
        let mut control_telemetry: Option<Telemetry> = None;

        if self.autotune.is_running() {
            match self.autotune.fast_step(&frame) {
                | FastAction::Inject(duty) => {
                    v_alpha = self.reconstruct_alpha(&duty);
                },
                | FastAction::Off => {},
                | FastAction::Delegate => {
                    let output = self.controller.step(frame, self.rotor());
                    control_telemetry = Some(output.telemetry);
                    let stationary = inverse_park(
                        output.telemetry.voltage_dq,
                        self.plant.theta_e,
                    );
                    v_alpha = stationary.alpha;
                    v_beta = stationary.beta;
                },
            }
        } else {
            let output = self.controller.step(frame, self.rotor());
            control_telemetry = Some(output.telemetry);
        }

        // dq-frame plant dynamics with back-EMF.
        let dq_v = park(
            AlphaBeta {
                alpha: v_alpha,
                beta: v_beta,
            },
            self.plant.theta_e,
        );
        let p = &mut self.plant;
        p.id += (dq_v.d - p.rs * p.id + p.omega_e * p.ls * p.iq) / p.ls * DT;
        p.iq += (dq_v.q - p.rs * p.iq - p.omega_e * (p.ls * p.id + p.flux))
            / p.ls
            * DT;
        p.theta_e = normalize_angle(p.theta_e + p.omega_e * DT);

        self.frame_count = self.frame_count.wrapping_add(1);
        if self.autotune.is_running() && self.frame_count.is_multiple_of(100) {
            let mut view = control_telemetry.unwrap_or_default();
            view.state = self.controller.state();
            self.autotune.task_tick(DT * 100.0, Some(&view));
            // Read the parameter set after the tick: requests queued by
            // this tick must carry the freshly computed values.
            while let Some(request) = self.autotune.pop_request() {
                let param = self.autotune.motor_param();
                self.apply(request, &param);
            }
            if self.autotune.state() == AutoTuneState::Save
                && let Some(saved) = self.autotune.take_save_request()
            {
                self.autotune.notify_saved(saved.is_commissioned());
            }
            self.status_updates += 1;
            let _ = self.autotune.status();
        } else if !self.autotune.is_running()
            && self.autotune.has_pending_requests()
        {
            while let Some(request) = self.autotune.pop_request() {
                let param = self.autotune.motor_param();
                self.apply(request, &param);
            }
        }
    }

    /// Runs boot calibration and returns when the controller is idle.
    fn boot(&mut self) {
        for _ in 0..(FocConfig::default().calibration_samples + 64) as usize {
            self.step();
        }
        assert_eq!(self.controller.state(), MotorState::Idle);
    }

    fn run_until<F: Fn(&Sim) -> bool>(&mut self, done: F, max_frames: usize) {
        let mut frames = 0;
        while !done(self) {
            self.step();
            frames += 1;
            assert!(
                frames < max_frames,
                "simulation did not converge: state={:?} error={:?} controller={:?} attempts={} bw={}",
                self.autotune.state(),
                self.autotune.error(),
                self.controller.state(),
                self.autotune.status().attempts,
                self.autotune.status().current_bandwidth_hz,
            );
        }
    }
}

#[test]
fn full_procedure_identifies_motor_and_completes() {
    let mut sim = Sim::new();
    sim.boot();

    let seed = MotorParam::from_foc_config(&sim.controller.config());
    assert!(sim.autotune.start(seed));

    sim.run_until(
        |sim| sim.autotune.state() == AutoTuneState::Done,
        (FREQ * 8.0) as usize,
    );

    let motor = sim.autotune.motor_param();
    let rs_true = sim.plant.rs;
    let ls_true = sim.plant.ls;
    let flux_true = sim.plant.flux;
    assert!(
        (motor.rs - rs_true).abs() / rs_true < 0.05,
        "rs {} vs {rs_true}",
        motor.rs
    );
    assert!(
        (motor.ld - ls_true).abs() / ls_true < 0.15,
        "ld {} vs {ls_true}",
        motor.ld
    );
    assert!(
        (motor.lq - ls_true).abs() / ls_true < 0.15,
        "lq {} vs {ls_true}",
        motor.lq
    );
    assert!(
        (motor.flux - flux_true).abs() / flux_true < 0.05,
        "flux {} vs {flux_true}",
        motor.flux
    );
    assert!(motor.is_commissioned());
    assert!(motor.validate().is_ok());

    // Controller runs on the identified gains (exact transfer of the
    // commissioning result into the live controller).
    let config = sim.controller.config();
    assert_eq!(config.current_pid.kp, motor.iq_kp);
    assert_eq!(config.current_pid.ki, motor.iq_ki);
    assert_eq!(config.velocity_pid.kp, motor.speed_kp);
    assert_eq!(config.current_limit, motor.max_current);
    assert_eq!(sim.controller.state(), MotorState::Idle);
    assert!(sim.autotune.pi_metrics().is_some());
    assert!(sim.status_updates > 0);
}

#[test]
fn overcurrent_during_identification_latches_error() {
    let mut sim = Sim::new();
    sim.boot();
    let seed = MotorParam::from_foc_config(&sim.controller.config());
    assert!(sim.autotune.start(seed));

    sim.run_until(
        |sim| sim.autotune.state() == AutoTuneState::RsMeasure,
        (FREQ * 0.2) as usize,
    );

    // Force the plant winding into a massive overcurrent.
    sim.plant.ls = 1.0e-6;
    sim.plant.rs = 1.0e-3;
    sim.run_until(
        |sim| sim.autotune.state() == AutoTuneState::Error,
        (FREQ * 0.2) as usize,
    );
    assert_eq!(
        sim.autotune.error(),
        foc_firmware::autotune::AutoTuneError::OverCurrent
    );
}

#[test]
fn stop_mid_procedure_returns_to_idle() {
    let mut sim = Sim::new();
    sim.boot();
    let seed = MotorParam::from_foc_config(&sim.controller.config());
    assert!(sim.autotune.start(seed));
    sim.run_until(
        |sim| sim.autotune.state() == AutoTuneState::RsMeasure,
        (FREQ * 0.2) as usize,
    );
    sim.autotune.stop();
    assert_eq!(sim.autotune.state(), AutoTuneState::Idle);
    assert_eq!(
        sim.autotune.error(),
        foc_firmware::autotune::AutoTuneError::Aborted
    );
    // The bridge request is released and the controller returns to idle.
    sim.run_until(
        |sim| sim.controller.state() == MotorState::Idle,
        (FREQ * 0.1) as usize,
    );
    assert!(!sim.autotune.is_running());
}
