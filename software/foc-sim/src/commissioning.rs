use foc_firmware::autotune::{
    AutoTuneConfig, AutoTuneController, AutoTuneError, AutoTuneState, ControlRequest, FastAction,
};
use foc_firmware::control::foc_core::FocConfig;
use foc_firmware::control::foc_math::PhaseDuty;
use foc_firmware::interfaces::{Command, ControlMode, ControlOutput};
use foc_firmware::params::motor_param::MotorParam;

use crate::plant::PlantParams;
use crate::runner::Runner;

#[derive(Clone, Copy, Debug)]
pub enum Fixture {
    /// Hold the rotor through winding identification and current PI verification.
    ReleaseForOpenLoop,
    /// Negative control: physical rotor cannot pass the open-loop speed check.
    AlwaysLocked,
}

#[derive(Clone, Copy)]
pub struct CommissioningConfig {
    pub motor: PlantParams,
    pub controller: FocConfig,
    pub tune: AutoTuneConfig,
    pub fixture: Fixture,
    pub initial_mechanical_angle: f64,
    pub max_frames: u32,
}

impl CommissioningConfig {
    pub fn baseline() -> Self {
        let controller = crate::runner::controller_config();
        let mut tune = AutoTuneConfig::from_foc_config(&controller);
        tune.offset_frames = 256;
        tune.rs_settle_frames = 4_000;
        tune.rs_sample_frames = 2_000;
        tune.pi_step_ratio = 0.7;
        Self {
            motor: PlantParams::default(),
            controller,
            tune,
            fixture: Fixture::ReleaseForOpenLoop,
            initial_mechanical_angle: 0.0,
            max_frames: 1_000_000,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StageRecord {
    pub time_ns: u64,
    pub state: AutoTuneState,
    pub error: AutoTuneError,
    pub mechanical_speed: f64,
    pub locked_rotor: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Accuracy {
    pub rs_relative: f64,
    pub ld_relative: f64,
    pub lq_relative: f64,
    pub flux_relative: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct CommissioningResult {
    pub state: AutoTuneState,
    pub error: AutoTuneError,
    pub motor: MotorParam,
    pub truth: PlantParams,
    pub frames: u32,
    pub released: bool,
    pub max_speed: f64,
    pub save_requested: bool,
    pub pi_verified: bool,
    pub pi_metrics: Option<foc_firmware::autotune::measure::StepMetrics>,
    pub verified_bandwidth_hz: f32,
    pub live_current_kp: f32,
    pub live_current_ki: f32,
    pub live_speed_kp: f32,
    pub live_speed_ki: f32,
    pub gains_applied: bool,
    pub accuracy: Accuracy,
}

impl CommissioningResult {
    pub fn passed(&self) -> bool {
        self.state == AutoTuneState::Done
            && self.error == AutoTuneError::None
            && self.save_requested
            && self.released
            && self.max_speed > 1.0
            && self.pi_verified
            && self.gains_applied
            && self.live_current_kp == self.motor.iq_kp
            && self.live_current_ki == self.motor.iq_ki
            && self.live_speed_kp == self.motor.speed_kp
            && self.live_speed_ki == self.motor.speed_ki
            && self.motor.is_commissioned()
            && self.motor.validate().is_ok()
            && self.accuracy.rs_relative < 0.05
            && self.accuracy.ld_relative < 0.15
            && self.accuracy.lq_relative < 0.15
            && self.accuracy.flux_relative < 0.05
    }
}

fn apply_request(runner: &mut Runner, request: ControlRequest, param: &MotorParam) -> bool {
    let controller = &mut runner.controller;
    match request {
        ControlRequest::DisableAndResync => {
            controller.handle_command(Command::Disable);
            controller.resynchronize_control_input();
            runner.stop_bridge();
            true
        }
        ControlRequest::EnableCurrent => {
            controller.handle_command(Command::Enable(ControlMode::Current))
        }
        ControlRequest::EnableOpenLoop => {
            controller.handle_command(Command::Enable(ControlMode::OpenLoop))
        }
        ControlRequest::SetIq(iq) => controller.handle_command(Command::SetIq(iq)),
        ControlRequest::SetOpenLoop {
            electrical_velocity,
            q_voltage,
        } => controller.handle_command(Command::SetOpenLoop {
            electrical_velocity,
            q_voltage,
        }),
        ControlRequest::ApplyMotorParam => controller.apply_motor_param(param),
    }
}

fn inject_output(duty: PhaseDuty) -> ControlOutput {
    ControlOutput {
        duty,
        bridge_enabled: true,
        ..ControlOutput::default()
    }
}

pub fn run_commissioning(
    config: CommissioningConfig,
    mut log: impl FnMut(StageRecord),
) -> Result<CommissioningResult, &'static str> {
    if config.max_frames == 0 || config.tune.pwm_frequency_hz != config.controller.pwm_frequency_hz
    {
        return Err("invalid commissioning timing");
    }
    let mut runner = Runner::new(config.motor, config.controller)?;
    if !config.initial_mechanical_angle.is_finite() {
        return Err("invalid initial rotor angle");
    }
    runner.plant.state.mechanical_angle = config
        .initial_mechanical_angle
        .rem_euclid(std::f64::consts::TAU);
    runner.calibrate()?;
    runner.plant.locked_rotor = true;
    let seed = MotorParam::from_foc_config(&runner.controller.config());
    let mut autotune = AutoTuneController::new(config.tune);
    if !autotune.start(seed) {
        return Err("auto-tune rejected initial parameters");
    }

    let mut view = runner.controller.telemetry();
    let mut frames = 0;
    let mut save_requested = false;
    let mut released = false;
    let mut pi_verified = false;
    let mut gains_applied = true;
    let mut max_speed = 0.0_f64;
    let mut previous = autotune.state();
    while frames < config.max_frames && !autotune.state().is_terminal() {
        let (raw, rotor) = runner.capture();
        let output = match autotune.fast_step(&raw) {
            FastAction::Off => ControlOutput::default(),
            FastAction::Inject(duty) => inject_output(duty),
            FastAction::Delegate => {
                let output = runner.controller.step(raw, rotor);
                view = output.telemetry;
                output
            }
        };
        frames += 1;
        if frames.is_multiple_of(config.controller.pwm_frequency_hz / 1_000)
            && autotune.is_running()
        {
            view.state = runner.controller.state();
            autotune.task_tick(0.001, Some(&view));
            if autotune.state() == AutoTuneState::OpenLoopPrepare {
                pi_verified = autotune.pi_metrics().is_some();
            }
        }
        let mut requested_stop = false;
        while let Some(request) = autotune.pop_request() {
            if matches!(request, ControlRequest::DisableAndResync) {
                requested_stop = true;
            }
            let applied = apply_request(&mut runner, request, &autotune.motor_param());
            gains_applied &= applied;
        }
        if matches!(config.fixture, Fixture::ReleaseForOpenLoop)
            && runner.plant.locked_rotor
            && autotune.state() == AutoTuneState::OpenLoopRun
        {
            runner.plant.locked_rotor = false;
            released = true;
        }
        if autotune.state() == AutoTuneState::Save
            && let Some(param) = autotune.take_save_request()
        {
            save_requested = true;
            // No Flash emulation: acknowledge only an internally valid request.
            autotune.notify_saved(param.is_commissioned() && param.validate().is_ok());
        }
        let output = if requested_stop || autotune.state() == AutoTuneState::Error {
            ControlOutput::default()
        } else {
            output
        };
        runner.drive(output)?;
        max_speed = max_speed.max(runner.plant.state.mechanical_speed.abs());
        if autotune.state() != previous || frames == 1 {
            previous = autotune.state();
            log(StageRecord {
                time_ns: runner.time_ns,
                state: previous,
                error: autotune.error(),
                mechanical_speed: runner.plant.state.mechanical_speed,
                locked_rotor: runner.plant.locked_rotor,
            });
        }
    }
    let motor = autotune.motor_param();
    let live = runner.controller.config();
    let relative = |found: f32, truth: f64| (f64::from(found) - truth).abs() / truth;
    let accuracy = Accuracy {
        rs_relative: relative(motor.rs, config.motor.resistance),
        ld_relative: relative(motor.ld, config.motor.ld),
        lq_relative: relative(motor.lq, config.motor.lq),
        flux_relative: relative(motor.flux, config.motor.flux),
    };
    Ok(CommissioningResult {
        state: autotune.state(),
        error: autotune.error(),
        motor,
        truth: config.motor,
        frames,
        released,
        max_speed,
        save_requested,
        pi_verified,
        pi_metrics: autotune.pi_metrics(),
        verified_bandwidth_hz: autotune.status().current_bandwidth_hz,
        live_current_kp: live.current_pid.kp,
        live_current_ki: live.current_pid.ki,
        live_speed_kp: live.velocity_pid.kp,
        live_speed_ki: live.velocity_pid.ki,
        gains_applied,
        accuracy,
    })
}
