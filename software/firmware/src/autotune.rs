//! Motor auto-tune state machine (commissioning procedure).
//!
//! Architecture follows the firmware split: the fast loop (`fast_step`)
//! runs from the ADC-frame-driven control task and only performs bounded,
//! deterministic work — current sampling for identification and direct
//! PWM voltage injection. All sequencing, fitting, and decisions run in
//! the slow task (`task_tick`, called at about 1 kHz). Every state has a
//! timeout and explicit success/failure conditions; any protection trip
//! immediately requests the bridge off.
//!
//! During verification states the procedure delegates PWM generation to
//! the regular [`crate::foc_core::FocController`] (via [`FastAction::Delegate`])
//! and drives it through [`ControlRequest`]s, acting as a test conductor.
//! Protection during delegated frames is provided by the controller; during
//! injection frames this module enforces over-current, residual, and bus
//! voltage limits per sample.

use crate::autotune_measure::{
    StepLimits, StepMetrics, analyze_rejected_median, fit_rs, linear_fit,
};
use crate::foc_core::FocConfig;
use crate::foc_math::{AlphaBeta, PhaseCurrents, PhaseDuty, clarke, svpwm};
use crate::interfaces::{
    ControlMode, FaultFlags, MotorState, RawAdcFrame, Telemetry,
};
use crate::motor_param::MotorParam;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum AutoTuneState {
    Idle = 0,
    Init,
    CurrentOffset,
    RsPrepare,
    RsMeasure,
    LPrepare,
    LMeasure,
    CurrentPiCalc,
    CurrentPiVerify,
    OpenLoopPrepare,
    OpenLoopRun,
    FluxMeasure,
    ObserverInit,
    ObserverVerify,
    PlLConfig,
    TransitionVerify,
    Save,
    Done,
    Error,
}

impl AutoTuneState {
    pub const fn name(self) -> &'static str {
        match self {
            | Self::Idle => "idle",
            | Self::Init => "init",
            | Self::CurrentOffset => "current-offset",
            | Self::RsPrepare => "rs-prepare",
            | Self::RsMeasure => "rs-measure",
            | Self::LPrepare => "l-prepare",
            | Self::LMeasure => "l-measure",
            | Self::CurrentPiCalc => "current-pi-calc",
            | Self::CurrentPiVerify => "current-pi-verify",
            | Self::OpenLoopPrepare => "open-loop-prepare",
            | Self::OpenLoopRun => "open-loop-run",
            | Self::FluxMeasure => "flux-measure",
            | Self::ObserverInit => "observer-init",
            | Self::ObserverVerify => "observer-verify",
            | Self::PlLConfig => "pll-config",
            | Self::TransitionVerify => "transition-verify",
            | Self::Save => "save",
            | Self::Done => "done",
            | Self::Error => "error",
        }
    }

    pub const fn is_terminal(self) -> bool {
        matches!(self, Self::Idle | Self::Done | Self::Error)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum AutoTuneError {
    #[default]
    None = 0,
    Aborted,
    NotReady,
    Timeout,
    OverCurrent,
    OverVoltage,
    UnderVoltage,
    ResidualCurrent,
    SensorInvalid,
    FrameOverrun,
    RsFit,
    LFit,
    PiUnstable,
    OpenLoopFailed,
    FluxRange,
    SaveFailed,
}

impl AutoTuneError {
    pub const fn name(self) -> &'static str {
        match self {
            | Self::None => "none",
            | Self::Aborted => "aborted",
            | Self::NotReady => "not-ready",
            | Self::Timeout => "timeout",
            | Self::OverCurrent => "over-current",
            | Self::OverVoltage => "over-voltage",
            | Self::UnderVoltage => "under-voltage",
            | Self::ResidualCurrent => "residual-current",
            | Self::SensorInvalid => "sensor-invalid",
            | Self::FrameOverrun => "frame-overrun",
            | Self::RsFit => "rs-fit",
            | Self::LFit => "l-fit",
            | Self::PiUnstable => "pi-unstable",
            | Self::OpenLoopFailed => "open-loop-failed",
            | Self::FluxRange => "flux-range",
            | Self::SaveFailed => "save-failed",
        }
    }
}

/// What the fast loop wants the bridge to do for this ADC frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FastAction {
    /// Bridge off this frame.
    Off,
    /// Autotune drives the bridge directly with this duty.
    Inject(PhaseDuty),
    /// Run the regular FOC controller for this frame.
    Delegate,
}

/// Requests the auto-tune procedure issues to the motor task, which applies
/// them to the regular FOC controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlRequest {
    DisableAndResync,
    EnableCurrent,
    EnableOpenLoop,
    SetIq(f32),
    SetOpenLoop {
        electrical_velocity: f32,
        q_voltage: f32,
    },
    ApplyMotorParam,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutoTuneStatus {
    pub state: AutoTuneState,
    pub error: AutoTuneError,
    pub progress: u8,
    pub attempts: u8,
    pub current_bandwidth_hz: f32,
    pub rs: f32,
    pub ld: f32,
    pub flux: f32,
    pub commissioned: bool,
}

/// Procedural limits and timings. Electrical I/O scaling comes from
/// [`AutoTuneConfig::from_foc_config`]; the rest are conservative
/// commissioning defaults.
#[derive(Clone, Copy, Debug)]
pub struct AutoTuneConfig {
    pub pwm_frequency_hz: u32,
    pub adc_reference_voltage: f32,
    pub adc_full_scale: f32,
    pub current_sense_gain: f32,
    pub shunt_resistance: f32,
    pub bus_divider_high: f32,
    pub bus_divider_low: f32,
    pub minimum_duty: f32,
    pub maximum_duty: f32,

    pub max_current: f32,
    pub over_current_threshold: f32,
    pub residual_threshold: f32,
    pub battery_cells: u8,
    pub cell_under_voltage: f32,
    pub cell_over_voltage: f32,

    pub offset_frames: u32,
    pub prepare_current: f32,

    pub rs_current_ratios: [f32; 4],
    pub rs_guess: f32,
    pub rs_search_ramp: f32,
    pub rs_min: f32,
    pub rs_max: f32,
    pub rs_min_r_squared: f32,
    pub rs_settle_frames: u32,
    pub rs_sample_frames: u32,

    pub l_guess: f32,
    pub l_min: f32,
    pub l_max: f32,
    pub l_pulse_frames: u32,
    pub l_cooldown_frames: u32,
    pub l_pulse_pairs: u32,
    pub l_min_valid_pulses: u32,
    pub l_pulse_current_ratio: f32,

    pub current_bandwidth_hz: f32,
    pub minimum_current_bandwidth_hz: f32,
    pub bandwidth_rollback: f32,
    pub max_pi_retries: u8,
    pub pi_step_ratio: f32,
    pub pi_prep_timeout: f32,
    pub pi_window: f32,
    pub step_limits: StepLimits,

    pub open_loop_hz: f32,
    pub open_loop_current_ratio: f32,
    pub open_loop_voltage_ramp: f32,
    pub open_loop_max_voltage_ratio: f32,
    pub open_loop_ratio_tolerance: f32,

    pub flux_time: f32,
    pub flux_stride: u32,
    pub flux_min: f32,
    pub flux_max: f32,

    pub observer_bandwidth_hz: f32,

    pub state_timeout: f32,
}

impl AutoTuneConfig {
    /// Electrical scaling and protection limits follow the running
    /// configuration; procedure timings keep documented defaults.
    pub fn from_foc_config(config: &FocConfig) -> Self {
        Self {
            pwm_frequency_hz: config.pwm_frequency_hz,
            adc_reference_voltage: config.adc_reference_voltage,
            adc_full_scale: config.adc_full_scale,
            current_sense_gain: config.current_sense_gain,
            shunt_resistance: config.shunt_resistance,
            bus_divider_high: config.bus_divider_high,
            bus_divider_low: config.bus_divider_low,
            minimum_duty: config.minimum_duty,
            maximum_duty: config.maximum_duty,
            max_current: (config.current_limit * 0.5)
                .min(config.over_current_threshold * 0.6),
            over_current_threshold: config.over_current_threshold,
            residual_threshold: config.residual_threshold,
            battery_cells: config.battery_cells,
            cell_under_voltage: config.cell_under_voltage,
            cell_over_voltage: config.cell_over_voltage,
            ..Self::default()
        }
    }
}

impl Default for AutoTuneConfig {
    fn default() -> Self {
        Self {
            pwm_frequency_hz: 100_000,
            adc_reference_voltage: 3.3,
            adc_full_scale: 4_095.0,
            current_sense_gain: 50.0,
            shunt_resistance: 0.000_5,
            bus_divider_high: 100_000.0,
            bus_divider_low: 10_000.0,
            minimum_duty: 0.05,
            maximum_duty: 0.95,
            max_current: 10.0,
            over_current_threshold: 55.0,
            residual_threshold: 3.0,
            battery_cells: 6,
            cell_under_voltage: 3.0,
            cell_over_voltage: 4.25,
            offset_frames: 1_024,
            prepare_current: 0.2,
            rs_current_ratios: [0.05, 0.10, 0.15, 0.20],
            rs_guess: 0.05,
            rs_search_ramp: 150.0,
            rs_min: 0.002,
            rs_max: 2.0,
            rs_min_r_squared: 0.98,
            rs_settle_frames: 8_000,
            rs_sample_frames: 4_000,
            l_guess: 0.000_3,
            l_min: 5.0e-6,
            l_max: 0.05,
            l_pulse_frames: 32,
            l_cooldown_frames: 128,
            l_pulse_pairs: 4,
            l_min_valid_pulses: 5,
            l_pulse_current_ratio: 0.10,
            current_bandwidth_hz: 500.0,
            minimum_current_bandwidth_hz: 100.0,
            bandwidth_rollback: 0.75,
            max_pi_retries: 3,
            pi_step_ratio: 0.08,
            pi_prep_timeout: 0.3,
            pi_window: 0.2,
            step_limits: StepLimits::default(),
            open_loop_hz: 5.0,
            open_loop_current_ratio: 0.15,
            open_loop_voltage_ramp: 4.0,
            open_loop_max_voltage_ratio: 0.45,
            open_loop_ratio_tolerance: 0.2,
            flux_time: 0.3,
            flux_stride: 3,
            flux_min: 1.0e-4,
            flux_max: 0.5,
            observer_bandwidth_hz: 50.0,
            state_timeout: 12.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RsPhase {
    /// First point only: ramp the voltage until the current approaches the
    /// smallest target so a bad initial Rs guess cannot skip every point.
    Ramp,
    Settle,
    Sample,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LPulsePhase {
    Idle,
    Pulse,
    Cooldown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PiPhase {
    WaitRunning,
    Baseline,
    Step,
    RetryWait,
}

const L_SAMPLE_CAPACITY: usize = 128;
const STEP_SAMPLE_CAPACITY: usize = 256;
const FLUX_SAMPLE_CAPACITY: usize = 128;
const MAX_RS_POINTS: usize = 4;
const MAX_L_ESTIMATES: usize = 8;
const CONTROL_REQUEST_CAPACITY: usize = 4;

pub struct AutoTuneController {
    config: AutoTuneConfig,
    state: AutoTuneState,
    error: AutoTuneError,
    motor: MotorParam,
    elapsed: f32,
    frame_count: u32,
    attempts: u8,
    current_bandwidth_hz: f32,
    step_target: f32,

    offsets: [f32; 3],
    offset_sum: [f64; 3],
    offset_count: u32,

    last_currents: PhaseCurrents,
    last_bus_voltage: f32,
    last_sequence: Option<u32>,

    rs_phase: RsPhase,
    rs_phase_frames: u32,
    rs_point_index: usize,
    rs_points: [(f32, f32); MAX_RS_POINTS],
    rs_point_voltage: f32,
    rs_sum_current: f64,
    rs_sample_count: u32,
    rs_rescale: bool,
    offsets_valid: bool,

    l_pulse_phase: LPulsePhase,
    l_pulse_frames_left: u32,
    l_pulse_index: u32,
    l_pulse_polarity: f32,
    l_pulse_voltage: f32,
    l_samples: [f32; L_SAMPLE_CAPACITY],
    l_sample_count: usize,
    l_pulse_ready: bool,
    l_estimates: [f32; MAX_L_ESTIMATES],
    l_valid_pulses: u32,

    pi_phase: PiPhase,
    pi_phase_elapsed: f32,
    pi_window_elapsed: f32,
    pi_step_samples: [(f32, f32); STEP_SAMPLE_CAPACITY],
    pi_sample_count: usize,
    pi_last_metrics: Option<StepMetrics>,
    sensor_invalid_time: f32,

    open_loop_velocity: f32,
    open_loop_voltage: f32,
    open_loop_target_current: f32,
    open_loop_settled_time: f32,
    open_loop_saturated_time: f32,

    flux_elapsed: f32,
    flux_samples: [f32; FLUX_SAMPLE_CAPACITY],
    flux_sample_count: usize,

    /// Pending control requests, drained FIFO by the motor task. Small
    /// fixed capacity; the procedure never queues more than a few.
    requests: [Option<ControlRequest>; CONTROL_REQUEST_CAPACITY],
    request_write: usize,
    request_read: usize,
    save_request: Option<MotorParam>,
}

impl AutoTuneController {
    pub fn new(config: AutoTuneConfig) -> Self {
        Self {
            config,
            state: AutoTuneState::Idle,
            error: AutoTuneError::None,
            motor: MotorParam::default(),
            elapsed: 0.0,
            frame_count: 0,
            attempts: 0,
            current_bandwidth_hz: config.current_bandwidth_hz,
            step_target: 0.0,
            offsets: [0.0; 3],
            offset_sum: [0.0; 3],
            offset_count: 0,
            last_currents: PhaseCurrents::default(),
            last_bus_voltage: 0.0,
            last_sequence: None,
            rs_phase: RsPhase::Settle,
            rs_phase_frames: 0,
            rs_point_index: 0,
            rs_points: [(0.0, 0.0); MAX_RS_POINTS],
            rs_point_voltage: 0.0,
            rs_sum_current: 0.0,
            rs_sample_count: 0,
            rs_rescale: true,
            offsets_valid: false,
            l_pulse_phase: LPulsePhase::Idle,
            l_pulse_frames_left: 0,
            l_pulse_index: 0,
            l_pulse_polarity: 1.0,
            l_pulse_voltage: 0.0,
            l_samples: [0.0; L_SAMPLE_CAPACITY],
            l_sample_count: 0,
            l_pulse_ready: false,
            l_estimates: [0.0; MAX_L_ESTIMATES],
            l_valid_pulses: 0,
            pi_phase: PiPhase::WaitRunning,
            pi_phase_elapsed: 0.0,
            pi_window_elapsed: 0.0,
            pi_step_samples: [(0.0, 0.0); STEP_SAMPLE_CAPACITY],
            pi_sample_count: 0,
            pi_last_metrics: None,
            sensor_invalid_time: 0.0,
            open_loop_velocity: 0.0,
            open_loop_voltage: 0.0,
            open_loop_target_current: 0.0,
            open_loop_settled_time: 0.0,
            open_loop_saturated_time: 0.0,
            flux_elapsed: 0.0,
            flux_samples: [0.0; FLUX_SAMPLE_CAPACITY],
            flux_sample_count: 0,
            requests: [None; CONTROL_REQUEST_CAPACITY],
            request_write: 0,
            request_read: 0,
            save_request: None,
        }
    }

    pub fn is_running(&self) -> bool {
        !self.state.is_terminal()
    }

    pub const fn state(&self) -> AutoTuneState {
        self.state
    }

    pub const fn config(&self) -> AutoTuneConfig {
        self.config
    }

    pub const fn error(&self) -> AutoTuneError {
        self.error
    }

    pub const fn motor_param(&self) -> MotorParam {
        self.motor
    }

    pub fn pi_metrics(&self) -> Option<StepMetrics> {
        self.pi_last_metrics
    }

    pub fn status(&self) -> AutoTuneStatus {
        AutoTuneStatus {
            state: self.state,
            error: self.error,
            progress: self.progress(),
            attempts: self.attempts,
            current_bandwidth_hz: self.current_bandwidth_hz,
            rs: self.motor.rs,
            ld: self.motor.ld,
            flux: self.motor.flux,
            commissioned: self.motor.is_commissioned(),
        }
    }

    /// Starts a procedure from a terminal state. `seed` supplies values the
    /// procedure does not measure (pole pairs, speed gains).
    pub fn start(&mut self, seed: MotorParam) -> bool {
        if self.is_running() || seed.validate().is_err() {
            return false;
        }
        self.motor = seed;
        self.motor.valid_flag = 0;
        self.motor.checksum = 0;
        self.error = AutoTuneError::None;
        self.attempts = 0;
        self.current_bandwidth_hz = self.config.current_bandwidth_hz;
        self.offsets = [0.0; 3];
        self.offsets_valid = false;
        self.last_sequence = None;
        self.requests = [None; CONTROL_REQUEST_CAPACITY];
        self.request_write = 0;
        self.request_read = 0;
        self.save_request = None;
        self.enter(AutoTuneState::Init);
        true
    }

    /// Aborts a running procedure and returns to idle.
    pub fn stop(&mut self) {
        if self.is_running() {
            self.error = AutoTuneError::Aborted;
            self.push_request(ControlRequest::DisableAndResync);
            self.state = AutoTuneState::Idle;
        }
    }

    /// Queues a control request for the motor task. The capacity bound is
    /// never reached by the procedure (at most three are queued at once).
    fn push_request(&mut self, request: ControlRequest) {
        if self.request_write - self.request_read < CONTROL_REQUEST_CAPACITY {
            self.requests[self.request_write % CONTROL_REQUEST_CAPACITY] =
                Some(request);
            self.request_write += 1;
        }
    }

    pub fn has_pending_requests(&self) -> bool {
        self.request_read < self.request_write
    }

    pub fn pop_request(&mut self) -> Option<ControlRequest> {
        let request = if self.has_pending_requests() {
            self.requests[self.request_read % CONTROL_REQUEST_CAPACITY].take()
        } else {
            None
        };
        if request.is_some() {
            self.request_read += 1;
        }
        request
    }

    pub fn take_save_request(&mut self) -> Option<MotorParam> {
        self.save_request.take()
    }

    pub fn notify_saved(&mut self, saved: bool) {
        if self.state != AutoTuneState::Save {
            return;
        }
        if saved {
            self.push_request(ControlRequest::DisableAndResync);
            self.state = AutoTuneState::Done;
        } else {
            self.fail(AutoTuneError::SaveFailed);
        }
    }

    fn enter(&mut self, state: AutoTuneState) {
        self.state = state;
        self.elapsed = 0.0;
        self.frame_count = 0;
        match state {
            | AutoTuneState::CurrentOffset => {
                self.offset_sum = [0.0; 3];
                self.offset_count = 0;
                self.offsets_valid = false;
            },
            | AutoTuneState::RsPrepare => {
                self.rs_point_index = 0;
                self.rs_rescale = true;
                self.rs_point_voltage = 0.0;
            },
            | AutoTuneState::RsMeasure => {
                self.rs_phase = if self.rs_rescale {
                    RsPhase::Ramp
                } else {
                    RsPhase::Settle
                };
                self.rs_phase_frames = self.config.rs_settle_frames;
                self.rs_sum_current = 0.0;
                self.rs_sample_count = 0;
                self.rs_point_voltage = self.rs_point_voltage(0);
            },
            | AutoTuneState::LPrepare => {
                self.l_pulse_phase = LPulsePhase::Idle;
                self.l_pulse_index = 0;
                self.l_pulse_polarity = 1.0;
                self.l_valid_pulses = 0;
                self.l_estimates = [0.0; MAX_L_ESTIMATES];
                self.l_pulse_voltage = self.l_pulse_voltage();
            },
            | AutoTuneState::LMeasure => {
                self.l_pulse_phase = LPulsePhase::Idle;
                self.l_pulse_frames_left = 0;
                self.l_sample_count = 0;
                self.l_pulse_ready = false;
            },
            | AutoTuneState::CurrentPiVerify => {
                self.pi_phase = PiPhase::WaitRunning;
                self.pi_phase_elapsed = 0.0;
                self.pi_window_elapsed = 0.0;
                self.pi_sample_count = 0;
                self.sensor_invalid_time = 0.0;
            },
            | AutoTuneState::OpenLoopPrepare => {
                self.open_loop_velocity =
                    core::f32::consts::TAU * self.config.open_loop_hz;
                self.open_loop_voltage = 0.0;
                self.open_loop_target_current =
                    self.config.open_loop_current_ratio
                        * self.config.max_current;
                self.open_loop_settled_time = 0.0;
                self.open_loop_saturated_time = 0.0;
            },
            | AutoTuneState::FluxMeasure => {
                self.flux_elapsed = 0.0;
                self.flux_sample_count = 0;
                self.flux_samples = [0.0; FLUX_SAMPLE_CAPACITY];
            },
            | _ => {},
        }
    }

    fn fail(&mut self, error: AutoTuneError) {
        self.error = error;
        self.push_request(ControlRequest::DisableAndResync);
        self.state = AutoTuneState::Error;
    }

    fn progress(&self) -> u8 {
        let base = match self.state {
            | AutoTuneState::Idle => 0,
            | AutoTuneState::Init => 2,
            | AutoTuneState::CurrentOffset => 5,
            | AutoTuneState::RsPrepare => 10,
            | AutoTuneState::RsMeasure => 12,
            | AutoTuneState::LPrepare => 32,
            | AutoTuneState::LMeasure => 34,
            | AutoTuneState::CurrentPiCalc => 52,
            | AutoTuneState::CurrentPiVerify => 55,
            | AutoTuneState::OpenLoopPrepare => 70,
            | AutoTuneState::OpenLoopRun => 73,
            | AutoTuneState::FluxMeasure => 78,
            | AutoTuneState::ObserverInit => 84,
            | AutoTuneState::ObserverVerify => 88,
            | AutoTuneState::PlLConfig => 91,
            | AutoTuneState::TransitionVerify => 93,
            | AutoTuneState::Save => 96,
            | AutoTuneState::Done => 100,
            | AutoTuneState::Error => 0,
        };
        let span = match self.state {
            | AutoTuneState::RsMeasure => 20,
            | AutoTuneState::LMeasure => 18,
            | AutoTuneState::CurrentPiVerify => 14,
            | AutoTuneState::FluxMeasure => 5,
            | _ => 1,
        };
        let fraction = match self.state {
            | AutoTuneState::RsMeasure => {
                self.rs_point_index as f32
                    / self.config.rs_current_ratios.len() as f32
            },
            | AutoTuneState::LMeasure => {
                self.l_valid_pulses as f32
                    / (self.config.l_pulse_pairs * 2) as f32
            },
            | AutoTuneState::CurrentPiVerify => {
                (self.pi_window_elapsed / self.config.pi_window).min(1.0)
            },
            | AutoTuneState::FluxMeasure => {
                (self.flux_elapsed / self.config.flux_time).min(1.0)
            },
            | _ => 0.0,
        };
        (base as f32 + span as f32 * fraction.clamp(0.0, 1.0)) as u8
    }

    fn amperes_per_count(&self) -> f32 {
        self.config.adc_reference_voltage
            / self.config.adc_full_scale
            / (self.config.current_sense_gain * self.config.shunt_resistance)
    }

    fn raw_to_bus_voltage(&self, raw: u16) -> f32 {
        f32::from(raw)
            * (self.config.adc_reference_voltage / self.config.adc_full_scale)
            * ((self.config.bus_divider_high + self.config.bus_divider_low)
                / self.config.bus_divider_low)
    }

    fn bus_limits(&self) -> (f32, f32) {
        let cells = f32::from(self.config.battery_cells);
        (
            cells * self.config.cell_under_voltage,
            cells * self.config.cell_over_voltage,
        )
    }

    fn rs_point_voltage(&self, index: usize) -> f32 {
        let ratios = self.config.rs_current_ratios;
        let ratio = ratios[index.min(ratios.len() - 1)];
        let guess = if self.rs_rescale {
            self.config.rs_guess
        } else {
            self.motor.rs.max(self.config.rs_guess * 0.2)
        };
        (guess * ratio * self.config.max_current).clamp(0.02, 6.0)
    }

    fn l_pulse_voltage(&self) -> f32 {
        let target_rise =
            self.config.l_pulse_current_ratio * self.config.max_current;
        let pulse_time = self.config.l_pulse_frames as f32
            / self.config.pwm_frequency_hz as f32;
        let inductive = self.config.l_guess * target_rise / pulse_time;
        let resistive = self.motor.rs * 0.5 * self.config.max_current;
        (inductive + resistive).clamp(0.3, 12.0)
    }

    fn peak_current(&self) -> f32 {
        self.last_currents
            .a
            .abs()
            .max(self.last_currents.b.abs())
            .max(self.last_currents.c.abs())
    }

    /// Fast path: called once per ADC frame (100 kHz). Performs protection,
    /// measurement sampling, and injection duty computation only.
    pub fn fast_step(&mut self, raw: &RawAdcFrame) -> FastAction {
        if !self.is_running() {
            return FastAction::Off;
        }
        self.frame_count = self.frame_count.wrapping_add(1);

        // Frame continuity is checked here whenever the regular controller
        // is not consuming frames itself (delegate states).
        let delegating = matches!(
            self.state,
            AutoTuneState::CurrentPiVerify
                | AutoTuneState::OpenLoopRun
                | AutoTuneState::FluxMeasure
        );
        if let Some(previous) = self.last_sequence
            && raw.sequence != previous.wrapping_add(1)
            && !delegating
        {
            self.fail(AutoTuneError::FrameOverrun);
            return FastAction::Off;
        }
        self.last_sequence = Some(raw.sequence);

        let bus_voltage = self.raw_to_bus_voltage(raw.bus_voltage);
        self.last_bus_voltage = bus_voltage;
        let (bus_min, bus_max) = self.bus_limits();
        if bus_voltage < bus_min {
            self.fail(AutoTuneError::UnderVoltage);
            return FastAction::Off;
        }
        if bus_voltage > bus_max {
            self.fail(AutoTuneError::OverVoltage);
            return FastAction::Off;
        }

        let currents = if self.state == AutoTuneState::CurrentOffset {
            self.offset_sum[0] += f64::from(raw.phase_a);
            self.offset_sum[1] += f64::from(raw.phase_b);
            self.offset_sum[2] += f64::from(raw.phase_c);
            self.offset_count += 1;
            PhaseCurrents::default()
        } else {
            let amperes_per_count = self.amperes_per_count();
            PhaseCurrents {
                a: (f32::from(raw.phase_a) - self.offsets[0])
                    * amperes_per_count,
                b: (f32::from(raw.phase_b) - self.offsets[1])
                    * amperes_per_count,
                c: (f32::from(raw.phase_c) - self.offsets[2])
                    * amperes_per_count,
            }
        };
        self.last_currents = currents;

        // Current-based protection needs calibrated offsets; before that the
        // raw values would read as a huge common-mode current.
        if self.offsets_valid {
            let residual = (currents.a + currents.b + currents.c).abs();
            if residual > self.config.residual_threshold {
                self.fail(AutoTuneError::ResidualCurrent);
                return FastAction::Off;
            }
            let peak =
                currents.a.abs().max(currents.b.abs()).max(currents.c.abs());
            if peak > self.config.over_current_threshold {
                self.fail(AutoTuneError::OverCurrent);
                return FastAction::Off;
            }
        }

        match self.state {
            | AutoTuneState::RsMeasure => self.fast_rs_measure(currents),
            | AutoTuneState::LMeasure => self.fast_l_measure(currents),
            | AutoTuneState::CurrentPiVerify
            | AutoTuneState::OpenLoopRun
            | AutoTuneState::FluxMeasure => FastAction::Delegate,
            | _ => FastAction::Off,
        }
    }

    fn inject_alpha(&self, voltage: f32) -> FastAction {
        let duty = svpwm(
            AlphaBeta {
                alpha: voltage,
                beta: 0.0,
            },
            self.last_bus_voltage,
            self.config.minimum_duty,
            self.config.maximum_duty,
        );
        FastAction::Inject(duty)
    }

    fn fast_rs_measure(&mut self, currents: PhaseCurrents) -> FastAction {
        match self.rs_phase {
            | RsPhase::Ramp => {
                let frame = 1.0 / self.config.pwm_frequency_hz as f32;
                self.rs_point_voltage = (self.rs_point_voltage
                    + self.config.rs_search_ramp * frame)
                    .min(6.0);
                let alpha = clarke(currents).alpha;
                let first_target =
                    self.config.rs_current_ratios[0] * self.config.max_current;
                if alpha.abs() >= 0.9 * first_target
                    || self.rs_point_voltage >= 6.0
                {
                    self.rs_phase = RsPhase::Settle;
                    self.rs_phase_frames = self.config.rs_settle_frames;
                }
            },
            | RsPhase::Settle => {
                self.rs_phase_frames = self.rs_phase_frames.saturating_sub(1);
                if self.rs_phase_frames == 0 {
                    self.rs_phase = RsPhase::Sample;
                    self.rs_phase_frames = self.config.rs_sample_frames;
                    self.rs_sum_current = 0.0;
                    self.rs_sample_count = 0;
                }
            },
            | RsPhase::Sample => {
                let alpha = clarke(currents).alpha;
                self.rs_sum_current += f64::from(alpha);
                self.rs_sample_count += 1;
                self.rs_phase_frames = self.rs_phase_frames.saturating_sub(1);
            },
        }
        self.inject_alpha(self.rs_point_voltage)
    }

    fn fast_l_measure(&mut self, currents: PhaseCurrents) -> FastAction {
        let alpha = clarke(currents).alpha;
        let pulses_total = self.config.l_pulse_pairs * 2;
        match self.l_pulse_phase {
            | LPulsePhase::Idle => {
                if self.l_pulse_index >= pulses_total {
                    // All pulses issued; wait for the slow task to finish.
                } else {
                    self.l_pulse_frames_left =
                        self.l_pulse_frames_left.saturating_sub(1);
                    if self.l_pulse_frames_left == 0 {
                        self.l_pulse_phase = LPulsePhase::Pulse;
                        self.l_pulse_frames_left = self.config.l_pulse_frames;
                        self.l_sample_count = 0;
                    }
                }
            },
            | LPulsePhase::Pulse => {
                if self.l_sample_count < L_SAMPLE_CAPACITY {
                    self.l_samples[self.l_sample_count] = alpha;
                    self.l_sample_count += 1;
                }
                self.l_pulse_frames_left =
                    self.l_pulse_frames_left.saturating_sub(1);
                if self.l_pulse_frames_left == 0 {
                    self.l_pulse_phase = LPulsePhase::Cooldown;
                    self.l_pulse_frames_left = self.config.l_cooldown_frames;
                    self.l_pulse_ready = true;
                }
            },
            | LPulsePhase::Cooldown => {
                self.l_pulse_frames_left =
                    self.l_pulse_frames_left.saturating_sub(1);
                if self.l_pulse_frames_left == 0 {
                    self.l_pulse_index += 1;
                    if self.l_pulse_index >= pulses_total {
                        self.l_pulse_phase = LPulsePhase::Idle;
                        self.l_pulse_frames_left = 0;
                    } else {
                        self.l_pulse_polarity = -self.l_pulse_polarity;
                        self.l_pulse_phase = LPulsePhase::Idle;
                        self.l_pulse_frames_left = 16;
                    }
                }
            },
        }
        let voltage = match self.l_pulse_phase {
            | LPulsePhase::Pulse => {
                self.l_pulse_polarity * self.l_pulse_voltage
            },
            | _ => 0.0,
        };
        self.inject_alpha(voltage)
    }

    /// Slow path: called at about 1 kHz. Runs sequencing, fitting, and
    /// decisions. `control` is the latest controller view; the motor task
    /// refreshes its `state` field from the live controller.
    pub fn task_tick(&mut self, dt: f32, control: Option<&Telemetry>) {
        if !self.is_running() || dt <= 0.0 || !dt.is_finite() {
            return;
        }
        self.elapsed += dt;

        if let Some(telemetry) = control {
            if telemetry.sensor_valid {
                self.sensor_invalid_time = 0.0;
            } else {
                self.sensor_invalid_time += dt;
            }
        }

        match self.state {
            | AutoTuneState::Init => self.tick_init(),
            | AutoTuneState::CurrentOffset => self.tick_current_offset(),
            | AutoTuneState::RsPrepare => self.tick_rs_prepare(),
            | AutoTuneState::RsMeasure => self.tick_rs_measure(),
            | AutoTuneState::LPrepare => self.tick_l_prepare(),
            | AutoTuneState::LMeasure => self.tick_l_measure(),
            | AutoTuneState::CurrentPiCalc => self.tick_current_pi_calc(),
            | AutoTuneState::CurrentPiVerify => {
                self.tick_current_pi_verify(dt, control);
            },
            | AutoTuneState::OpenLoopPrepare => {
                self.tick_open_loop_prepare();
            },
            | AutoTuneState::OpenLoopRun => {
                self.tick_open_loop_run(dt, control);
            },
            | AutoTuneState::FluxMeasure => self.tick_flux_measure(dt, control),
            | AutoTuneState::ObserverInit => self.tick_observer_init(),
            | AutoTuneState::ObserverVerify => self.tick_observer_verify(),
            | AutoTuneState::PlLConfig => {
                self.enter(AutoTuneState::TransitionVerify);
            },
            | AutoTuneState::TransitionVerify => self.tick_transition_verify(),
            | AutoTuneState::Save
            | AutoTuneState::Idle
            | AutoTuneState::Done
            | AutoTuneState::Error => {},
        }

        if self.is_running() && self.elapsed > self.config.state_timeout {
            self.fail(AutoTuneError::Timeout);
        }
    }

    fn tick_init(&mut self) {
        self.enter(AutoTuneState::CurrentOffset);
    }

    fn tick_current_offset(&mut self) {
        if self.offset_count >= self.config.offset_frames {
            let count = self.offset_count as f32;
            self.offsets = [
                (self.offset_sum[0] / f64::from(count)) as f32,
                (self.offset_sum[1] / f64::from(count)) as f32,
                (self.offset_sum[2] / f64::from(count)) as f32,
            ];
            self.offsets_valid = true;
            self.enter(AutoTuneState::RsPrepare);
        }
    }

    fn tick_rs_prepare(&mut self) {
        if self.elapsed > 0.02
            && self.peak_current() < self.config.prepare_current
        {
            self.enter(AutoTuneState::RsMeasure);
        }
    }

    fn tick_rs_measure(&mut self) {
        if self.rs_phase != RsPhase::Sample
            || self.rs_sample_count == 0
            || self.rs_phase_frames > 0
        {
            return;
        }
        let mean_current =
            (self.rs_sum_current / f64::from(self.rs_sample_count)) as f32;
        let point = (mean_current.abs(), self.rs_point_voltage);
        let target = self.config.rs_current_ratios[self.rs_point_index.min(3)]
            * self.config.max_current;
        let plausible = point.0 > 0.3 * target && point.0 < 2.0 * target;
        if plausible {
            if self.rs_rescale && point.0 > 0.0 {
                // Re-scale remaining point voltages from the first
                // measurement so a bad initial guess cannot saturate the
                // winding.
                self.motor.rs = (point.1 / point.0)
                    .clamp(self.config.rs_min, self.config.rs_max);
                self.rs_rescale = false;
            }
            self.rs_points[self.rs_point_index.min(MAX_RS_POINTS - 1)] = point;
            self.rs_point_index += 1;
        }

        if self.rs_point_index >= self.config.rs_current_ratios.len() {
            let slice =
                &self.rs_points[..self.rs_point_index.min(MAX_RS_POINTS)];
            match fit_rs(slice) {
                | Some(fit)
                    if fit.rs >= self.config.rs_min
                        && fit.rs <= self.config.rs_max
                        && fit.r_squared >= self.config.rs_min_r_squared =>
                {
                    self.motor.rs = fit.rs;
                    self.enter(AutoTuneState::LPrepare);
                },
                | _ => self.fail(AutoTuneError::RsFit),
            }
        } else {
            self.rs_point_voltage = self.rs_point_voltage(self.rs_point_index);
            self.rs_phase = RsPhase::Settle;
            self.rs_phase_frames = self.config.rs_settle_frames;
            self.rs_sum_current = 0.0;
            self.rs_sample_count = 0;
        }
    }

    fn tick_l_prepare(&mut self) {
        if self.elapsed > 0.02
            && self.peak_current() < self.config.prepare_current
        {
            self.enter(AutoTuneState::LMeasure);
        }
    }

    fn tick_l_measure(&mut self) {
        if self.l_pulse_ready {
            self.l_pulse_ready = false;
            if let Some(inductance) = self.fit_last_pulse()
                && self.l_valid_pulses < MAX_L_ESTIMATES as u32
            {
                self.l_estimates[self.l_valid_pulses as usize] = inductance;
                self.l_valid_pulses += 1;
            }
        }
        let pulses_total = self.config.l_pulse_pairs * 2;
        if self.l_pulse_index >= pulses_total
            && self.l_pulse_phase == LPulsePhase::Idle
            && self.l_pulse_frames_left == 0
            && !self.l_pulse_ready
        {
            self.finish_l_measure();
        }
    }

    fn fit_last_pulse(&self) -> Option<f32> {
        let count = self.l_sample_count;
        if count < 4 {
            return None;
        }
        let dt = 1.0 / self.config.pwm_frequency_hz as f32;
        let mut points = [(0.0f32, 0.0f32); L_SAMPLE_CAPACITY];
        for (index, sample) in self.l_samples[..count].iter().enumerate() {
            points[index] = (index as f32 * dt, *sample);
        }
        let fit = linear_fit(&points[..count])?;
        let mean_current =
            self.l_samples[..count].iter().sum::<f32>() / count as f32;
        let voltage = self.l_pulse_polarity * self.l_pulse_voltage;
        let slope = fit.slope;
        if slope * voltage <= 0.0 {
            return None;
        }
        let inductance = (voltage - self.motor.rs * mean_current) / slope;
        if !inductance.is_finite()
            || inductance.abs() < self.config.l_min * 0.1
            || inductance.abs() > self.config.l_max * 10.0
        {
            return None;
        }
        Some(inductance.abs())
    }

    fn finish_l_measure(&mut self) {
        let count = self.l_valid_pulses as usize;
        let Some(inductance) =
            analyze_rejected_median(&self.l_estimates[..count])
        else {
            self.fail(AutoTuneError::LFit);
            return;
        };
        if !(self.config.l_min..=self.config.l_max).contains(&inductance) {
            self.fail(AutoTuneError::LFit);
            return;
        }
        self.motor.ld = inductance;
        self.motor.lq = inductance;
        self.enter(AutoTuneState::CurrentPiCalc);
    }

    fn tick_current_pi_calc(&mut self) {
        let bandwidth = core::f32::consts::TAU * self.current_bandwidth_hz;
        self.motor.id_kp = self.motor.ld * bandwidth;
        self.motor.id_ki = self.motor.rs * bandwidth;
        self.motor.iq_kp = self.motor.lq * bandwidth;
        self.motor.iq_ki = self.motor.rs * bandwidth;
        // Resynchronize the controller before handing control back: it has
        // not consumed ADC frames during the injection phases.
        self.push_request(ControlRequest::ApplyMotorParam);
        self.push_request(ControlRequest::DisableAndResync);
        self.push_request(ControlRequest::EnableCurrent);
        self.step_target = self.config.pi_step_ratio * self.config.max_current;
        self.enter(AutoTuneState::CurrentPiVerify);
    }

    fn tick_current_pi_verify(&mut self, dt: f32, control: Option<&Telemetry>) {
        self.pi_phase_elapsed += dt;
        let Some(telemetry) = control else {
            if self.pi_phase_elapsed > self.config.pi_prep_timeout {
                self.fail(AutoTuneError::NotReady);
            }
            return;
        };
        if self.sensor_invalid_time > 0.05 {
            self.fail(AutoTuneError::SensorInvalid);
            return;
        }
        if telemetry.state == MotorState::Fault {
            let error = if telemetry.faults.contains(FaultFlags::OVER_CURRENT) {
                AutoTuneError::OverCurrent
            } else if telemetry.faults.contains(FaultFlags::OVER_VOLTAGE) {
                AutoTuneError::OverVoltage
            } else if telemetry.faults.contains(FaultFlags::UNDER_VOLTAGE) {
                AutoTuneError::UnderVoltage
            } else {
                AutoTuneError::NotReady
            };
            self.fail(error);
            return;
        }

        match self.pi_phase {
            | PiPhase::WaitRunning => {
                if telemetry.state == MotorState::Running(ControlMode::Current)
                {
                    self.pi_phase = PiPhase::Baseline;
                    self.pi_phase_elapsed = 0.0;
                } else if self.pi_phase_elapsed > self.config.pi_prep_timeout {
                    self.fail(AutoTuneError::NotReady);
                }
            },
            | PiPhase::Baseline => {
                if self.pi_phase_elapsed >= 0.03 {
                    self.push_request(ControlRequest::SetIq(self.step_target));
                    self.pi_phase = PiPhase::Step;
                    self.pi_phase_elapsed = 0.0;
                    self.pi_window_elapsed = 0.0;
                    self.pi_sample_count = 0;
                }
            },
            | PiPhase::Step => {
                self.pi_window_elapsed += dt;
                if self.pi_sample_count < STEP_SAMPLE_CAPACITY {
                    self.pi_step_samples[self.pi_sample_count] =
                        (self.pi_window_elapsed, telemetry.current_dq.q);
                    self.pi_sample_count += 1;
                }
                if self.pi_window_elapsed >= self.config.pi_window {
                    self.evaluate_step_response();
                }
            },
            | PiPhase::RetryWait => {
                if telemetry.state == MotorState::Idle
                    && self.peak_current() < self.config.prepare_current
                {
                    self.enter(AutoTuneState::CurrentPiCalc);
                } else if self.pi_phase_elapsed > self.config.pi_prep_timeout {
                    self.fail(AutoTuneError::NotReady);
                }
            },
        }
    }

    fn evaluate_step_response(&mut self) {
        let samples = &self.pi_step_samples[..self.pi_sample_count];
        let window = self.config.pi_window;
        let Some(metrics) = StepMetrics::analyze(samples, self.step_target)
        else {
            self.fail(AutoTuneError::PiUnstable);
            return;
        };
        self.pi_last_metrics = Some(metrics);
        if metrics.passes(&self.config.step_limits, self.step_target, window) {
            self.push_request(ControlRequest::SetIq(0.0));
            self.enter(AutoTuneState::OpenLoopPrepare);
            return;
        }
        self.attempts += 1;
        if self.attempts > self.config.max_pi_retries
            || self.current_bandwidth_hz * self.config.bandwidth_rollback
                < self.config.minimum_current_bandwidth_hz
        {
            self.fail(AutoTuneError::PiUnstable);
            return;
        }
        self.current_bandwidth_hz *= self.config.bandwidth_rollback;
        self.push_request(ControlRequest::DisableAndResync);
        self.pi_phase = PiPhase::RetryWait;
        self.pi_phase_elapsed = 0.0;
    }

    fn tick_open_loop_prepare(&mut self) {
        if self.elapsed > 0.02
            && self.peak_current() < self.config.prepare_current
        {
            self.push_request(ControlRequest::DisableAndResync);
            self.push_request(ControlRequest::EnableOpenLoop);
            self.enter(AutoTuneState::OpenLoopRun);
        }
    }

    fn open_loop_max_voltage(&self) -> f32 {
        self.config.open_loop_max_voltage_ratio
            * 0.5
            * self.last_bus_voltage.max(1.0)
    }

    fn tick_open_loop_run(&mut self, dt: f32, control: Option<&Telemetry>) {
        let Some(telemetry) = control else {
            return;
        };
        if telemetry.state != MotorState::Running(ControlMode::OpenLoop) {
            return;
        }
        let iq = telemetry.current_dq.q.abs();
        let reached = iq >= 0.8 * self.open_loop_target_current;
        let max_voltage = self.open_loop_max_voltage();
        if !reached {
            if self.open_loop_voltage < max_voltage {
                self.open_loop_voltage = (self.open_loop_voltage
                    + self.config.open_loop_voltage_ramp * dt)
                    .min(max_voltage);
                self.push_request(ControlRequest::SetOpenLoop {
                    electrical_velocity: self.open_loop_velocity,
                    q_voltage: self.open_loop_voltage,
                });
                self.open_loop_saturated_time = 0.0;
            } else {
                self.open_loop_saturated_time += dt;
                if self.open_loop_saturated_time > 0.5 {
                    self.fail(AutoTuneError::OpenLoopFailed);
                }
            }
        } else {
            // Synchronization check against the rotor sensor: the imposed
            // electrical speed must match the measured mechanical speed.
            let ratio = (telemetry.mechanical_velocity
                * f32::from(self.motor.pole_pairs)
                / self.open_loop_velocity)
                .abs();
            if (ratio - 1.0).abs() > self.config.open_loop_ratio_tolerance {
                self.fail(AutoTuneError::OpenLoopFailed);
                return;
            }
            self.open_loop_settled_time += dt;
            if self.open_loop_settled_time >= 0.1 {
                self.enter(AutoTuneState::FluxMeasure);
            }
        }
    }

    fn tick_flux_measure(&mut self, dt: f32, control: Option<&Telemetry>) {
        let Some(telemetry) = control else {
            return;
        };
        if telemetry.state != MotorState::Running(ControlMode::OpenLoop) {
            return;
        }
        self.flux_elapsed += dt;
        let tick_index = (self.flux_elapsed / dt) as u32;
        if tick_index.is_multiple_of(self.config.flux_stride.max(1))
            && self.flux_sample_count < FLUX_SAMPLE_CAPACITY
        {
            let omega = self.open_loop_velocity;
            let flux = (telemetry.voltage_dq.q
                - self.motor.rs * telemetry.current_dq.q)
                / omega;
            if flux.is_finite() && self.flux_sample_count < FLUX_SAMPLE_CAPACITY
            {
                self.flux_samples[self.flux_sample_count] = flux;
                self.flux_sample_count += 1;
            }
        }
        if self.flux_elapsed >= self.config.flux_time {
            let Some(flux) = analyze_rejected_median(
                &self.flux_samples[..self.flux_sample_count],
            ) else {
                self.fail(AutoTuneError::FluxRange);
                return;
            };
            if !(self.config.flux_min..=self.config.flux_max).contains(&flux) {
                self.fail(AutoTuneError::FluxRange);
                return;
            }
            self.motor.flux = flux;
            self.push_request(ControlRequest::DisableAndResync);
            self.enter(AutoTuneState::ObserverInit);
        }
    }

    fn tick_observer_init(&mut self) {
        let bandwidth =
            core::f32::consts::TAU * self.config.observer_bandwidth_hz;
        self.motor.observer_bandwidth = bandwidth;
        // Normalized back-EMF observer gain convention: the correction term
        // is scaled by flux * omega_o. Finalized together with the future
        // observer module.
        self.motor.observer_gain = bandwidth * self.motor.flux;
        self.motor.pll_kp = 2.0 * bandwidth;
        self.motor.pll_ki = bandwidth * bandwidth;
        self.motor.observer_switch_speed = self.open_loop_velocity;
        self.enter(AutoTuneState::ObserverVerify);
    }

    fn tick_observer_verify(&mut self) {
        // Parameter-level verification until the sensorless observer module
        // lands: all observer quantities must be finite and positive where
        // applicable, and the full parameter set must validate.
        let sane = self.motor.observer_bandwidth > 0.0
            && self.motor.observer_gain > 0.0
            && self.motor.pll_kp > 0.0
            && self.motor.pll_ki > 0.0
            && self.motor.observer_switch_speed > 0.0
            && self.motor.validate().is_ok();
        if sane {
            self.enter(AutoTuneState::PlLConfig);
        } else {
            self.fail(AutoTuneError::FluxRange);
        }
    }

    fn tick_transition_verify(&mut self) {
        let complete = self.motor.rs > 0.0
            && self.motor.ld > 0.0
            && self.motor.lq > 0.0
            && self.motor.flux > 0.0
            && self.motor.id_kp > 0.0
            && self.motor.iq_kp > 0.0
            && self.motor.validate().is_ok();
        if !complete {
            self.fail(AutoTuneError::NotReady);
            return;
        }
        self.motor.mark_commissioned();
        self.save_request = Some(self.motor);
        self.enter(AutoTuneState::Save);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc_math::Dq;

    const FREQ: f32 = 100_000.0;

    /// Simulates a single-phase R-L winding (alpha axis) driven by the
    /// injected alpha voltage, locked rotor, ideal inverter.
    struct Harness {
        autotune: AutoTuneController,
        sequence: u32,
        offsets: [f32; 3],
        plant_current: f32,
        plant_voltage: f32,
        rs: f32,
        ls: f32,
    }

    impl Harness {
        fn new(rs: f32, ls: f32) -> Self {
            let config = AutoTuneConfig {
                rs_guess: rs,
                l_guess: ls,
                rs_settle_frames: 400,
                rs_sample_frames: 200,
                offset_frames: 64,
                ..AutoTuneConfig::default()
            };
            Self {
                autotune: AutoTuneController::new(config),
                sequence: 0,
                offsets: [2_048.0; 3],
                plant_current: 0.0,
                plant_voltage: 0.0,
                rs,
                ls,
            }
        }

        fn amperes_per_count(config: &AutoTuneConfig) -> f32 {
            config.adc_reference_voltage
                / config.adc_full_scale
                / (config.current_sense_gain * config.shunt_resistance)
        }

        fn frame(&mut self) -> RawAdcFrame {
            let dt = 1.0 / FREQ;
            self.plant_current +=
                (self.plant_voltage - self.rs * self.plant_current) / self.ls
                    * dt;
            let apc = Self::amperes_per_count(&self.autotune.config);
            let alpha = self.plant_current;
            self.sequence += 1;
            RawAdcFrame {
                phase_a: (self.offsets[0] + alpha / apc) as u16,
                phase_b: (self.offsets[1] - alpha / apc / 2.0) as u16,
                phase_c: (self.offsets[2] - alpha / apc / 2.0) as u16,
                bus_voltage: (24.0 / (3.3 / 4095.0 * 11.0)) as u16,
                ntc: 930,
                sequence: self.sequence,
                overrun: false,
            }
        }

        fn run_frame(&mut self) -> FastAction {
            let frame = self.frame();
            let action = self.autotune.fast_step(&frame);
            self.plant_voltage = match action {
                | FastAction::Inject(duty) => {
                    // Reconstruct the alpha-axis voltage from all three
                    // phase duties, cancelling the zero-sequence component.
                    let va = (duty.a - 0.5) * 24.0;
                    let vb = (duty.b - 0.5) * 24.0;
                    let vc = (duty.c - 0.5) * 24.0;
                    (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc)
                },
                | _ => 0.0,
            };
            action
        }
    }

    fn running_telemetry(
        mode: ControlMode,
        iq: f32,
        uq: f32,
        mechanical_velocity: f32,
    ) -> Telemetry {
        Telemetry {
            state: MotorState::Running(mode),
            currents: PhaseCurrents {
                a: iq,
                b: -iq / 2.0,
                c: -iq / 2.0,
            },
            current_dq: Dq { d: 0.0, q: iq },
            voltage_dq: Dq { d: 0.0, q: uq },
            sensor_valid: true,
            bus_voltage: 24.0,
            mechanical_velocity,
            ..Telemetry::default()
        }
    }

    fn running_current_telemetry(iq: f32) -> Telemetry {
        running_telemetry(ControlMode::Current, iq, 0.0, 0.0)
    }

    #[test]
    fn start_stop_and_states_report_correctly() {
        let mut harness = Harness::new(0.05, 0.000_2);
        assert!(!harness.autotune.is_running());
        assert!(harness.autotune.start(MotorParam::default()));
        // Starting again while running is rejected.
        assert!(!harness.autotune.start(MotorParam::default()));
        assert_eq!(harness.autotune.state(), AutoTuneState::Init);
        assert!(harness.autotune.is_running());
        harness.autotune.task_tick(0.001, None);
        assert_eq!(harness.autotune.state(), AutoTuneState::CurrentOffset);
        harness.autotune.stop();
        assert_eq!(harness.autotune.state(), AutoTuneState::Idle);
        assert_eq!(harness.autotune.error(), AutoTuneError::Aborted);
        assert!(!harness.autotune.is_running());
        // Terminal states accept a fresh start.
        assert!(harness.autotune.start(MotorParam::default()));
        harness.autotune.stop();
    }

    #[test]
    fn overcurrent_immediately_fails_and_turns_bridge_off() {
        let mut harness = Harness::new(0.05, 0.000_2);
        harness.autotune.start(MotorParam::default());
        // Move Init -> CurrentOffset, then let offset calibration complete
        // so current protection is active.
        harness.autotune.task_tick(0.001, None);
        for _ in 0..150 {
            harness.run_frame();
        }
        harness.autotune.task_tick(0.001, None);
        assert!(harness.autotune.offsets_valid);
        // Force a large plant current regardless of injected duty.
        harness.plant_voltage = 100.0;
        let mut action = FastAction::Off;
        for _ in 0..500 {
            let frame = harness.frame();
            action = harness.autotune.fast_step(&frame);
            if harness.autotune.error() == AutoTuneError::OverCurrent {
                break;
            }
        }
        assert_eq!(harness.autotune.error(), AutoTuneError::OverCurrent);
        assert_eq!(action, FastAction::Off);
        assert_eq!(harness.autotune.state(), AutoTuneState::Error);
    }

    #[test]
    fn sequence_gap_fails_in_injection_states() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.motor.ld = 0.000_2;
        harness.autotune.offsets = [2_048.0; 3];
        harness.autotune.offsets_valid = true;
        harness.autotune.enter(AutoTuneState::RsMeasure);
        // Establish the previous sequence number with a normal frame.
        let frame = harness.frame();
        assert!(matches!(
            harness.autotune.fast_step(&frame),
            FastAction::Inject(_)
        ));
        let mut gapped = harness.frame();
        gapped.sequence = harness.sequence.wrapping_add(100);
        assert_eq!(harness.autotune.fast_step(&gapped), FastAction::Off);
        assert_eq!(harness.autotune.error(), AutoTuneError::FrameOverrun);
    }

    #[test]
    fn identifies_rs_and_l_on_simulated_winding() {
        let rs_true = 0.5;
        let ls_true = 0.000_2;
        let mut harness = Harness::new(rs_true, ls_true);
        assert!(harness.autotune.start(MotorParam::default()));

        let mut guard = 0;
        while harness.autotune.state() != AutoTuneState::LPrepare
            && guard < 4_000_000
        {
            harness.run_frame();
            if harness.autotune.frame_count.is_multiple_of(100) {
                harness.autotune.task_tick(0.001, None);
            }
            if harness.autotune.state() == AutoTuneState::Error {
                panic!("rs phase failed: {:?}", harness.autotune.error());
            }
            guard += 1;
        }
        assert_eq!(harness.autotune.state(), AutoTuneState::LPrepare);
        let rs = harness.autotune.motor_param().rs;
        assert!(
            (rs - rs_true).abs() / rs_true < 0.05,
            "rs {rs} vs {rs_true}"
        );

        let mut guard = 0;
        while harness.autotune.state() != AutoTuneState::CurrentPiCalc
            && guard < 4_000_000
        {
            harness.run_frame();
            if harness.autotune.frame_count.is_multiple_of(100) {
                harness.autotune.task_tick(0.001, None);
            }
            if harness.autotune.state() == AutoTuneState::Error {
                panic!("l phase failed: {:?}", harness.autotune.error());
            }
            guard += 1;
        }
        assert_eq!(harness.autotune.state(), AutoTuneState::CurrentPiCalc);
        let l = harness.autotune.motor_param().ld;
        assert!((l - ls_true).abs() / ls_true < 0.15, "l {l} vs {ls_true}");
        assert_eq!(harness.autotune.motor_param().lq, l);
    }

    #[test]
    fn pi_gains_follow_bandwidth_formulas() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.motor.ld = 0.000_2;
        harness.autotune.motor.lq = 0.000_2;
        harness.autotune.enter(AutoTuneState::CurrentPiCalc);
        harness.autotune.task_tick(0.001, None);
        let bandwidth =
            core::f32::consts::TAU * harness.autotune.current_bandwidth_hz;
        let motor = harness.autotune.motor_param();
        assert!((motor.id_kp - 0.000_2 * bandwidth).abs() < 1.0e-6);
        assert!((motor.id_ki - 0.5 * bandwidth).abs() < 1.0e-3);
        assert_eq!(harness.autotune.state(), AutoTuneState::CurrentPiVerify);
        assert_eq!(
            harness.autotune.pop_request(),
            Some(ControlRequest::ApplyMotorParam)
        );
    }

    #[test]
    fn pi_verify_accepts_well_damped_response() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.motor.ld = 0.000_2;
        harness.autotune.enter(AutoTuneState::CurrentPiVerify);
        harness.autotune.step_target = 0.8;

        harness
            .autotune
            .task_tick(0.001, Some(&running_current_telemetry(0.0)));
        assert_eq!(harness.autotune.pop_request(), None);
        for _ in 0..35 {
            harness
                .autotune
                .task_tick(0.001, Some(&running_current_telemetry(0.0)));
        }
        assert_eq!(
            harness.autotune.pop_request(),
            Some(ControlRequest::SetIq(0.8))
        );
        let tau = 0.002;
        for index in 0..220 {
            let time = index as f32 * 0.001;
            let iq = 0.8 * (1.0 - (-time / tau).exp());
            harness
                .autotune
                .task_tick(0.001, Some(&running_current_telemetry(iq)));
        }
        // Step verified; the procedure advances into the open-loop phase.
        assert!(matches!(
            harness.autotune.state(),
            AutoTuneState::OpenLoopPrepare | AutoTuneState::OpenLoopRun
        ));
        assert!(harness.autotune.pi_metrics().is_some());
    }

    #[test]
    fn pi_verify_rolls_back_bandwidth_on_oscillation() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.motor.ld = 0.000_2;
        harness.autotune.enter(AutoTuneState::CurrentPiVerify);
        harness.autotune.step_target = 0.8;

        harness
            .autotune
            .task_tick(0.001, Some(&running_current_telemetry(0.0)));
        for _ in 0..35 {
            harness
                .autotune
                .task_tick(0.001, Some(&running_current_telemetry(0.0)));
        }
        for index in 0..220 {
            let time = index as f32 * 0.001;
            let iq =
                0.8 + (-time / 0.02).exp() * (time * 900.0).sin() * 0.9 * 0.8;
            harness
                .autotune
                .task_tick(0.001, Some(&running_current_telemetry(iq)));
        }
        assert_eq!(harness.autotune.state(), AutoTuneState::CurrentPiVerify);
        assert_eq!(harness.autotune.attempts, 1);
        assert!(
            (harness.autotune.current_bandwidth_hz - 500.0 * 0.75).abs()
                < 1.0e-3
        );
        // The step request is still queued ahead of the rollback request.
        assert_eq!(
            harness.autotune.pop_request(),
            Some(ControlRequest::SetIq(0.8))
        );
        assert_eq!(
            harness.autotune.pop_request(),
            Some(ControlRequest::DisableAndResync)
        );
    }

    #[test]
    fn pi_verify_fails_after_exhausted_retries() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.motor.ld = 0.000_2;
        harness.autotune.step_target = 0.8;
        // Four consecutive unstable verification rounds exhaust the retries.
        for _ in 0..4 {
            harness.autotune.enter(AutoTuneState::CurrentPiVerify);
            harness
                .autotune
                .task_tick(0.001, Some(&running_current_telemetry(0.0)));
            for _ in 0..35 {
                harness
                    .autotune
                    .task_tick(0.001, Some(&running_current_telemetry(0.0)));
            }
            for index in 0..220 {
                let time = index as f32 * 0.001;
                let iq = 0.8
                    + (-time / 0.02).exp() * (time * 900.0).sin() * 0.9 * 0.8;
                harness
                    .autotune
                    .task_tick(0.001, Some(&running_current_telemetry(iq)));
            }
            if harness.autotune.state() == AutoTuneState::Error {
                break;
            }
        }
        assert_eq!(harness.autotune.state(), AutoTuneState::Error);
        assert_eq!(harness.autotune.error(), AutoTuneError::PiUnstable);
    }

    #[test]
    fn flux_measurement_extracts_back_emf_constant() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.open_loop_velocity = core::f32::consts::TAU * 5.0;
        harness.autotune.enter(AutoTuneState::FluxMeasure);

        let omega = harness.autotune.open_loop_velocity;
        let flux_true = 0.01;
        let iq = 1.5;
        let uq = 0.5 * iq + omega * flux_true;
        let telemetry =
            running_telemetry(ControlMode::OpenLoop, iq, uq, omega / 4.0);
        // Tick until the flux window completes.
        let mut guard = 0;
        while harness.autotune.state() == AutoTuneState::FluxMeasure
            && guard < 1_000
        {
            harness.autotune.task_tick(0.001, Some(&telemetry));
            guard += 1;
        }
        assert!(guard < 1_000);
        assert!(
            (harness.autotune.motor_param().flux - flux_true).abs() < 1.0e-3
        );
        // The parameter-only tail runs to completion in a few ticks and
        // parks in Save with a commissioned record.
        for _ in 0..8 {
            harness.autotune.task_tick(0.001, Some(&telemetry));
        }
        assert_eq!(harness.autotune.state(), AutoTuneState::Save);
        let motor = harness.autotune.take_save_request().unwrap();
        assert!(motor.is_commissioned());
        let bandwidth = core::f32::consts::TAU * 50.0;
        assert!((motor.observer_bandwidth - bandwidth).abs() < 1.0e-2);
        assert!((motor.observer_gain - bandwidth * flux_true).abs() < 1.0e-3);
        assert!((motor.pll_kp - 2.0 * bandwidth).abs() < 1.0e-2);
        assert!((motor.pll_ki - bandwidth * bandwidth).abs() < 1.0e-1);
    }

    #[test]
    fn open_loop_run_ramps_voltage_and_advances_on_current() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.enter(AutoTuneState::OpenLoopPrepare);
        // Prepare waits for the decay window, then disables/resynchronizes
        // the controller before enabling open loop.
        for _ in 0..40 {
            harness.autotune.task_tick(0.001, None);
        }
        assert_eq!(
            harness.autotune.pop_request(),
            Some(ControlRequest::DisableAndResync)
        );
        assert_eq!(
            harness.autotune.pop_request(),
            Some(ControlRequest::EnableOpenLoop)
        );
        assert_eq!(harness.autotune.state(), AutoTuneState::OpenLoopRun);
        let omega = harness.autotune.open_loop_velocity;
        let mut voltage: f32 = 0.0;
        for _ in 0..400 {
            voltage = (voltage + 4.0 * 0.001)
                .min(harness.autotune.open_loop_max_voltage());
            let telemetry = running_telemetry(
                ControlMode::OpenLoop,
                harness.autotune.open_loop_target_current,
                voltage,
                omega / 4.0,
            );
            harness.autotune.task_tick(0.001, Some(&telemetry));
        }
        assert_eq!(harness.autotune.state(), AutoTuneState::FluxMeasure);
    }

    #[test]
    fn save_flow_requests_record_and_completes_on_ack() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.motor.ld = 0.000_2;
        harness.autotune.motor.lq = 0.000_2;
        harness.autotune.motor.flux = 0.01;
        harness.autotune.motor.id_kp = 1.0;
        harness.autotune.motor.iq_kp = 1.0;
        harness.autotune.enter(AutoTuneState::TransitionVerify);
        harness.autotune.task_tick(0.001, None);
        assert_eq!(harness.autotune.state(), AutoTuneState::Save);
        let param = harness.autotune.take_save_request().unwrap();
        assert!(param.is_commissioned());
        assert!(param.validate().is_ok());
        harness.autotune.notify_saved(true);
        assert_eq!(harness.autotune.state(), AutoTuneState::Done);
        assert!(!harness.autotune.is_running());
        assert_eq!(harness.autotune.status().progress, 100);
        // Repeated notifications are ignored.
        harness.autotune.notify_saved(true);
        assert_eq!(harness.autotune.state(), AutoTuneState::Done);
    }

    #[test]
    fn save_failure_latches_error() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.enter(AutoTuneState::Save);
        harness.autotune.notify_saved(false);
        assert_eq!(harness.autotune.state(), AutoTuneState::Error);
        assert_eq!(harness.autotune.error(), AutoTuneError::SaveFailed);
    }

    #[test]
    fn timeout_fails_stuck_state() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.config.state_timeout = 0.05;
        harness.autotune.start(MotorParam::default());
        // Sit in a prepare state without current decay.
        for _ in 0..100 {
            harness.autotune.task_tick(0.001, None);
        }
        assert_eq!(harness.autotune.state(), AutoTuneState::Error);
        assert_eq!(harness.autotune.error(), AutoTuneError::Timeout);
    }

    #[test]
    fn observer_states_pass_through_to_save_request() {
        let mut harness = Harness::new(0.5, 0.000_2);
        harness.autotune.start(MotorParam::default());
        harness.autotune.motor.rs = 0.5;
        harness.autotune.motor.ld = 0.000_2;
        harness.autotune.motor.lq = 0.000_2;
        harness.autotune.motor.flux = 0.01;
        harness.autotune.open_loop_velocity = core::f32::consts::TAU * 5.0;
        harness.autotune.enter(AutoTuneState::ObserverInit);
        harness.autotune.task_tick(0.001, None);
        assert_eq!(harness.autotune.state(), AutoTuneState::ObserverVerify);
        harness.autotune.task_tick(0.001, None);
        assert_eq!(harness.autotune.state(), AutoTuneState::PlLConfig);
        harness.autotune.task_tick(0.001, None);
        assert_eq!(harness.autotune.state(), AutoTuneState::TransitionVerify);
        harness.autotune.task_tick(0.001, None);
        assert_eq!(harness.autotune.state(), AutoTuneState::Save);
        assert!(harness.autotune.take_save_request().is_some());
    }
}
