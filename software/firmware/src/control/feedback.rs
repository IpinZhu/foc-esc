//! Adapter between the FOC modes (what is controlled) and the source of
//! rotor feedback. An unavailable sensorless backend never falls back to
//! physical inputs, defaults or uncontrolled voltage drive.
use super::foc_core::FocConfig;
use super::foc_math::{AlphaBeta, PhaseDuty};
use super::sensorless::{Config, Guidance, Stage, Tracker};
use crate::interfaces::{ControlMode, RotorSample};
use crate::params::motor_param::MotorParam;

pub(super) enum Feedback {
    Physical,
    Sensorless(Option<Tracker>),
}
impl Feedback {
    pub fn sensorless(cfg: FocConfig, motor: Option<&MotorParam>) -> Self {
        Self::Sensorless(
            motor
                .and_then(|m| {
                    Config::from_record(
                        m,
                        cfg.pwm_frequency_hz,
                        cfg.pole_pairs,
                        cfg.maximum_velocity,
                    )
                })
                .map(Tracker::new),
        )
    }
    pub fn physical(&self) -> bool {
        matches!(self, Self::Physical)
    }
    pub fn stage(&self) -> Option<Stage> {
        match self {
            | Self::Sensorless(Some(t)) => Some(t.stage),
            | _ => None,
        }
    }
    pub fn accepts_target(&self, mode: ControlMode, target: f32) -> bool {
        match self {
            | Self::Physical => true,
            | Self::Sensorless(Some(t)) => {
                t.same_direction(target)
                    && t.accepts(target, mode == ControlMode::Velocity)
            },
            | Self::Sensorless(None) => false,
        }
    }
    pub fn start(
        &mut self,
        mode: ControlMode,
        target: f32,
        sensor_valid: bool,
    ) -> bool {
        match self {
            | Self::Physical => sensor_valid || mode == ControlMode::OpenLoop,
            | Self::Sensorless(Some(t))
                if matches!(
                    mode,
                    ControlMode::Velocity | ControlMode::Current
                ) && t.accepts(target, mode == ControlMode::Velocity) =>
            {
                t.start(target);
                true
            },
            | _ => false,
        }
    }
    pub fn velocity_reference(&mut self, target: f32) -> f32 {
        match self {
            | Self::Sensorless(Some(t)) => t.velocity_reference(target),
            | _ => target,
        }
    }
    pub fn current_limit(&self, limit: f32) -> f32 {
        match self {
            | Self::Sensorless(Some(t)) => limit.min(t.cfg.max_current),
            | _ => limit,
        }
    }
    pub fn axis_voltage_limit(&self, limit: f32) -> f32 {
        match self {
            | Self::Sensorless(Some(t)) => {
                limit.min(t.cfg.max_voltage * core::f32::consts::FRAC_1_SQRT_2)
            },
            | _ => limit,
        }
    }
    pub fn update(
        &mut self,
        i: AlphaBeta,
        bus: f32,
        duty: PhaseDuty,
        max_velocity: f32,
    ) -> Option<(RotorSample, Guidance)> {
        let Self::Sensorless(Some(t)) = self else {
            return None;
        };
        let g = t.update(i, bus, duty)?;
        let speed = t.mechanical_speed(g.estimate.speed);
        (speed.abs() <= max_velocity).then_some((
            RotorSample {
                mechanical_angle: g.mechanical_angle,
                mechanical_velocity: speed,
                valid: g.closed,
            },
            g,
        ))
    }
}
