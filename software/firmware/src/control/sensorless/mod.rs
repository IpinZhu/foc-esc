//! Electrical feedback for light-load, stationary-start SPMSMs. No registers,
//! sensor, plant state, PID, Flash layout or command protocol in the fast path.
mod observer;
use crate::control::foc_math::{
    AlphaBeta, INV_SQRT_3, PhaseDuty, normalize_angle, shortest_angle_delta,
};
use crate::params::motor_param::MotorParam;
pub use observer::Estimate;
use observer::Observer;

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pairs: f32,
    rs: f32,
    l: f32,
    flux: f32,
    dt: f32,
    bandwidth: f32,
    filter_step: f32,
    pll_p_step: f32,
    pll_i_step: f32,
    switch_speed: f32,
    speed_ceiling: f32,
    pub(super) max_current: f32,
    pub(super) max_voltage: f32,
}
impl Config {
    pub fn from_record(
        m: &MotorParam,
        hz: u32,
        pole_pairs: u8,
        maximum_mechanical_speed: f32,
    ) -> Option<Self> {
        let dt = 1.0 / hz as f32;
        let w = m.observer_bandwidth;
        let switch_speed = m.observer_switch_speed;
        let ceiling =
            (maximum_mechanical_speed * f32::from(pole_pairs)).min(2.0 * w);
        if !m.is_commissioned()
            || m.validate().is_err()
            || m.pole_pairs != u16::from(pole_pairs)
            || (m.lq / m.ld - 1.0).abs() > 0.15
            || !(0.0..0.2).contains(&(w * dt))
            || w <= 0.0
            || m.pll_kp <= 0.0
            || m.pll_kp * dt >= 0.2
            || m.pll_ki <= 0.0
            || m.pll_ki * dt * dt >= m.pll_kp * dt
            || switch_speed <= 0.0
            || switch_speed >= ceiling
            || !maximum_mechanical_speed.is_finite()
        {
            return None;
        }
        Some(Self {
            pairs: f32::from(pole_pairs),
            rs: m.rs,
            l: (m.ld + m.lq) * 0.5,
            flux: m.flux,
            dt,
            bandwidth: w,
            filter_step: w * dt / (1.0 + w * dt),
            pll_p_step: m.pll_kp * dt,
            pll_i_step: m.pll_ki * dt,
            switch_speed,
            speed_ceiling: ceiling,
            max_current: m.max_current,
            max_voltage: m.max_voltage,
        })
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Align,
    Ramp,
    Blend,
    ClosedLoop,
}
#[derive(Clone, Copy)]
pub(super) struct Guidance {
    pub mechanical_angle: f32,
    pub angle: f32,
    pub estimate: Estimate,
    pub closed: bool,
    pub blend: f32,
    align_scale: f32,
    damping: f32,
}
impl Guidance {
    pub fn current_target(
        self,
        q: f32,
        startup_current: f32,
    ) -> crate::control::foc_math::Dq {
        let startup = startup_current * self.align_scale * (1.0 - self.blend);
        crate::control::foc_math::Dq {
            d: 0.85 * startup,
            q: self.damping * startup + q * self.blend,
        }
    }
}
fn voltage(d: PhaseDuty, bus: f32) -> AlphaBeta {
    AlphaBeta {
        alpha: (2.0 * d.a - d.b - d.c) * (bus / 3.0),
        beta: (d.b - d.c) * (INV_SQRT_3 * bus),
    }
}

pub(super) struct Tracker {
    pub cfg: Config,
    pub stage: Stage,
    observer: Observer,
    sign: f32,
    elapsed: f32,
    qualified: f32,
    lost: f32,
    angle: f32,
    speed: f32,
    previous_angle: f32,
    sector: i16,
    velocity_reference: f32,
    previous: PhaseDuty,
}
impl Tracker {
    pub fn new(cfg: Config) -> Self {
        Self {
            cfg,
            stage: Stage::Align,
            observer: Observer::default(),
            sign: 1.0,
            elapsed: 0.0,
            qualified: 0.0,
            lost: 0.0,
            angle: 0.0,
            speed: 0.0,
            previous_angle: 0.0,
            sector: 0,
            velocity_reference: 0.0,
            previous: PhaseDuty::DISABLED,
        }
    }
    pub fn mechanical_speed(&self, electrical: f32) -> f32 {
        electrical / self.cfg.pairs
    }
    pub fn accepts(&self, target: f32, velocity_mode: bool) -> bool {
        let speed = target.abs() * self.cfg.pairs;
        target.is_finite()
            && target != 0.0
            && (!velocity_mode
                || (self.cfg.switch_speed..self.cfg.speed_ceiling)
                    .contains(&speed))
    }
    pub fn start(&mut self, target: f32) {
        *self = Self::new(self.cfg);
        self.sign = target.signum();
    }
    pub fn same_direction(&self, target: f32) -> bool {
        self.sign * target > 0.0
    }
    // Start the outer speed loop near the forced speed rather than feeding
    // it the entire final setpoint error on the first tick of handover.
    pub fn velocity_reference(&mut self, target: f32) -> f32 {
        let step = 8.0 * self.cfg.switch_speed / self.cfg.pairs * self.cfg.dt;
        self.velocity_reference +=
            (target - self.velocity_reference).clamp(-step, step);
        self.velocity_reference
    }
    pub fn update(
        &mut self,
        i: AlphaBeta,
        bus: f32,
        last_command: PhaseDuty,
    ) -> Option<Guidance> {
        // One tick of PWM latch latency: the just-sampled current ends the
        // interval driven by command k-2, not k or k-1. Verify on the board.
        let d = core::mem::replace(&mut self.previous, last_command);
        let e =
            self.observer
                .update(&self.cfg, self.sign, voltage(d, bus), i)?;
        let dt = self.cfg.dt;
        let switch = self.cfg.switch_speed;
        let signed_speed = self.sign * e.speed;
        let expected_emf = e.speed.abs() * self.cfg.flux;
        let plausible = e.emf > 0.35 * switch * self.cfg.flux
            && e.emf > 0.5 * expected_emf
            && e.emf < 1.6 * expected_emf
            && e.phase_error.abs() < 0.6
            && signed_speed > 0.45 * switch;
        if self.stage != Stage::ClosedLoop {
            self.elapsed += dt;
        }
        let mut blend = 0.0;
        let mut align_scale = 1.0;
        match self.stage {
            | Stage::Align => {
                align_scale = (self.elapsed / 0.05).min(1.0);
                if self.elapsed >= 0.20 {
                    self.stage = Stage::Ramp;
                    self.elapsed = 0.0;
                }
            },
            | Stage::Ramp => {
                self.speed = self.sign * switch * (self.elapsed / 0.5).min(1.0);
                self.angle = normalize_angle(self.angle + self.speed * dt);
                let sync = plausible
                    && (e.speed - self.speed).abs() < 0.22 * switch
                    && shortest_angle_delta(e.angle, self.angle).abs() < 1.0;
                if self.elapsed >= 0.5 && sync {
                    self.qualified += dt;
                } else {
                    self.qualified = 0.0;
                }
                if self.qualified >= 0.035 {
                    self.stage = Stage::Blend;
                    self.elapsed = 0.0;
                } else if self.elapsed >= 1.25 {
                    return None;
                }
            },
            | Stage::Blend | Stage::ClosedLoop => {
                self.lost = if plausible { 0.0 } else { self.lost + dt };
                if self.lost >= 0.005 {
                    return None;
                }
                if self.stage == Stage::Blend {
                    self.angle = normalize_angle(self.angle + self.speed * dt);
                    blend = (self.elapsed / 0.04).min(1.0);
                    if blend >= 1.0 {
                        self.stage = Stage::ClosedLoop;
                        self.velocity_reference = self.speed / self.cfg.pairs;
                    }
                } else {
                    blend = 1.0;
                }
            },
        }
        let angle = if self.stage == Stage::ClosedLoop {
            e.angle
        } else {
            normalize_angle(
                self.angle + blend * shortest_angle_delta(e.angle, self.angle),
            )
        };
        // Keep electrical-sector count instead of integrating small f32
        // angle deltas indefinitely. Shaft origin is only known modulo a pole pair.
        let raw_delta = angle - self.previous_angle;
        if raw_delta.abs() > core::f32::consts::PI {
            self.sector = (self.sector - if raw_delta > 0.0 { 1 } else { -1 })
                .rem_euclid(self.cfg.pairs as i16);
        }
        let mechanical_angle =
            (f32::from(self.sector) * core::f32::consts::TAU + angle)
                / self.cfg.pairs;
        self.previous_angle = angle;
        // Dampen the spring-like oscillation of a rotor pulled by the
        // forced d-axis field. Project *measured EMF*, not PLL speed, onto
        // that field: this signal remains signed during alignment swings.
        let damping = if blend < 1.0 {
            let (sin, cos) = crate::control::foc_math::sin_cos(self.angle);
            let local_speed = (cos * e.filtered_emf.beta
                - sin * e.filtered_emf.alpha)
                / self.cfg.flux;
            ((self.speed - local_speed) / switch).clamp(-0.5, 0.5)
        } else {
            0.0
        };
        Some(Guidance {
            damping,
            mechanical_angle,
            angle,
            estimate: e,
            closed: self.stage == Stage::ClosedLoop && plausible,
            blend,
            align_scale,
        })
    }
}
