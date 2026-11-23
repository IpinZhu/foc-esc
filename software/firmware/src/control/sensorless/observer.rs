//! Filtered SPMSM voltage model: e = LPF(v - R*i) - L*s*LPF(i).
//! No numerical differentiation of two quantized ADC samples.
use super::Config;
use crate::control::foc_math::{
    AlphaBeta, normalize_angle, shortest_angle_delta,
};
use core::f32::consts::FRAC_PI_2;

#[derive(Clone, Copy, Debug, Default)]
pub struct Estimate {
    pub angle: f32, // electrical rad
    pub speed: f32, // electrical rad/s
    pub emf: f32,   // phase peak V, corrected for filter attenuation
    pub phase_error: f32,
    pub(super) filtered_emf: AlphaBeta,
}
#[derive(Default)]
pub(super) struct Observer {
    current_lp: AlphaBeta,
    voltage_lp: AlphaBeta,
    angle: f32,
    speed: f32,
}
impl Observer {
    pub(super) fn update(
        &mut self,
        cfg: &Config,
        sign: f32,
        v: AlphaBeta,
        i: AlphaBeta,
    ) -> Option<Estimate> {
        // Backward Euler. Both paths have the same pole in rad/s.
        let axis = |v: f32, i: f32, i_lp: &mut f32, v_lp: &mut f32| {
            *i_lp += cfg.filter_step * (i - *i_lp);
            *v_lp += cfg.filter_step * (v - cfg.rs * i - *v_lp);
            *v_lp - cfg.l * cfg.bandwidth * (i - *i_lp)
        };
        let a = axis(
            v.alpha,
            i.alpha,
            &mut self.current_lp.alpha,
            &mut self.voltage_lp.alpha,
        );
        let b = axis(
            v.beta,
            i.beta,
            &mut self.current_lp.beta,
            &mut self.voltage_lp.beta,
        );
        // Compensate filter delay at the estimated signed frequency.
        let filtered_emf = AlphaBeta { alpha: a, beta: b };
        let ratio = self.speed / cfg.bandwidth;
        let (a, b) = (a - ratio * b, b + ratio * a);
        let emf = libm::sqrtf(a * a + b * b);
        let predicted = normalize_angle(self.angle + self.speed * cfg.dt);
        let error = if emf > 0.1 * cfg.switch_speed * cfg.flux {
            shortest_angle_delta(
                libm::atan2f(b, a) - sign * FRAC_PI_2,
                predicted,
            )
        } else {
            0.0
        };
        self.angle = normalize_angle(predicted + cfg.pll_p_step * error);
        self.speed += cfg.pll_i_step * error;
        (emf.is_finite()
            && self.angle.is_finite()
            && self.speed.is_finite()
            && self.speed.abs() <= cfg.speed_ceiling)
            .then_some(Estimate {
                angle: self.angle,
                speed: self.speed,
                emf,
                phase_error: error,
                filtered_emf,
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::motor_param::MotorParam;

    #[test]
    fn tracks_signed_rotating_emf_without_motor_or_controller() {
        let mut motor = MotorParam {
            observer_bandwidth: 314.0,
            pll_kp: 628.0,
            pll_ki: 314.0 * 314.0,
            observer_switch_speed: 40.0,
            ..MotorParam::default()
        };
        motor.mark_commissioned();
        let cfg = Config::from_record(&motor, 100_000, 4, 1000.0).unwrap();
        for omega in [-150.0_f32, 100.0] {
            let mut observer = Observer::default();
            for tick in 0..30_000 {
                let angle = 0.2 + omega * (tick as f32 * cfg.dt);
                let (s, c) = angle.sin_cos();
                let v = AlphaBeta {
                    alpha: -omega * cfg.flux * s,
                    beta: omega * cfg.flux * c,
                };
                let e = observer
                    .update(&cfg, omega.signum(), v, AlphaBeta::default())
                    .unwrap();
                if tick >= 20_000 {
                    assert!(shortest_angle_delta(e.angle, angle).abs() < 0.02);
                    assert!((e.speed - omega).abs() < 1.0);
                }
            }
            assert!(
                observer
                    .update(
                        &cfg,
                        1.0,
                        AlphaBeta {
                            alpha: f32::NAN,
                            beta: 0.0
                        },
                        AlphaBeta::default()
                    )
                    .is_none()
            );
        }
    }
}
