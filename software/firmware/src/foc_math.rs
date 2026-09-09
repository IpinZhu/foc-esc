use core::f32::consts::{FRAC_PI_2, PI, TAU};

pub const SQRT_3: f32 = 1.732_050_8;
pub const INV_SQRT_3: f32 = 0.577_350_26;
pub const SQRT_3_OVER_2: f32 = 0.866_025_4;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PhaseCurrents {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AlphaBeta {
    pub alpha: f32,
    pub beta: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Dq {
    pub d: f32,
    pub q: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PhaseDuty {
    pub a: f32,
    pub b: f32,
    pub c: f32,
}

impl PhaseDuty {
    pub const DISABLED: Self = Self {
        a: 0.5,
        b: 0.5,
        c: 0.5,
    };
}

pub fn normalize_angle(angle: f32) -> f32 {
    let wrapped = angle % TAU;
    if wrapped < 0.0 {
        wrapped + TAU
    } else {
        wrapped
    }
}

pub fn shortest_angle_delta(current: f32, previous: f32) -> f32 {
    normalize_angle(current - previous + PI) - PI
}

pub fn sin_cos(angle: f32) -> (f32, f32) {
    let angle = normalize_angle(angle);
    let quadrant = (angle / FRAC_PI_2) as u8;
    let x = angle - quadrant as f32 * FRAC_PI_2;
    let x2 = x * x;
    let sin = x
        * (1.0
            + x2 * (-1.0 / 6.0
                + x2 * (1.0 / 120.0 + x2 * (-1.0 / 5_040.0 + x2 / 362_880.0))));
    let cos = 1.0
        + x2 * (-1.0 / 2.0
            + x2 * (1.0 / 24.0 + x2 * (-1.0 / 720.0 + x2 / 40_320.0)));

    match quadrant {
        | 0 => (sin, cos),
        | 1 => (cos, -sin),
        | 2 => (-sin, -cos),
        | _ => (-cos, sin),
    }
}

pub fn clarke(currents: PhaseCurrents) -> AlphaBeta {
    AlphaBeta {
        alpha: (2.0 / 3.0) * (currents.a - 0.5 * currents.b - 0.5 * currents.c),
        beta: (2.0 / 3.0) * SQRT_3_OVER_2 * (currents.b - currents.c),
    }
}

pub fn park(stationary: AlphaBeta, electrical_angle: f32) -> Dq {
    let (sin, cos) = sin_cos(electrical_angle);
    Dq {
        d: cos * stationary.alpha + sin * stationary.beta,
        q: cos * stationary.beta - sin * stationary.alpha,
    }
}

pub fn inverse_park(rotating: Dq, electrical_angle: f32) -> AlphaBeta {
    let (sin, cos) = sin_cos(electrical_angle);
    AlphaBeta {
        alpha: cos * rotating.d - sin * rotating.q,
        beta: sin * rotating.d + cos * rotating.q,
    }
}

pub fn svpwm(
    voltage: AlphaBeta,
    bus_voltage: f32,
    min_duty: f32,
    max_duty: f32,
) -> PhaseDuty {
    if !bus_voltage.is_finite()
        || bus_voltage <= 0.0
        || !voltage.alpha.is_finite()
        || !voltage.beta.is_finite()
        || !min_duty.is_finite()
        || !max_duty.is_finite()
        || min_duty >= max_duty
    {
        return PhaseDuty::DISABLED;
    }

    let phase_a = voltage.alpha;
    let phase_b = -0.5 * voltage.alpha + SQRT_3_OVER_2 * voltage.beta;
    let phase_c = -0.5 * voltage.alpha - SQRT_3_OVER_2 * voltage.beta;
    let maximum = phase_a.max(phase_b).max(phase_c);
    let minimum = phase_a.min(phase_b).min(phase_c);
    let common_mode = -0.5 * (maximum + minimum);
    let centered_a = phase_a + common_mode;
    let centered_b = phase_b + common_mode;
    let centered_c = phase_c + common_mode;
    let available = 0.5 * (max_duty - min_duty) * bus_voltage;
    let requested =
        centered_a.abs().max(centered_b.abs()).max(centered_c.abs());
    let scale = if requested > available {
        available / requested
    } else {
        1.0
    };

    PhaseDuty {
        a: (0.5 + centered_a * scale / bus_voltage).clamp(min_duty, max_duty),
        b: (0.5 + centered_b * scale / bus_voltage).clamp(min_duty, max_duty),
        c: (0.5 + centered_c * scale / bus_voltage).clamp(min_duty, max_duty),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn sin_cos_matches_cardinal_angles() {
        for (angle, expected_sin, expected_cos) in [
            (0.0, 0.0, 1.0),
            (FRAC_PI_2, 1.0, 0.0),
            (PI, 0.0, -1.0),
            (3.0 * FRAC_PI_2, -1.0, 0.0),
            (TAU, 0.0, 1.0),
        ] {
            let (sin, cos) = sin_cos(angle);
            close(sin, expected_sin, 3.0e-5);
            close(cos, expected_cos, 3.0e-5);
        }
    }

    #[test]
    fn clarke_uses_amplitude_invariant_scaling() {
        let transformed = clarke(PhaseCurrents {
            a: 10.0,
            b: -5.0,
            c: -5.0,
        });
        close(transformed.alpha, 10.0, 1.0e-5);
        close(transformed.beta, 0.0, 1.0e-5);
    }

    #[test]
    fn park_round_trip() {
        let stationary = AlphaBeta {
            alpha: 3.2,
            beta: -1.7,
        };
        let angle = 1.234;
        let restored = inverse_park(park(stationary, angle), angle);
        close(restored.alpha, stationary.alpha, 2.0e-4);
        close(restored.beta, stationary.beta, 2.0e-4);
    }

    #[test]
    fn svpwm_centers_zero_vector() {
        assert_eq!(
            svpwm(AlphaBeta::default(), 24.0, 0.05, 0.95),
            PhaseDuty::DISABLED
        );
    }

    #[test]
    fn svpwm_disables_non_finite_voltage() {
        let duty = svpwm(
            AlphaBeta {
                alpha: f32::NAN,
                beta: 0.0,
            },
            24.0,
            0.05,
            0.95,
        );
        assert_eq!(duty, PhaseDuty::DISABLED);
    }
    #[test]
    fn svpwm_clamps_overmodulation() {
        let duty = svpwm(
            AlphaBeta {
                alpha: 100.0,
                beta: 100.0,
            },
            24.0,
            0.05,
            0.95,
        );
        for value in [duty.a, duty.b, duty.c] {
            assert!((0.05..=0.95).contains(&value));
        }
        close(duty.a.max(duty.b).max(duty.c), 0.95, 1.0e-6);
        close(duty.a.min(duty.b).min(duty.c), 0.05, 1.0e-6);
    }
}
