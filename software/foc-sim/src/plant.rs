use std::f64::consts::TAU;

#[derive(Clone, Copy, Debug)]
pub struct PlantParams {
    pub resistance: f64,
    pub ld: f64,
    pub lq: f64,
    pub flux: f64,
    pub pole_pairs: u8,
    pub inertia: f64,
    pub friction: f64,
}

impl Default for PlantParams {
    fn default() -> Self {
        Self {
            resistance: 0.5,
            ld: 0.0002,
            lq: 0.0002,
            flux: 0.01,
            pole_pairs: 4,
            inertia: 2.0e-5,
            friction: 2.0e-5,
        }
    }
}

impl PlantParams {
    pub fn validate(self) -> Result<(), &'static str> {
        if ![self.resistance, self.ld, self.lq, self.flux, self.inertia]
            .into_iter()
            .all(|value| value.is_finite() && value > 0.0)
            || !self.friction.is_finite()
            || self.friction < 0.0
            || self.pole_pairs == 0
        {
            return Err("invalid motor parameters");
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PlantState {
    pub id: f64,
    pub iq: f64,
    pub mechanical_angle: f64,
    pub mechanical_speed: f64,
}

impl PlantState {
    pub fn electrical_angle(self, params: PlantParams) -> f64 {
        (self.mechanical_angle * f64::from(params.pole_pairs)).rem_euclid(TAU)
    }

    pub fn phase_currents(self, params: PlantParams) -> [f64; 3] {
        let (sin, cos) = self.electrical_angle(params).sin_cos();
        let alpha = cos * self.id - sin * self.iq;
        let beta = sin * self.id + cos * self.iq;
        let root3_over_2 = 3.0_f64.sqrt() / 2.0;
        [
            alpha,
            -0.5 * alpha + root3_over_2 * beta,
            -0.5 * alpha - root3_over_2 * beta,
        ]
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StationaryVoltage {
    pub alpha: f64,
    pub beta: f64,
}

#[derive(Clone, Copy, Debug)]
pub enum Load {
    Signed(f64),
    Opposing(f64),
}

impl Default for Load {
    fn default() -> Self {
        Self::Signed(0.0)
    }
}

impl Load {
    fn torque(self, speed: f64) -> f64 {
        match self {
            Self::Signed(value) => value,
            Self::Opposing(value) => value * speed.signum(),
        }
    }
}

pub struct Plant {
    pub params: PlantParams,
    pub state: PlantState,
    pub load: Load,
    pub locked_rotor: bool,
    /// Open-circuit current decay approximation; not a diode or regeneration model.
    pub off_decay_seconds: f64,
}

impl Plant {
    pub fn new(params: PlantParams) -> Result<Self, &'static str> {
        params.validate()?;
        Ok(Self {
            params,
            state: PlantState::default(),
            load: Load::default(),
            locked_rotor: false,
            off_decay_seconds: 20.0e-6,
        })
    }

    fn derivative(&self, state: PlantState, voltage: Option<StationaryVoltage>) -> PlantState {
        let p = self.params;
        let omega_e = state.mechanical_speed * f64::from(p.pole_pairs);
        let (did, diq) = if let Some(v) = voltage {
            let (sin, cos) = state.electrical_angle(p).sin_cos();
            let vd = cos * v.alpha + sin * v.beta;
            let vq = cos * v.beta - sin * v.alpha;
            (
                (vd - p.resistance * state.id + omega_e * p.lq * state.iq) / p.ld,
                (vq - p.resistance * state.iq - omega_e * (p.ld * state.id + p.flux)) / p.lq,
            )
        } else {
            (
                -state.id / self.off_decay_seconds,
                -state.iq / self.off_decay_seconds,
            )
        };
        let torque = 1.5
            * f64::from(p.pole_pairs)
            * (p.flux * state.iq + (p.ld - p.lq) * state.id * state.iq);
        PlantState {
            id: did,
            iq: diq,
            mechanical_angle: if self.locked_rotor {
                0.0
            } else {
                state.mechanical_speed
            },
            mechanical_speed: if self.locked_rotor {
                0.0
            } else {
                (torque
                    - self.load.torque(state.mechanical_speed)
                    - p.friction * state.mechanical_speed)
                    / p.inertia
            },
        }
    }

    pub fn advance(&mut self, voltage: Option<StationaryVoltage>, dt: f64) {
        assert!(dt.is_finite() && dt > 0.0);
        assert!(!self.locked_rotor || self.state.mechanical_speed == 0.0);
        assert!(self.off_decay_seconds.is_finite() && self.off_decay_seconds > 0.0);
        let start = self.state;
        let k1 = self.derivative(start, voltage);
        let k2 = self.derivative(add(start, k1, dt * 0.5), voltage);
        let k3 = self.derivative(add(start, k2, dt * 0.5), voltage);
        let k4 = self.derivative(add(start, k3, dt), voltage);
        self.state = PlantState {
            id: start.id + dt / 6.0 * (k1.id + 2.0 * k2.id + 2.0 * k3.id + k4.id),
            iq: start.iq + dt / 6.0 * (k1.iq + 2.0 * k2.iq + 2.0 * k3.iq + k4.iq),
            mechanical_angle: (start.mechanical_angle
                + dt / 6.0
                    * (k1.mechanical_angle
                        + 2.0 * k2.mechanical_angle
                        + 2.0 * k3.mechanical_angle
                        + k4.mechanical_angle))
                .rem_euclid(TAU),
            mechanical_speed: start.mechanical_speed
                + dt / 6.0
                    * (k1.mechanical_speed
                        + 2.0 * k2.mechanical_speed
                        + 2.0 * k3.mechanical_speed
                        + k4.mechanical_speed),
        };
        assert!(
            self.state.id.is_finite()
                && self.state.iq.is_finite()
                && self.state.mechanical_speed.is_finite()
        );
    }
}

fn add(state: PlantState, rate: PlantState, dt: f64) -> PlantState {
    PlantState {
        id: state.id + rate.id * dt,
        iq: state.iq + rate.iq * dt,
        mechanical_angle: state.mechanical_angle + rate.mechanical_angle * dt,
        mechanical_speed: state.mechanical_speed + rate.mechanical_speed * dt,
    }
}
