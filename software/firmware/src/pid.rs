#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub output_limit: f32,
    pub output_ramp: f32,
}

impl PidConfig {
    pub const fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        output_limit: f32,
        output_ramp: f32,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            output_limit,
            output_ramp,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PidController {
    config: PidConfig,
    integral: f32,
    previous_error: f32,
    previous_output: f32,
    initialized: bool,
}

impl PidController {
    pub const fn new(config: PidConfig) -> Self {
        Self {
            config,
            integral: 0.0,
            previous_error: 0.0,
            previous_output: 0.0,
            initialized: false,
        }
    }

    pub fn config(&self) -> PidConfig {
        self.config
    }

    pub fn set_config(&mut self, config: PidConfig) {
        self.config = config;
        self.integral = self
            .integral
            .clamp(-config.output_limit.abs(), config.output_limit.abs());
    }

    pub fn set_output_limit(&mut self, limit: f32) {
        self.config.output_limit = limit.abs();
        self.integral = self.integral.clamp(-limit.abs(), limit.abs());
        self.previous_output =
            self.previous_output.clamp(-limit.abs(), limit.abs());
    }

    pub fn update(&mut self, error: f32, dt: f32) -> f32 {
        if !error.is_finite() || !dt.is_finite() || dt <= 0.0 {
            return self.previous_output;
        }

        let limit = self.config.output_limit.abs();
        let proportional = self.config.kp * error;
        let derivative = if self.initialized {
            self.config.kd * (error - self.previous_error) / dt
        } else {
            0.0
        };
        let integral_candidate =
            (self.integral + self.config.ki * error * dt).clamp(-limit, limit);
        let unsaturated = proportional + integral_candidate + derivative;
        if !unsaturated.is_finite() {
            self.reset();
            return 0.0;
        }
        let saturated = unsaturated.clamp(-limit, limit);

        if unsaturated == saturated
            || (unsaturated > limit && error < 0.0)
            || (unsaturated < -limit && error > 0.0)
        {
            self.integral = integral_candidate;
        }

        let mut output =
            (proportional + self.integral + derivative).clamp(-limit, limit);
        if self.initialized
            && self.config.output_ramp.is_finite()
            && self.config.output_ramp > 0.0
        {
            let max_delta = self.config.output_ramp * dt;
            output = output.clamp(
                self.previous_output - max_delta,
                self.previous_output + max_delta,
            );
        }

        self.previous_error = error;
        self.previous_output = output;
        self.initialized = true;
        output
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.previous_output = 0.0;
        self.initialized = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_output_and_prevents_windup() {
        let mut pid = PidController::new(PidConfig::new(
            1.0,
            100.0,
            0.0,
            2.0,
            f32::INFINITY,
        ));
        for _ in 0..1_000 {
            assert!(pid.update(10.0, 0.001) <= 2.0);
        }
        assert!(pid.update(-1.0, 0.001) < 2.0);
    }

    #[test]
    fn limits_output_slew_rate() {
        let mut pid =
            PidController::new(PidConfig::new(10.0, 0.0, 0.0, 100.0, 5.0));
        assert_eq!(pid.update(0.0, 0.1), 0.0);
        let output = pid.update(10.0, 0.1);
        assert!((output - 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn non_finite_intermediate_resets_output() {
        let mut pid = PidController::new(PidConfig::new(
            f32::MAX,
            0.0,
            -f32::MAX,
            100.0,
            f32::INFINITY,
        ));
        assert_eq!(pid.update(1.0, 0.001), 100.0);
        assert_eq!(pid.update(0.0, 0.001), 0.0);
    }
    #[test]
    fn reset_clears_controller_history() {
        let mut pid = PidController::new(PidConfig::new(
            1.0,
            1.0,
            1.0,
            100.0,
            f32::INFINITY,
        ));
        pid.update(2.0, 0.01);
        pid.reset();
        assert!((pid.update(0.0, 0.01)).abs() < 1.0e-6);
    }
}
