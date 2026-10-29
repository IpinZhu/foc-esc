//! UI-independent device model. Hardware protocols implement DeviceDriver.
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Parameters {
    pub target_speed: f64,
    pub current_limit: f64,
    pub kp: f64,
    pub ki: f64,
}
impl Default for Parameters {
    fn default() -> Self {
        Self {
            target_speed: 1500.,
            current_limit: 8.,
            kp: 0.12,
            ki: 0.8,
        }
    }
}
impl Parameters {
    pub fn validate(&self) -> Result<(), String> {
        for (name, value, min, max) in [
            ("目标转速", self.target_speed, 0., 6000.),
            ("电流限制", self.current_limit, 0.1, 20.),
            ("Kp", self.kp, 0., 10.),
            ("Ki", self.ki, 0., 100.),
        ] {
            if !value.is_finite() || value < min || value > max {
                return Err(format!("{name} 必须在 {min}–{max} 之间"));
            }
        }
        Ok(())
    }
}
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Sample {
    pub time: f64,
    pub speed: f64,
    pub current: f64,
    pub voltage: f64,
    pub temperature: f64,
}
/// Implementations must distinguish link availability from protocol readiness.
pub trait DeviceDriver {
    fn read_parameters(&self) -> Parameters;
    fn write_parameters(&mut self, params: Parameters) -> Result<(), String>;
    fn sample(&mut self, dt: f64) -> Sample;
}
#[derive(Default)]
pub struct SimulatedDevice {
    params: Parameters,
    time: f64,
    speed: f64,
}
impl DeviceDriver for SimulatedDevice {
    fn read_parameters(&self) -> Parameters {
        self.params.clone()
    }
    fn write_parameters(&mut self, params: Parameters) -> Result<(), String> {
        params.validate()?;
        self.params = params;
        Ok(())
    }
    fn sample(&mut self, dt: f64) -> Sample {
        self.time += dt;
        self.speed += (self.params.target_speed - self.speed) * (1. - (-dt * 2.).exp());
        Sample {
            time: self.time,
            speed: self.speed + (self.time * 3.).sin() * 12.,
            current: (1.8 + (self.time * 4.).sin() * 0.25).min(self.params.current_limit),
            voltage: 24. + (self.time * 1.7).sin() * 0.08,
            temperature: 32. + (self.time * 0.15).sin() * 0.5,
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_nonfinite_and_out_of_range() {
        for value in [f64::NAN, f64::INFINITY, -1., 6001.] {
            let p = Parameters {
                target_speed: value,
                ..Default::default()
            };
            assert!(p.validate().is_err());
        }
    }
    #[test]
    fn invalid_write_is_atomic() {
        let mut device = SimulatedDevice::default();
        let p = Parameters {
            current_limit: 21.,
            ..Default::default()
        };
        assert!(device.write_parameters(p).is_err());
        assert_eq!(device.read_parameters().current_limit, 8.);
    }
    #[test]
    fn simulator_tracks_parameter_changes() {
        let mut device = SimulatedDevice::default();
        device
            .write_parameters(Parameters {
                target_speed: 3000.,
                ..Default::default()
            })
            .unwrap();
        let mut value = 0.;
        for _ in 0..200 {
            value = device.sample(0.05).speed;
        }
        assert!((value - 3000.).abs() < 15.);
    }
}
