use crate::control::foc_math::PhaseDuty;
use crate::interfaces::RotorSample;

pub trait PwmBridge {
    fn set_duty(&mut self, duty: PhaseDuty);
    fn enable(&mut self);
    fn disable(&mut self);
    fn is_enabled(&self) -> bool;
}

pub trait RotorSensor {
    fn sample(&mut self, dt: f32) -> RotorSample;
}
