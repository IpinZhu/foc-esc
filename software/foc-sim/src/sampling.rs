use foc_firmware::control::foc_core::FocConfig;
use foc_firmware::interfaces::{RawAdcFrame, RotorSample};

use crate::plant::Plant;

pub fn sample(
    plant: &Plant,
    config: &FocConfig,
    bus_voltage: f64,
    sequence: u32,
) -> (RawAdcFrame, RotorSample) {
    let phase = plant.state.phase_currents(plant.params);
    let current_per_count = f64::from(config.adc_reference_voltage)
        / f64::from(config.adc_full_scale)
        / f64::from(config.current_sense_gain * config.shunt_resistance);
    let bus_per_count = f64::from(config.adc_reference_voltage) / f64::from(config.adc_full_scale)
        * f64::from((config.bus_divider_high + config.bus_divider_low) / config.bus_divider_low);
    let temperature_voltage = f64::from(config.temperature_sensor_zero_voltage)
        + f64::from(config.ambient_temperature * config.temperature_sensor_volts_per_degree);
    let raw = RawAdcFrame {
        phase_a: adc(2048.0 + phase[0] / current_per_count),
        phase_b: adc(2048.0 + phase[1] / current_per_count),
        phase_c: adc(2048.0 + phase[2] / current_per_count),
        bus_voltage: adc(bus_voltage / bus_per_count),
        ntc: adc(
            temperature_voltage / f64::from(config.adc_reference_voltage)
                * f64::from(config.adc_full_scale),
        ),
        sequence,
        overrun: false,
    };
    let rotor = RotorSample {
        mechanical_angle: plant.state.mechanical_angle as f32,
        mechanical_velocity: plant.state.mechanical_speed as f32,
        valid: true,
    };
    (raw, rotor)
}

fn adc(value: f64) -> u16 {
    value.round().clamp(0.0, 4095.0) as u16
}
