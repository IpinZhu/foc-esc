use foc_firmware::control::foc_math::PhaseDuty;
use foc_firmware::interfaces::ControlOutput;
use foc_sim::inverter::voltage;
use foc_sim::plant::{Plant, PlantParams, StationaryVoltage};
use foc_sim::runner::controller_config;
use foc_sim::sampling::sample;

#[test]
fn average_inverter_respects_common_mode_and_bridge_enable() {
    let mut output = ControlOutput {
        bridge_enabled: true,
        duty: PhaseDuty {
            a: 0.6,
            b: 0.45,
            c: 0.45,
        },
        ..ControlOutput::default()
    };
    let v = voltage(output, 24.0).unwrap().unwrap();
    assert!((v.alpha - 2.4).abs() < 1.0e-5);
    assert!(v.beta.abs() < 1.0e-5);
    output.duty = PhaseDuty {
        a: 0.7,
        b: 0.55,
        c: 0.55,
    };
    assert!((voltage(output, 24.0).unwrap().unwrap().alpha - v.alpha).abs() < 1.0e-5);
    output.bridge_enabled = false;
    assert!(voltage(output, 24.0).unwrap().is_none());
}

#[test]
fn invalid_effective_duty_is_not_silently_clamped() {
    let output = ControlOutput {
        bridge_enabled: true,
        duty: PhaseDuty {
            a: f32::NAN,
            b: 0.5,
            c: 0.5,
        },
        ..ControlOutput::default()
    };
    assert!(voltage(output, 24.0).is_err());
}

#[test]
fn adc_quantization_is_independent_of_firmware_current_transform() {
    let params = PlantParams::default();
    let mut plant = Plant::new(params).unwrap();
    plant.state.id = 1.0;
    let config = controller_config();
    let (raw, rotor) = sample(&plant, &config, 24.0, 7);
    let current_per_count = f64::from(config.adc_reference_voltage)
        / f64::from(config.adc_full_scale)
        / (f64::from(config.current_sense_gain) * f64::from(config.shunt_resistance));
    assert_eq!(
        raw.phase_a,
        (2048.0 + 1.0 / current_per_count).round() as u16
    );
    assert_eq!(raw.sequence, 7);
    assert!(rotor.valid);
    plant.advance(
        Some(StationaryVoltage {
            alpha: 0.0,
            beta: 0.0,
        }),
        1.0e-6,
    );
    assert!(plant.state.id < 1.0);
}
