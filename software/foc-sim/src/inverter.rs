use foc_firmware::interfaces::ControlOutput;

use crate::plant::StationaryVoltage;

/// Average line-neutral voltage from effective PWM duty, not requested dq voltage.
pub fn voltage(
    output: ControlOutput,
    bus_voltage: f64,
) -> Result<Option<StationaryVoltage>, &'static str> {
    if !bus_voltage.is_finite() || bus_voltage <= 0.0 {
        return Err("invalid bus voltage");
    }
    if !output.bridge_enabled {
        return Ok(None);
    }
    let duty = output.duty;
    if ![duty.a, duty.b, duty.c]
        .into_iter()
        .all(|d| d.is_finite() && (0.0..=1.0).contains(&d))
    {
        return Err("invalid effective duty");
    }
    let a = f64::from(duty.a);
    let b = f64::from(duty.b);
    let c = f64::from(duty.c);
    let mean = (a + b + c) / 3.0;
    let va = (a - mean) * bus_voltage;
    let vb = (b - mean) * bus_voltage;
    let vc = (c - mean) * bus_voltage;
    Ok(Some(StationaryVoltage {
        alpha: (2.0 * va - vb - vc) / 3.0,
        beta: (vb - vc) / 3.0_f64.sqrt(),
    }))
}
