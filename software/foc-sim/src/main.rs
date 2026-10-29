mod commissioning_cli;

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use foc_sim::plant::PlantParams;
use foc_sim::runner::{
    CONTROL_PERIOD_NS, PHYSICS_STEP_NS, Record, Scenario, controller_config, run,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let selection = args.next();
    if selection.as_deref() == Some("autotune") {
        return commissioning_cli::run(args);
    }
    let scenario = match selection.as_deref() {
        Some("current") => Scenario::Current,
        Some("speed") => Scenario::Speed,
        Some("open-loop") => Scenario::OpenLoop,
        _ => {
            return Err(
                "usage: foc-sim <current|speed|open-loop> [--csv path] [--json path]".into(),
            );
        }
    };
    let mut csv_path = None;
    let mut json_path = None;
    while let Some(flag) = args.next() {
        let path = args.next().ok_or("missing output path")?;
        match flag.as_str() {
            "--csv" if csv_path.is_none() => csv_path = Some(path),
            "--json" if json_path.is_none() => json_path = Some(path),
            _ => {
                return Err(
                    "usage: foc-sim <current|speed|open-loop> [--csv path] [--json path]".into(),
                );
            }
        }
    }
    let mut csv = if let Some(path) = csv_path {
        let mut writer = BufWriter::new(File::create(Path::new(&path))?);
        writeln!(
            writer,
            "time_s,state,id_true,iq_true,speed_true,iq_measured,duty_a,duty_b,duty_c,bridge_commanded,bridge_effective"
        )?;
        Some(writer)
    } else {
        None
    };
    let mut write_error = None;
    let summary = run(scenario, |record: Record| {
        if let Some(writer) = &mut csv
            && let Err(error) = writeln!(
                writer,
                "{:.6},{:?},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{}",
                record.time_ns as f64 * 1.0e-9,
                record.state,
                record.truth.id,
                record.truth.iq,
                record.truth.mechanical_speed,
                record.iq_measured,
                record.output.duty.a,
                record.output.duty.b,
                record.output.duty.c,
                record.output.bridge_enabled,
                record.effective.bridge_enabled
            )
        {
            write_error = Some(error);
        }
    })?;
    if let Some(error) = write_error {
        return Err(error.into());
    }
    if let Some(mut writer) = csv {
        writer.flush()?;
    }
    let iq_json = if matches!(scenario, Scenario::Current) {
        summary.mean_iq_error.to_string()
    } else {
        "null".to_owned()
    };
    let speed_json = if matches!(scenario, Scenario::Speed) {
        summary.mean_speed_error.to_string()
    } else {
        "null".to_owned()
    };
    if let Some(path) = json_path {
        let plant = PlantParams::default();
        let config = controller_config();
        let mut writer = BufWriter::new(File::create(Path::new(&path))?);
        writeln!(
            writer,
            "{{\n  \"phase\": \"M1-sensored-average\",\n  \"scenario\": \"{}\",\n  \"passed\": {},\n  \"faulted\": {},\n  \"final_iq_a\": {},\n  \"final_speed_rad_s\": {},\n  \"mean_iq_error_a\": {},\n  \"mean_speed_error_rad_s\": {},\n  \"peak_phase_current_a\": {},\n  \"control_period_ns\": {},\n  \"physics_step_ns\": {},\n  \"pwm_delay_ticks\": 1,\n  \"bus_voltage_v\": 24.0,\n  \"plant\": {{\"rs_ohm\": {}, \"ld_h\": {}, \"lq_h\": {}, \"flux_wb\": {}, \"pole_pairs\": {}, \"inertia_kg_m2\": {}, \"friction_nm_s_rad\": {}}},\n  \"controller\": {{\"current_kp\": {}, \"current_ki\": {}, \"speed_kp\": {}, \"speed_ki\": {}, \"current_limit_a\": {}}}\n}}",
            summary.scenario.name(),
            summary.passed(),
            summary.faulted,
            summary.final_iq,
            summary.final_speed,
            iq_json,
            speed_json,
            summary.peak_phase_current,
            CONTROL_PERIOD_NS,
            PHYSICS_STEP_NS,
            plant.resistance,
            plant.ld,
            plant.lq,
            plant.flux,
            plant.pole_pairs,
            plant.inertia,
            plant.friction,
            config.current_pid.kp,
            config.current_pid.ki,
            config.velocity_pid.kp,
            config.velocity_pid.ki,
            config.current_limit
        )?;
        writer.flush()?;
    }
    let iq_display = if matches!(scenario, Scenario::Current) {
        format!("{:.3} A", summary.mean_iq_error)
    } else {
        "n/a".to_owned()
    };
    let speed_display = if matches!(scenario, Scenario::Speed) {
        format!("{:.3} rad/s", summary.mean_speed_error)
    } else {
        "n/a".to_owned()
    };
    println!(
        "scenario={} passed={} final_iq={:.3} A final_speed={:.3} rad/s iq_error={} speed_error={} peak_phase={:.3} A faulted={}",
        summary.scenario.name(),
        summary.passed(),
        summary.final_iq,
        summary.final_speed,
        iq_display,
        speed_display,
        summary.peak_phase_current,
        summary.faulted
    );
    if !summary.passed() {
        return Err("scenario failed acceptance checks".into());
    }
    Ok(())
}
