use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use foc_sim::commissioning::{CommissioningConfig, Fixture, StageRecord, run_commissioning};

pub fn run(mut args: impl Iterator<Item = String>) -> Result<(), Box<dyn std::error::Error>> {
    let mut config = CommissioningConfig::baseline();
    let mut csv_path = None;
    let mut json_path = None;
    while let Some(flag) = args.next() {
        let value = args.next().ok_or("missing autotune option value")?;
        match flag.as_str() {
            "--csv" if csv_path.is_none() => csv_path = Some(value),
            "--json" if json_path.is_none() => json_path = Some(value),
            "--angle-electrical" => {
                let angle: f64 = value.parse()?;
                if !angle.is_finite() {
                    return Err("angle must be finite".into());
                }
                config.initial_mechanical_angle = angle / f64::from(config.motor.pole_pairs);
            }
            "--fixture" if value == "locked" => config.fixture = Fixture::AlwaysLocked,
            _ => return Err("usage: foc-sim autotune [--angle-electrical rad] [--fixture locked] [--csv path] [--json path]".into()),
        }
    }
    let mut csv = if let Some(path) = csv_path {
        let mut writer = BufWriter::new(File::create(Path::new(&path))?);
        writeln!(
            writer,
            "time_s,state,error,mechanical_speed_rad_s,rotor_locked"
        )?;
        Some(writer)
    } else {
        None
    };
    let mut write_error = None;
    let result = run_commissioning(config, |stage: StageRecord| {
        if let Some(writer) = &mut csv
            && let Err(error) = writeln!(
                writer,
                "{:.6},{},{},{:.6},{}",
                stage.time_ns as f64 * 1.0e-9,
                stage.state.name(),
                stage.error.name(),
                stage.mechanical_speed,
                stage.locked_rotor
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
    if let Some(path) = json_path {
        let mut writer = BufWriter::new(File::create(Path::new(&path))?);
        let m = result.motor;
        let t = result.truth;
        let a = result.accuracy;
        let pi = result.pi_metrics;
        let pi_metrics = if let Some(pi) = pi {
            format!(
                "{{\"peak_a\":{},\"overshoot_ratio\":{},\"rms_error_a\":{},\"oscillations\":{}}}",
                pi.peak, pi.overshoot, pi.rms_error, pi.oscillations
            )
        } else {
            "null".to_owned()
        };
        writeln!(
            writer,
            "{{\n  \"scenario\": \"autotune\",\n  \"passed\": {},\n  \"state\": \"{}\",\n  \"error\": \"{}\",\n  \"frames\": {},\n  \"rotor_released\": {},\n  \"peak_speed_rad_s\": {},\n  \"pi_verified\": {},\n  \"pi_metrics\": {},\n  \"verified_bandwidth_hz\": {},\n  \"save_request_only\": {},\n  \"flash_persisted\": false,\n  \"fixture\": \"{}\",\n  \"initial_mechanical_angle_rad\": {},\n  \"pi_step_ratio\": {},\n  \"truth\": {{\"rs_ohm\": {}, \"ld_h\": {}, \"lq_h\": {}, \"flux_wb\": {}}},\n  \"estimated\": {{\"rs_ohm\": {}, \"ld_h\": {}, \"lq_h\": {}, \"flux_wb\": {}, \"id_kp\": {}, \"id_ki\": {}, \"iq_kp\": {}, \"iq_ki\": {}, \"speed_kp_inherited\": {}, \"speed_ki_inherited\": {}}},\n  \"live\": {{\"current_kp\": {}, \"current_ki\": {}, \"speed_kp\": {}, \"speed_ki\": {}}},\n  \"relative_error\": {{\"rs\": {}, \"ld\": {}, \"lq\": {}, \"flux\": {}}}\n}}",
            result.passed(),
            result.state.name(),
            result.error.name(),
            result.frames,
            result.released,
            result.max_speed,
            result.pi_verified,
            pi_metrics,
            result.verified_bandwidth_hz,
            result.save_requested,
            if matches!(config.fixture, Fixture::AlwaysLocked) {
                "locked"
            } else {
                "release-for-open-loop"
            },
            config.initial_mechanical_angle,
            config.tune.pi_step_ratio,
            t.resistance,
            t.ld,
            t.lq,
            t.flux,
            m.rs,
            m.ld,
            m.lq,
            m.flux,
            m.id_kp,
            m.id_ki,
            m.iq_kp,
            m.iq_ki,
            m.speed_kp,
            m.speed_ki,
            result.live_current_kp,
            result.live_current_ki,
            result.live_speed_kp,
            result.live_speed_ki,
            a.rs_relative,
            a.ld_relative,
            a.lq_relative,
            a.flux_relative
        )?;
        writer.flush()?;
    }
    println!(
        "autotune passed={} state={} error={} Rs={:.5} Ω ({:.2}%) Ld={:.7} H ({:.2}%) Lq={:.7} H ({:.2}%) flux={:.6} Wb ({:.2}%) current_Kp={:.4} Ki={:.2} save_request_only={}",
        result.passed(),
        result.state.name(),
        result.error.name(),
        result.motor.rs,
        result.accuracy.rs_relative * 100.0,
        result.motor.ld,
        result.accuracy.ld_relative * 100.0,
        result.motor.lq,
        result.accuracy.lq_relative * 100.0,
        result.motor.flux,
        result.accuracy.flux_relative * 100.0,
        result.motor.iq_kp,
        result.motor.iq_ki,
        result.save_requested
    );
    if !result.passed() {
        return Err("commissioning failed simulated accuracy or safety checks".into());
    }
    Ok(())
}
