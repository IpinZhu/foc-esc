use foc_firmware::params::parameters::{
    ParameterId, ParameterKind, ParameterProfileV1, ParameterValue,
};
use serde::Serialize;
use std::collections::BTreeMap;

pub type Parameters = BTreeMap<String, f64>;

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ParameterDescriptor {
    pub name: String,
    pub label: &'static str,
    pub group: &'static str,
    pub unit: &'static str,
    pub kind: &'static str,
    pub min: f64,
    pub max: f64,
    pub startup_only: bool,
}

pub fn descriptors() -> Vec<ParameterDescriptor> {
    ParameterId::ALL
        .into_iter()
        .map(|id| {
            use ParameterId::*;
            let (label, group, unit, min, max) = match id {
                BatteryCells => ("电池串数", "电机与传感器", "S", 2., 6.),
                PolePairs => ("极对数", "电机与传感器", "", 1., 64.),
                SensorDirection => ("传感器方向", "电机与传感器", "±1", -1., 1.),
                EncoderCpr => ("编码器计数/转", "电机与传感器", "count", 2., 65535.),
                ElectricalZero => (
                    "电角度零点",
                    "电机与传感器",
                    "rad",
                    0.,
                    std::f64::consts::TAU,
                ),
                CurrentKp => ("电流环 Kp", "电流环", "", 0., 1000000.),
                CurrentKi => ("电流环 Ki", "电流环", "", 0., 1000000.),
                CurrentKd => ("电流环 Kd", "电流环", "", 0., 1000000.),
                CurrentOutputLimit => (
                    "电流环输出限制",
                    "电流环",
                    "V",
                    f32::from_bits(1) as f64,
                    1000000.,
                ),
                CurrentOutputRamp => ("电流环输出斜率", "电流环", "V/s", 0., 1000000.),
                VelocityKp => ("速度环 Kp", "速度环", "", 0., 1000000.),
                VelocityKi => ("速度环 Ki", "速度环", "", 0., 1000000.),
                VelocityKd => ("速度环 Kd", "速度环", "", 0., 1000000.),
                VelocityOutputLimit => (
                    "速度环输出限制",
                    "速度环",
                    "A",
                    f32::from_bits(1) as f64,
                    1000000.,
                ),
                VelocityOutputRamp => ("速度环输出斜率", "速度环", "A/s", 0., 1000000.),
                AdcReferenceVoltage => ("ADC 参考电压", "采样标定", "V", 2.5, 3.6),
                CurrentSenseGain => ("电流放大倍数", "采样标定", "", 1., 500.),
                ShuntResistance => ("分流电阻", "采样标定", "Ω", 0.00005, 0.1),
                BusDividerHigh => ("母线分压高侧", "采样标定", "Ω", 100., 10000000.),
                BusDividerLow => ("母线分压低侧", "采样标定", "Ω", 100., 10000000.),
                CurrentLimit => ("运行电流限制", "软件保护", "A", 0.01, 50.),
                OverCurrentThreshold => ("过流阈值", "软件保护", "A", 0.01, 55.),
                OverCurrentDebounce => ("过流去抖", "软件保护", "sample", 1., 10000.),
                ResidualThreshold => ("电流残差阈值", "软件保护", "A", 0., 3.),
                ResidualDebounce => ("残差去抖", "软件保护", "sample", 1., 10000.),
                CellUnderVoltage => ("单节欠压", "软件保护", "V", 2., 3.5),
                CellOverVoltage => ("单节过压", "软件保护", "V", 3.6, 4.25),
                BusFilterTimeConstant => ("母线滤波时间", "采样标定", "s", 0.00001, 1.),
                MosfetHotResistance => ("MOS 热态电阻", "热模型", "Ω", 0.00001, 0.1),
                ThermalResistance => ("热阻", "热模型", "°C/W", 0.01, 100.),
                ThermalTimeConstant => ("热时间常数", "热模型", "s", 0.01, 1000.),
                AmbientTemperature => ("环境温度", "热模型", "°C", -40., 99.),
                DerateTemperature => ("降额温度", "热模型", "°C", -39., 100.),
                ShutdownTemperature => ("关断温度", "热模型", "°C", -38., 125.),
                StallTargetVelocity => ("堵转目标速度阈值", "软件保护", "rad/s", 0.01, 1000.),
                StallMaxVelocity => ("堵转实际速度阈值", "软件保护", "rad/s", 0., 1000.),
                StallMinIq => ("堵转最小 Iq", "软件保护", "A", 0., 50.),
                StallTime => ("堵转持续时间", "软件保护", "s", 0.01, 10.),
                SensorFaultTime => ("传感器无效持续时间", "软件保护", "s", 0.00001, 1.),
                MaximumVelocity => ("最大机械角速度", "速度环", "rad/s", 0.01, 1000.),
                MaximumOpenLoopVelocity => {
                    ("最大开环电角速度", "电机与传感器", "rad/s", 0.01, 10000.)
                }
                TemperatureSensorZeroVoltage => ("温度传感器零点", "采样标定", "V", 0., 3.6),
                TemperatureSensorVoltsPerDegree => {
                    ("温度传感器斜率", "采样标定", "V/°C", 0.0001, 0.1)
                }
            };
            ParameterDescriptor {
                name: id.name().into(),
                label,
                group,
                unit,
                min,
                max,
                kind: match id.kind() {
                    ParameterKind::U8 => "u8",
                    ParameterKind::I8 => "i8",
                    ParameterKind::U16 => "u16",
                    ParameterKind::F32 => "f32",
                },
                startup_only: id.startup_only(),
            }
        })
        .collect()
}

pub fn number(value: ParameterValue) -> f64 {
    match value {
        ParameterValue::U8(v) => v as f64,
        ParameterValue::I8(v) => v as f64,
        ParameterValue::U16(v) => v as f64,
        ParameterValue::F32(v) => v.to_string().parse().unwrap_or(v as f64),
    }
}
pub fn defaults() -> Parameters {
    let profile = ParameterProfileV1::default();
    ParameterId::ALL
        .into_iter()
        .map(|id| (id.name().into(), number(profile.get(id))))
        .collect()
}
pub fn value(id: ParameterId, v: f64) -> Result<ParameterValue, String> {
    if !v.is_finite() {
        return Err("参数必须为有限数值".into());
    }
    let descriptor = descriptors()
        .into_iter()
        .find(|d| d.name == id.name())
        .unwrap();
    if v < descriptor.min || v > descriptor.max {
        return Err(format!("{} 超出范围", id.name()));
    }
    Ok(match id.kind() {
        ParameterKind::U8 | ParameterKind::I8 | ParameterKind::U16 if v.fract() != 0. => {
            return Err("整数参数不能包含小数".into());
        }
        ParameterKind::U8 => ParameterValue::U8(v as u8),
        ParameterKind::I8 => ParameterValue::I8(v as i8),
        ParameterKind::U16 => ParameterValue::U16(v as u16),
        ParameterKind::F32 => ParameterValue::F32(v as f32),
    })
}
pub fn profile(values: &Parameters) -> Result<ParameterProfileV1, String> {
    if values.len() != ParameterId::ALL.len() {
        return Err("需先读取全部设备参数".into());
    }
    let mut profile = ParameterProfileV1::default();
    for id in ParameterId::ALL {
        let raw = *values.get(id.name()).ok_or("缺少设备参数")?;
        let typed = value(id, raw)?;
        macro_rules! f {
            ($field:expr) => {
                if let ParameterValue::F32(v) = typed {
                    $field = v;
                }
            };
        }
        use ParameterId::*;
        match id {
            BatteryCells => {
                if let ParameterValue::U8(v) = typed {
                    profile.battery_cells = v;
                }
            }
            PolePairs => {
                if let ParameterValue::U8(v) = typed {
                    profile.pole_pairs = v;
                }
            }
            SensorDirection => {
                if let ParameterValue::I8(v) = typed {
                    profile.sensor_direction = v;
                }
            }
            EncoderCpr => {
                if let ParameterValue::U16(v) = typed {
                    profile.encoder_cpr = v;
                }
            }
            ElectricalZero => f!(profile.electrical_zero),
            CurrentKp => f!(profile.current_pid.kp),
            CurrentKi => f!(profile.current_pid.ki),
            CurrentKd => f!(profile.current_pid.kd),
            CurrentOutputLimit => f!(profile.current_pid.output_limit),
            CurrentOutputRamp => f!(profile.current_pid.output_ramp),
            VelocityKp => f!(profile.velocity_pid.kp),
            VelocityKi => f!(profile.velocity_pid.ki),
            VelocityKd => f!(profile.velocity_pid.kd),
            VelocityOutputLimit => f!(profile.velocity_pid.output_limit),
            VelocityOutputRamp => f!(profile.velocity_pid.output_ramp),
            AdcReferenceVoltage => f!(profile.adc_reference_voltage),
            CurrentSenseGain => f!(profile.current_sense_gain),
            ShuntResistance => f!(profile.shunt_resistance),
            BusDividerHigh => f!(profile.bus_divider_high),
            BusDividerLow => f!(profile.bus_divider_low),
            CurrentLimit => f!(profile.current_limit),
            OverCurrentThreshold => f!(profile.over_current_threshold),
            OverCurrentDebounce => {
                if let ParameterValue::U16(v) = typed {
                    profile.over_current_debounce = v;
                }
            }
            ResidualThreshold => f!(profile.residual_threshold),
            ResidualDebounce => {
                if let ParameterValue::U16(v) = typed {
                    profile.residual_debounce = v;
                }
            }
            CellUnderVoltage => f!(profile.cell_under_voltage),
            CellOverVoltage => f!(profile.cell_over_voltage),
            BusFilterTimeConstant => f!(profile.bus_filter_time_constant),
            MosfetHotResistance => f!(profile.mosfet_hot_resistance),
            ThermalResistance => f!(profile.thermal_resistance),
            ThermalTimeConstant => f!(profile.thermal_time_constant),
            AmbientTemperature => f!(profile.ambient_temperature),
            DerateTemperature => f!(profile.derate_temperature),
            ShutdownTemperature => f!(profile.shutdown_temperature),
            StallTargetVelocity => f!(profile.stall_target_velocity),
            StallMaxVelocity => f!(profile.stall_max_velocity),
            StallMinIq => f!(profile.stall_min_iq),
            StallTime => f!(profile.stall_time),
            SensorFaultTime => f!(profile.sensor_fault_time),
            MaximumVelocity => f!(profile.maximum_velocity),
            MaximumOpenLoopVelocity => f!(profile.maximum_open_loop_velocity),
            TemperatureSensorZeroVoltage => f!(profile.temperature_sensor_zero_voltage),
            TemperatureSensorVoltsPerDegree => f!(profile.temperature_sensor_volts_per_degree),
        }
    }
    profile
        .validate()
        .map_err(|e| format!("参数组合无效: {e:?}"))?;
    Ok(profile)
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Telemetry {
    pub sequence: u32,
    pub state: u8,
    pub faults: u32,
    pub ia: f64,
    pub ib: f64,
    pub ic: f64,
    pub id: f64,
    pub iq: f64,
    pub vbus: f64,
    pub velocity: f64,
    pub temperature: f64,
    pub time: f64,
}
impl Telemetry {
    pub fn parse(line: &str, time: f64) -> Result<Self, String> {
        let mut fields = BTreeMap::new();
        for token in line.split_ascii_whitespace() {
            let (key, value) = token.split_once('=').ok_or("遥测字段格式错误")?;
            if fields.insert(key, value).is_some() {
                return Err("重复遥测字段".into());
            }
        }
        let get = |key| {
            fields
                .get(key)
                .copied()
                .ok_or_else(|| format!("遥测缺少 {key}"))
        };
        let finite = |key| -> Result<f64, String> {
            let v: f64 = get(key)?.parse().map_err(|_| format!("无效遥测 {key}"))?;
            if !v.is_finite() {
                return Err(format!("非有限遥测 {key}"));
            }
            Ok(v)
        };
        let state = get("state")?.parse::<u8>().map_err(|e| e.to_string())?;
        if state > 5 {
            return Err("未知电机状态".into());
        }
        Ok(Self {
            sequence: get("seq")?.parse::<u32>().map_err(|e| e.to_string())?,
            state,
            faults: u32::from_str_radix(get("faults")?, 16).map_err(|e| e.to_string())?,
            ia: finite("ia")?,
            ib: finite("ib")?,
            ic: finite("ic")?,
            id: finite("id")?,
            iq: finite("iq")?,
            vbus: finite("vbus")?,
            velocity: finite("vel")?,
            temperature: finite("temp")?,
            time,
        })
    }
}

#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Snapshot {
    pub connected: bool,
    pub ready: bool,
    pub controlled: bool,
    pub source: String,
    pub identity: String,
    pub bridge: String,
    pub telemetry: Option<Telemetry>,
    pub autotune: String,
    pub recording: bool,
    pub record_count: usize,
    pub dropped: u64,
    pub error: Option<String>,
    pub packet_age_ms: Option<u64>,
}
impl Snapshot {
    pub fn new(source: &str) -> Self {
        Self {
            connected: true,
            ready: false,
            controlled: false,
            source: source.into(),
            identity: String::new(),
            bridge: String::new(),
            telemetry: None,
            autotune: String::new(),
            recording: false,
            record_count: 0,
            dropped: 0,
            error: None,
            packet_age_ms: None,
        }
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct Batch {
    pub snapshot: Snapshot,
    pub samples: Vec<Telemetry>,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn actual_firmware_descriptors_and_defaults() {
        assert_eq!(descriptors().len(), 43);
        assert!(profile(&defaults()).is_ok());
    }
    #[test]
    fn reject_fractional_and_nonfinite_parameters() {
        assert!(value(ParameterId::BatteryCells, 2.5).is_err());
        assert!(value(ParameterId::CurrentKp, f64::NAN).is_err());
        assert!(value(ParameterId::SensorDirection, 0.).is_ok());
        let mut values = defaults();
        values.insert("sensor-direction".into(), 0.);
        assert!(profile(&values).is_err());
    }
    #[test]
    fn telemetry_requires_all_finite_fields() {
        let line =
            "seq=100 state=1 faults=00000000 ia=1 ib=-0.5 ic=-0.5 id=0 iq=1 vbus=24 vel=12 temp=35";
        assert_eq!(Telemetry::parse(line, 1.).unwrap().velocity, 12.);
        assert!(Telemetry::parse(&line.replace("vbus=24", "vbus=NaN"), 1.).is_err());
        assert!(Telemetry::parse(&(line.to_owned() + " state=5"), 1.).is_err());
    }
}
