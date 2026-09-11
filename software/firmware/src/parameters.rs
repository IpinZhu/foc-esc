use core::f32::consts::TAU;

use crate::foc_core::FocConfig;
use crate::foc_math::normalize_angle;
use crate::pid::PidConfig;

pub const DEFAULT_ENCODER_CPR: u16 = 4_096;
pub const PARAMETER_PAYLOAD_SIZE: usize = 160;
pub const PARAMETER_COUNT: usize = 43;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterKind {
    U8,
    I8,
    U16,
    F32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterValue {
    U8(u8),
    I8(i8),
    U16(u16),
    F32(f32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ParameterId {
    BatteryCells = 0x01,
    PolePairs = 0x02,
    SensorDirection = 0x03,
    EncoderCpr = 0x04,
    ElectricalZero = 0x05,
    CurrentKp = 0x10,
    CurrentKi = 0x11,
    CurrentKd = 0x12,
    CurrentOutputLimit = 0x13,
    CurrentOutputRamp = 0x14,
    VelocityKp = 0x18,
    VelocityKi = 0x19,
    VelocityKd = 0x1a,
    VelocityOutputLimit = 0x1b,
    VelocityOutputRamp = 0x1c,
    AdcReferenceVoltage = 0x20,
    CurrentSenseGain = 0x21,
    ShuntResistance = 0x22,
    BusDividerHigh = 0x23,
    BusDividerLow = 0x24,
    CurrentLimit = 0x28,
    OverCurrentThreshold = 0x29,
    OverCurrentDebounce = 0x2a,
    ResidualThreshold = 0x2b,
    ResidualDebounce = 0x2c,
    CellUnderVoltage = 0x2d,
    CellOverVoltage = 0x2e,
    BusFilterTimeConstant = 0x30,
    MosfetHotResistance = 0x31,
    ThermalResistance = 0x32,
    ThermalTimeConstant = 0x33,
    AmbientTemperature = 0x34,
    DerateTemperature = 0x35,
    ShutdownTemperature = 0x36,
    StallTargetVelocity = 0x38,
    StallMaxVelocity = 0x39,
    StallMinIq = 0x3a,
    StallTime = 0x3b,
    SensorFaultTime = 0x3c,
    MaximumVelocity = 0x3d,
    MaximumOpenLoopVelocity = 0x3e,
    TemperatureSensorZeroVoltage = 0x40,
    TemperatureSensorVoltsPerDegree = 0x41,
}

impl ParameterId {
    pub const ALL: [Self; PARAMETER_COUNT] = [
        Self::BatteryCells,
        Self::PolePairs,
        Self::SensorDirection,
        Self::EncoderCpr,
        Self::ElectricalZero,
        Self::CurrentKp,
        Self::CurrentKi,
        Self::CurrentKd,
        Self::CurrentOutputLimit,
        Self::CurrentOutputRamp,
        Self::VelocityKp,
        Self::VelocityKi,
        Self::VelocityKd,
        Self::VelocityOutputLimit,
        Self::VelocityOutputRamp,
        Self::AdcReferenceVoltage,
        Self::CurrentSenseGain,
        Self::ShuntResistance,
        Self::BusDividerHigh,
        Self::BusDividerLow,
        Self::CurrentLimit,
        Self::OverCurrentThreshold,
        Self::OverCurrentDebounce,
        Self::ResidualThreshold,
        Self::ResidualDebounce,
        Self::CellUnderVoltage,
        Self::CellOverVoltage,
        Self::BusFilterTimeConstant,
        Self::MosfetHotResistance,
        Self::ThermalResistance,
        Self::ThermalTimeConstant,
        Self::AmbientTemperature,
        Self::DerateTemperature,
        Self::ShutdownTemperature,
        Self::StallTargetVelocity,
        Self::StallMaxVelocity,
        Self::StallMinIq,
        Self::StallTime,
        Self::SensorFaultTime,
        Self::MaximumVelocity,
        Self::MaximumOpenLoopVelocity,
        Self::TemperatureSensorZeroVoltage,
        Self::TemperatureSensorVoltsPerDegree,
    ];

    pub const fn from_raw(raw: u8) -> Option<Self> {
        Some(match raw {
            | 0x01 => Self::BatteryCells,
            | 0x02 => Self::PolePairs,
            | 0x03 => Self::SensorDirection,
            | 0x04 => Self::EncoderCpr,
            | 0x05 => Self::ElectricalZero,
            | 0x10 => Self::CurrentKp,
            | 0x11 => Self::CurrentKi,
            | 0x12 => Self::CurrentKd,
            | 0x13 => Self::CurrentOutputLimit,
            | 0x14 => Self::CurrentOutputRamp,
            | 0x18 => Self::VelocityKp,
            | 0x19 => Self::VelocityKi,
            | 0x1a => Self::VelocityKd,
            | 0x1b => Self::VelocityOutputLimit,
            | 0x1c => Self::VelocityOutputRamp,
            | 0x20 => Self::AdcReferenceVoltage,
            | 0x21 => Self::CurrentSenseGain,
            | 0x22 => Self::ShuntResistance,
            | 0x23 => Self::BusDividerHigh,
            | 0x24 => Self::BusDividerLow,
            | 0x28 => Self::CurrentLimit,
            | 0x29 => Self::OverCurrentThreshold,
            | 0x2a => Self::OverCurrentDebounce,
            | 0x2b => Self::ResidualThreshold,
            | 0x2c => Self::ResidualDebounce,
            | 0x2d => Self::CellUnderVoltage,
            | 0x2e => Self::CellOverVoltage,
            | 0x30 => Self::BusFilterTimeConstant,
            | 0x31 => Self::MosfetHotResistance,
            | 0x32 => Self::ThermalResistance,
            | 0x33 => Self::ThermalTimeConstant,
            | 0x34 => Self::AmbientTemperature,
            | 0x35 => Self::DerateTemperature,
            | 0x36 => Self::ShutdownTemperature,
            | 0x38 => Self::StallTargetVelocity,
            | 0x39 => Self::StallMaxVelocity,
            | 0x3a => Self::StallMinIq,
            | 0x3b => Self::StallTime,
            | 0x3c => Self::SensorFaultTime,
            | 0x3d => Self::MaximumVelocity,
            | 0x3e => Self::MaximumOpenLoopVelocity,
            | 0x40 => Self::TemperatureSensorZeroVoltage,
            | 0x41 => Self::TemperatureSensorVoltsPerDegree,
            | _ => return None,
        })
    }

    pub const fn name(self) -> &'static str {
        match self {
            | Self::BatteryCells => "battery-cells",
            | Self::PolePairs => "pole-pairs",
            | Self::SensorDirection => "sensor-direction",
            | Self::EncoderCpr => "encoder-cpr",
            | Self::ElectricalZero => "electrical-zero",
            | Self::CurrentKp => "current-kp",
            | Self::CurrentKi => "current-ki",
            | Self::CurrentKd => "current-kd",
            | Self::CurrentOutputLimit => "current-output-limit",
            | Self::CurrentOutputRamp => "current-output-ramp",
            | Self::VelocityKp => "velocity-kp",
            | Self::VelocityKi => "velocity-ki",
            | Self::VelocityKd => "velocity-kd",
            | Self::VelocityOutputLimit => "velocity-output-limit",
            | Self::VelocityOutputRamp => "velocity-output-ramp",
            | Self::AdcReferenceVoltage => "adc-reference-voltage",
            | Self::CurrentSenseGain => "current-sense-gain",
            | Self::ShuntResistance => "shunt-resistance",
            | Self::BusDividerHigh => "bus-divider-high",
            | Self::BusDividerLow => "bus-divider-low",
            | Self::CurrentLimit => "current-limit",
            | Self::OverCurrentThreshold => "over-current-threshold",
            | Self::OverCurrentDebounce => "over-current-debounce",
            | Self::ResidualThreshold => "residual-threshold",
            | Self::ResidualDebounce => "residual-debounce",
            | Self::CellUnderVoltage => "cell-under-voltage",
            | Self::CellOverVoltage => "cell-over-voltage",
            | Self::BusFilterTimeConstant => "bus-filter-time-constant",
            | Self::MosfetHotResistance => "mosfet-hot-resistance",
            | Self::ThermalResistance => "thermal-resistance",
            | Self::ThermalTimeConstant => "thermal-time-constant",
            | Self::AmbientTemperature => "ambient-temperature",
            | Self::DerateTemperature => "derate-temperature",
            | Self::ShutdownTemperature => "shutdown-temperature",
            | Self::StallTargetVelocity => "stall-target-velocity",
            | Self::StallMaxVelocity => "stall-max-velocity",
            | Self::StallMinIq => "stall-min-iq",
            | Self::StallTime => "stall-time",
            | Self::SensorFaultTime => "sensor-fault-time",
            | Self::MaximumVelocity => "maximum-velocity",
            | Self::MaximumOpenLoopVelocity => "maximum-open-loop-velocity",
            | Self::TemperatureSensorZeroVoltage => {
                "temperature-sensor-zero-voltage"
            },
            | Self::TemperatureSensorVoltsPerDegree => {
                "temperature-sensor-volts-per-degree"
            },
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|id| id.name() == name)
    }

    pub const fn kind(self) -> ParameterKind {
        match self {
            | Self::BatteryCells | Self::PolePairs => ParameterKind::U8,
            | Self::SensorDirection => ParameterKind::I8,
            | Self::EncoderCpr
            | Self::OverCurrentDebounce
            | Self::ResidualDebounce => ParameterKind::U16,
            | _ => ParameterKind::F32,
        }
    }

    pub const fn startup_only(self) -> bool {
        matches!(self, Self::PolePairs | Self::EncoderCpr)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterError {
    WrongType,
    OutOfRange,
    InvalidRelationship,
    ReservedBytes,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParameterProfileV1 {
    pub battery_cells: u8,
    pub pole_pairs: u8,
    pub sensor_direction: i8,
    pub encoder_cpr: u16,
    pub electrical_zero: f32,
    pub current_pid: PidConfig,
    pub velocity_pid: PidConfig,
    pub adc_reference_voltage: f32,
    pub current_sense_gain: f32,
    pub shunt_resistance: f32,
    pub bus_divider_high: f32,
    pub bus_divider_low: f32,
    pub current_limit: f32,
    pub over_current_threshold: f32,
    pub over_current_debounce: u16,
    pub residual_threshold: f32,
    pub residual_debounce: u16,
    pub cell_under_voltage: f32,
    pub cell_over_voltage: f32,
    pub bus_filter_time_constant: f32,
    pub mosfet_hot_resistance: f32,
    pub thermal_resistance: f32,
    pub thermal_time_constant: f32,
    pub ambient_temperature: f32,
    pub derate_temperature: f32,
    pub shutdown_temperature: f32,
    pub stall_target_velocity: f32,
    pub stall_max_velocity: f32,
    pub stall_min_iq: f32,
    pub stall_time: f32,
    pub sensor_fault_time: f32,
    pub maximum_velocity: f32,
    pub maximum_open_loop_velocity: f32,
    pub temperature_sensor_zero_voltage: f32,
    pub temperature_sensor_volts_per_degree: f32,
}

impl Default for ParameterProfileV1 {
    fn default() -> Self {
        Self::from_config(&FocConfig::default(), DEFAULT_ENCODER_CPR)
    }
}

impl ParameterProfileV1 {
    pub fn from_config(config: &FocConfig, encoder_cpr: u16) -> Self {
        Self {
            battery_cells: config.battery_cells,
            pole_pairs: config.pole_pairs,
            sensor_direction: if config.sensor_direction < 0.0 {
                -1
            } else {
                1
            },
            encoder_cpr,
            electrical_zero: normalize_angle(config.electrical_zero),
            current_pid: canonical_pid(config.current_pid),
            velocity_pid: canonical_pid(config.velocity_pid),
            adc_reference_voltage: config.adc_reference_voltage,
            current_sense_gain: config.current_sense_gain,
            shunt_resistance: config.shunt_resistance,
            bus_divider_high: config.bus_divider_high,
            bus_divider_low: config.bus_divider_low,
            current_limit: config.current_limit,
            over_current_threshold: config.over_current_threshold,
            over_current_debounce: config.over_current_debounce,
            residual_threshold: config.residual_threshold,
            residual_debounce: config.residual_debounce,
            cell_under_voltage: config.cell_under_voltage,
            cell_over_voltage: config.cell_over_voltage,
            bus_filter_time_constant: config.bus_filter_time_constant,
            mosfet_hot_resistance: config.mosfet_hot_resistance,
            thermal_resistance: config.thermal_resistance,
            thermal_time_constant: config.thermal_time_constant,
            ambient_temperature: config.ambient_temperature,
            derate_temperature: config.derate_temperature,
            shutdown_temperature: config.shutdown_temperature,
            stall_target_velocity: config.stall_target_velocity,
            stall_max_velocity: config.stall_max_velocity,
            stall_min_iq: config.stall_min_iq,
            stall_time: config.stall_time,
            sensor_fault_time: config.sensor_fault_time,
            maximum_velocity: config.maximum_velocity,
            maximum_open_loop_velocity: config.maximum_open_loop_velocity,
            temperature_sensor_zero_voltage: config
                .temperature_sensor_zero_voltage,
            temperature_sensor_volts_per_degree: config
                .temperature_sensor_volts_per_degree,
        }
    }

    pub fn apply_boot(self, config: &mut FocConfig) {
        self.apply_live(config);
        config.pole_pairs = self.pole_pairs;
    }

    pub fn apply_live(self, config: &mut FocConfig) {
        config.battery_cells = self.battery_cells;
        config.sensor_direction = self.sensor_direction as f32;
        config.electrical_zero = self.electrical_zero;
        config.current_pid = self.current_pid;
        config.velocity_pid = self.velocity_pid;
        config.adc_reference_voltage = self.adc_reference_voltage;
        config.current_sense_gain = self.current_sense_gain;
        config.shunt_resistance = self.shunt_resistance;
        config.bus_divider_high = self.bus_divider_high;
        config.bus_divider_low = self.bus_divider_low;
        config.current_limit = self.current_limit;
        config.over_current_threshold = self.over_current_threshold;
        config.over_current_debounce = self.over_current_debounce;
        config.residual_threshold = self.residual_threshold;
        config.residual_debounce = self.residual_debounce;
        config.cell_under_voltage = self.cell_under_voltage;
        config.cell_over_voltage = self.cell_over_voltage;
        config.bus_filter_time_constant = self.bus_filter_time_constant;
        config.mosfet_hot_resistance = self.mosfet_hot_resistance;
        config.thermal_resistance = self.thermal_resistance;
        config.thermal_time_constant = self.thermal_time_constant;
        config.ambient_temperature = self.ambient_temperature;
        config.derate_temperature = self.derate_temperature;
        config.shutdown_temperature = self.shutdown_temperature;
        config.stall_target_velocity = self.stall_target_velocity;
        config.stall_max_velocity = self.stall_max_velocity;
        config.stall_min_iq = self.stall_min_iq;
        config.stall_time = self.stall_time;
        config.sensor_fault_time = self.sensor_fault_time;
        config.maximum_velocity = self.maximum_velocity;
        config.maximum_open_loop_velocity = self.maximum_open_loop_velocity;
        config.temperature_sensor_zero_voltage =
            self.temperature_sensor_zero_voltage;
        config.temperature_sensor_volts_per_degree =
            self.temperature_sensor_volts_per_degree;
    }

    pub fn validate(&self) -> Result<(), ParameterError> {
        if !(2..=6).contains(&self.battery_cells)
            || !(1..=64).contains(&self.pole_pairs)
            || !matches!(self.sensor_direction, -1 | 1)
            || self.encoder_cpr < 2
            || !finite_in(self.electrical_zero, 0.0, TAU)
            || self.electrical_zero >= TAU
            || !valid_pid(self.current_pid)
            || !valid_pid(self.velocity_pid)
            || !finite_in(self.adc_reference_voltage, 2.5, 3.6)
            || !finite_in(self.current_sense_gain, 1.0, 500.0)
            || !finite_in(self.shunt_resistance, 0.000_05, 0.1)
            || !finite_in(self.bus_divider_high, 100.0, 10_000_000.0)
            || !finite_in(self.bus_divider_low, 100.0, 10_000_000.0)
            || !finite_in(self.current_limit, 0.01, 50.0)
            || !finite_in(self.over_current_threshold, 0.01, 55.0)
            || !(1..=10_000).contains(&self.over_current_debounce)
            || !finite_in(self.residual_threshold, 0.0, 3.0)
            || !(1..=10_000).contains(&self.residual_debounce)
            || !finite_in(self.cell_under_voltage, 2.0, 3.5)
            || !finite_in(self.cell_over_voltage, 3.6, 4.25)
            || !finite_in(self.bus_filter_time_constant, 0.000_01, 1.0)
            || !finite_in(self.mosfet_hot_resistance, 0.000_01, 0.1)
            || !finite_in(self.thermal_resistance, 0.01, 100.0)
            || !finite_in(self.thermal_time_constant, 0.01, 1_000.0)
            || !finite_in(self.ambient_temperature, -40.0, 99.0)
            || !finite_in(self.derate_temperature, -39.0, 100.0)
            || !finite_in(self.shutdown_temperature, -38.0, 125.0)
            || !finite_in(self.stall_target_velocity, 0.01, 1_000.0)
            || !finite_in(self.stall_max_velocity, 0.0, 1_000.0)
            || !finite_in(self.stall_min_iq, 0.0, 50.0)
            || !finite_in(self.stall_time, 0.01, 10.0)
            || !finite_in(self.sensor_fault_time, 0.000_01, 1.0)
            || !finite_in(self.maximum_velocity, 0.01, 1_000.0)
            || !finite_in(self.maximum_open_loop_velocity, 0.01, 10_000.0)
            || !finite_in(self.temperature_sensor_zero_voltage, 0.0, 3.6)
            || !finite_in(
                self.temperature_sensor_volts_per_degree,
                0.000_1,
                0.1,
            )
        {
            return Err(ParameterError::OutOfRange);
        }

        let divider_ratio = (self.bus_divider_high + self.bus_divider_low)
            / self.bus_divider_low;
        if !(1.0..=100.0).contains(&divider_ratio)
            || self.current_limit > self.over_current_threshold
            || self.cell_under_voltage >= self.cell_over_voltage
            || self.ambient_temperature >= self.derate_temperature
            || self.derate_temperature >= self.shutdown_temperature
            || self.stall_max_velocity > self.stall_target_velocity
            || self.stall_target_velocity > self.maximum_velocity
            || self.stall_min_iq > self.current_limit
        {
            return Err(ParameterError::InvalidRelationship);
        }
        Ok(())
    }

    pub fn get(&self, id: ParameterId) -> ParameterValue {
        match id {
            | ParameterId::BatteryCells => {
                ParameterValue::U8(self.battery_cells)
            },
            | ParameterId::PolePairs => ParameterValue::U8(self.pole_pairs),
            | ParameterId::SensorDirection => {
                ParameterValue::I8(self.sensor_direction)
            },
            | ParameterId::EncoderCpr => ParameterValue::U16(self.encoder_cpr),
            | ParameterId::ElectricalZero => {
                ParameterValue::F32(self.electrical_zero)
            },
            | ParameterId::CurrentKp => {
                ParameterValue::F32(self.current_pid.kp)
            },
            | ParameterId::CurrentKi => {
                ParameterValue::F32(self.current_pid.ki)
            },
            | ParameterId::CurrentKd => {
                ParameterValue::F32(self.current_pid.kd)
            },
            | ParameterId::CurrentOutputLimit => {
                ParameterValue::F32(self.current_pid.output_limit)
            },
            | ParameterId::CurrentOutputRamp => {
                ParameterValue::F32(self.current_pid.output_ramp)
            },
            | ParameterId::VelocityKp => {
                ParameterValue::F32(self.velocity_pid.kp)
            },
            | ParameterId::VelocityKi => {
                ParameterValue::F32(self.velocity_pid.ki)
            },
            | ParameterId::VelocityKd => {
                ParameterValue::F32(self.velocity_pid.kd)
            },
            | ParameterId::VelocityOutputLimit => {
                ParameterValue::F32(self.velocity_pid.output_limit)
            },
            | ParameterId::VelocityOutputRamp => {
                ParameterValue::F32(self.velocity_pid.output_ramp)
            },
            | ParameterId::AdcReferenceVoltage => {
                ParameterValue::F32(self.adc_reference_voltage)
            },
            | ParameterId::CurrentSenseGain => {
                ParameterValue::F32(self.current_sense_gain)
            },
            | ParameterId::ShuntResistance => {
                ParameterValue::F32(self.shunt_resistance)
            },
            | ParameterId::BusDividerHigh => {
                ParameterValue::F32(self.bus_divider_high)
            },
            | ParameterId::BusDividerLow => {
                ParameterValue::F32(self.bus_divider_low)
            },
            | ParameterId::CurrentLimit => {
                ParameterValue::F32(self.current_limit)
            },
            | ParameterId::OverCurrentThreshold => {
                ParameterValue::F32(self.over_current_threshold)
            },
            | ParameterId::OverCurrentDebounce => {
                ParameterValue::U16(self.over_current_debounce)
            },
            | ParameterId::ResidualThreshold => {
                ParameterValue::F32(self.residual_threshold)
            },
            | ParameterId::ResidualDebounce => {
                ParameterValue::U16(self.residual_debounce)
            },
            | ParameterId::CellUnderVoltage => {
                ParameterValue::F32(self.cell_under_voltage)
            },
            | ParameterId::CellOverVoltage => {
                ParameterValue::F32(self.cell_over_voltage)
            },
            | ParameterId::BusFilterTimeConstant => {
                ParameterValue::F32(self.bus_filter_time_constant)
            },
            | ParameterId::MosfetHotResistance => {
                ParameterValue::F32(self.mosfet_hot_resistance)
            },
            | ParameterId::ThermalResistance => {
                ParameterValue::F32(self.thermal_resistance)
            },
            | ParameterId::ThermalTimeConstant => {
                ParameterValue::F32(self.thermal_time_constant)
            },
            | ParameterId::AmbientTemperature => {
                ParameterValue::F32(self.ambient_temperature)
            },
            | ParameterId::DerateTemperature => {
                ParameterValue::F32(self.derate_temperature)
            },
            | ParameterId::ShutdownTemperature => {
                ParameterValue::F32(self.shutdown_temperature)
            },
            | ParameterId::StallTargetVelocity => {
                ParameterValue::F32(self.stall_target_velocity)
            },
            | ParameterId::StallMaxVelocity => {
                ParameterValue::F32(self.stall_max_velocity)
            },
            | ParameterId::StallMinIq => ParameterValue::F32(self.stall_min_iq),
            | ParameterId::StallTime => ParameterValue::F32(self.stall_time),
            | ParameterId::SensorFaultTime => {
                ParameterValue::F32(self.sensor_fault_time)
            },
            | ParameterId::MaximumVelocity => {
                ParameterValue::F32(self.maximum_velocity)
            },
            | ParameterId::MaximumOpenLoopVelocity => {
                ParameterValue::F32(self.maximum_open_loop_velocity)
            },
            | ParameterId::TemperatureSensorZeroVoltage => {
                ParameterValue::F32(self.temperature_sensor_zero_voltage)
            },
            | ParameterId::TemperatureSensorVoltsPerDegree => {
                ParameterValue::F32(self.temperature_sensor_volts_per_degree)
            },
        }
    }

    pub fn set(
        &mut self,
        id: ParameterId,
        value: ParameterValue,
    ) -> Result<(), ParameterError> {
        let mut candidate = *self;
        match (id, value) {
            | (ParameterId::BatteryCells, ParameterValue::U8(value)) => {
                candidate.battery_cells = value
            },
            | (ParameterId::PolePairs, ParameterValue::U8(value)) => {
                candidate.pole_pairs = value
            },
            | (ParameterId::SensorDirection, ParameterValue::I8(value)) => {
                candidate.sensor_direction = value
            },
            | (ParameterId::EncoderCpr, ParameterValue::U16(value)) => {
                candidate.encoder_cpr = value
            },
            | (ParameterId::ElectricalZero, ParameterValue::F32(value)) => {
                if !value.is_finite() {
                    return Err(ParameterError::OutOfRange);
                }
                candidate.electrical_zero = normalize_angle(value);
            },
            | (ParameterId::CurrentKp, ParameterValue::F32(value)) => {
                candidate.current_pid.kp = value
            },
            | (ParameterId::CurrentKi, ParameterValue::F32(value)) => {
                candidate.current_pid.ki = value
            },
            | (ParameterId::CurrentKd, ParameterValue::F32(value)) => {
                candidate.current_pid.kd = value
            },
            | (ParameterId::CurrentOutputLimit, ParameterValue::F32(value)) => {
                candidate.current_pid.output_limit = value
            },
            | (ParameterId::CurrentOutputRamp, ParameterValue::F32(value)) => {
                candidate.current_pid.output_ramp = value
            },
            | (ParameterId::VelocityKp, ParameterValue::F32(value)) => {
                candidate.velocity_pid.kp = value
            },
            | (ParameterId::VelocityKi, ParameterValue::F32(value)) => {
                candidate.velocity_pid.ki = value
            },
            | (ParameterId::VelocityKd, ParameterValue::F32(value)) => {
                candidate.velocity_pid.kd = value
            },
            | (
                ParameterId::VelocityOutputLimit,
                ParameterValue::F32(value),
            ) => candidate.velocity_pid.output_limit = value,
            | (ParameterId::VelocityOutputRamp, ParameterValue::F32(value)) => {
                candidate.velocity_pid.output_ramp = value
            },
            | (
                ParameterId::AdcReferenceVoltage,
                ParameterValue::F32(value),
            ) => candidate.adc_reference_voltage = value,
            | (ParameterId::CurrentSenseGain, ParameterValue::F32(value)) => {
                candidate.current_sense_gain = value
            },
            | (ParameterId::ShuntResistance, ParameterValue::F32(value)) => {
                candidate.shunt_resistance = value
            },
            | (ParameterId::BusDividerHigh, ParameterValue::F32(value)) => {
                candidate.bus_divider_high = value
            },
            | (ParameterId::BusDividerLow, ParameterValue::F32(value)) => {
                candidate.bus_divider_low = value
            },
            | (ParameterId::CurrentLimit, ParameterValue::F32(value)) => {
                candidate.current_limit = value
            },
            | (
                ParameterId::OverCurrentThreshold,
                ParameterValue::F32(value),
            ) => candidate.over_current_threshold = value,
            | (
                ParameterId::OverCurrentDebounce,
                ParameterValue::U16(value),
            ) => candidate.over_current_debounce = value,
            | (ParameterId::ResidualThreshold, ParameterValue::F32(value)) => {
                candidate.residual_threshold = value
            },
            | (ParameterId::ResidualDebounce, ParameterValue::U16(value)) => {
                candidate.residual_debounce = value
            },
            | (ParameterId::CellUnderVoltage, ParameterValue::F32(value)) => {
                candidate.cell_under_voltage = value
            },
            | (ParameterId::CellOverVoltage, ParameterValue::F32(value)) => {
                candidate.cell_over_voltage = value
            },
            | (
                ParameterId::BusFilterTimeConstant,
                ParameterValue::F32(value),
            ) => candidate.bus_filter_time_constant = value,
            | (
                ParameterId::MosfetHotResistance,
                ParameterValue::F32(value),
            ) => candidate.mosfet_hot_resistance = value,
            | (ParameterId::ThermalResistance, ParameterValue::F32(value)) => {
                candidate.thermal_resistance = value
            },
            | (
                ParameterId::ThermalTimeConstant,
                ParameterValue::F32(value),
            ) => candidate.thermal_time_constant = value,
            | (ParameterId::AmbientTemperature, ParameterValue::F32(value)) => {
                candidate.ambient_temperature = value
            },
            | (ParameterId::DerateTemperature, ParameterValue::F32(value)) => {
                candidate.derate_temperature = value
            },
            | (
                ParameterId::ShutdownTemperature,
                ParameterValue::F32(value),
            ) => candidate.shutdown_temperature = value,
            | (
                ParameterId::StallTargetVelocity,
                ParameterValue::F32(value),
            ) => candidate.stall_target_velocity = value,
            | (ParameterId::StallMaxVelocity, ParameterValue::F32(value)) => {
                candidate.stall_max_velocity = value
            },
            | (ParameterId::StallMinIq, ParameterValue::F32(value)) => {
                candidate.stall_min_iq = value
            },
            | (ParameterId::StallTime, ParameterValue::F32(value)) => {
                candidate.stall_time = value
            },
            | (ParameterId::SensorFaultTime, ParameterValue::F32(value)) => {
                candidate.sensor_fault_time = value
            },
            | (ParameterId::MaximumVelocity, ParameterValue::F32(value)) => {
                candidate.maximum_velocity = value
            },
            | (
                ParameterId::MaximumOpenLoopVelocity,
                ParameterValue::F32(value),
            ) => candidate.maximum_open_loop_velocity = value,
            | (
                ParameterId::TemperatureSensorZeroVoltage,
                ParameterValue::F32(value),
            ) => candidate.temperature_sensor_zero_voltage = value,
            | (
                ParameterId::TemperatureSensorVoltsPerDegree,
                ParameterValue::F32(value),
            ) => candidate.temperature_sensor_volts_per_degree = value,
            | _ => return Err(ParameterError::WrongType),
        }
        candidate.validate()?;
        *self = candidate;
        Ok(())
    }

    pub fn encode_payload(&self) -> [u8; PARAMETER_PAYLOAD_SIZE] {
        let mut data = [0u8; PARAMETER_PAYLOAD_SIZE];
        data[0] = self.battery_cells;
        data[1] = self.pole_pairs;
        data[2] = self.sensor_direction as u8;
        data[4..6].copy_from_slice(&self.encoder_cpr.to_le_bytes());
        data[6..8].copy_from_slice(&self.over_current_debounce.to_le_bytes());
        data[8..10].copy_from_slice(&self.residual_debounce.to_le_bytes());

        let values = self.float_values();
        for (index, value) in values.into_iter().enumerate() {
            let offset = 12 + index * 4;
            data[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        data
    }

    pub fn decode_payload(
        data: &[u8; PARAMETER_PAYLOAD_SIZE],
    ) -> Result<Self, ParameterError> {
        if data[3] != 0 || data[10] != 0 || data[11] != 0 {
            return Err(ParameterError::ReservedBytes);
        }
        let mut offset = 12;
        let mut next = || {
            let bytes: [u8; 4] = data[offset..offset + 4].try_into().unwrap();
            offset += 4;
            f32::from_le_bytes(bytes)
        };
        let profile = Self {
            battery_cells: data[0],
            pole_pairs: data[1],
            sensor_direction: data[2] as i8,
            encoder_cpr: u16::from_le_bytes([data[4], data[5]]),
            over_current_debounce: u16::from_le_bytes([data[6], data[7]]),
            residual_debounce: u16::from_le_bytes([data[8], data[9]]),
            electrical_zero: next(),
            current_pid: PidConfig::new(next(), next(), next(), next(), next()),
            velocity_pid: PidConfig::new(
                next(),
                next(),
                next(),
                next(),
                next(),
            ),
            adc_reference_voltage: next(),
            current_sense_gain: next(),
            shunt_resistance: next(),
            bus_divider_high: next(),
            bus_divider_low: next(),
            current_limit: next(),
            over_current_threshold: next(),
            residual_threshold: next(),
            cell_under_voltage: next(),
            cell_over_voltage: next(),
            bus_filter_time_constant: next(),
            mosfet_hot_resistance: next(),
            thermal_resistance: next(),
            thermal_time_constant: next(),
            ambient_temperature: next(),
            derate_temperature: next(),
            shutdown_temperature: next(),
            stall_target_velocity: next(),
            stall_max_velocity: next(),
            stall_min_iq: next(),
            stall_time: next(),
            sensor_fault_time: next(),
            maximum_velocity: next(),
            maximum_open_loop_velocity: next(),
            temperature_sensor_zero_voltage: next(),
            temperature_sensor_volts_per_degree: next(),
        };
        profile.validate()?;
        Ok(profile)
    }

    fn float_values(&self) -> [f32; 37] {
        [
            self.electrical_zero,
            self.current_pid.kp,
            self.current_pid.ki,
            self.current_pid.kd,
            self.current_pid.output_limit,
            self.current_pid.output_ramp,
            self.velocity_pid.kp,
            self.velocity_pid.ki,
            self.velocity_pid.kd,
            self.velocity_pid.output_limit,
            self.velocity_pid.output_ramp,
            self.adc_reference_voltage,
            self.current_sense_gain,
            self.shunt_resistance,
            self.bus_divider_high,
            self.bus_divider_low,
            self.current_limit,
            self.over_current_threshold,
            self.residual_threshold,
            self.cell_under_voltage,
            self.cell_over_voltage,
            self.bus_filter_time_constant,
            self.mosfet_hot_resistance,
            self.thermal_resistance,
            self.thermal_time_constant,
            self.ambient_temperature,
            self.derate_temperature,
            self.shutdown_temperature,
            self.stall_target_velocity,
            self.stall_max_velocity,
            self.stall_min_iq,
            self.stall_time,
            self.sensor_fault_time,
            self.maximum_velocity,
            self.maximum_open_loop_velocity,
            self.temperature_sensor_zero_voltage,
            self.temperature_sensor_volts_per_degree,
        ]
    }
}

fn canonical_pid(mut config: PidConfig) -> PidConfig {
    if !config.output_ramp.is_finite() || config.output_ramp <= 0.0 {
        config.output_ramp = 0.0;
    }
    config
}

fn valid_pid(config: PidConfig) -> bool {
    [
        config.kp,
        config.ki,
        config.kd,
        config.output_limit,
        config.output_ramp,
    ]
    .into_iter()
    .all(|value| value.is_finite() && (0.0..=1_000_000.0).contains(&value))
        && config.output_limit > 0.0
}

fn finite_in(value: f32, minimum: f32, maximum: f32) -> bool {
    value.is_finite() && value >= minimum && value <= maximum
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_profile_is_valid_and_canonical() {
        let profile = ParameterProfileV1::default();
        assert_eq!(profile.current_pid.output_ramp, 0.0);
        assert_eq!(profile.validate(), Ok(()));
        assert_eq!(profile.encode_payload().len(), PARAMETER_PAYLOAD_SIZE);
    }

    #[test]
    fn payload_round_trip_is_exact() {
        let profile = ParameterProfileV1::default();
        let encoded = profile.encode_payload();
        assert_eq!(ParameterProfileV1::decode_payload(&encoded), Ok(profile));
    }

    #[test]
    fn setting_is_transactional_and_normalizes_angle() {
        let mut profile = ParameterProfileV1::default();
        profile
            .set(ParameterId::ElectricalZero, ParameterValue::F32(-0.5))
            .unwrap();
        assert!(profile.electrical_zero >= 0.0);
        assert!(profile.electrical_zero < TAU);

        let before = profile;
        assert_eq!(
            profile.set(ParameterId::BatteryCells, ParameterValue::U8(1)),
            Err(ParameterError::OutOfRange)
        );
        assert_eq!(profile, before);
    }

    #[test]
    fn rejects_wrong_types_nonfinite_values_and_bad_relationships() {
        let mut profile = ParameterProfileV1::default();
        assert_eq!(
            profile.set(ParameterId::BatteryCells, ParameterValue::F32(6.0)),
            Err(ParameterError::WrongType)
        );
        assert_eq!(
            profile.set(ParameterId::CurrentKp, ParameterValue::F32(f32::NAN)),
            Err(ParameterError::OutOfRange)
        );
        assert_eq!(
            profile.set(ParameterId::CurrentLimit, ParameterValue::F32(56.0)),
            Err(ParameterError::OutOfRange)
        );
        assert_eq!(
            profile
                .set(ParameterId::CellUnderVoltage, ParameterValue::F32(4.0)),
            Err(ParameterError::OutOfRange)
        );
    }

    #[test]
    fn ids_and_names_are_unique_and_complete() {
        for (index, id) in ParameterId::ALL.into_iter().enumerate() {
            assert_eq!(ParameterId::from_raw(id as u8), Some(id));
            assert_eq!(ParameterId::from_name(id.name()), Some(id));
            for other in ParameterId::ALL.into_iter().skip(index + 1) {
                assert_ne!(id as u8, other as u8);
                assert_ne!(id.name(), other.name());
            }
        }
    }

    #[test]
    fn apply_boot_restores_persisted_values_including_pole_pairs() {
        let mut profile = ParameterProfileV1::default();
        profile
            .set(ParameterId::PolePairs, ParameterValue::U8(7))
            .unwrap();
        profile
            .set(ParameterId::CurrentKi, ParameterValue::F32(15.0))
            .unwrap();

        let mut config = FocConfig::default();
        profile.apply_boot(&mut config);
        assert_eq!(config.pole_pairs, 7);
        assert_eq!(config.current_pid.ki, 15.0);

        config.pole_pairs = 2;
        profile.apply_live(&mut config);
        assert_eq!(config.pole_pairs, 2);
        assert_eq!(config.current_pid.ki, 15.0);
    }

    #[test]
    fn from_config_captures_runtime_config_changes() {
        let base = FocConfig::default();
        let config = FocConfig {
            battery_cells: 3,
            sensor_direction: -1.0,
            current_pid: PidConfig {
                kp: 0.9,
                ..base.current_pid
            },
            ..base
        };
        let profile = ParameterProfileV1::from_config(&config, 8_192);
        assert_eq!(profile.battery_cells, 3);
        assert_eq!(profile.encoder_cpr, 8_192);
        assert_eq!(profile.sensor_direction, -1);
        assert_eq!(profile.current_pid.kp, 0.9);
        assert_eq!(profile.validate(), Ok(()));
    }
}
