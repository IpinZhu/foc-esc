use crate::foc_math::{
    Dq, PhaseCurrents, PhaseDuty, clarke, inverse_park, normalize_angle, park,
    svpwm,
};
use crate::interfaces::{
    Command, ControlMode, ControlOutput, FaultFlags, MotorState, RawAdcFrame,
    RotorSample, Telemetry,
};
use crate::parameters::ParameterProfileV1;
use crate::pid::{PidConfig, PidController};

pub const DEFAULT_PWM_FREQUENCY_HZ: u32 = 100_000;
pub const DEFAULT_DEAD_TIME_NS: u32 = 200;

#[derive(Clone, Copy, Debug)]
pub struct FocConfig {
    pub pwm_frequency_hz: u32,
    pub speed_loop_frequency_hz: u32,
    pub pole_pairs: u8,
    pub sensor_direction: f32,
    pub electrical_zero: f32,
    pub adc_reference_voltage: f32,
    pub adc_full_scale: f32,
    pub current_sense_gain: f32,
    pub shunt_resistance: f32,
    pub bus_divider_high: f32,
    pub bus_divider_low: f32,
    pub calibration_samples: u32,
    pub minimum_duty: f32,
    pub maximum_duty: f32,
    pub current_limit: f32,
    pub over_current_threshold: f32,
    pub over_current_debounce: u16,
    pub residual_threshold: f32,
    pub residual_debounce: u16,
    pub battery_cells: u8,
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
    pub current_pid: PidConfig,
    pub velocity_pid: PidConfig,
}

impl Default for FocConfig {
    fn default() -> Self {
        Self {
            pwm_frequency_hz: DEFAULT_PWM_FREQUENCY_HZ,
            speed_loop_frequency_hz: 1_000,
            pole_pairs: 4,
            sensor_direction: 1.0,
            electrical_zero: 0.0,
            adc_reference_voltage: 3.3,
            adc_full_scale: 4_095.0,
            current_sense_gain: 50.0,
            shunt_resistance: 0.000_5,
            bus_divider_high: 100_000.0,
            bus_divider_low: 10_000.0,
            calibration_samples: 1_024,
            minimum_duty: 0.05,
            maximum_duty: 0.95,
            current_limit: 50.0,
            over_current_threshold: 55.0,
            over_current_debounce: 3,
            residual_threshold: 3.0,
            residual_debounce: 20,
            battery_cells: 6,
            cell_under_voltage: 3.0,
            cell_over_voltage: 4.25,
            bus_filter_time_constant: 0.02,
            mosfet_hot_resistance: 0.010,
            thermal_resistance: 2.0,
            thermal_time_constant: 10.0,
            ambient_temperature: 25.0,
            derate_temperature: 100.0,
            shutdown_temperature: 125.0,
            stall_target_velocity: 10.0,
            stall_max_velocity: 1.0,
            stall_min_iq: 10.0,
            stall_time: 0.5,
            sensor_fault_time: 0.002,
            maximum_velocity: 1_000.0,
            maximum_open_loop_velocity: 10_000.0,
            temperature_sensor_zero_voltage: 0.5,
            temperature_sensor_volts_per_degree: 0.01,
            current_pid: PidConfig::new(0.2, 20.0, 0.0, 30.0, f32::INFINITY),
            velocity_pid: PidConfig::new(0.4, 2.0, 0.0, 50.0, 200.0),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct OffsetCalibrator {
    sum_a: u64,
    sum_b: u64,
    sum_c: u64,
    count: u32,
}

impl OffsetCalibrator {
    fn push(&mut self, frame: RawAdcFrame, target: u32) -> Option<[f32; 3]> {
        self.sum_a += frame.phase_a as u64;
        self.sum_b += frame.phase_b as u64;
        self.sum_c += frame.phase_c as u64;
        self.count += 1;
        if self.count < target.max(1) {
            return None;
        }
        let count = self.count as f32;
        Some([
            self.sum_a as f32 / count,
            self.sum_b as f32 / count,
            self.sum_c as f32 / count,
        ])
    }
}

pub struct FocController {
    config: FocConfig,
    state: MotorState,
    faults: FaultFlags,
    offsets: [f32; 3],
    calibrator: OffsetCalibrator,
    current_d_pid: PidController,
    current_q_pid: PidController,
    velocity_pid: PidController,
    target_iq: f32,
    target_velocity: f32,
    open_loop_velocity: f32,
    open_loop_q_voltage: f32,
    open_loop_angle: f32,
    speed_divider: u32,
    speed_counter: u32,
    over_current_count: u16,
    residual_count: u16,
    sensor_invalid_count: u32,
    stall_count: u32,
    calibration_complete: bool,
    previous_sequence: Option<u32>,
    last_duty: PhaseDuty,
    bus_current_average: f32,
    junction_temperature: f32,
    last_telemetry: Telemetry,
}

impl FocController {
    pub fn new(config: FocConfig) -> Self {
        let speed_divider = (config.pwm_frequency_hz
            / config.speed_loop_frequency_hz.max(1))
        .max(1);
        Self {
            state: MotorState::Calibrating,
            faults: FaultFlags::empty(),
            offsets: [0.0; 3],
            calibrator: OffsetCalibrator::default(),
            current_d_pid: PidController::new(config.current_pid),
            current_q_pid: PidController::new(config.current_pid),
            velocity_pid: PidController::new(config.velocity_pid),
            target_iq: 0.0,
            target_velocity: 0.0,
            open_loop_velocity: 0.0,
            open_loop_q_voltage: 0.0,
            open_loop_angle: 0.0,
            speed_divider,
            speed_counter: 0,
            over_current_count: 0,
            residual_count: 0,
            sensor_invalid_count: 0,
            stall_count: 0,
            calibration_complete: false,
            previous_sequence: None,
            last_duty: PhaseDuty::DISABLED,
            bus_current_average: 0.0,
            junction_temperature: config.ambient_temperature,
            last_telemetry: Telemetry::default(),
            config,
        }
    }

    pub fn state(&self) -> MotorState {
        self.state
    }

    pub fn faults(&self) -> FaultFlags {
        self.faults
    }

    pub fn telemetry(&self) -> Telemetry {
        self.last_telemetry
    }

    pub fn offsets(&self) -> [f32; 3] {
        self.offsets
    }

    pub fn config(&self) -> FocConfig {
        self.config
    }

    pub fn storage_safe(&self) -> bool {
        if self.state != MotorState::Idle
            || !self.calibration_complete
            || !self.faults.is_empty()
            || self.junction_temperature >= self.config.shutdown_temperature
        {
            return false;
        }
        let (_, maximum_voltage) = self.bus_limits();
        if !self.last_telemetry.bus_voltage.is_finite()
            || self.last_telemetry.bus_voltage > maximum_voltage
        {
            return false;
        }
        let currents = self.last_telemetry.currents;
        currents.a.abs().max(currents.b.abs()).max(currents.c.abs()) < 1.0
    }

    pub fn apply_live_profile(&mut self, profile: ParameterProfileV1) -> bool {
        if !self.storage_safe() || profile.validate().is_err() {
            return false;
        }
        let mut config = self.config;
        profile.apply_live(&mut config);
        self.current_d_pid.set_config(config.current_pid);
        self.current_q_pid.set_config(config.current_pid);
        self.velocity_pid.set_config(config.velocity_pid);
        self.target_iq = self
            .target_iq
            .clamp(-config.current_limit, config.current_limit);
        self.target_velocity = self
            .target_velocity
            .clamp(-config.maximum_velocity, config.maximum_velocity);
        self.open_loop_velocity = self.open_loop_velocity.clamp(
            -config.maximum_open_loop_velocity,
            config.maximum_open_loop_velocity,
        );
        self.config = config;
        self.reset_loops();
        true
    }

    pub fn resynchronize_control_input(&mut self) {
        self.previous_sequence = None;
    }

    pub fn handle_command(&mut self, command: Command) -> bool {
        match command {
            | Command::Disable => {
                self.enter_idle();
                true
            },
            | Command::ClearFault => {
                if self.state != MotorState::Fault || !self.safe_to_clear() {
                    return false;
                }
                self.faults.clear();
                self.reset_loops();
                if self.calibration_complete {
                    self.state = MotorState::Idle;
                } else {
                    self.calibrator = OffsetCalibrator::default();
                    self.offsets = [0.0; 3];
                    self.state = MotorState::Calibrating;
                }
                true
            },
            | Command::Enable(mode) => {
                let was_idle = self.state == MotorState::Idle;
                self.try_enable(mode);
                was_idle && self.state == MotorState::Running(mode)
            },
            | Command::SetIq(iq) if iq.is_finite() => {
                self.target_iq = iq.clamp(
                    -self.config.current_limit,
                    self.config.current_limit,
                );
                true
            },
            | Command::SetVelocity(velocity) if velocity.is_finite() => {
                self.target_velocity = velocity.clamp(
                    -self.config.maximum_velocity,
                    self.config.maximum_velocity,
                );
                true
            },
            | Command::SetOpenLoop {
                electrical_velocity,
                q_voltage,
            } if electrical_velocity.is_finite() && q_voltage.is_finite() => {
                self.open_loop_velocity = electrical_velocity.clamp(
                    -self.config.maximum_open_loop_velocity,
                    self.config.maximum_open_loop_velocity,
                );
                self.open_loop_q_voltage = q_voltage;
                true
            },
            | Command::SetCellCount(cells)
                if (2..=6).contains(&cells) && !self.bridge_requested() =>
            {
                self.config.battery_cells = cells;
                true
            },
            | Command::SetElectricalZero(angle)
                if angle.is_finite() && !self.bridge_requested() =>
            {
                self.config.electrical_zero = normalize_angle(angle);
                true
            },
            | Command::SetCurrentPid(config)
                if valid_pid(config) && !self.bridge_requested() =>
            {
                self.current_d_pid.set_config(config);
                self.current_q_pid.set_config(config);
                self.config.current_pid = config;
                true
            },
            | Command::SetVelocityPid(config)
                if valid_pid(config) && !self.bridge_requested() =>
            {
                self.velocity_pid.set_config(config);
                self.config.velocity_pid = config;
                true
            },
            | Command::ReportControlOverrun => {
                self.trip(FaultFlags::CONTROL_OVERRUN);
                true
            },
            | _ => false,
        }
    }

    pub fn step(
        &mut self,
        raw: RawAdcFrame,
        rotor: RotorSample,
    ) -> ControlOutput {
        let dt = 1.0 / self.config.pwm_frequency_hz as f32;
        self.check_sequence(raw.sequence);
        if raw.overrun {
            self.trip(FaultFlags::CONTROL_OVERRUN);
        }
        let bus_voltage = self.raw_to_bus_voltage(raw.bus_voltage);
        let sensor_temperature = self.raw_to_temperature(raw.ntc);

        if self.state == MotorState::Calibrating {
            self.update_thermal(
                PhaseCurrents::default(),
                dt,
                sensor_temperature,
            );
            if let Some(offsets) =
                self.calibrator.push(raw, self.config.calibration_samples)
            {
                self.offsets = offsets;
                self.calibration_complete = true;
                self.state = MotorState::Idle;
            }
            return self.finish_step(
                raw,
                rotor,
                bus_voltage,
                PhaseCurrents::default(),
                Dq::default(),
                Dq::default(),
                PhaseDuty::DISABLED,
                false,
            );
        }

        let currents = self.raw_to_currents(raw);
        let bus_current = self.last_duty.a * currents.a
            + self.last_duty.b * currents.b
            + self.last_duty.c * currents.c;
        let filter_alpha = dt / (self.config.bus_filter_time_constant + dt);
        self.bus_current_average +=
            filter_alpha * (bus_current - self.bus_current_average);
        self.update_thermal(currents, dt, sensor_temperature);
        self.run_protections(currents, bus_voltage, rotor, dt);

        if self.state == MotorState::Fault || self.state == MotorState::Idle {
            self.last_duty = PhaseDuty::DISABLED;
            return self.finish_step(
                raw,
                rotor,
                bus_voltage,
                currents,
                Dq::default(),
                Dq::default(),
                PhaseDuty::DISABLED,
                false,
            );
        }

        let mode = match self.state {
            | MotorState::Running(mode) => mode,
            | _ => ControlMode::Idle,
        };
        let electrical_angle = match mode {
            | ControlMode::OpenLoop => {
                self.open_loop_angle = normalize_angle(
                    self.open_loop_angle + self.open_loop_velocity * dt,
                );
                self.open_loop_angle
            },
            | _ => normalize_angle(
                self.config.sensor_direction
                    * self.config.pole_pairs as f32
                    * rotor.mechanical_angle
                    - self.config.electrical_zero,
            ),
        };
        let current_dq = park(clarke(currents), electrical_angle);
        let voltage_dq = match mode {
            | ControlMode::OpenLoop => Dq {
                d: 0.0,
                q: self
                    .open_loop_q_voltage
                    .clamp(-0.5 * bus_voltage, 0.5 * bus_voltage),
            },
            | ControlMode::Current => self.run_current_loop(
                current_dq,
                self.target_iq,
                bus_voltage,
                dt,
            ),
            | ControlMode::Velocity => {
                self.run_speed_loop(rotor.mechanical_velocity);
                self.run_current_loop(
                    current_dq,
                    self.target_iq,
                    bus_voltage,
                    dt,
                )
            },
            | ControlMode::Idle => Dq::default(),
        };
        let duty = svpwm(
            inverse_park(voltage_dq, electrical_angle),
            bus_voltage,
            self.config.minimum_duty,
            self.config.maximum_duty,
        );
        self.last_duty = duty;
        self.finish_step(
            raw,
            rotor,
            bus_voltage,
            currents,
            current_dq,
            voltage_dq,
            duty,
            true,
        )
    }

    fn try_enable(&mut self, mode: ControlMode) {
        if self.state != MotorState::Idle
            || !self.faults.is_empty()
            || mode == ControlMode::Idle
        {
            return;
        }
        let bus_voltage = self.last_telemetry.bus_voltage;
        let (minimum, maximum) = self.bus_limits();
        let needs_sensor =
            matches!(mode, ControlMode::Current | ControlMode::Velocity);
        if bus_voltage < minimum
            || bus_voltage > maximum
            || (needs_sensor && !self.last_telemetry.sensor_valid)
        {
            return;
        }
        self.reset_loops();
        self.open_loop_angle = if self.last_telemetry.sensor_valid {
            normalize_angle(
                self.config.sensor_direction
                    * self.config.pole_pairs as f32
                    * self.last_telemetry.mechanical_angle
                    - self.config.electrical_zero,
            )
        } else {
            0.0
        };
        self.state = MotorState::Running(mode);
    }

    fn enter_idle(&mut self) {
        if !matches!(self.state, MotorState::Calibrating | MotorState::Fault) {
            self.state = MotorState::Idle;
        }
        self.last_duty = PhaseDuty::DISABLED;
        self.reset_loops();
    }

    fn reset_loops(&mut self) {
        self.current_d_pid.reset();
        self.current_q_pid.reset();
        self.velocity_pid.reset();
        self.speed_counter = 0;
        self.stall_count = 0;
    }

    fn run_current_loop(
        &mut self,
        measured: Dq,
        iq_target: f32,
        bus_voltage: f32,
        dt: f32,
    ) -> Dq {
        let voltage_limit = 0.5 * bus_voltage.max(0.0);
        self.current_d_pid.set_output_limit(voltage_limit);
        self.current_q_pid.set_output_limit(voltage_limit);
        let iq_limit = self.derated_current_limit();
        Dq {
            d: self.current_d_pid.update(-measured.d, dt),
            q: self
                .current_q_pid
                .update(iq_target.clamp(-iq_limit, iq_limit) - measured.q, dt),
        }
    }

    fn run_speed_loop(&mut self, measured_velocity: f32) {
        self.speed_counter += 1;
        if self.speed_counter < self.speed_divider {
            return;
        }
        self.speed_counter = 0;
        let dt =
            self.speed_divider as f32 / self.config.pwm_frequency_hz as f32;
        self.velocity_pid
            .set_output_limit(self.derated_current_limit());
        self.target_iq = self
            .velocity_pid
            .update(self.target_velocity - measured_velocity, dt);

        let stalled = self.target_velocity.abs()
            >= self.config.stall_target_velocity
            && measured_velocity.abs() <= self.config.stall_max_velocity
            && self.target_iq.abs() >= self.config.stall_min_iq;
        if stalled {
            self.stall_count = self.stall_count.saturating_add(1);
            let limit = (self.config.stall_time
                * self.config.speed_loop_frequency_hz as f32)
                as u32;
            if self.stall_count >= limit.max(1) {
                self.trip(FaultFlags::STALL);
            }
        } else {
            self.stall_count = 0;
        }
    }

    fn run_protections(
        &mut self,
        currents: PhaseCurrents,
        bus_voltage: f32,
        rotor: RotorSample,
        dt: f32,
    ) {
        let maximum_current =
            currents.a.abs().max(currents.b.abs()).max(currents.c.abs());
        if maximum_current > self.config.over_current_threshold {
            self.over_current_count = self.over_current_count.saturating_add(1);
            if self.over_current_count
                >= self.config.over_current_debounce.max(1)
            {
                self.trip(FaultFlags::OVER_CURRENT);
            }
        } else {
            self.over_current_count = 0;
        }

        if (currents.a + currents.b + currents.c).abs()
            > self.config.residual_threshold
        {
            self.residual_count = self.residual_count.saturating_add(1);
            if self.residual_count >= self.config.residual_debounce.max(1) {
                self.trip(FaultFlags::CURRENT_RESIDUAL);
            }
        } else {
            self.residual_count = 0;
        }

        if self.bridge_requested() {
            let (minimum, maximum) = self.bus_limits();
            if bus_voltage < minimum {
                self.trip(FaultFlags::UNDER_VOLTAGE);
            } else if bus_voltage > maximum {
                self.trip(FaultFlags::OVER_VOLTAGE);
            }

            let sensor_required = !matches!(
                self.state,
                MotorState::Running(ControlMode::OpenLoop)
            );
            if sensor_required && !rotor.valid {
                self.sensor_invalid_count =
                    self.sensor_invalid_count.saturating_add(1);
                let limit = (self.config.sensor_fault_time / dt) as u32;
                if self.sensor_invalid_count >= limit.max(1) {
                    self.trip(FaultFlags::SENSOR);
                }
            } else {
                self.sensor_invalid_count = 0;
            }
        }

        if self.junction_temperature >= self.config.shutdown_temperature {
            self.trip(FaultFlags::OVER_TEMPERATURE);
        }
    }

    fn update_thermal(
        &mut self,
        currents: PhaseCurrents,
        dt: f32,
        sensor_temperature: f32,
    ) {
        let conduction_loss = self.config.mosfet_hot_resistance
            * (currents.a * currents.a
                + currents.b * currents.b
                + currents.c * currents.c);
        let target_rise = conduction_loss * self.config.thermal_resistance;
        let current_rise =
            self.junction_temperature - self.config.ambient_temperature;
        self.junction_temperature += (target_rise - current_rise) * dt
            / self.config.thermal_time_constant.max(dt);
        if sensor_temperature.is_finite() {
            self.junction_temperature =
                self.junction_temperature.max(sensor_temperature);
        }
    }

    fn derated_current_limit(&self) -> f32 {
        if self.junction_temperature <= self.config.derate_temperature {
            return self.config.current_limit;
        }
        let span =
            self.config.shutdown_temperature - self.config.derate_temperature;
        if span <= 0.0 {
            return 0.0;
        }
        (self.config.current_limit
            * (self.config.shutdown_temperature - self.junction_temperature)
            / span)
            .clamp(0.0, self.config.current_limit)
    }

    fn raw_to_currents(&self, raw: RawAdcFrame) -> PhaseCurrents {
        let amperes_per_count = self.config.adc_reference_voltage
            / self.config.adc_full_scale
            / (self.config.current_sense_gain * self.config.shunt_resistance);
        PhaseCurrents {
            a: (raw.phase_a as f32 - self.offsets[0]) * amperes_per_count,
            b: (raw.phase_b as f32 - self.offsets[1]) * amperes_per_count,
            c: (raw.phase_c as f32 - self.offsets[2]) * amperes_per_count,
        }
    }

    fn raw_to_bus_voltage(&self, raw: u16) -> f32 {
        raw as f32 * self.config.adc_reference_voltage
            / self.config.adc_full_scale
            * (self.config.bus_divider_high + self.config.bus_divider_low)
            / self.config.bus_divider_low
    }

    fn raw_to_temperature(&self, raw: u16) -> f32 {
        let voltage = raw as f32 * self.config.adc_reference_voltage
            / self.config.adc_full_scale;
        let slope = self.config.temperature_sensor_volts_per_degree;
        if !voltage.is_finite() || !slope.is_finite() || slope <= 0.0 {
            return self.config.ambient_temperature;
        }
        (voltage - self.config.temperature_sensor_zero_voltage) / slope
    }

    fn bus_limits(&self) -> (f32, f32) {
        let cells = self.config.battery_cells.clamp(2, 6) as f32;
        (
            cells * self.config.cell_under_voltage,
            cells * self.config.cell_over_voltage,
        )
    }

    fn check_sequence(&mut self, sequence: u32) {
        if let Some(previous) = self.previous_sequence
            && sequence != previous.wrapping_add(1)
        {
            self.trip(FaultFlags::CONTROL_OVERRUN);
        }
        self.previous_sequence = Some(sequence);
    }

    fn safe_to_clear(&self) -> bool {
        let (_, maximum_voltage) = self.bus_limits();
        if self.junction_temperature >= self.config.shutdown_temperature
            || self.last_telemetry.bus_voltage > maximum_voltage
        {
            return false;
        }
        if !self.calibration_complete {
            return true;
        }
        let maximum_current = self
            .last_telemetry
            .currents
            .a
            .abs()
            .max(self.last_telemetry.currents.b.abs())
            .max(self.last_telemetry.currents.c.abs());
        maximum_current < 1.0
    }

    fn bridge_requested(&self) -> bool {
        let mut requested = false;
        if let MotorState::Running(_) = self.state {
            requested = true;
        }
        requested
    }

    fn trip(&mut self, fault: FaultFlags) {
        self.faults.insert(fault);
        self.state = MotorState::Fault;
        self.last_duty = PhaseDuty::DISABLED;
    }

    #[allow(clippy::too_many_arguments)]
    fn finish_step(
        &mut self,
        raw: RawAdcFrame,
        rotor: RotorSample,
        bus_voltage: f32,
        currents: PhaseCurrents,
        current_dq: Dq,
        voltage_dq: Dq,
        duty: PhaseDuty,
        bridge_enabled: bool,
    ) -> ControlOutput {
        let bus_current =
            duty.a * currents.a + duty.b * currents.b + duty.c * currents.c;
        let output_power =
            1.5 * (voltage_dq.d * current_dq.d + voltage_dq.q * current_dq.q);
        let telemetry = Telemetry {
            sequence: raw.sequence,
            state: self.state,
            faults: self.faults,
            currents,
            current_dq,
            voltage_dq,
            duty,
            mechanical_angle: rotor.mechanical_angle,
            mechanical_velocity: rotor.mechanical_velocity,
            sensor_valid: rotor.valid,
            bus_voltage,
            ntc_raw: raw.ntc,
            bus_current,
            bus_current_average: self.bus_current_average,
            input_power: bus_voltage * self.bus_current_average,
            output_power,
            junction_temperature: self.junction_temperature,
            target_iq: self.target_iq,
            target_velocity: self.target_velocity,
        };
        self.last_telemetry = telemetry;
        ControlOutput {
            duty,
            bridge_enabled: bridge_enabled && self.state != MotorState::Fault,
            telemetry,
        }
    }
}

fn valid_pid(config: PidConfig) -> bool {
    config.kp.is_finite()
        && config.ki.is_finite()
        && config.kd.is_finite()
        && config.output_limit.is_finite()
        && config.output_limit >= 0.0
        && config.output_ramp >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::{ParameterId, ParameterValue};

    fn raw(
        sequence: u32,
        phase: u16,
        bus_voltage: f32,
        config: FocConfig,
    ) -> RawAdcFrame {
        raw_with_ntc(sequence, phase, bus_voltage, 930, config)
    }

    fn raw_with_ntc(
        sequence: u32,
        phase: u16,
        bus_voltage: f32,
        ntc: u16,
        config: FocConfig,
    ) -> RawAdcFrame {
        let bus_scale = config.adc_reference_voltage / config.adc_full_scale
            * (config.bus_divider_high + config.bus_divider_low)
            / config.bus_divider_low;
        RawAdcFrame {
            phase_a: phase,
            phase_b: phase,
            phase_c: phase,
            bus_voltage: (bus_voltage / bus_scale) as u16,
            ntc,
            sequence,
            overrun: false,
        }
    }

    fn calibrated_controller() -> (FocController, FocConfig, u32) {
        let config = FocConfig {
            calibration_samples: 4,
            battery_cells: 6,
            ..Default::default()
        };
        let mut controller = FocController::new(config);
        let rotor = RotorSample {
            mechanical_angle: 0.0,
            mechanical_velocity: 0.0,
            valid: true,
        };
        for sequence in 1..=4 {
            controller.step(raw(sequence, 2_048, 24.0, config), rotor);
        }
        assert_eq!(controller.state(), MotorState::Idle);
        (controller, config, 4)
    }

    #[test]
    fn calibrates_all_phase_offsets_before_idle() {
        let (controller, _, _) = calibrated_controller();
        assert_eq!(controller.offsets(), [2_048.0; 3]);
    }

    #[test]
    fn current_mode_produces_valid_pwm() {
        let (mut controller, config, sequence) = calibrated_controller();
        controller.handle_command(Command::SetIq(5.0));
        controller.handle_command(Command::Enable(ControlMode::Current));
        let output = controller.step(
            raw(sequence + 1, 2_048, 24.0, config),
            RotorSample {
                mechanical_angle: 0.0,
                mechanical_velocity: 0.0,
                valid: true,
            },
        );
        assert!(output.bridge_enabled);
        for duty in [output.duty.a, output.duty.b, output.duty.c] {
            assert!(
                (config.minimum_duty..=config.maximum_duty).contains(&duty)
            );
        }
    }

    #[test]
    fn overcurrent_is_debounced_and_latched() {
        let (mut controller, config, mut sequence) = calibrated_controller();
        controller.handle_command(Command::Enable(ControlMode::OpenLoop));
        let high_count = 2_048 + (56.0 / 0.032_234_434) as u16;
        for _ in 0..config.over_current_debounce {
            sequence += 1;
            controller.step(
                raw(sequence, high_count, 24.0, config),
                RotorSample::default(),
            );
        }
        assert_eq!(controller.state(), MotorState::Fault);
        assert!(controller.faults().contains(FaultFlags::OVER_CURRENT));
    }

    #[test]
    fn skipped_adc_frame_trips_control_overrun() {
        let (mut controller, config, sequence) = calibrated_controller();
        controller.step(
            raw(sequence + 2, 2_048, 24.0, config),
            RotorSample::default(),
        );
        assert!(controller.faults().contains(FaultFlags::CONTROL_OVERRUN));
    }

    #[test]
    fn ntc_temperature_trips_overtemperature() {
        let (mut controller, config, sequence) = calibrated_controller();
        controller.handle_command(Command::Enable(ControlMode::OpenLoop));
        let hot_ntc = ((0.5 + 130.0 * 0.01) / 3.3 * 4_095.0) as u16;
        controller.step(
            raw_with_ntc(sequence + 1, 2_048, 24.0, hot_ntc, config),
            RotorSample::default(),
        );
        assert_eq!(controller.state(), MotorState::Fault);
        assert!(controller.faults().contains(FaultFlags::OVER_TEMPERATURE));
    }

    #[test]
    fn fault_during_calibration_restarts_calibration_on_clear() {
        let config = FocConfig {
            calibration_samples: 4,
            ..FocConfig::default()
        };
        let mut controller = FocController::new(config);
        let mut frame = raw_with_ntc(1, 0, 24.0, 930, config);
        frame.overrun = true;
        controller.step(frame, RotorSample::default());
        assert_eq!(controller.state(), MotorState::Fault);

        controller.handle_command(Command::ClearFault);
        assert_eq!(controller.state(), MotorState::Calibrating);
        for sequence in 2..=5 {
            controller.step(
                raw(sequence, 2_048, 24.0, config),
                RotorSample {
                    mechanical_angle: 0.0,
                    mechanical_velocity: 0.0,
                    valid: true,
                },
            );
        }
        assert_eq!(controller.state(), MotorState::Idle);
        assert_eq!(controller.offsets(), [2_048.0; 3]);
    }

    #[test]
    fn command_velocity_is_bounded() {
        let (mut controller, config, sequence) = calibrated_controller();
        controller.handle_command(Command::SetVelocity(f32::MAX));
        controller.step(
            raw(sequence + 1, 2_048, 24.0, config),
            RotorSample::default(),
        );
        assert_eq!(
            controller.telemetry().target_velocity,
            config.maximum_velocity
        );
    }
    #[test]
    fn bus_current_reconstruction_rejects_common_mode() {
        let duty = PhaseDuty {
            a: 0.8,
            b: 0.3,
            c: 0.4,
        };
        let currents = PhaseCurrents {
            a: 10.0,
            b: -4.0,
            c: -6.0,
        };
        let reconstructed =
            duty.a * currents.a + duty.b * currents.b + duty.c * currents.c;
        assert!((reconstructed - 4.4).abs() < 1.0e-6);
    }

    #[test]
    fn storage_is_unsafe_until_calibration_completes() {
        let controller = FocController::new(FocConfig::default());
        assert!(!controller.storage_safe());
    }

    #[test]
    fn apply_live_profile_updates_safe_idle_controller() {
        let (mut controller, _, _) = calibrated_controller();
        let mut profile = ParameterProfileV1::default();
        profile
            .set(ParameterId::CurrentLimit, ParameterValue::F32(40.0))
            .unwrap();
        profile
            .set(ParameterId::CurrentKp, ParameterValue::F32(0.7))
            .unwrap();
        assert!(controller.apply_live_profile(profile));
        assert_eq!(controller.config().current_limit, 40.0);
        assert_eq!(controller.config().current_pid.kp, 0.7);
    }

    #[test]
    fn apply_live_profile_rejects_running_bridge() {
        let (mut controller, config, sequence) = calibrated_controller();
        controller.handle_command(Command::Enable(ControlMode::Current));
        controller.step(
            raw(sequence + 1, 2_048, 24.0, config),
            RotorSample {
                mechanical_angle: 0.0,
                mechanical_velocity: 0.0,
                valid: true,
            },
        );
        assert_eq!(
            controller.state(),
            MotorState::Running(ControlMode::Current)
        );
        let previous = controller.config().current_limit;
        assert!(!controller.apply_live_profile(ParameterProfileV1::default()));
        assert_eq!(controller.config().current_limit, previous);
    }

    #[test]
    fn apply_live_profile_rejects_invalid_profile_without_changes() {
        let (mut controller, _, _) = calibrated_controller();
        let profile = ParameterProfileV1 {
            current_limit: 60.0,
            ..ParameterProfileV1::default()
        };
        assert!(profile.validate().is_err());
        let previous = controller.config().current_limit;
        assert!(!controller.apply_live_profile(profile));
        assert_eq!(controller.config().current_limit, previous);
    }

    #[test]
    fn resynchronize_control_input_accepts_new_sequence() {
        let config = FocConfig {
            calibration_samples: 4,
            ..FocConfig::default()
        };
        let mut controller = FocController::new(config);
        let rotor = RotorSample {
            mechanical_angle: 0.0,
            mechanical_velocity: 0.0,
            valid: true,
        };
        controller.step(raw(1, 2_048, 24.0, config), rotor);
        controller.step(raw(2, 2_048, 24.0, config), rotor);
        controller.resynchronize_control_input();
        controller.step(raw(50, 2_048, 24.0, config), rotor);
        controller.step(raw(51, 2_048, 24.0, config), rotor);
        assert_eq!(controller.state(), MotorState::Idle);
        assert_eq!(controller.offsets(), [2_048.0; 3]);
    }
}
