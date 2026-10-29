use foc_firmware::control::foc_core::{FocConfig, FocController};
use foc_firmware::control::pid::PidConfig;
use foc_firmware::interfaces::{
    Command, ControlMode, ControlOutput, MotorState, RawAdcFrame, RotorSample,
};

use crate::inverter;
use crate::plant::{Load, Plant, PlantParams, PlantState};
use crate::sampling;

pub const CONTROL_PERIOD_NS: u64 = 10_000;
pub const PHYSICS_STEP_NS: u64 = 1_000;

#[derive(Clone, Copy, Debug)]
pub enum Scenario {
    Current,
    Speed,
    OpenLoop,
}

impl Scenario {
    pub fn name(self) -> &'static str {
        match self {
            Self::Current => "current",
            Self::Speed => "speed",
            Self::OpenLoop => "open-loop",
        }
    }
}

pub fn controller_config() -> FocConfig {
    FocConfig {
        current_limit: 5.0,
        over_current_threshold: 6.0,
        current_pid: PidConfig::new(0.63, 3_000.0, 0.0, 10.0, f32::INFINITY),
        velocity_pid: PidConfig::new(0.15, 3.0, 0.0, 5.0, f32::INFINITY),
        ..FocConfig::default()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Record {
    pub time_ns: u64,
    pub state: MotorState,
    pub truth: PlantState,
    pub iq_measured: f32,
    pub output: ControlOutput,
    pub effective: ControlOutput,
}

#[derive(Clone, Copy, Debug)]
pub struct Summary {
    pub scenario: Scenario,
    pub final_speed: f64,
    pub final_iq: f64,
    pub mean_iq_error: f64,
    pub mean_speed_error: f64,
    pub peak_phase_current: f64,
    pub faulted: bool,
}

impl Summary {
    pub fn passed(self) -> bool {
        !self.faulted
            && self.peak_phase_current < 6.0
            && match self.scenario {
                Scenario::Current => self.mean_iq_error < 0.1,
                Scenario::Speed => self.mean_speed_error < 2.5,
                Scenario::OpenLoop => (self.final_speed - 30.0).abs() > 5.0,
            }
    }
}

pub struct Runner {
    pub controller: FocController,
    pub plant: Plant,
    pub bus_voltage: f64,
    pub time_ns: u64,
    pub sequence: u32,
    pub physics_step_ns: u64,
    pending: Option<ControlOutput>,
    effective: ControlOutput,
}

impl Runner {
    pub fn new(params: PlantParams, config: FocConfig) -> Result<Self, &'static str> {
        if config.pwm_frequency_hz != 100_000 || config.pole_pairs != params.pole_pairs {
            return Err("simulation timing or pole pairs do not match controller");
        }
        Ok(Self {
            controller: FocController::new(config),
            plant: Plant::new(params)?,
            bus_voltage: 24.0,
            time_ns: 0,
            sequence: 0,
            physics_step_ns: PHYSICS_STEP_NS,
            pending: None,
            effective: ControlOutput::default(),
        })
    }

    /// Capture the physical state before this control period's output can take effect.
    pub fn capture(&mut self) -> (RawAdcFrame, RotorSample) {
        self.sequence = self.sequence.wrapping_add(1);
        sampling::sample(
            &self.plant,
            &self.controller.config(),
            self.bus_voltage,
            self.sequence,
        )
    }

    /// Apply the computed output with one period of PWM latency.
    pub fn drive(&mut self, output: ControlOutput) -> Result<ControlOutput, &'static str> {
        if self.physics_step_ns == 0 || !CONTROL_PERIOD_NS.is_multiple_of(self.physics_step_ns) {
            return Err("physics step must divide control period");
        }
        if output.bridge_enabled {
            self.effective = self.pending.take().unwrap_or_default();
            self.pending = Some(output);
        } else {
            self.stop_bridge();
        }
        let voltage = inverter::voltage(self.effective, self.bus_voltage)?;
        let applied = self.effective;
        let mut elapsed = 0;
        while elapsed < CONTROL_PERIOD_NS {
            self.plant
                .advance(voltage, self.physics_step_ns as f64 * 1.0e-9);
            elapsed += self.physics_step_ns;
        }
        self.time_ns += CONTROL_PERIOD_NS;
        Ok(applied)
    }

    pub fn stop_bridge(&mut self) {
        self.effective = ControlOutput::default();
        self.pending = None;
    }

    /// Sample at t_k, calculate once, then integrate to t_(k+1).
    pub fn step(&mut self) -> Result<Record, &'static str> {
        if self.physics_step_ns == 0 || !CONTROL_PERIOD_NS.is_multiple_of(self.physics_step_ns) {
            return Err("physics step must divide control period");
        }
        let (raw, rotor) = self.capture();
        let output = self.controller.step(raw, rotor);
        let truth = self.plant.state;
        let effective = self.drive(output)?;
        Ok(Record {
            time_ns: self.time_ns - CONTROL_PERIOD_NS,
            state: self.controller.state(),
            truth,
            iq_measured: output.telemetry.current_dq.q,
            output,
            effective,
        })
    }

    pub fn calibrate(&mut self) -> Result<(), &'static str> {
        for _ in 0..self.controller.config().calibration_samples {
            let frame = self.step()?;
            if frame.effective.bridge_enabled {
                return Err("bridge enabled during calibration");
            }
        }
        if self.controller.state() != MotorState::Idle {
            return Err("calibration did not reach idle");
        }
        self.step()?;
        Ok(())
    }
}

pub fn run(scenario: Scenario, mut log: impl FnMut(Record)) -> Result<Summary, &'static str> {
    let mut runner = Runner::new(PlantParams::default(), controller_config())?;
    runner.calibrate()?;
    let (mode, target, duration, load) = match scenario {
        Scenario::Current => (ControlMode::Current, 2.0, 2_000, Load::Signed(0.0)),
        Scenario::Speed => (ControlMode::Velocity, 50.0, 30_000, Load::Opposing(0.015)),
        Scenario::OpenLoop => (ControlMode::OpenLoop, 120.0, 30_000, Load::Opposing(0.015)),
    };
    runner.plant.load = load;
    let command = match scenario {
        Scenario::Current => Command::SetIq(target as f32),
        Scenario::Speed => Command::SetVelocity(target as f32),
        Scenario::OpenLoop => Command::SetOpenLoop {
            electrical_velocity: target as f32,
            q_voltage: 1.0,
        },
    };
    if !runner.controller.handle_command(command)
        || !runner.controller.handle_command(Command::Enable(mode))
    {
        return Err("controller rejected run command");
    }
    let mut sum_iq = 0.0;
    let mut sum_speed = 0.0;
    let mut count = 0;
    let mut peak = 0.0_f64;
    for tick in 0..duration {
        let record = runner.step()?;
        peak = peak.max(
            record
                .truth
                .phase_currents(runner.plant.params)
                .into_iter()
                .map(f64::abs)
                .fold(0.0, f64::max),
        );
        if tick >= duration - 1_000 {
            if matches!(scenario, Scenario::Current) {
                sum_iq += (f64::from(record.iq_measured) - target).abs();
            }
            if matches!(scenario, Scenario::Speed) {
                sum_speed += (record.truth.mechanical_speed - target).abs();
            }
            count += 1;
        }
        log(record);
    }
    Ok(Summary {
        scenario,
        final_speed: runner.plant.state.mechanical_speed,
        final_iq: runner.plant.state.iq,
        mean_iq_error: sum_iq / f64::from(count),
        mean_speed_error: sum_speed / f64::from(count),
        peak_phase_current: peak,
        faulted: runner.controller.state() == MotorState::Fault,
    })
}
