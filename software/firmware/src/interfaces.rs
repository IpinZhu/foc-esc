use crate::control::foc_math::{Dq, PhaseCurrents, PhaseDuty};
use crate::control::pid::PidConfig;
use crate::params::parameters::{ParameterId, ParameterValue};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ControlMode {
    #[default]
    Idle,
    OpenLoop,
    Current,
    Velocity,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MotorState {
    #[default]
    Calibrating,
    Idle,
    Running(ControlMode),
    Fault,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaultFlags(u32);

impl FaultFlags {
    pub const OVER_CURRENT: Self = Self(1 << 0);
    pub const CURRENT_RESIDUAL: Self = Self(1 << 1);
    pub const UNDER_VOLTAGE: Self = Self(1 << 2);
    pub const OVER_VOLTAGE: Self = Self(1 << 3);
    pub const OVER_TEMPERATURE: Self = Self(1 << 4);
    pub const STALL: Self = Self(1 << 5);
    pub const SENSOR: Self = Self(1 << 6);
    pub const CONTROL_OVERRUN: Self = Self(1 << 7);
    pub const INVALID_COMMAND: Self = Self(1 << 8);

    pub const fn empty() -> Self {
        Self(0)
    }

    pub const fn bits(self) -> u32 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    pub fn insert(&mut self, other: Self) {
        self.0 |= other.0;
    }

    pub fn clear(&mut self) {
        self.0 = 0;
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RawAdcFrame {
    pub phase_a: u16,
    pub phase_b: u16,
    pub phase_c: u16,
    pub bus_voltage: u16,
    pub ntc: u16,
    pub sequence: u32,
    pub overrun: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotorSample {
    pub mechanical_angle: f32,
    pub mechanical_velocity: f32,
    pub valid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    Enable(ControlMode),
    Disable,
    ClearFault,
    SetIq(f32),
    SetVelocity(f32),
    SetOpenLoop {
        electrical_velocity: f32,
        q_voltage: f32,
    },
    SetCellCount(u8),
    SetElectricalZero(f32),
    SetCurrentPid(PidConfig),
    SetVelocityPid(PidConfig),
    AutoTuneStart,
    AutoTuneStop,
    ReportControlOverrun,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParameterRoute {
    Uart,
    Can {
        transaction: u8,
        request_id: u16,
        request_data: [u8; 8],
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParameterAction {
    Get(ParameterId),
    Set(ParameterId, ParameterValue),
    Status,
    Save,
    Load,
    Defaults,
    FactoryReset,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParameterRequest {
    pub route: ParameterRoute,
    pub action: ParameterAction,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ParameterStorageStatus {
    pub persisted_valid: bool,
    pub source_defaults: bool,
    pub generation: u32,
    pub dirty: bool,
    pub restart_required: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ParameterResultCode {
    Success = 0,
    Unchanged = 1,
    Busy = 2,
    Invalid = 3,
    NoValidRecord = 4,
    FlashRead = 5,
    FlashErase = 6,
    FlashProgram = 7,
    Verify = 8,
    Unknown = 9,
    QueueFull = 10,
    Conflict = 11,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParameterResponse {
    pub route: ParameterRoute,
    pub action: ParameterAction,
    pub result: ParameterResultCode,
    pub value: Option<ParameterValue>,
    pub storage: ParameterStorageStatus,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Telemetry {
    pub sequence: u32,
    pub state: MotorState,
    pub faults: FaultFlags,
    pub currents: PhaseCurrents,
    pub current_dq: Dq,
    pub voltage_dq: Dq,
    pub duty: PhaseDuty,
    pub mechanical_angle: f32,
    pub mechanical_velocity: f32,
    pub sensor_valid: bool,
    pub bus_voltage: f32,
    pub ntc_raw: u16,
    pub bus_current: f32,
    pub bus_current_average: f32,
    pub input_power: f32,
    pub output_power: f32,
    pub junction_temperature: f32,
    pub target_iq: f32,
    pub target_velocity: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlOutput {
    pub duty: PhaseDuty,
    pub bridge_enabled: bool,
    pub telemetry: Telemetry,
}

impl Default for ControlOutput {
    fn default() -> Self {
        Self {
            duty: PhaseDuty::DISABLED,
            bridge_enabled: false,
            telemetry: Telemetry::default(),
        }
    }
}
