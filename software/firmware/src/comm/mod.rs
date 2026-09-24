//! Communication paths: UART line protocol, Classic CAN control and
//! telemetry, and the shared command/telemetry channels.

mod can;
mod request;
mod uart;

#[cfg(target_arch = "arm")]
mod broker;
#[cfg(target_arch = "arm")]
mod tasks;

/// Compatibility facade for the original communication API.
pub mod comm_task;

pub use request::{
    ParameterRequestEnvelope, ParameterTransactionCache, TransactionLookup,
};

pub(crate) const fn state_code(state: crate::interfaces::MotorState) -> u8 {
    use crate::interfaces::{ControlMode, MotorState};

    match state {
        | MotorState::Calibrating => 0,
        | MotorState::Idle => 1,
        | MotorState::Running(ControlMode::OpenLoop) => 2,
        | MotorState::Running(ControlMode::Current) => 3,
        | MotorState::Running(ControlMode::Velocity) => 4,
        | MotorState::Running(ControlMode::Idle) => 1,
        | MotorState::Fault => 5,
    }
}
