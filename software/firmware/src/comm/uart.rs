#[cfg(target_arch = "arm")]
use core::fmt::{self, Write};

#[cfg(target_arch = "arm")]
use crate::autotune::AutoTuneStatus;
use crate::control::pid::PidConfig;
use crate::interfaces::{Command, ControlMode, ParameterAction};
#[cfg(target_arch = "arm")]
use crate::interfaces::{
    ParameterResponse, ParameterResultCode, ParameterStorageStatus, Telemetry,
};
use crate::params::parameters::{ParameterId, ParameterKind, ParameterValue};

#[cfg(target_arch = "arm")]
use super::state_code;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UartRequest {
    Command(Command),
    Parameter(ParameterAction),
    ParameterShow,
    AutoTune(AutoTuneAction),
    Status,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AutoTuneAction {
    Start,
    Stop,
    Status,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    Empty,
    Unknown,
    MissingArgument,
    InvalidArgument,
    ExtraArgument,
}

pub fn parse_uart_request(line: &str) -> Result<UartRequest, ParseError> {
    let mut words = line.split_ascii_whitespace();
    let Some(command) = words.next() else {
        return Err(ParseError::Empty);
    };

    let request = match command {
        | "enable" | "mode" => {
            let mode = match next_word(&mut words)? {
                | "open" => ControlMode::OpenLoop,
                | "current" => ControlMode::Current,
                | "velocity" => ControlMode::Velocity,
                | _ => return Err(ParseError::InvalidArgument),
            };
            UartRequest::Command(Command::Enable(mode))
        },
        | "disable" => UartRequest::Command(Command::Disable),
        | "clear" | "clear-fault" => UartRequest::Command(Command::ClearFault),
        | "iq" => UartRequest::Command(Command::SetIq(next_f32(&mut words)?)),
        | "velocity" => {
            UartRequest::Command(Command::SetVelocity(next_f32(&mut words)?))
        },
        | "open" => UartRequest::Command(Command::SetOpenLoop {
            electrical_velocity: next_f32(&mut words)?,
            q_voltage: next_f32(&mut words)?,
        }),
        | "cells" => {
            UartRequest::Command(Command::SetCellCount(next_u8(&mut words)?))
        },
        | "zero" => UartRequest::Command(Command::SetElectricalZero(next_f32(
            &mut words,
        )?)),
        | "current-pid" => {
            UartRequest::Command(Command::SetCurrentPid(next_pid(&mut words)?))
        },
        | "velocity-pid" => {
            UartRequest::Command(Command::SetVelocityPid(next_pid(&mut words)?))
        },
        | "param" => parse_parameter_request(&mut words)?,
        | "autotune" => {
            let action = match next_word(&mut words)? {
                | "start" => AutoTuneAction::Start,
                | "stop" => AutoTuneAction::Stop,
                | "status" => AutoTuneAction::Status,
                | _ => return Err(ParseError::InvalidArgument),
            };
            UartRequest::AutoTune(action)
        },
        | "status" => UartRequest::Status,
        | _ => return Err(ParseError::Unknown),
    };

    if words.next().is_some() {
        return Err(ParseError::ExtraArgument);
    }
    Ok(request)
}

fn next_word<'a>(
    words: &mut impl Iterator<Item = &'a str>,
) -> Result<&'a str, ParseError> {
    words.next().ok_or(ParseError::MissingArgument)
}

fn next_f32<'a>(
    words: &mut impl Iterator<Item = &'a str>,
) -> Result<f32, ParseError> {
    let value = next_word(words)?
        .parse::<f32>()
        .map_err(|_| ParseError::InvalidArgument)?;
    value
        .is_finite()
        .then_some(value)
        .ok_or(ParseError::InvalidArgument)
}

fn next_u8<'a>(
    words: &mut impl Iterator<Item = &'a str>,
) -> Result<u8, ParseError> {
    next_word(words)?
        .parse::<u8>()
        .map_err(|_| ParseError::InvalidArgument)
}

fn next_pid<'a>(
    words: &mut impl Iterator<Item = &'a str>,
) -> Result<PidConfig, ParseError> {
    Ok(PidConfig::new(
        next_f32(words)?,
        next_f32(words)?,
        next_f32(words)?,
        next_f32(words)?,
        next_f32(words)?,
    ))
}

fn parse_parameter_request<'a>(
    words: &mut impl Iterator<Item = &'a str>,
) -> Result<UartRequest, ParseError> {
    let action = match next_word(words)? {
        | "get" => {
            let id = ParameterId::from_name(next_word(words)?)
                .ok_or(ParseError::InvalidArgument)?;
            ParameterAction::Get(id)
        },
        | "set" => {
            let id = ParameterId::from_name(next_word(words)?)
                .ok_or(ParseError::InvalidArgument)?;
            let value = match id.kind() {
                | ParameterKind::U8 => ParameterValue::U8(next_u8(words)?),
                | ParameterKind::I8 => ParameterValue::I8(
                    next_word(words)?
                        .parse::<i8>()
                        .map_err(|_| ParseError::InvalidArgument)?,
                ),
                | ParameterKind::U16 => ParameterValue::U16(
                    next_word(words)?
                        .parse::<u16>()
                        .map_err(|_| ParseError::InvalidArgument)?,
                ),
                | ParameterKind::F32 => ParameterValue::F32(next_f32(words)?),
            };
            ParameterAction::Set(id, value)
        },
        | "status" => ParameterAction::Status,
        | "save" => ParameterAction::Save,
        | "load" => ParameterAction::Load,
        | "defaults" => ParameterAction::Defaults,
        | "factory-reset" => ParameterAction::FactoryReset,
        | "show" => return Ok(UartRequest::ParameterShow),
        | _ => return Err(ParseError::InvalidArgument),
    };
    Ok(UartRequest::Parameter(action))
}

#[cfg(target_arch = "arm")]
pub(crate) struct TextBuffer<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

#[cfg(target_arch = "arm")]
impl<const N: usize> TextBuffer<N> {
    pub(crate) const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
        }
    }

    pub(crate) fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

#[cfg(target_arch = "arm")]
impl<const N: usize> Write for TextBuffer<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.len.checked_add(value.len()).ok_or(fmt::Error)?;
        if end > N {
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(value.as_bytes());
        self.len = end;
        Ok(())
    }
}

#[cfg(target_arch = "arm")]
pub(crate) fn format_telemetry(
    telemetry: &Telemetry,
    output: &mut TextBuffer<256>,
) -> fmt::Result {
    output.write_fmt(format_args!(
        "seq={} state={} faults={:08x} ia={:.2} ib={:.2} ic={:.2} id={:.2} iq={:.2} vbus={:.2} vel={:.2} temp={:.1}\r\n",
        telemetry.sequence,
        state_code(telemetry.state),
        telemetry.faults.bits(),
        telemetry.currents.a,
        telemetry.currents.b,
        telemetry.currents.c,
        telemetry.current_dq.d,
        telemetry.current_dq.q,
        telemetry.bus_voltage,
        telemetry.mechanical_velocity,
        telemetry.junction_temperature,
    ))
}

#[cfg(target_arch = "arm")]
fn write_parameter_value(
    output: &mut impl Write,
    value: ParameterValue,
) -> fmt::Result {
    match value {
        | ParameterValue::U8(value) => write!(output, "{value}"),
        | ParameterValue::I8(value) => write!(output, "{value}"),
        | ParameterValue::U16(value) => write!(output, "{value}"),
        | ParameterValue::F32(value) => write!(output, "{value:.7}"),
    }
}

#[cfg(target_arch = "arm")]
pub(crate) fn format_parameter_response(
    response: &ParameterResponse,
    output: &mut TextBuffer<256>,
) -> fmt::Result {
    if !matches!(
        response.result,
        ParameterResultCode::Success | ParameterResultCode::Unchanged
    ) {
        return writeln!(output, "error {}\r", result_name(response.result));
    }

    match response.action {
        | ParameterAction::Get(parameter) => {
            write!(output, "param {}=", parameter.name())?;
            write_parameter_value(output, response.value.ok_or(fmt::Error)?)?;
            output.write_str("\r\n")
        },
        | ParameterAction::Set(parameter, _) => {
            write!(output, "ok set {}=", parameter.name())?;
            write_parameter_value(output, response.value.ok_or(fmt::Error)?)?;
            write!(
                output,
                " dirty={} restart-required={}\r\n",
                response.storage.dirty as u8,
                response.storage.restart_required as u8,
            )
        },
        | ParameterAction::Status => {
            write_parameter_status(response.storage, output)
        },
        | ParameterAction::Save => write!(
            output,
            "ok {} generation={}\r\n",
            if response.result == ParameterResultCode::Unchanged {
                "unchanged"
            } else {
                "saved"
            },
            response.storage.generation,
        ),
        | ParameterAction::Load => write!(
            output,
            "ok loaded generation={} restart-required={}\r\n",
            response.storage.generation,
            response.storage.restart_required as u8,
        ),
        | ParameterAction::Defaults => write!(
            output,
            "ok defaults dirty={} restart-required={}\r\n",
            response.storage.dirty as u8,
            response.storage.restart_required as u8,
        ),
        | ParameterAction::FactoryReset => write!(
            output,
            "ok factory-reset generation={} restart-required={}\r\n",
            response.storage.generation,
            response.storage.restart_required as u8,
        ),
    }
}

#[cfg(target_arch = "arm")]
fn write_parameter_status(
    status: ParameterStorageStatus,
    output: &mut TextBuffer<256>,
) -> fmt::Result {
    write!(
        output,
        "param-status valid={} source={} generation={} dirty={} restart-required={}\r\n",
        status.persisted_valid as u8,
        if status.source_defaults {
            "defaults"
        } else {
            "flash"
        },
        status.generation,
        status.dirty as u8,
        status.restart_required as u8,
    )
}

#[cfg(target_arch = "arm")]
pub(crate) fn format_autotune_status(
    status: Option<AutoTuneStatus>,
    output: &mut TextBuffer<256>,
) -> fmt::Result {
    match status {
        | None => output.write_str("at state=unknown error=none\r\n"),
        | Some(status) => write!(
            output,
            "at state={} error={} progress={} bw={:?} rs={:?} ld={:?} flux={:?} attempts={} commissioned={}\r\n",
            status.state.name(),
            status.error.name(),
            status.progress,
            status.current_bandwidth_hz,
            status.rs,
            status.ld,
            status.flux,
            status.attempts,
            status.commissioned as u8,
        ),
    }
}

#[cfg(target_arch = "arm")]
fn result_name(result: ParameterResultCode) -> &'static str {
    match result {
        | ParameterResultCode::Success => "success",
        | ParameterResultCode::Unchanged => "unchanged",
        | ParameterResultCode::Busy => "busy",
        | ParameterResultCode::Invalid => "invalid",
        | ParameterResultCode::NoValidRecord => "no-valid-record",
        | ParameterResultCode::FlashRead => "flash-read",
        | ParameterResultCode::FlashErase => "flash-erase",
        | ParameterResultCode::FlashProgram => "flash-program",
        | ParameterResultCode::Verify => "verify",
        | ParameterResultCode::Unknown => "unknown-parameter",
        | ParameterResultCode::QueueFull => "queue-full",
        | ParameterResultCode::Conflict => "transaction-conflict",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_uart_commands() {
        assert_eq!(
            parse_uart_request("enable current"),
            Ok(UartRequest::Command(Command::Enable(ControlMode::Current)))
        );
        assert_eq!(
            parse_uart_request("open 12.5 3.0"),
            Ok(UartRequest::Command(Command::SetOpenLoop {
                electrical_velocity: 12.5,
                q_voltage: 3.0,
            }))
        );
        assert_eq!(parse_uart_request("status"), Ok(UartRequest::Status));
        assert_eq!(
            parse_uart_request("iq nan"),
            Err(ParseError::InvalidArgument)
        );
    }

    #[test]
    fn parses_autotune_requests() {
        assert_eq!(
            parse_uart_request("autotune start"),
            Ok(UartRequest::AutoTune(AutoTuneAction::Start))
        );
        assert_eq!(
            parse_uart_request("autotune stop"),
            Ok(UartRequest::AutoTune(AutoTuneAction::Stop))
        );
        assert_eq!(
            parse_uart_request("autotune status"),
            Ok(UartRequest::AutoTune(AutoTuneAction::Status))
        );
        assert_eq!(
            parse_uart_request("autotune"),
            Err(ParseError::MissingArgument)
        );
        assert_eq!(
            parse_uart_request("autotune reboot"),
            Err(ParseError::InvalidArgument)
        );
    }

    #[test]
    fn parses_uart_parameter_requests_with_declared_types() {
        assert_eq!(
            parse_uart_request("param get current-kp"),
            Ok(UartRequest::Parameter(ParameterAction::Get(
                ParameterId::CurrentKp
            )))
        );
        assert_eq!(
            parse_uart_request("param set battery-cells 4"),
            Ok(UartRequest::Parameter(ParameterAction::Set(
                ParameterId::BatteryCells,
                ParameterValue::U8(4),
            )))
        );
        assert_eq!(
            parse_uart_request("param set sensor-direction -1"),
            Ok(UartRequest::Parameter(ParameterAction::Set(
                ParameterId::SensorDirection,
                ParameterValue::I8(-1),
            )))
        );
        assert_eq!(
            parse_uart_request("param show"),
            Ok(UartRequest::ParameterShow)
        );
        assert_eq!(
            parse_uart_request("param set current-kp inf"),
            Err(ParseError::InvalidArgument)
        );
    }
}
