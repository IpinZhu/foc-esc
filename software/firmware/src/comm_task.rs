#[cfg(target_arch = "arm")]
use core::fmt::{self, Write};

#[cfg(target_arch = "arm")]
use crate::autotune::AutoTuneStatus;
#[cfg(target_arch = "arm")]
use crate::interfaces::ParameterResultCode;
use crate::interfaces::{
    Command, ControlMode, MotorState, ParameterAction, ParameterRequest,
    ParameterResponse, ParameterRoute, ParameterStorageStatus, Telemetry,
};
use crate::parameters::{ParameterId, ParameterKind, ParameterValue};
use crate::pid::PidConfig;

pub const CAN_CONTROL_ID: u16 = 0x100;
pub const CAN_IQ_TARGET_ID: u16 = 0x101;
pub const CAN_VELOCITY_TARGET_ID: u16 = 0x102;
pub const CAN_OPEN_LOOP_ID: u16 = 0x103;
pub const CAN_CELL_COUNT_ID: u16 = 0x104;
pub const CAN_ELECTRICAL_ZERO_ID: u16 = 0x105;
pub const CAN_PARAMETER_OPERATION_ID: u16 = 0x106;
pub const CAN_PARAMETER_ACCESS_ID: u16 = 0x107;
pub const CAN_STATUS_ID: u16 = 0x180;
pub const CAN_CURRENTS_ID: u16 = 0x181;
pub const CAN_MOTION_ID: u16 = 0x182;
pub const CAN_PARAMETER_OPERATION_RESULT_ID: u16 = 0x183;
pub const CAN_PARAMETER_ACCESS_RESULT_ID: u16 = 0x184;

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

pub fn decode_can_command(id: u16, data: &[u8]) -> Option<Command> {
    let expected_length = match id {
        | CAN_CONTROL_ID | CAN_CELL_COUNT_ID => 1,
        | CAN_IQ_TARGET_ID
        | CAN_VELOCITY_TARGET_ID
        | CAN_ELECTRICAL_ZERO_ID => 4,
        | CAN_OPEN_LOOP_ID => 8,
        | _ => return None,
    };
    if data.len() != expected_length {
        return None;
    }

    match id {
        | CAN_CONTROL_ID => match *data.first()? {
            | 0 => Some(Command::Disable),
            | 1 => Some(Command::ClearFault),
            | 2 => Some(Command::Enable(ControlMode::OpenLoop)),
            | 3 => Some(Command::Enable(ControlMode::Current)),
            | 4 => Some(Command::Enable(ControlMode::Velocity)),
            | _ => None,
        },
        | CAN_IQ_TARGET_ID => Some(Command::SetIq(read_f32(data, 0)?)),
        | CAN_VELOCITY_TARGET_ID => {
            Some(Command::SetVelocity(read_f32(data, 0)?))
        },
        | CAN_OPEN_LOOP_ID => Some(Command::SetOpenLoop {
            electrical_velocity: read_f32(data, 0)?,
            q_voltage: read_f32(data, 4)?,
        }),
        | CAN_CELL_COUNT_ID => Some(Command::SetCellCount(*data.first()?)),
        | CAN_ELECTRICAL_ZERO_ID => {
            Some(Command::SetElectricalZero(read_f32(data, 0)?))
        },
        | _ => None,
    }
}

fn read_f32(data: &[u8], offset: usize) -> Option<f32> {
    let bytes: [u8; 4] = data.get(offset..offset + 4)?.try_into().ok()?;
    let value = f32::from_le_bytes(bytes);
    value.is_finite().then_some(value)
}

pub fn decode_can_parameter_request(
    id: u16,
    data: &[u8],
) -> Option<ParameterRequest> {
    if data.len() != 8 {
        return None;
    }
    let request_data: [u8; 8] = data.try_into().ok()?;
    let route = ParameterRoute::Can {
        transaction: data[0],
        request_id: id,
        request_data,
    };
    let action = match id {
        | CAN_PARAMETER_OPERATION_ID => {
            if data[2..].iter().any(|value| *value != 0) {
                return None;
            }
            match data[1] {
                | 0 => ParameterAction::Status,
                | 1 => ParameterAction::Save,
                | 2 => ParameterAction::Load,
                | 3 => ParameterAction::Defaults,
                | 4 => ParameterAction::FactoryReset,
                | _ => return None,
            }
        },
        | CAN_PARAMETER_ACCESS_ID => {
            let parameter = ParameterId::from_raw(data[2])?;
            if data[3] != 0 {
                return None;
            }
            match data[1] {
                | 0 if data[4..].iter().all(|value| *value == 0) => {
                    ParameterAction::Get(parameter)
                },
                | 1 => ParameterAction::Set(
                    parameter,
                    decode_parameter_value(parameter, &data[4..8])?,
                ),
                | _ => return None,
            }
        },
        | _ => return None,
    };
    Some(ParameterRequest { route, action })
}

fn decode_parameter_value(
    parameter: ParameterId,
    data: &[u8],
) -> Option<ParameterValue> {
    let bytes: [u8; 4] = data.try_into().ok()?;
    match parameter.kind() {
        | ParameterKind::U8 if bytes[1..] == [0; 3] => {
            Some(ParameterValue::U8(bytes[0]))
        },
        | ParameterKind::I8 if bytes[1..] == [0; 3] => {
            Some(ParameterValue::I8(bytes[0] as i8))
        },
        | ParameterKind::U16 if bytes[2..] == [0; 2] => {
            Some(ParameterValue::U16(u16::from_le_bytes([
                bytes[0], bytes[1],
            ])))
        },
        | ParameterKind::F32 => {
            let value = f32::from_le_bytes(bytes);
            value.is_finite().then_some(ParameterValue::F32(value))
        },
        | _ => None,
    }
}

pub fn encode_can_parameter_response(
    response: &ParameterResponse,
) -> Option<(u16, [u8; 8])> {
    let ParameterRoute::Can { transaction, .. } = response.route else {
        return None;
    };
    let mut data = [0u8; 8];
    data[0] = transaction;
    match response.action {
        | ParameterAction::Get(parameter)
        | ParameterAction::Set(parameter, _) => {
            data[1] =
                matches!(response.action, ParameterAction::Set(_, _)) as u8;
            data[2] = parameter as u8;
            data[3] = response.result as u8
                | if response.storage.restart_required {
                    0x80
                } else {
                    0
                };
            if let Some(value) = response.value {
                data[4..8].copy_from_slice(&encode_parameter_value(value));
            }
            Some((CAN_PARAMETER_ACCESS_RESULT_ID, data))
        },
        | action => {
            data[1] = operation_code(action)?;
            data[2] = response.result as u8;
            data[3] = storage_flags(response.storage);
            data[4..8]
                .copy_from_slice(&response.storage.generation.to_le_bytes());
            Some((CAN_PARAMETER_OPERATION_RESULT_ID, data))
        },
    }
}

fn operation_code(action: ParameterAction) -> Option<u8> {
    match action {
        | ParameterAction::Status => Some(0),
        | ParameterAction::Save => Some(1),
        | ParameterAction::Load => Some(2),
        | ParameterAction::Defaults => Some(3),
        | ParameterAction::FactoryReset => Some(4),
        | _ => None,
    }
}

fn storage_flags(status: ParameterStorageStatus) -> u8 {
    status.persisted_valid as u8
        | (status.dirty as u8) << 1
        | (status.restart_required as u8) << 2
        | (status.source_defaults as u8) << 3
}

fn encode_parameter_value(value: ParameterValue) -> [u8; 4] {
    match value {
        | ParameterValue::U8(value) => [value, 0, 0, 0],
        | ParameterValue::I8(value) => [value as u8, 0, 0, 0],
        | ParameterValue::U16(value) => {
            let [low, high] = value.to_le_bytes();
            [low, high, 0, 0]
        },
        | ParameterValue::F32(value) => value.to_le_bytes(),
    }
}

pub fn encode_can_status(telemetry: &Telemetry) -> [u8; 8] {
    let mut data = [0; 8];
    data[0] = state_code(telemetry.state);
    data[1..5].copy_from_slice(&telemetry.faults.bits().to_le_bytes());
    data[5] = telemetry.sensor_valid as u8;
    data[6..8].copy_from_slice(&(telemetry.sequence as u16).to_le_bytes());
    data
}

pub fn encode_can_currents(telemetry: &Telemetry) -> [u8; 8] {
    let values = [
        scaled_i16(telemetry.currents.a, 100.0),
        scaled_i16(telemetry.currents.b, 100.0),
        scaled_i16(telemetry.currents.c, 100.0),
        scaled_i16(telemetry.bus_current_average, 100.0),
    ];
    let mut data = [0; 8];
    for (index, value) in values.into_iter().enumerate() {
        data[index * 2..index * 2 + 2].copy_from_slice(&value.to_le_bytes());
    }
    data
}

pub fn encode_can_motion(telemetry: &Telemetry) -> [u8; 8] {
    let values = [
        scaled_u16(telemetry.bus_voltage, 100.0),
        scaled_i16(telemetry.mechanical_velocity, 100.0) as u16,
        scaled_u16(telemetry.mechanical_angle, 1_000.0),
        scaled_i16(telemetry.junction_temperature, 10.0) as u16,
    ];
    let mut data = [0; 8];
    for (index, value) in values.into_iter().enumerate() {
        data[index * 2..index * 2 + 2].copy_from_slice(&value.to_le_bytes());
    }
    data
}

fn state_code(state: MotorState) -> u8 {
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

fn scaled_i16(value: f32, scale: f32) -> i16 {
    (value * scale).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

fn scaled_u16(value: f32, scale: f32) -> u16 {
    (value * scale).clamp(0.0, u16::MAX as f32) as u16
}

#[cfg(target_arch = "arm")]
struct TextBuffer<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

#[cfg(target_arch = "arm")]
impl<const N: usize> TextBuffer<N> {
    const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
        }
    }

    fn as_bytes(&self) -> &[u8] {
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
fn format_telemetry(
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
fn format_parameter_response(
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
fn format_autotune_status(
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

#[cfg(target_arch = "arm")]
mod tasks {
    use embassy_futures::select::{Either, select};
    use embassy_stm32::can;
    use embassy_stm32::mode::Async;
    use embassy_stm32::usart::Uart;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;
    use embassy_sync::watch::Watch;
    use embassy_time::Timer;
    use embedded_can::Id;

    use super::*;

    static COMMANDS: Channel<CriticalSectionRawMutex, Command, 16> =
        Channel::new();
    static PARAMETER_REQUESTS: Channel<
        CriticalSectionRawMutex,
        ParameterRequest,
        8,
    > = Channel::new();
    static UART_PARAMETER_RESPONSES: Channel<
        CriticalSectionRawMutex,
        ParameterResponse,
        1,
    > = Channel::new();
    static CAN_PARAMETER_RESPONSES: Channel<
        CriticalSectionRawMutex,
        ParameterResponse,
        8,
    > = Channel::new();
    static TELEMETRY: Watch<CriticalSectionRawMutex, Telemetry, 2> =
        Watch::new();
    static AUTOTUNE_STATUS: Watch<CriticalSectionRawMutex, AutoTuneStatus, 4> =
        Watch::new();

    pub fn try_receive_command() -> Option<Command> {
        COMMANDS.try_receive().ok()
    }

    pub fn try_receive_parameter_request() -> Option<ParameterRequest> {
        PARAMETER_REQUESTS.try_receive().ok()
    }

    pub fn publish_parameter_response(response: ParameterResponse) -> bool {
        match response.route {
            | ParameterRoute::Uart => {
                UART_PARAMETER_RESPONSES.try_send(response)
            },
            | ParameterRoute::Can { .. } => {
                CAN_PARAMETER_RESPONSES.try_send(response)
            },
        }
        .is_ok()
    }

    pub fn publish_telemetry(telemetry: Telemetry) {
        TELEMETRY.sender().send(telemetry);
    }

    pub fn publish_autotune_status(status: AutoTuneStatus) {
        AUTOTUNE_STATUS.sender().send(status);
    }

    pub fn try_get_autotune_status() -> Option<AutoTuneStatus> {
        AUTOTUNE_STATUS.try_get()
    }

    #[embassy_executor::task]
    pub async fn uart_task(mut uart: Uart<'static, Async>) {
        let mut line = [0u8; 128];
        let mut length = 0;
        let mut discarding_line = false;

        loop {
            let mut byte = [0u8; 1];
            if uart.read(&mut byte).await.is_err() {
                Timer::after_millis(1).await;
                continue;
            }

            match byte[0] {
                | b'\r' => {},
                | b'\n' => {
                    if discarding_line {
                        discarding_line = false;
                        length = 0;
                        let _ = uart.write(b"line-too-long\r\n").await;
                        continue;
                    }
                    let request = core::str::from_utf8(&line[..length])
                        .map_err(|_| ParseError::InvalidArgument)
                        .and_then(parse_uart_request);
                    length = 0;

                    match request {
                        | Ok(UartRequest::Command(command)) => {
                            COMMANDS.send(command).await;
                            let _ = uart.write(b"ok\r\n").await;
                        },
                        | Ok(UartRequest::Parameter(action)) => {
                            PARAMETER_REQUESTS
                                .send(ParameterRequest {
                                    route: ParameterRoute::Uart,
                                    action,
                                })
                                .await;
                            let response =
                                UART_PARAMETER_RESPONSES.receive().await;
                            let mut output = TextBuffer::new();
                            if format_parameter_response(&response, &mut output)
                                .is_ok()
                            {
                                let _ = uart.write(output.as_bytes()).await;
                            } else {
                                let _ = uart.write(b"error format\r\n").await;
                            }
                        },
                        | Ok(UartRequest::ParameterShow) => {
                            for parameter in ParameterId::ALL {
                                PARAMETER_REQUESTS
                                    .send(ParameterRequest {
                                        route: ParameterRoute::Uart,
                                        action: ParameterAction::Get(parameter),
                                    })
                                    .await;
                                let response =
                                    UART_PARAMETER_RESPONSES.receive().await;
                                let mut output = TextBuffer::new();
                                if format_parameter_response(
                                    &response,
                                    &mut output,
                                )
                                .is_ok()
                                {
                                    let _ = uart.write(output.as_bytes()).await;
                                }
                            }
                        },
                        | Ok(UartRequest::AutoTune(action)) => match action {
                            | AutoTuneAction::Start => {
                                COMMANDS.send(Command::AutoTuneStart).await;
                                let _ = uart
                                    .write(b"ok autotune-starting\r\n")
                                    .await;
                            },
                            | AutoTuneAction::Stop => {
                                COMMANDS.send(Command::AutoTuneStop).await;
                                let _ = uart
                                    .write(b"ok autotune-stopping\r\n")
                                    .await;
                            },
                            | AutoTuneAction::Status => {
                                let mut output = TextBuffer::new();
                                if format_autotune_status(
                                    try_get_autotune_status(),
                                    &mut output,
                                )
                                .is_ok()
                                {
                                    let _ = uart.write(output.as_bytes()).await;
                                } else {
                                    let _ =
                                        uart.write(b"error format\r\n").await;
                                }
                            },
                        },
                        | Ok(UartRequest::Status) => {
                            if let Some(telemetry) = TELEMETRY.try_get() {
                                let mut response = TextBuffer::new();
                                if format_telemetry(&telemetry, &mut response)
                                    .is_ok()
                                {
                                    let _ =
                                        uart.write(response.as_bytes()).await;
                                }
                            } else {
                                let _ = uart.write(b"not-ready\r\n").await;
                            }
                        },
                        | Err(_) => {
                            let _ = uart.write(b"error\r\n").await;
                        },
                    }
                },
                | value if !discarding_line && length < line.len() => {
                    line[length] = value;
                    length += 1;
                },
                | _ => {
                    discarding_line = true;
                    length = 0;
                },
            }
        }
    }

    #[embassy_executor::task]
    pub async fn can_receive_task(mut receiver: can::CanRx<'static>) {
        loop {
            match receiver.read().await {
                | Ok(envelope) => {
                    let frame = envelope.frame;
                    if frame.header().rtr() || frame.header().fdcan() {
                        continue;
                    }
                    let id = match frame.header().id() {
                        | Id::Standard(id) => id.as_raw(),
                        | Id::Extended(_) => continue,
                    };
                    if let Some(request) =
                        decode_can_parameter_request(id, frame.data())
                    {
                        PARAMETER_REQUESTS.send(request).await;
                    } else if let Some(command) =
                        decode_can_command(id, frame.data())
                    {
                        COMMANDS.send(command).await;
                    }
                },
                | Err(_) => Timer::after_millis(1).await,
            }
        }
    }

    #[embassy_executor::task]
    pub async fn can_telemetry_task(mut transmitter: can::CanTx<'static>) {
        loop {
            match select(
                CAN_PARAMETER_RESPONSES.receive(),
                Timer::after_millis(10),
            )
            .await
            {
                | Either::First(response) => {
                    let Some((id, data)) =
                        encode_can_parameter_response(&response)
                    else {
                        continue;
                    };
                    if let Ok(frame) =
                        can::frame::Frame::new_standard(id, &data)
                    {
                        let _ = transmitter.write(&frame).await;
                    }
                },
                | Either::Second(_) => {
                    let Some(telemetry) = TELEMETRY.try_get() else {
                        continue;
                    };
                    for (id, data) in [
                        (CAN_STATUS_ID, encode_can_status(&telemetry)),
                        (CAN_CURRENTS_ID, encode_can_currents(&telemetry)),
                        (CAN_MOTION_ID, encode_can_motion(&telemetry)),
                    ] {
                        if let Ok(frame) =
                            can::frame::Frame::new_standard(id, &data)
                        {
                            let _ = transmitter.write(&frame).await;
                        }
                    }
                },
            }
        }
    }
}

#[cfg(target_arch = "arm")]
pub use tasks::{
    can_receive_task, can_telemetry_task, publish_autotune_status,
    publish_parameter_response, publish_telemetry, try_get_autotune_status,
    try_receive_command, try_receive_parameter_request, uart_task,
};

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
    fn decodes_can_commands() {
        assert_eq!(
            decode_can_command(CAN_CONTROL_ID, &[4]),
            Some(Command::Enable(ControlMode::Velocity))
        );
        assert_eq!(
            decode_can_command(CAN_IQ_TARGET_ID, &12.5f32.to_le_bytes()),
            Some(Command::SetIq(12.5))
        );
        assert_eq!(decode_can_command(CAN_OPEN_LOOP_ID, &[0; 4]), None);
        assert_eq!(decode_can_command(CAN_CONTROL_ID, &[0, 0]), None);
        assert_eq!(
            decode_can_command(CAN_IQ_TARGET_ID, &[0, 0, 0, 0, 0]),
            None
        );
    }

    #[test]
    fn encodes_can_telemetry_little_endian() {
        let mut telemetry = Telemetry::default();
        telemetry.currents.a = 1.25;
        telemetry.currents.b = -2.5;
        telemetry.currents.c = 0.75;
        telemetry.bus_current_average = 3.0;
        let data = encode_can_currents(&telemetry);
        assert_eq!(i16::from_le_bytes([data[0], data[1]]), 125);
        assert_eq!(i16::from_le_bytes([data[2], data[3]]), -250);
        assert_eq!(i16::from_le_bytes([data[6], data[7]]), 300);
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

    #[test]
    fn decodes_strict_can_parameter_frames() {
        assert_eq!(
            decode_can_parameter_request(
                CAN_PARAMETER_OPERATION_ID,
                &[7, 1, 0, 0, 0, 0, 0, 0],
            ),
            Some(ParameterRequest {
                route: ParameterRoute::Can {
                    transaction: 7,
                    request_id: CAN_PARAMETER_OPERATION_ID,
                    request_data: [7, 1, 0, 0, 0, 0, 0, 0],
                },
                action: ParameterAction::Save,
            })
        );
        let mut set = [0u8; 8];
        set[0] = 8;
        set[1] = 1;
        set[2] = ParameterId::CurrentKp as u8;
        set[4..8].copy_from_slice(&0.25f32.to_le_bytes());
        assert_eq!(
            decode_can_parameter_request(CAN_PARAMETER_ACCESS_ID, &set),
            Some(ParameterRequest {
                route: ParameterRoute::Can {
                    transaction: 8,
                    request_id: CAN_PARAMETER_ACCESS_ID,
                    request_data: set,
                },
                action: ParameterAction::Set(
                    ParameterId::CurrentKp,
                    ParameterValue::F32(0.25),
                ),
            })
        );
        set[3] = 1;
        assert_eq!(
            decode_can_parameter_request(CAN_PARAMETER_ACCESS_ID, &set),
            None
        );
        assert_eq!(
            decode_can_parameter_request(CAN_PARAMETER_OPERATION_ID, &[0; 7]),
            None
        );
    }

    #[test]
    fn encodes_can_parameter_results() {
        use crate::interfaces::ParameterResultCode;

        let response = ParameterResponse {
            route: ParameterRoute::Can {
                transaction: 9,
                request_id: CAN_PARAMETER_ACCESS_ID,
                request_data: [0; 8],
            },
            action: ParameterAction::Get(ParameterId::EncoderCpr),
            result: ParameterResultCode::Success,
            value: Some(ParameterValue::U16(4_096)),
            storage: ParameterStorageStatus {
                persisted_valid: true,
                generation: 12,
                restart_required: true,
                ..ParameterStorageStatus::default()
            },
        };
        let (id, data) = encode_can_parameter_response(&response).unwrap();
        assert_eq!(id, CAN_PARAMETER_ACCESS_RESULT_ID);
        assert_eq!(data, [9, 0, 4, 0x80, 0x00, 0x10, 0, 0]);

        let operation = ParameterResponse {
            route: ParameterRoute::Can {
                transaction: 10,
                request_id: CAN_PARAMETER_OPERATION_ID,
                request_data: [0; 8],
            },
            action: ParameterAction::Save,
            result: ParameterResultCode::Unchanged,
            value: None,
            storage: ParameterStorageStatus {
                persisted_valid: true,
                generation: 12,
                dirty: false,
                ..ParameterStorageStatus::default()
            },
        };
        let (id, data) = encode_can_parameter_response(&operation).unwrap();
        assert_eq!(id, CAN_PARAMETER_OPERATION_RESULT_ID);
        assert_eq!(data, [10, 1, 1, 1, 12, 0, 0, 0]);
    }
}
