use core::fmt::{self, Write};

use crate::interfaces::{Command, ControlMode, MotorState, Telemetry};
use crate::pid::PidConfig;

pub const CAN_CONTROL_ID: u16 = 0x100;
pub const CAN_IQ_TARGET_ID: u16 = 0x101;
pub const CAN_VELOCITY_TARGET_ID: u16 = 0x102;
pub const CAN_OPEN_LOOP_ID: u16 = 0x103;
pub const CAN_CELL_COUNT_ID: u16 = 0x104;
pub const CAN_ELECTRICAL_ZERO_ID: u16 = 0x105;
pub const CAN_STATUS_ID: u16 = 0x180;
pub const CAN_CURRENTS_ID: u16 = 0x181;
pub const CAN_MOTION_ID: u16 = 0x182;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UartRequest {
    Command(Command),
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

pub fn decode_can_command(id: u16, data: &[u8]) -> Option<Command> {
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

struct TextBuffer<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

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

fn format_telemetry(
    telemetry: &Telemetry,
    output: &mut TextBuffer<256>,
) -> fmt::Result {
    write!(
        output,
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
    )
}

#[cfg(target_arch = "arm")]
mod tasks {
    use embassy_stm32::can;
    use embassy_stm32::mode::Async;
    use embassy_stm32::usart::Uart;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;
    use embassy_sync::watch::Watch;
    use embassy_time::Timer;

    use super::*;

    static COMMANDS: Channel<CriticalSectionRawMutex, Command, 16> =
        Channel::new();
    static TELEMETRY: Watch<CriticalSectionRawMutex, Telemetry, 2> =
        Watch::new();

    pub fn try_receive_command() -> Option<Command> {
        COMMANDS.try_receive().ok()
    }

    pub fn publish_telemetry(telemetry: Telemetry) {
        TELEMETRY.sender().send(telemetry);
    }

    #[embassy_executor::task]
    pub async fn uart_task(mut uart: Uart<'static, Async>) {
        let mut line = [0u8; 128];
        let mut length = 0;

        loop {
            let mut byte = [0u8; 1];
            if uart.read(&mut byte).await.is_err() {
                Timer::after_millis(1).await;
                continue;
            }

            match byte[0] {
                | b'\r' => {},
                | b'\n' => {
                    let request = core::str::from_utf8(&line[..length])
                        .map_err(|_| ParseError::InvalidArgument)
                        .and_then(parse_uart_request);
                    length = 0;

                    match request {
                        | Ok(UartRequest::Command(command)) => {
                            COMMANDS.send(command).await;
                            let _ = uart.write(b"ok\r\n").await;
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
                | value if length < line.len() => {
                    line[length] = value;
                    length += 1;
                },
                | _ => {
                    length = 0;
                    let _ = uart.write(b"line-too-long\r\n").await;
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
                    let id = (frame.priority() >> 18) as u16;
                    if let Some(command) = decode_can_command(id, frame.data())
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
            Timer::after_millis(10).await;
            let Some(telemetry) = TELEMETRY.try_get() else {
                continue;
            };
            for (id, data) in [
                (CAN_STATUS_ID, encode_can_status(&telemetry)),
                (CAN_CURRENTS_ID, encode_can_currents(&telemetry)),
                (CAN_MOTION_ID, encode_can_motion(&telemetry)),
            ] {
                if let Ok(frame) = can::frame::Frame::new_standard(id, &data) {
                    let _ = transmitter.write(&frame).await;
                }
            }
        }
    }
}

#[cfg(target_arch = "arm")]
pub use tasks::{
    can_receive_task, can_telemetry_task, publish_telemetry,
    try_receive_command, uart_task,
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
}
