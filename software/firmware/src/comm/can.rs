use crate::interfaces::{
    Command, ControlMode, ParameterAction, ParameterRequest, ParameterResponse,
    ParameterRoute, ParameterStorageStatus, Telemetry,
};
use crate::params::parameters::{ParameterId, ParameterKind, ParameterValue};

use super::request::ParameterRequestEnvelope;
use super::state_code;

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
    decode_can_parameter_envelope(id, data)
        .map(ParameterRequestEnvelope::request)
}

pub(crate) fn decode_can_parameter_envelope(
    id: u16,
    data: &[u8],
) -> Option<ParameterRequestEnvelope> {
    if data.len() != 8 {
        return None;
    }
    let request_data: [u8; 8] = data.try_into().ok()?;
    let route = ParameterRoute::Can {
        transaction: data[0],
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
    Some(ParameterRequestEnvelope::can(
        ParameterRequest { route, action },
        id,
        request_data,
    ))
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
    let ParameterRoute::Can { transaction } = response.route else {
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

fn scaled_i16(value: f32, scale: f32) -> i16 {
    (value * scale).clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

fn scaled_u16(value: f32, scale: f32) -> u16 {
    (value * scale).clamp(0.0, u16::MAX as f32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn decodes_strict_can_parameter_frames() {
        assert_eq!(
            decode_can_parameter_request(
                CAN_PARAMETER_OPERATION_ID,
                &[7, 1, 0, 0, 0, 0, 0, 0],
            ),
            Some(ParameterRequest {
                route: ParameterRoute::Can { transaction: 7 },
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
                route: ParameterRoute::Can { transaction: 8 },
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
            route: ParameterRoute::Can { transaction: 9 },
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
            route: ParameterRoute::Can { transaction: 10 },
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
