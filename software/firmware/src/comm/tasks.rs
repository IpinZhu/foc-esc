use embassy_futures::select::{Either, select};
use embassy_stm32::can;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::Uart;
use embassy_time::Timer;
use embedded_can::Id;
use foc_wire::{Decoder, MAX_FRAME};

use crate::interfaces::{
    Command, ParameterAction, ParameterRequest, ParameterRoute,
};
use crate::params::parameters::ParameterId;

use super::broker;
use super::can::{
    CAN_CURRENTS_ID, CAN_MOTION_ID, CAN_STATUS_ID, decode_can_command,
    decode_can_parameter_envelope, encode_can_currents, encode_can_motion,
    encode_can_parameter_response, encode_can_status,
};
use super::request::ParameterRequestEnvelope;
use super::uart::{
    AutoTuneAction, ParseError, TextBuffer, UartRequest,
    format_autotune_status, format_parameter_response, format_telemetry,
    parse_uart_request,
};

#[embassy_executor::task]
pub async fn uart_task(mut uart: Uart<'static, Async>) {
    let mut line = [0u8; 128];
    let mut length = 0;
    let mut discarding_line = false;
    let mut decoder = Decoder::new();
    let mut binary_mode = false;
    let mut pending_f = false;

    loop {
        let mut byte = [0u8; 1];
        match select(uart.read(&mut byte), Timer::after_millis(100)).await {
            | Either::First(Ok(())) => {},
            | Either::First(Err(_)) => continue,
            | Either::Second(_) => {
                if binary_mode {
                    decoder.reset();
                    binary_mode = false;
                }
                continue;
            },
        }

        let value = byte[0];
        if binary_mode {
            if let Some(request) = decoder.push(value) {
                binary_mode = false;
                broker::send_wire_request(request).await;
                let response = broker::receive_wire_response().await;
                let mut encoded = [0u8; MAX_FRAME];
                let count = response.encode(&mut encoded);
                if count != 0 {
                    let _ = uart.write(&encoded[..count]).await;
                }
            }
            continue;
        }
        if pending_f {
            pending_f = false;
            if value == b'C' {
                // Framed traffic must never fall back into ASCII execution.
                binary_mode = true;
                length = 0;
                discarding_line = false;
                decoder.reset();
                let _ = decoder.push(b'F');
                let _ = decoder.push(b'C');
                continue;
            }
            if length < line.len() {
                line[length] = b'F';
                length += 1;
            } else {
                discarding_line = true;
                length = 0;
            }
        }
        if value == b'F' && !discarding_line {
            pending_f = true;
            continue;
        }

        match value {
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
                        broker::send_command(command).await;
                        let _ = uart.write(b"ok\r\n").await;
                    },
                    | Ok(UartRequest::Parameter(action)) => {
                        broker::send_parameter_request(
                            ParameterRequestEnvelope::uart(ParameterRequest {
                                route: ParameterRoute::Uart,
                                action,
                            }),
                        )
                        .await;
                        let response =
                            broker::receive_uart_parameter_response().await;
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
                            broker::send_parameter_request(
                                ParameterRequestEnvelope::uart(
                                    ParameterRequest {
                                        route: ParameterRoute::Uart,
                                        action: ParameterAction::Get(parameter),
                                    },
                                ),
                            )
                            .await;
                            let response =
                                broker::receive_uart_parameter_response().await;
                            let mut output = TextBuffer::new();
                            if format_parameter_response(&response, &mut output)
                                .is_ok()
                            {
                                let _ = uart.write(output.as_bytes()).await;
                            }
                        }
                    },
                    | Ok(UartRequest::AutoTune(action)) => match action {
                        | AutoTuneAction::Start => {
                            broker::send_command(Command::AutoTuneStart).await;
                            let _ =
                                uart.write(b"ok autotune-starting\r\n").await;
                        },
                        | AutoTuneAction::Stop => {
                            broker::send_command(Command::AutoTuneStop).await;
                            let _ =
                                uart.write(b"ok autotune-stopping\r\n").await;
                        },
                        | AutoTuneAction::Status => {
                            let mut output = TextBuffer::new();
                            if format_autotune_status(
                                broker::try_get_autotune_status(),
                                &mut output,
                            )
                            .is_ok()
                            {
                                let _ = uart.write(output.as_bytes()).await;
                            } else {
                                let _ = uart.write(b"error format\r\n").await;
                            }
                        },
                    },
                    | Ok(UartRequest::Status) => {
                        if let Some(telemetry) = broker::latest_telemetry() {
                            let mut response = TextBuffer::new();
                            if format_telemetry(&telemetry, &mut response)
                                .is_ok()
                            {
                                let _ = uart.write(response.as_bytes()).await;
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
                    decode_can_parameter_envelope(id, frame.data())
                {
                    broker::send_parameter_request(request).await;
                } else if let Some(command) =
                    decode_can_command(id, frame.data())
                {
                    broker::send_command(command).await;
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
            broker::receive_can_parameter_response(),
            Timer::after_millis(10),
        )
        .await
        {
            | Either::First(response) => {
                let Some((id, data)) = encode_can_parameter_response(&response)
                else {
                    continue;
                };
                if let Ok(frame) = can::frame::Frame::new_standard(id, &data) {
                    let _ = transmitter.write(&frame).await;
                }
            },
            | Either::Second(_) => {
                let Some(telemetry) = broker::latest_telemetry() else {
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
