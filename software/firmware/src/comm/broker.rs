use crate::autotune::AutoTuneStatus;
use crate::interfaces::{
    Command, ParameterResponse, ParameterRoute, Telemetry,
};

use super::request::ParameterRequestEnvelope;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use foc_wire::Frame;

static COMMANDS: Channel<CriticalSectionRawMutex, Command, 16> = Channel::new();
static PARAMETER_REQUESTS: Channel<
    CriticalSectionRawMutex,
    ParameterRequestEnvelope,
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
static TELEMETRY_AVAILABLE: AtomicBool = AtomicBool::new(false);
static TELEMETRY: Watch<CriticalSectionRawMutex, Telemetry, 2> = Watch::new();
static AUTOTUNE_STATUS: Watch<CriticalSectionRawMutex, AutoTuneStatus, 4> =
    Watch::new();
static WIRE_REQUESTS: Channel<CriticalSectionRawMutex, Frame, 1> =
    Channel::new();
static WIRE_RESPONSES: Channel<CriticalSectionRawMutex, Frame, 1> =
    Channel::new();

pub(crate) async fn send_wire_request(frame: Frame) {
    WIRE_REQUESTS.send(frame).await;
}

pub fn try_receive_wire_request() -> Option<Frame> {
    WIRE_REQUESTS.try_receive().ok()
}

pub(crate) async fn receive_wire_response() -> Frame {
    WIRE_RESPONSES.receive().await
}

pub fn publish_wire_response(frame: Frame) -> bool {
    WIRE_RESPONSES.try_send(frame).is_ok()
}

pub(crate) async fn send_command(command: Command) {
    COMMANDS.send(command).await;
}

pub(crate) async fn send_parameter_request(request: ParameterRequestEnvelope) {
    PARAMETER_REQUESTS.send(request).await;
}

pub(crate) async fn receive_uart_parameter_response() -> ParameterResponse {
    UART_PARAMETER_RESPONSES.receive().await
}

pub(crate) async fn receive_can_parameter_response() -> ParameterResponse {
    CAN_PARAMETER_RESPONSES.receive().await
}

pub fn latest_telemetry() -> Option<Telemetry> {
    if !TELEMETRY_AVAILABLE.load(Ordering::Acquire) {
        return None;
    }
    TELEMETRY.try_get()
}

pub fn mark_telemetry_unavailable() {
    TELEMETRY_AVAILABLE.store(false, Ordering::Release);
}

pub fn try_receive_command() -> Option<Command> {
    COMMANDS.try_receive().ok()
}

pub fn try_receive_parameter_request() -> Option<ParameterRequestEnvelope> {
    PARAMETER_REQUESTS.try_receive().ok()
}

pub fn publish_parameter_response(response: ParameterResponse) -> bool {
    match response.route {
        | ParameterRoute::Uart => UART_PARAMETER_RESPONSES.try_send(response),
        | ParameterRoute::Can { .. } => {
            CAN_PARAMETER_RESPONSES.try_send(response)
        },
    }
    .is_ok()
}

pub fn publish_telemetry(telemetry: Telemetry) {
    TELEMETRY.sender().send(telemetry);
    TELEMETRY_AVAILABLE.store(true, Ordering::Release);
}

pub fn publish_autotune_status(status: AutoTuneStatus) {
    AUTOTUNE_STATUS.sender().send(status);
}

pub fn try_get_autotune_status() -> Option<AutoTuneStatus> {
    AUTOTUNE_STATUS.try_get()
}
