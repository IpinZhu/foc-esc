use crate::model::{Batch, Parameters, Snapshot, Telemetry, defaults, profile, value};
use crate::simulator::{Simulator, parse_parameter};
use foc_firmware::comm::comm_task::{AutoTuneAction, UartRequest, parse_uart_request};
use foc_firmware::interfaces::{Command, ParameterAction};
use foc_firmware::params::parameters::ParameterId;
use foc_wire::{
    BRIDGE_HELLO, BRIDGE_INFO, Decoder, Frame, HELLO, IDENTITY, MAX_FRAME, REQUEST, RESPONSE,
};
use host_transport::Link;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, oneshot};

type Reply = oneshot::Sender<Result<String, String>>;
struct Call {
    command: String,
    queued: Instant,
    reply: Reply,
}
struct Shared {
    snapshot: Snapshot,
    parameters: Parameters,
    records: Vec<Telemetry>,
}
#[derive(Clone)]
pub struct Session {
    tx: mpsc::Sender<Call>,
    stop_tx: mpsc::Sender<Call>,
    shared: Arc<Mutex<Shared>>,
    touched: Arc<Mutex<Instant>>,
}
impl Session {
    pub async fn connect(
        source: &str,
        port: &str,
        sink: impl Fn(Batch) -> bool + Send + 'static,
    ) -> Result<Self, String> {
        let mut snapshot = Snapshot::new(source);
        let (mut device, parameters) = match source {
            "simulator" => {
                snapshot.identity = "SIMULATOR · STM32G473 / encoder".into();
                snapshot.bridge = "本地模拟 · 无真实硬件".into();
                (Device::Simulator(Box::default()), defaults())
            }
            "serial" => {
                let mut serial = Serial::open(port).await?;
                let bridge = serial.exchange(BRIDGE_HELLO, b"", BRIDGE_INFO).await?;
                if !bridge.split_ascii_whitespace().any(|p| p == "secure=1") {
                    return Err("bridge 未配置加密单播，拒绝开放无线控制".into());
                }
                snapshot.bridge = bridge;
                snapshot.identity = serial.exchange(HELLO, b"", IDENTITY).await?;
                if !snapshot
                    .identity
                    .split_ascii_whitespace()
                    .any(|p| p == "model=stm32g473")
                    || !snapshot
                        .identity
                        .split_ascii_whitespace()
                        .any(|p| p == "lease=500")
                    || !snapshot
                        .identity
                        .split_ascii_whitespace()
                        .any(|p| p == "params=1")
                    || !snapshot
                        .identity
                        .split_ascii_whitespace()
                        .any(|p| p == "capabilities=control,params,autotune")
                    || serial.boot == 0
                {
                    return Err("设备身份或安全协议能力不匹配".into());
                }
                let mut parameters = Parameters::new();
                for id in ParameterId::ALL {
                    let response = serial.command(&format!("param get {}", id.name())).await?;
                    let (name, raw) = parse_parameter(&response).ok_or("参数读回格式错误")?;
                    if name != id.name() {
                        return Err("参数应答身份不匹配".into());
                    }
                    value(id, raw)?;
                    parameters.insert(name.into(), raw);
                }
                profile(&parameters)?;
                (Device::Serial(Box::new(serial)), parameters)
            }
            _ => return Err("不支持的连接方式".into()),
        };
        snapshot.ready = true;
        if let Device::Simulator(sim) = &mut device {
            snapshot.telemetry = Some(sim.sample(0.));
        }
        let shared = Arc::new(Mutex::new(Shared {
            snapshot,
            parameters,
            records: Vec::new(),
        }));
        let touched = Arc::new(Mutex::new(Instant::now()));
        let (tx, rx) = mpsc::channel(8);
        let (stop_tx, stop_rx) = mpsc::channel(1);
        let handle = Self {
            tx,
            stop_tx,
            shared: shared.clone(),
            touched: touched.clone(),
        };
        tokio::spawn(run(device, shared, touched, rx, stop_rx, sink));
        Ok(handle)
    }
    pub fn snapshot(&self) -> Snapshot {
        self.shared.lock().unwrap().snapshot.clone()
    }
    pub fn touch(&self) {
        *self.touched.lock().unwrap() = Instant::now();
    }
    pub async fn command(&self, command: String) -> Result<String, String> {
        if command.len() > foc_wire::MAX_PAYLOAD
            || !command.is_ascii()
            || command.contains(['\r', '\n'])
        {
            return Err("无效命令长度或字符".into());
        }
        let (reply, wait) = oneshot::channel();
        let queue = if matches!(
            command.as_str(),
            "disable" | "release" | "autotune stop" | "__disconnect"
        ) {
            &self.stop_tx
        } else {
            &self.tx
        };
        queue
            .try_send(Call {
                command,
                queued: Instant::now(),
                reply,
            })
            .map_err(|_| "命令队列满或会话已结束")?;
        tokio::time::timeout(Duration::from_secs(3), wait)
            .await
            .map_err(|_| "命令超时，执行结果未知；请查询状态，不要盲目重试".to_string())?
            .map_err(|_| "会话结束，执行结果未知".to_string())?
    }
    pub async fn disconnect(&self) {
        let _ = self.command("__disconnect".into()).await;
    }
    pub async fn read_parameters(&self) -> Result<Parameters, String> {
        for id in ParameterId::ALL {
            self.command(format!("param get {}", id.name())).await?;
        }
        let state = self.shared.lock().unwrap();
        profile(&state.parameters)?;
        Ok(state.parameters.clone())
    }
    pub async fn write_parameter(&self, name: &str, raw: f64) -> Result<String, String> {
        let id = ParameterId::from_name(name).ok_or("未知参数")?;
        value(id, raw)?;
        self.command(format!("param set {name} {raw}")).await
    }
    pub fn set_recording(&self, enabled: bool) -> Result<(), String> {
        let mut state = self.shared.lock().unwrap();
        if enabled && !state.snapshot.ready {
            return Err("设备未就绪".into());
        }
        if enabled {
            state.records.clear();
            state.snapshot.record_count = 0;
        }
        state.snapshot.recording = enabled;
        Ok(())
    }
    pub fn export_csv(&self) -> String {
        let state = self.shared.lock().unwrap();
        let mut csv = String::from(
            "time_s,adc_sequence,state,faults,ia_A,ib_A,ic_A,id_A,iq_A,vbus_V,velocity_rad_s,speed_rpm,temperature_C\n",
        );
        use std::fmt::Write;
        for s in &state.records {
            let _ = writeln!(
                csv,
                "{},{},{},{:08x},{},{},{},{},{},{},{},{},{}",
                s.time,
                s.sequence,
                s.state,
                s.faults,
                s.ia,
                s.ib,
                s.ic,
                s.id,
                s.iq,
                s.vbus,
                s.velocity,
                s.velocity * 60. / std::f64::consts::TAU,
                s.temperature
            );
        }
        csv
    }
}

struct Serial {
    link: Link,
    decoder: Decoder,
    session: u64,
    boot: u64,
    sequence: u32,
    permit: u32,
}
impl Serial {
    async fn open(port: &str) -> Result<Self, String> {
        if port.trim().is_empty() || port.len() > 128 {
            return Err("请选择有效串口".into());
        }
        let mut random = [0; 8];
        getrandom::fill(&mut random).map_err(|e| e.to_string())?;
        let session = u64::from_le_bytes(random).max(1);
        let link = Link::serial(port, 115200)
            .await
            .map_err(|e| e.to_string())?;
        Ok(Self {
            link,
            decoder: Decoder::new(),
            session,
            boot: 0,
            sequence: 0,
            permit: 0,
        })
    }
    async fn exchange(&mut self, kind: u8, body: &[u8], expected: u8) -> Result<String, String> {
        self.sequence = self
            .sequence
            .checked_add(1)
            .ok_or("请求序号已耗尽，请重新连接")?;
        let mut frame = Frame::new(kind, self.session, body).ok_or("命令过长")?;
        frame.boot = self.boot;
        frame.request = self.sequence;
        frame.sequence = self.sequence;
        frame.permit = self.permit;
        let mut bytes = [0; MAX_FRAME];
        let size = frame.encode(&mut bytes);
        let result = tokio::time::timeout(Duration::from_millis(220), async {
            self.link
                .send(&bytes[..size])
                .await
                .map_err(|e| e.to_string())?;
            loop {
                let bytes = self.link.receive().await.map_err(|e| e.to_string())?;
                for byte in bytes {
                    if let Some(response) = self.decoder.push(byte) {
                        if response.session != self.session
                            || response.request != frame.request
                            || response.sequence != frame.sequence
                        {
                            continue;
                        }
                        if response.kind != expected {
                            continue;
                        }
                        if expected == IDENTITY {
                            self.boot = response.boot;
                        } else if expected != BRIDGE_INFO && response.boot != self.boot {
                            return Err("设备已重启，需要重新握手".into());
                        }
                        if expected != BRIDGE_INFO {
                            self.permit = response.permit;
                        }
                        let text = std::str::from_utf8(response.data())
                            .map_err(|_| "设备响应不是有效 ASCII/UTF-8")?
                            .trim()
                            .to_owned();
                        return Ok(text);
                    }
                }
            }
        })
        .await;
        result.map_err(|_| "设备应答超时，执行结果未知".to_string())?
    }
    async fn command(&mut self, command: &str) -> Result<String, String> {
        if command == "claim" {
            let mut random = [0; 8];
            getrandom::fill(&mut random).map_err(|e| e.to_string())?;
            self.session = u64::from_le_bytes(random).max(1);
        }
        let response = self.exchange(REQUEST, command.as_bytes(), RESPONSE).await?;
        if response.starts_with("rejected") || response.starts_with("error") {
            return Err(response);
        }
        let accepted = if command == "status" {
            response.starts_with("seq=") || response == "not-ready"
        } else if command == "autotune status" {
            response.starts_with("at ")
        } else if command == "autotune start" {
            response == "accepted"
        } else if command.starts_with("param get ") {
            response.starts_with("param ")
        } else if command.starts_with("param set ") {
            response.starts_with("ok set ")
        } else if command == "param status" {
            response.starts_with("param-status ")
        } else if command == "param save" {
            response.starts_with("ok saved ") || response.starts_with("ok unchanged ")
        } else if command == "param load" {
            response.starts_with("ok loaded ")
        } else if command == "param defaults" {
            response.starts_with("ok defaults ")
        } else if command == "param factory-reset" {
            response.starts_with("ok factory-reset ")
        } else if command == "release" {
            response == "released"
        } else {
            response == "applied"
        };
        if !accepted {
            return Err(format!("未收到设备执行确认: {response}"));
        }
        if command.starts_with("param get ") || command.starts_with("param set ") {
            let expected = command
                .split_ascii_whitespace()
                .nth(2)
                .ok_or("缺少参数名")?;
            let (name, raw) = parse_parameter(&response).ok_or("参数应答格式错误")?;
            if name != expected {
                return Err("参数应答身份不匹配".into());
            }
            value(ParameterId::from_name(name).ok_or("未知应答参数")?, raw)?;
        }
        Ok(response)
    }
}

enum Device {
    Simulator(Box<Simulator>),
    Serial(Box<Serial>),
}
impl Device {
    async fn execute(&mut self, command: &str) -> Result<String, String> {
        match self {
            Self::Simulator(sim) => sim.execute(command),
            Self::Serial(serial) => serial.command(command).await,
        }
    }
    fn tuning(&self) -> bool {
        matches!(self, Self::Simulator(sim) if sim.tuning())
    }
}

fn read_only(request: &UartRequest) -> bool {
    matches!(
        request,
        UartRequest::Status
            | UartRequest::AutoTune(AutoTuneAction::Status)
            | UartRequest::Parameter(ParameterAction::Get(_) | ParameterAction::Status)
    )
}
fn validate(command: &str, state: &Shared, tuning: bool) -> Result<(), String> {
    if !state.snapshot.ready {
        return Err("设备未就绪，请重新连接".into());
    }
    if command == "claim" {
        if state.snapshot.controlled {
            return Err("已取得控制权".into());
        }
        if !state
            .snapshot
            .telemetry
            .as_ref()
            .is_some_and(|t| t.state == 1 && t.faults == 0)
            || tuning
        {
            return Err("请等待电机空闲、无故障且整定停止".into());
        }
        return Ok(());
    }
    if command == "disable" || command == "autotune stop" || command == "release" {
        return Ok(());
    }
    let request = parse_uart_request(command).map_err(|e| format!("无效命令: {e:?}"))?;
    if read_only(&request) {
        return Ok(());
    }
    if !state.snapshot.controlled {
        return Err("请先取得控制权".into());
    }
    let telemetry = state.snapshot.telemetry.as_ref().ok_or("等待新鲜遥测")?;
    if state.snapshot.packet_age_ms.is_none_or(|age| age > 250) {
        return Err("遥测已过期，禁止控制".into());
    }
    let config = profile(&state.parameters)?;
    match request {
        UartRequest::Command(Command::Enable(_)) => {
            if telemetry.state != 1 || telemetry.faults != 0 || tuning {
                return Err("使能需要空闲、无故障、未整定".into());
            }
        }
        UartRequest::Command(Command::SetIq(v)) if v.abs() > config.current_limit => {
            return Err("Iq 超出设备电流限制".into());
        }
        UartRequest::Command(Command::SetVelocity(v)) if v.abs() > config.maximum_velocity => {
            return Err("速度超出设备限制".into());
        }
        UartRequest::Command(Command::SetOpenLoop {
            electrical_velocity,
            q_voltage,
        }) if electrical_velocity.abs() > config.maximum_open_loop_velocity
            || q_voltage.abs() as f64 > telemetry.vbus / 2. =>
        {
            return Err("开环目标超出设备/母线限制".into());
        }
        UartRequest::Command(
            Command::SetCellCount(_)
            | Command::SetElectricalZero(_)
            | Command::SetCurrentPid(_)
            | Command::SetVelocityPid(_),
        ) => return Err("配置请使用参数页面".into()),
        UartRequest::Parameter(action) => {
            if telemetry.state != 1 && telemetry.state != 5 || tuning {
                return Err("参数修改/持久化需要停机".into());
            }
            if let ParameterAction::Set(id, v) = action {
                let mut candidate = config;
                candidate
                    .set(id, v)
                    .map_err(|e| format!("参数无效: {e:?}"))?;
            }
        }
        UartRequest::AutoTune(AutoTuneAction::Start)
            if telemetry.state != 1 || telemetry.faults != 0 || tuning =>
        {
            return Err("整定需要空闲且无故障".into());
        }
        UartRequest::ParameterShow => return Err("请逐项读取参数".into()),
        _ => {}
    }
    Ok(())
}

async fn run(
    mut device: Device,
    shared: Arc<Mutex<Shared>>,
    touched: Arc<Mutex<Instant>>,
    mut rx: mpsc::Receiver<Call>,
    mut stop_rx: mpsc::Receiver<Call>,
    sink: impl Fn(Batch) -> bool + Send + 'static,
) {
    let start = Instant::now();
    let mut last_packet = Instant::now();
    let mut heartbeat = Instant::now();
    let mut tune_poll = Instant::now();
    let mut disconnect_reply = None;
    let mut tick = tokio::time::interval(Duration::from_millis(50));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        {
            let mut state = shared.lock().unwrap();
            state.snapshot.packet_age_ms = Some(last_packet.elapsed().as_millis() as u64);
            let silent = touched.lock().unwrap().elapsed();
            if silent > Duration::from_millis(1500)
                || (state.snapshot.controlled && silent > Duration::from_millis(500))
            {
                state.snapshot.error = Some("界面心跳中断，控制会话已关闭；请确认设备停机".into());
                break;
            }
        }
        tokio::select! {
            call = async {
                tokio::select! { biased;
                    call = stop_rx.recv() => call,
                    call = rx.recv() => call,
                }
            } => {
                let Some(call) = call else { break; };
                let stopping = matches!(call.command.as_str(), "disable" | "release" | "autotune stop" | "__disconnect");
                if stopping {
                    while let Ok(pending) = rx.try_recv() { let _ = pending.reply.send(Err("停止请求已取消排队命令".into())); }
                }
                if call.command == "__disconnect" { disconnect_reply = Some(call.reply); break; }
                if call.reply.is_closed() || (!stopping && call.queued.elapsed() > Duration::from_millis(200)) {
                    let _ = call.reply.send(Err("排队命令已过期，未发送".into())); continue;
                }
                let valid = { let state = shared.lock().unwrap(); validate(&call.command, &state, device.tuning()) };
                let result = match valid { Ok(()) => device.execute(&call.command).await, Err(e) => Err(e) };
                if let Ok(response) = &result {
                    let mut state = shared.lock().unwrap();
                    if call.command == "claim" { state.snapshot.controlled = true; heartbeat = Instant::now(); }
                    if call.command == "release" { state.snapshot.controlled = false; }
                    if let Some((name, v)) = parse_parameter(response) { state.parameters.insert(name.into(), v); }
                    if matches!(call.command.as_str(), "param load" | "param defaults" | "param factory-reset") {
                        state.parameters.clear();
                    }
                    if call.command.starts_with("autotune") { state.snapshot.autotune = response.clone(); }
                    state.snapshot.error = None;
                } else if let Err(e) = &result { shared.lock().unwrap().snapshot.error = Some(e.clone()); }
                let _ = call.reply.send(result);
            }
            _ = tick.tick() => {
                let controlled = shared.lock().unwrap().snapshot.controlled;
                if controlled && touched.lock().unwrap().elapsed() > Duration::from_millis(500) {
                    shared.lock().unwrap().snapshot.error = Some("界面心跳中断，控制会话已关闭；请确认设备停机".into());
                    break;
                } else if controlled && heartbeat.elapsed() >= Duration::from_millis(100) {
                    let result = match &mut device { Device::Simulator(_) => Ok("applied".into()), Device::Serial(serial) => serial.command("heartbeat").await };
                    heartbeat = Instant::now();
                    if let Err(error) = result { shared.lock().unwrap().snapshot.error = Some(error); break; }
                }
                let time = start.elapsed().as_secs_f64();
                let result = match &mut device {
                    Device::Simulator(sim) => Ok(Some(sim.sample(time))),
                    Device::Serial(serial) => serial.command("status").await.and_then(|line| {
                        if line == "not-ready" { Ok(None) } else { Telemetry::parse(&line, time).map(Some) }
                    }),
                };
                let mut samples = Vec::new();
                match result {
                    Ok(Some(sample)) => {
                        last_packet = Instant::now();
                        let mut state = shared.lock().unwrap(); state.snapshot.telemetry = Some(sample.clone());
                        if state.snapshot.recording {
                            if state.records.len() < 12000 { state.records.push(sample.clone()); }
                            else { state.snapshot.recording = false; state.snapshot.dropped += 1; state.snapshot.error = Some("已达到 12000 帧录制上限，请导出数据".into()); }
                        }
                        state.snapshot.record_count = state.records.len(); samples.push(sample);
                    }
                    Ok(None) => { shared.lock().unwrap().snapshot.telemetry = None; }
                    Err(error) => { shared.lock().unwrap().snapshot.error = Some(error); break; }
                }
                if tune_poll.elapsed() >= Duration::from_millis(500) {
                    tune_poll = Instant::now();
                    match device.execute("autotune status").await {
                        Ok(status) => shared.lock().unwrap().snapshot.autotune = status,
                        Err(e) => { shared.lock().unwrap().snapshot.error = Some(e); }
                    }
                }
                let snapshot = { let mut state = shared.lock().unwrap(); state.snapshot.packet_age_ms = Some(last_packet.elapsed().as_millis() as u64); state.snapshot.clone() };
                if !sink(Batch { snapshot, samples }) { break; }
            }
        }
        if shared.lock().unwrap().snapshot.controlled
            && heartbeat.elapsed() >= Duration::from_millis(100)
        {
            let result = match &mut device {
                Device::Simulator(_) => Ok(String::new()),
                Device::Serial(serial) => serial.command("heartbeat").await,
            };
            heartbeat = Instant::now();
            if let Err(error) = result {
                shared.lock().unwrap().snapshot.error = Some(error);
                break;
            }
        }
    }
    let _ = device.execute("disable").await;
    let _ = device.execute("release").await;
    if let Device::Serial(serial) = device {
        let _ = serial.link.close().await;
    }
    let snapshot = {
        let mut state = shared.lock().unwrap();
        state.snapshot.connected = false;
        state.snapshot.ready = false;
        state.snapshot.controlled = false;
        state.snapshot.recording = false;
        state.snapshot.packet_age_ms = Some(last_packet.elapsed().as_millis() as u64);
        state.snapshot.clone()
    };
    let _ = sink(Batch {
        snapshot,
        samples: Vec::new(),
    });
    if let Some(reply) = disconnect_reply {
        let _ = reply.send(Ok("disconnected".into()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn no_control_before_explicit_claim_and_fresh_telemetry() {
        let mut snapshot = Snapshot::new("simulator");
        snapshot.ready = true;
        snapshot.telemetry = Some(Simulator::default().sample(0.));
        snapshot.packet_age_ms = Some(0);
        let mut state = Shared {
            snapshot,
            parameters: defaults(),
            records: Vec::new(),
        };
        assert!(validate("enable current", &state, false).is_err());
        assert!(validate("claim", &state, false).is_ok());
        state.snapshot.controlled = true;
        assert!(validate("enable current", &state, false).is_ok());
        assert!(validate("iq NaN", &state, false).is_err());
        assert!(validate("iq 999", &state, false).is_err());
        state.snapshot.packet_age_ms = Some(251);
        assert!(validate("enable current", &state, false).is_err());
        assert!(validate("disable", &state, false).is_ok());
    }
    #[tokio::test]
    async fn framed_transport_discards_late_responses_and_rejects_queued_ack() {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let peer = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let mut decoder = Decoder::new();
            let mut bytes = [0; MAX_FRAME];
            for text in ["applied", "ok"] {
                let request = loop {
                    let byte = stream.read_u8().await.unwrap();
                    if let Some(frame) = decoder.push(byte) {
                        break frame;
                    }
                };
                let mut response = Frame::new(RESPONSE, request.session, text.as_bytes()).unwrap();
                response.boot = request.boot;
                response.request = request.request;
                response.sequence = request.sequence;
                response.permit = 123;
                let mut late = response;
                late.session ^= 1;
                let size = late.encode(&mut bytes);
                stream.write_all(&bytes[..size]).await.unwrap();
                let size = response.encode(&mut bytes);
                for part in bytes[..size].chunks(7) {
                    stream.write_all(part).await.unwrap();
                }
            }
        });
        let link = Link::tcp(&address.to_string()).await.unwrap();
        let mut serial = Serial {
            link,
            decoder: Decoder::new(),
            session: 77,
            boot: 99,
            sequence: 0,
            permit: 0,
        };
        assert_eq!(serial.command("disable").await.unwrap(), "applied");
        assert_eq!(serial.permit, 123);
        assert!(
            serial
                .command("enable current")
                .await
                .unwrap_err()
                .contains("执行确认")
        );
        peer.await.unwrap();
    }
    #[tokio::test]
    async fn unavailable_tuning_telemetry_does_not_cancel_control_lease() {
        use tokio::io::{AsyncReadExt, AsyncWriteExt};
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let peer = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let mut decoder = Decoder::new();
            let mut bytes = [0; MAX_FRAME];
            while let Ok(byte) = stream.read_u8().await {
                if let Some(request) = decoder.push(byte) {
                    let text = match request.data() {
                        b"status" => "not-ready",
                        b"release" => "released",
                        b"autotune status" => "at state=identifying progress=20 error=none",
                        _ => "applied",
                    };
                    let mut reply = Frame::new(RESPONSE, request.session, text.as_bytes()).unwrap();
                    reply.boot = 99;
                    reply.request = request.request;
                    reply.sequence = request.sequence;
                    let size = reply.encode(&mut bytes);
                    stream.write_all(&bytes[..size]).await.unwrap();
                }
            }
        });
        let link = Link::tcp(&address.to_string()).await.unwrap();
        let serial = Serial {
            link,
            decoder: Decoder::new(),
            session: 77,
            boot: 99,
            sequence: 0,
            permit: 0,
        };
        let mut snapshot = Snapshot::new("serial");
        snapshot.ready = true;
        snapshot.controlled = true;
        let shared = Arc::new(Mutex::new(Shared {
            snapshot,
            parameters: defaults(),
            records: Vec::new(),
        }));
        let touched = Arc::new(Mutex::new(Instant::now()));
        let (tx, rx) = mpsc::channel(8);
        let (stop_tx, stop_rx) = mpsc::channel(1);
        let session = Session {
            tx,
            stop_tx,
            shared: shared.clone(),
            touched: touched.clone(),
        };
        tokio::spawn(run(
            Device::Serial(Box::new(serial)),
            shared,
            touched,
            rx,
            stop_rx,
            |_| true,
        ));
        tokio::time::sleep(Duration::from_millis(180)).await;
        assert!(session.snapshot().connected);
        assert!(session.snapshot().controlled);
        assert!(session.snapshot().telemetry.is_none());
        session.disconnect().await;
        peer.await.unwrap();
    }
    #[tokio::test]
    async fn simulator_revokes_control_when_frontend_stops_touching() {
        let session = Session::connect("simulator", "", |_| true).await.unwrap();
        tokio::time::sleep(Duration::from_millis(60)).await;
        session.command("claim".into()).await.unwrap();
        session.command("enable current".into()).await.unwrap();
        tokio::time::sleep(Duration::from_millis(600)).await;
        assert!(!session.snapshot().controlled);
        assert!(!session.snapshot().connected);
    }
    #[tokio::test]
    async fn stop_cancels_queued_enable_requests() {
        let session = Session::connect("simulator", "", |_| true).await.unwrap();
        session.command("claim".into()).await.unwrap();
        let mut pending = Vec::new();
        for _ in 0..8 {
            let (reply, wait) = oneshot::channel();
            assert!(
                session
                    .tx
                    .try_send(Call {
                        command: "enable current".into(),
                        queued: Instant::now(),
                        reply
                    })
                    .is_ok()
            );
            pending.push(wait);
        }
        session.command("disable".into()).await.unwrap();
        for wait in pending {
            assert!(wait.await.unwrap().unwrap_err().contains("取消"));
        }
        tokio::time::sleep(Duration::from_millis(60)).await;
        assert_eq!(session.snapshot().telemetry.unwrap().state, 1);
        session.disconnect().await;
        assert!(!session.snapshot().connected);
    }
    #[tokio::test]
    async fn readonly_recording_is_independent_of_control_ownership() {
        let session = Session::connect("simulator", "", |_| true).await.unwrap();
        session.set_recording(true).unwrap();
        tokio::time::sleep(Duration::from_millis(60)).await;
        assert!(!session.snapshot().controlled);
        session.disconnect().await;
        assert!(session.export_csv().lines().count() > 1);
    }
    #[tokio::test]
    async fn simulator_recording_and_disconnect() {
        let session = Session::connect("simulator", "", |_| true).await.unwrap();
        session.touch();
        tokio::time::sleep(Duration::from_millis(60)).await;
        session.command("claim".into()).await.unwrap();
        session.command("enable current".into()).await.unwrap();
        session.set_recording(true).unwrap();
        tokio::time::sleep(Duration::from_millis(70)).await;
        assert!(session.export_csv().lines().count() > 1);
        session.disconnect().await;
    }
}
