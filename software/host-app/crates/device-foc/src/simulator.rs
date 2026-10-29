use crate::model::{Parameters, Telemetry, defaults};
use foc_firmware::comm::comm_task::{AutoTuneAction, UartRequest, parse_uart_request};
use foc_firmware::interfaces::{Command, ControlMode, ParameterAction};
use foc_firmware::params::parameters::{ParameterId, ParameterProfileV1};
use std::time::Instant;

pub struct Simulator {
    pub parameters: Parameters,
    profile: ParameterProfileV1,
    pub state: u8,
    iq: f64,
    velocity: f64,
    target_iq: f64,
    target_velocity: f64,
    tune: Option<Instant>,
    dirty: bool,
    generation: u32,
    saved: Parameters,
    sequence: u32,
}
impl Default for Simulator {
    fn default() -> Self {
        Self {
            parameters: defaults(),
            profile: ParameterProfileV1::default(),
            state: 1,
            iq: 0.,
            velocity: 0.,
            target_iq: 0.,
            target_velocity: 0.,
            tune: None,
            dirty: false,
            generation: 0,
            saved: defaults(),
            sequence: 0,
        }
    }
}
impl Simulator {
    pub fn stop(&mut self) {
        self.tune = None;
        self.state = 1;
        self.target_iq = 0.;
        self.target_velocity = 0.;
    }
    pub fn tuning(&self) -> bool {
        self.tune.is_some()
    }
    pub fn tune_status(&self) -> String {
        if let Some(start) = self.tune {
            let progress = (start.elapsed().as_secs_f64() / 8. * 100.).min(99.) as u32;
            format!(
                "at state=identifying error=none progress={progress} rs=None ld=None flux=None commissioned=0"
            )
        } else {
            "at state=idle error=none progress=0 commissioned=0".into()
        }
    }
    pub fn sample(&mut self, time: f64) -> Telemetry {
        self.sequence = self.sequence.wrapping_add(5000);
        if self
            .tune
            .is_some_and(|start| start.elapsed().as_secs() >= 8)
        {
            self.stop();
        }
        let target = match self.state {
            4 => self.target_velocity,
            2 => self.target_velocity / 4.,
            3 => self.target_iq * 8.,
            _ => 0.,
        };
        self.velocity += (target - self.velocity) * 0.12;
        let q = if self.state > 1 {
            if self.state == 4 {
                (target - self.velocity).mul_add(0.1, 0.3).clamp(-8., 8.)
            } else {
                self.target_iq
            }
        } else {
            0.
        };
        self.iq += (q - self.iq) * 0.2;
        let a = self.iq * (time * 10.).sin();
        let b = self.iq * (time * 10. - 2.094395).sin();
        Telemetry {
            sequence: self.sequence,
            state: self.state,
            faults: 0,
            ia: a,
            ib: b,
            ic: -a - b,
            id: 0.,
            iq: self.iq,
            vbus: 24. + (time * 0.8).sin() * 0.03,
            velocity: self.velocity,
            temperature: 32. + self.iq.abs() * 0.4,
            time,
        }
    }
    pub fn execute(&mut self, command: &str) -> Result<String, String> {
        if command == "claim" {
            return Ok("applied".into());
        }
        if command == "release" {
            self.stop();
            return Ok("released".into());
        }
        let request = parse_uart_request(command).map_err(|e| format!("{e:?}"))?;
        match request {
            UartRequest::Command(cmd) => match cmd {
                Command::Enable(mode) => {
                    if self.tune.is_some() {
                        return Err("rejected tuning".into());
                    }
                    self.state = match mode {
                        ControlMode::OpenLoop => 2,
                        ControlMode::Current => 3,
                        ControlMode::Velocity => 4,
                        _ => 1,
                    };
                }
                Command::Disable => self.stop(),
                Command::ClearFault => self.state = 1,
                Command::SetIq(v) => self.target_iq = v as f64,
                Command::SetVelocity(v) => self.target_velocity = v as f64,
                Command::SetOpenLoop {
                    electrical_velocity,
                    q_voltage,
                } => {
                    self.target_velocity = electrical_velocity as f64;
                    self.target_iq = q_voltage as f64;
                }
                _ => return Err("请在参数页面修改配置".into()),
            },
            UartRequest::Parameter(action) => {
                return match action {
                    ParameterAction::Get(id) => Ok(format!(
                        "param {}={}",
                        id.name(),
                        self.parameters[id.name()]
                    )),
                    ParameterAction::Set(id, typed) => {
                        self.profile
                            .set(id, typed)
                            .map_err(|e| format!("rejected {e:?}"))?;
                        let v = crate::model::number(self.profile.get(id));
                        self.parameters.insert(id.name().into(), v);
                        self.dirty = true;
                        Ok(format!(
                            "ok set {}={} dirty=1 restart-required={}",
                            id.name(),
                            v,
                            u8::from(id.startup_only())
                        ))
                    }
                    ParameterAction::Status => Ok(format!(
                        "param-status valid=1 source=flash generation={} dirty={} restart-required=0",
                        self.generation,
                        u8::from(self.dirty)
                    )),
                    ParameterAction::Save => {
                        self.saved = self.parameters.clone();
                        self.dirty = false;
                        self.generation += 1;
                        Ok(format!("ok saved generation={}", self.generation))
                    }
                    ParameterAction::Load => {
                        self.parameters = self.saved.clone();
                        self.profile = crate::model::profile(&self.parameters)?;
                        self.dirty = false;
                        Ok(format!("ok loaded generation={}", self.generation))
                    }
                    ParameterAction::Defaults | ParameterAction::FactoryReset => {
                        self.parameters = defaults();
                        self.profile = ParameterProfileV1::default();
                        self.dirty = true;
                        if matches!(action, ParameterAction::FactoryReset) {
                            self.saved = self.parameters.clone();
                            self.generation += 1;
                            self.dirty = false;
                        }
                        Ok("ok defaults".into())
                    }
                };
            }
            UartRequest::AutoTune(action) => {
                return match action {
                    AutoTuneAction::Start => {
                        if self.state != 1 || self.tune.is_some() {
                            return Err("rejected busy".into());
                        }
                        self.tune = Some(Instant::now());
                        Ok("accepted".into())
                    }
                    AutoTuneAction::Stop => {
                        self.stop();
                        Ok("applied".into())
                    }
                    AutoTuneAction::Status => Ok(self.tune_status()),
                };
            }
            UartRequest::Status => return Err("内部遥测请求".into()),
            UartRequest::ParameterShow => return Err("使用逐项参数读取".into()),
        }
        Ok("applied".into())
    }
}

pub fn parse_parameter(line: &str) -> Option<(&str, f64)> {
    let rest = line
        .strip_prefix("param ")
        .or_else(|| line.strip_prefix("ok set "))?;
    let token = rest.split_ascii_whitespace().next()?;
    let (key, value) = token.split_once('=')?;
    ParameterId::from_name(key)?;
    let value: f64 = value.parse().ok()?;
    value.is_finite().then_some((key, value))
}
