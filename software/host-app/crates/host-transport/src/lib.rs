//! Transport adapters. Stream, datagram, report and GATT boundaries remain explicit.
//! Opening a link does not imply a device protocol handshake has succeeded.
use btleplug::{
    api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType},
    platform::{Manager, Peripheral},
};
use futures_util::StreamExt;
use serde::Serialize;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpStream, UdpSocket},
    sync::mpsc,
};
use tokio_serial::SerialPortBuilderExt;

type Error = Box<dyn std::error::Error + Send + Sync>;
pub type Result<T> = std::result::Result<T, Error>;
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Endpoint {
    pub id: String,
    pub name: String,
    pub kind: String,
}
#[derive(Debug, Clone, Copy)]
pub enum Boundary {
    Stream,
    Datagram,
    HidReport,
    GattNotification,
}
pub enum Link {
    Serial(tokio_serial::SerialStream),
    Tcp(TcpStream),
    Udp(UdpSocket),
    Hid(Arc<Mutex<hidapi::HidDevice>>),
    Ble {
        peripheral: Peripheral,
        write: btleplug::api::Characteristic,
        receiver: mpsc::Receiver<Vec<u8>>,
        task: tokio::task::JoinHandle<()>,
    },
}
impl Link {
    pub fn boundary(&self) -> Boundary {
        match self {
            Self::Serial(_) | Self::Tcp(_) => Boundary::Stream,
            Self::Udp(_) => Boundary::Datagram,
            Self::Hid(_) => Boundary::HidReport,
            Self::Ble { .. } => Boundary::GattNotification,
        }
    }
    pub async fn serial(path: &str, baud: u32) -> Result<Self> {
        Ok(Self::Serial(
            tokio_serial::new(path, baud).open_native_async()?,
        ))
    }
    pub async fn tcp(address: &str) -> Result<Self> {
        Ok(Self::Tcp(
            tokio::time::timeout(Duration::from_secs(5), TcpStream::connect(address)).await??,
        ))
    }
    pub async fn udp(bind: &str, peer: &str) -> Result<Self> {
        let socket = UdpSocket::bind(bind).await?;
        socket.connect(peer).await?;
        Ok(Self::Udp(socket))
    }
    pub async fn hid(path: String) -> Result<Self> {
        let device = tokio::task::spawn_blocking(move || -> Result<_> {
            let api = hidapi::HidApi::new()?;
            let path = std::ffi::CString::new(path)?;
            Ok(api.open_path(&path)?)
        })
        .await??;
        Ok(Self::Hid(Arc::new(Mutex::new(device))))
    }
    /// The device adapter selects characteristics and owns GATT fragmentation.
    pub async fn ble(
        peripheral: Peripheral,
        write: btleplug::api::Characteristic,
        notify: btleplug::api::Characteristic,
    ) -> Result<Self> {
        peripheral.connect().await?;
        let setup = async {
            peripheral.discover_services().await?;
            let mut notifications = peripheral.notifications().await?;
            peripheral.subscribe(&notify).await?;
            let (sender, receiver) = mpsc::channel(128);
            let task = tokio::spawn(async move {
                while let Some(value) = notifications.next().await {
                    if value.uuid == notify.uuid && sender.try_send(value.value).is_err() {
                        break;
                    }
                }
            });
            Ok::<_, Error>((receiver, task))
        }
        .await;
        match setup {
            Ok((receiver, task)) => Ok(Self::Ble {
                peripheral,
                write,
                receiver,
                task,
            }),
            Err(e) => {
                let _ = peripheral.disconnect().await;
                Err(e)
            }
        }
    }
    pub async fn send(&mut self, bytes: &[u8]) -> Result<()> {
        if bytes.is_empty() || bytes.len() > 65507 {
            return Err("invalid transfer size".into());
        }
        match self {
            Self::Serial(stream) => stream.write_all(bytes).await?,
            Self::Tcp(stream) => stream.write_all(bytes).await?,
            Self::Udp(socket) => {
                socket.send(bytes).await?;
            }
            Self::Hid(device) => {
                // First byte is HID report ID, including 0 for unnumbered reports.
                let device = device.clone();
                let bytes = bytes.to_vec();
                tokio::task::spawn_blocking(move || -> Result<()> {
                    let count = device
                        .lock()
                        .map_err(|_| "HID lock poisoned")?
                        .write(&bytes)?;
                    if count != bytes.len() {
                        return Err("partial HID write".into());
                    }
                    Ok(())
                })
                .await??;
            }
            Self::Ble {
                peripheral, write, ..
            } => {
                peripheral
                    .write(write, bytes, WriteType::WithResponse)
                    .await?
            }
        }
        Ok(())
    }
    pub async fn receive(&mut self) -> Result<Vec<u8>> {
        let mut buffer = vec![0; 65536];
        let count = match self {
            Self::Serial(stream) => stream.read(&mut buffer).await?,
            Self::Tcp(stream) => stream.read(&mut buffer).await?,
            Self::Udp(socket) => socket.recv(&mut buffer).await?,
            Self::Hid(device) => {
                let device = device.clone();
                return tokio::task::spawn_blocking(move || -> Result<Vec<u8>> {
                    let mut buffer = vec![0; 65536];
                    let count = device
                        .lock()
                        .map_err(|_| "HID lock poisoned")?
                        .read_timeout(&mut buffer, 1000)?;
                    if count == 0 {
                        return Err("HID read timeout".into());
                    }
                    buffer.truncate(count);
                    Ok(buffer)
                })
                .await?;
            }
            Self::Ble { receiver, .. } => {
                return receiver
                    .recv()
                    .await
                    .ok_or_else(|| "BLE stream ended or overflowed".into())
            }
        };
        if count == 0 && matches!(self, Self::Tcp(_) | Self::Serial(_)) {
            return Err("connection closed".into());
        }
        buffer.truncate(count);
        Ok(buffer)
    }
    pub async fn close(self) -> Result<()> {
        if let Self::Ble {
            peripheral, task, ..
        } = self
        {
            task.abort();
            peripheral.disconnect().await?;
        }
        Ok(())
    }
}
pub async fn discover(kind: &str) -> Result<Vec<Endpoint>> {
    match kind {
        "serial" => Ok(tokio_serial::available_ports()?
            .into_iter()
            .map(|p| Endpoint {
                name: p.port_name.clone(),
                id: p.port_name,
                kind: "serial".into(),
            })
            .collect()),
        "usb" => {
            tokio::task::spawn_blocking(|| -> Result<Vec<Endpoint>> {
                let api = hidapi::HidApi::new()?;
                Ok(api
                    .device_list()
                    .map(|d| Endpoint {
                        id: d.path().to_string_lossy().into_owned(),
                        name: format!(
                            "{} · {:04x}:{:04x}",
                            d.product_string().unwrap_or("USB HID"),
                            d.vendor_id(),
                            d.product_id()
                        ),
                        kind: "usb".into(),
                    })
                    .collect())
            })
            .await?
        }
        "ble" => {
            let manager = Manager::new().await?;
            let mut found = Vec::new();
            for adapter in manager.adapters().await? {
                adapter.start_scan(ScanFilter::default()).await?;
                tokio::time::sleep(Duration::from_secs(3)).await;
                let peripherals = adapter.peripherals().await;
                adapter.stop_scan().await?;
                for p in peripherals? {
                    if let Some(properties) = p.properties().await? {
                        found.push(Endpoint {
                            id: p.id().to_string(),
                            name: properties.local_name.unwrap_or_else(|| p.id().to_string()),
                            kind: "ble".into(),
                        });
                    }
                }
            }
            Ok(found)
        }
        _ => Ok(Vec::new()),
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn tcp_roundtrip_preserves_bytes() {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(async {
                let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
                let address = listener.local_addr().unwrap();
                let server = tokio::spawn(async move {
                    let (mut peer, _) = listener.accept().await.unwrap();
                    let mut b = [0; 4];
                    peer.read_exact(&mut b).await.unwrap();
                    peer.write_all(&b).await.unwrap();
                });
                let mut link = Link::tcp(&address.to_string()).await.unwrap();
                link.send(&[0, 1, 254, 255]).await.unwrap();
                let mut response = Vec::new();
                while response.len() < 4 {
                    response.extend(link.receive().await.unwrap());
                }
                assert_eq!(response, [0, 1, 254, 255]);
                server.await.unwrap();
            });
    }
}
