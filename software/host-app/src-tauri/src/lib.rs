use device_foc::{Batch, ParameterDescriptor, Parameters, Session, Snapshot};
use tauri::{ipc::Channel, Manager, State};
use tokio::sync::Mutex;

#[derive(Default)]
struct Store(Mutex<Option<Session>>);
async fn current(store: &Store) -> Result<Session, String> {
    store.0.lock().await.clone().ok_or("设备未连接".into())
}
#[tauri::command]
async fn connect(
    source: String,
    port: String,
    channel: Channel<Batch>,
    store: State<'_, Store>,
) -> Result<Snapshot, String> {
    let mut guard = store.0.lock().await;
    if let Some(previous) = guard.take() {
        previous.disconnect().await;
    }
    let session =
        Session::connect(&source, &port, move |batch| channel.send(batch).is_ok()).await?;
    let snapshot = session.snapshot();
    *guard = Some(session);
    Ok(snapshot)
}
#[tauri::command]
async fn disconnect(store: State<'_, Store>) -> Result<(), String> {
    current(&store).await?.disconnect().await;
    Ok(())
}
#[tauri::command]
async fn command(command: String, store: State<'_, Store>) -> Result<String, String> {
    current(&store).await?.command(command).await
}
#[tauri::command]
fn parameter_descriptors() -> Vec<ParameterDescriptor> {
    device_foc::descriptors()
}
#[tauri::command]
async fn read_parameters(store: State<'_, Store>) -> Result<Parameters, String> {
    current(&store).await?.read_parameters().await
}
#[tauri::command]
async fn write_parameter(
    name: String,
    value: f64,
    store: State<'_, Store>,
) -> Result<String, String> {
    current(&store).await?.write_parameter(&name, value).await
}
#[tauri::command]
async fn set_recording(enabled: bool, store: State<'_, Store>) -> Result<(), String> {
    current(&store).await?.set_recording(enabled)
}
#[tauri::command]
async fn export_csv(store: State<'_, Store>) -> Result<String, String> {
    Ok(current(&store).await?.export_csv())
}
#[tauri::command]
async fn save_export(
    app: tauri::AppHandle,
    kind: String,
    contents: String,
) -> Result<String, String> {
    if contents.len() > 16 * 1024 * 1024 {
        return Err("导出内容超过 16 MB".into());
    }
    let ext = match kind.as_str() {
        "csv" => "csv",
        "json" => "json",
        _ => return Err("不支持的导出类型".into()),
    };
    let directory = app.path().download_dir().map_err(|e| e.to_string())?;
    tauri::async_runtime::spawn_blocking(move || {
        std::fs::create_dir_all(&directory).map_err(|e| e.to_string())?;
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_secs();
        let stem = if ext == "csv" {
            "foc-telemetry"
        } else {
            "foc-parameters"
        };
        let mut path = directory.join(format!("{stem}-{stamp}.{ext}"));
        let mut suffix = 1;
        while path.exists() {
            path = directory.join(format!("{stem}-{stamp}-{suffix}.{ext}"));
            suffix += 1;
        }
        std::fs::write(&path, contents).map_err(|e| e.to_string())?;
        Ok::<String, String>(path.display().to_string())
    })
    .await
    .map_err(|e| e.to_string())?
}
#[tauri::command]
async fn touch(store: State<'_, Store>) -> Result<(), String> {
    current(&store).await?.touch();
    Ok(())
}
#[tauri::command]
async fn discover() -> Result<Vec<host_transport::Endpoint>, String> {
    host_transport::discover("serial")
        .await
        .map_err(|e| e.to_string())
}
pub fn run() {
    tauri::Builder::default()
        .manage(Store::default())
        .invoke_handler(tauri::generate_handler![
            connect,
            disconnect,
            command,
            parameter_descriptors,
            read_parameters,
            write_parameter,
            set_recording,
            export_csv,
            save_export,
            touch,
            discover
        ])
        .run(tauri::generate_context!())
        .expect("启动 FOC Studio 失败");
}
