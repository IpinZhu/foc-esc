use chrono::Local;
use serde::Deserialize;
use std::{
    fs::{self, File},
    io::{self, Write},
    path::{Path, PathBuf},
    sync::Mutex,
};
use tauri::{Manager, State};

#[derive(Deserialize)]
#[serde(rename_all = "lowercase")]
pub(crate) enum Level {
    Info,
    Success,
    Warning,
    Error,
}
impl Level {
    fn label(&self) -> &'static str {
        match self {
            Self::Info => "INFO",
            Self::Success => "SUCCESS",
            Self::Warning => "WARN",
            Self::Error => "ERROR",
        }
    }
}

struct RunFile {
    path: PathBuf,
    file: Mutex<File>,
}
impl RunFile {
    fn create(dir: &Path) -> io::Result<Self> {
        fs::create_dir_all(dir)?;
        let name = format!(
            "foc-studio-{}-{}.log",
            Local::now().format("%Y%m%d-%H%M%S%.3f"),
            std::process::id()
        );
        let path = dir.join(name);
        let file = File::create_new(&path)?;
        let run = Self {
            path,
            file: Mutex::new(file),
        };
        run.write(
            Level::Info,
            "desktop",
            &format!(
                "FOC Studio {} 启动 · {} / {}",
                env!("CARGO_PKG_VERSION"),
                std::env::consts::OS,
                std::env::consts::ARCH
            ),
        )?;
        Ok(run)
    }
    fn write(&self, level: Level, source: &str, message: &str) -> io::Result<()> {
        let mut file = self
            .file
            .lock()
            .map_err(|_| io::Error::other("日志写入锁不可用"))?;
        // One event per line, including errors containing newlines. No buffered queue on exit.
        let mut text: String = message
            .chars()
            .take(2048)
            .map(|c| if c.is_control() { ' ' } else { c })
            .collect();
        if message.chars().nth(2048).is_some() {
            text.push('…');
        }
        let line = format!(
            "{} [{}] [{}] {}\n",
            Local::now().format("%Y-%m-%d %H:%M:%S%.3f %:z"),
            level.label(),
            source,
            text
        );
        file.write_all(line.as_bytes())
    }
}

pub(crate) struct RunLog(Result<RunFile, String>);
impl RunLog {
    pub(crate) fn init(app: &tauri::App) -> Self {
        let portable = std::env::current_exe().and_then(|exe| {
            exe.parent()
                .map(|p| p.join("logs"))
                .ok_or_else(|| io::Error::other("无法定位程序目录"))
        });
        Self::choose(portable, || {
            app.path().app_log_dir().map_err(io::Error::other)
        })
    }
    fn choose(
        portable: io::Result<PathBuf>,
        fallback: impl FnOnce() -> io::Result<PathBuf>,
    ) -> Self {
        let create =
            |dir: PathBuf| RunFile::create(&dir).map_err(|e| format!("{}：{e}", dir.display()));
        Self(
            portable
                .map_err(|e| e.to_string())
                .and_then(create)
                .or_else(|first| {
                    let run = fallback()
                        .map_err(|e| e.to_string())
                        .and_then(create)
                        .map_err(|second| format!("程序目录：{first}；用户日志目录：{second}"))?;
                    run.write(
                        Level::Warning,
                        "desktop",
                        &format!("改用用户日志目录；程序目录不可用：{first}"),
                    )
                    .map_err(|e| e.to_string())?;
                    Ok(run)
                }),
        )
    }
    fn active(&self) -> Result<&RunFile, String> {
        self.0.as_ref().map_err(Clone::clone)
    }
    pub(crate) fn finish(&self) {
        if let Ok(run) = self.active() {
            if let Err(e) = run.write(Level::Info, "desktop", "FOC Studio 正常退出") {
                eprintln!("写入退出日志失败：{e}");
            }
        }
    }
}

#[tauri::command]
pub(crate) fn log_file_path(log: State<RunLog>) -> Result<PathBuf, String> {
    Ok(log.active()?.path.clone())
}
#[tauri::command]
pub(crate) fn append_log(level: Level, message: String, log: State<RunLog>) -> Result<(), String> {
    log.active()?
        .write(level, "workbench", &message)
        .map_err(|e| e.to_string())
}
#[tauri::command]
pub(crate) fn open_log_folder(log: State<RunLog>) -> Result<(), String> {
    let dir = log.active()?.path.parent().ok_or("日志目录不存在")?;
    #[cfg(target_os = "windows")]
    let opener = PathBuf::from(std::env::var_os("SystemRoot").ok_or("无法定位 Windows 目录")?)
        .join("explorer.exe");
    #[cfg(target_os = "macos")]
    let opener = "/usr/bin/open";
    #[cfg(not(any(target_os = "windows", target_os = "macos")))]
    let opener = "xdg-open";
    let child = std::process::Command::new(opener)
        .arg(dir)
        .spawn()
        .map_err(|e| e.to_string())?;
    #[cfg(unix)]
    std::thread::spawn(move || {
        let mut child = child;
        let _ = child.wait();
    });
    #[cfg(not(unix))]
    drop(child);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn session_log_survives_restart_and_handles_unwritable_directory() {
        let root = std::env::temp_dir().join(format!(
            "foc-studio-test-{}-{}",
            std::process::id(),
            Local::now().timestamp_nanos_opt().unwrap()
        ));
        let primary = root.join("logs");
        let log = RunLog::choose(Ok(primary.clone()), || panic!("unexpected fallback"));
        let run = log.active().unwrap();
        assert_eq!(run.path.parent(), Some(primary.as_path()));
        run.write(Level::Error, "workbench", "读参数\r\n失败")
            .unwrap();
        log.finish();
        let first_path = run.path.clone();
        drop(log);
        let text = fs::read_to_string(&first_path).unwrap();
        assert!(text.contains("[ERROR] [workbench] 读参数  失败"));
        assert!(text.ends_with("正常退出\n"));
        // A regular file cannot act as the log directory (even under admin/root).
        let fallback = root.join("user-logs");
        let next = RunLog::choose(Ok(first_path.clone()), || Ok(fallback.clone()));
        let run = next.active().unwrap();
        assert_eq!(run.path.parent(), Some(fallback.as_path()));
        assert!(fs::read_to_string(&run.path)
            .unwrap()
            .contains("改用用户日志目录"));
        drop(next);
        assert!(
            RunLog::choose(Ok(first_path.clone()), || Ok(first_path.clone()))
                .active()
                .is_err()
        );
        assert_eq!(fs::read_to_string(first_path).unwrap(), text);
        fs::remove_dir_all(root).unwrap();
    }
}
