//! デーモン全体で使用するエラー型の一元管理

use thiserror::Error;

/// デーモン全体のエラー型
///
/// rusb::Error, std::io::Error, serde_json::Error などを統一的に扱い、
/// `?` オペレータで顺畅に変換できるようにする。
#[derive(Debug, Error)]
pub enum DaemonError {
    #[error("USB context creation failed: {0}")]
    UsbContext(#[from] rusb::Error),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("JSON parse error: {0}")]
    Json(#[from] serde_json::Error),

    #[error("tuner not found")]
    TunerNotFound,

    #[error("invalid channel setting: {0}")]
    InvalidChannel(String),

    #[error("tuner busy: {0}")]
    TunerBusy(String),

    #[error("tuner lock timeout")]
    TunerLockTimeout,

    #[error("unknown command: {0}")]
    UnknownCommand(String),

    #[error("invalid port for device")]
    InvalidPort,

    #[error("channel set failed")]
    ChannelSetFailed,

    #[error("unknown error: {0}")]
    Unknown(String),
}

/// デーモン用の Result 型
pub type DaemonResult<T> = Result<T, DaemonError>;
