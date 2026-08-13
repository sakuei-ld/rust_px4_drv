use serde::{Deserialize, Serialize};

// B-CASカード関連定数
/// JSON制御プロトコル用ポート
pub const BCAS_SERVER_PORT: u16 = 8000;

/// bcs-perl.pl 互換のバイナリプロトコル（1バイト長プレフィックス＋生APDU）用ポート。
/// 既存のJSON制御プロトコルとは別に待ち受ける。
pub const BCAS_RAW_SERVER_PORT: u16 = 6900;

// コマンドプロトコルの定義
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ChannelSpace {
    Terrestrial,                // 地上波(GR)
    CommunityAntennaTeleVision, // ケーブルテレビ
    BroadcastingSatellite,      // BS
    CommunicationSatellite,     // CS
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ChannelConfig {
    pub space: ChannelSpace,
    pub channel: u32,             // チャンネル番号
    pub sub_channel: Option<u16>, // BS/CSのストリームID / スロット番号用
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum LnbMode {
    Off,
    Volt11, // または Volt13
    Volt15, // または Volt18
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "command", content = "payload")]
pub enum DaemonCommand {
    /// 選局（チャンネル設定）
    SetChannel {
        port: usize,
        channel: ChannelConfig,
        lnb_voltage: Option<LnbMode>,
    },
    /// キャプチャ及びストリーミングの開始
    StartStream { port: usize },
    /// キャプチャの停止
    StopStream { port: usize },
    /// シグナル強度（C/N比）の取得
    GetSignal { port: usize },
}

// ステータスやりとり用
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct StatusResponse {
    pub status: String,
    pub message: Option<String>,
}

// Signal 用
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SignalResponse {
    pub status: String,
    pub cnr: Option<f64>,
    pub message: Option<String>,
}

// ===== B-CASカード関連 =====

/// B-CASカードの初期化コマンド (BonCasProxy用)
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BcasInitializeCommand {
    pub command: String, // "initialize"
}

/// B-CASカードの情報取得コマンド
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BcasGetInfoCommand {
    pub command: String, // "get_info"
}

/// B-CASカードのチャンネル情報取得コマンド
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BcasReadChannelCommand {
    pub command: String, // "read_channel"
}

/// B-CASカードID取得コマンド
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BcasGetCardIdCommand {
    pub command: String, // "get_card_id"
}

/// B-CASカード関連コマンド
#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "command", content = "payload", rename_all = "snake_case")]
pub enum BcasCommand {
    Initialize,
    GetInfo,
    ReadChannel,
    GetCardId,
}

/// B-CASカードIDレスポンス
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BcasCardIdResponse {
    pub status: String,
    pub card_id: Option<String>,
    pub card_version: Option<u8>,
    pub manufacturer_id: Option<u8>,
    pub message: Option<String>,
}

/// B-CASカード情報レスポンス
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BcasInfoResponse {
    pub status: String,
    pub atr: Option<Vec<u8>>,
    pub ts: Option<u8>,
    pub t0: Option<u8>,
    pub protocol: Option<String>,
    pub message: Option<String>,
}

/// B-CASチャンネル情報レスポンス
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BcasChannelResponse {
    pub status: String,
    pub channel_data: Option<Vec<u8>>,
    pub message: Option<String>,
}
