use serde::{Deserialize, Serialize};

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
