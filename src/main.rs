use crossbeam_channel::Receiver;
use rusb::{Context, UsbContext};
use serde::{Deserialize, Serialize};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::{Arc, Mutex};
use std::thread;

// 既存のドライバをインポート
use rust_px4_usr_drv::drivers::it930x::IT930x;
use rust_px4_usr_drv::drivers::itedtv_bus::{BusOps, UsbBusRusb};
use rust_px4_usr_drv::drivers::px4_device::Px4Device;

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
    pub sub_channel: Option<u32>, // BS/CSのストリームID / スロット番号用
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

// チャンネルから周波数(kHz)への変換ヘルパー
fn channel_to_freq_khz(config: &ChannelConfig) -> anyhow::Result<u32> {
    match config.space {
        ChannelSpace::Terrestrial => {
            // 地デジ: 13〜62ch
            if (13..=62).contains(&config.channel) {
                Ok(473143 + (config.channel - 13) * 6000)
            } else {
                anyhow::bail!(
                    "Invalid Terrestrial channel: {}. Must be between 13 and 62.",
                    config.channel
                )
            }
        }
        ChannelSpace::CommunityAntennaTeleVision => {
            // CATV: C13〜C63ch
            if (13..=22).contains(&config.channel) {
                Ok(111143 + (config.channel - 13) * 6000)
            } else if (23..=63).contains(&config.channel) {
                Ok(225143 + (config.channel - 23) * 6000)
            } else {
                anyhow::bail!(
                    "Invalid CATV channel: {}. Must be between 13 and 63.",
                    config.channel
                )
            }
        }
        ChannelSpace::BroadcastingSatellite => {
            // BS: 1〜23ch (奇数のみ)
            if (1..=23).contains(&config.channel) && config.channel % 2 != 0 {
                let ch_idx = config.channel / 2;
                Ok(1049480 + (38360 * ch_idx))
            } else {
                anyhow::bail!(
                    "Invalid BS channel: {}. Must be an odd number between 1 and 23.",
                    config.channel
                )
            }
        }
        ChannelSpace::CommunicationSatellite => {
            // CS: 2〜24ch (偶数のみ)
            if (2..=24).contains(&config.channel) && config.channel % 2 == 0 {
                let ch_idx = config.channel / 2 - 1;
                Ok(1613000 + (40000 * ch_idx))
            } else {
                anyhow::bail!(
                    "Invalid CS channel: {}. Must be an even number between 2 and 24.",
                    config.channel
                )
            }
        }
    }
}

fn main() -> anyhow::Result<()> {
    // まず、USB関連の準備
    let context = match Context::new() {
        Ok(c) => c,
        Err(e) => {
            anyhow::bail!("Failed to create USB context: {}", e);
        }
    };

    // USBデバイスの検索
    const PX4_VID: u16 = 0x0511;
    const PX4_PID: u16 = 0x083f;

    let devices = match context.devices() {
        Ok(d) => d,
        Err(e) => {
            anyhow::bail!("Failed to list USB devices: {}", e);
        }
    };

    let device = match devices.iter().find(|d| {
        d.device_descriptor()
            .map(|desc| desc.vendor_id() == PX4_VID && desc.product_id() == PX4_PID)
            .unwrap_or(false)
    }) {
        Some(d) => d,
        None => {
            anyhow::bail!("PX4 device not found.");
        }
    };

    // USBデバイスを開く
    let handle = match device.open() {
        Ok(h) => h,
        Err(e) => {
            anyhow::bail!("Failed to open device: {}", e);
        }
    };

    // 各種、デバイス操作用の準備
    let bus = match UsbBusRusb::new(handle) {
        Ok(b) => b,
        Err(e) => {
            anyhow::bail!("Failed to UsbBusRusb::new(): {:?}", e);
        }
    };

    let it930x = IT930x::new(bus);

    // 1. デバイスの初期化と共有管理
    let (device, receivers) = match Px4Device::new(&it930x) {
        Ok(v) => v,
        Err(e) => {
            anyhow::bail!("Failed to init Px4Device: {:?}", e);
        }
    };

    let shared_receivers = Arc::new(receivers);
    let shared_device = Arc::new(Mutex::new(device));

    // 2. Unix Domain Socket の準備 (シグナルハンドラ準備も含む)
    let socket_path = "/tmp/px4-tuner.sock";

    ctrlc::set_handler(move || {
        println!("\n[info] Received shutdown signal. Cleaning up...");

        // ソケットファイルの削除
        if let Err(e) = std::fs::remove_file(socket_path) {
            eprintln!("[error] Failed to remove socket file: {}", e);
        } else {
            println!("[info] Socket file removed.");
        }

        // プロセスを終了
        std::process::exit(0);
    })
    .expect("Error setting Ctrl-C handler");

    let _ = std::fs::remove_file(socket_path)?;
    let listener = UnixListener::bind(socket_path)?;
    println!(
        "[server] Daemon started. Waiting for connections on {}...",
        socket_path
    );

    // 3. クライアント接続待ちループ
    std::thread::scope(|s| {
        for stream in listener.incoming() {
            match stream {
                Ok(stream) => {
                    let rx_clone = Arc::clone(&shared_receivers);
                    let dev_clone = Arc::clone(&shared_device);

                    // クライアントごとにスレッドを立てる
                    s.spawn(move || {
                        handle_client(stream, rx_clone, dev_clone);
                    });
                }
                Err(e) => println!("[error] Connection failed: {}", e),
            }
        }
    });

    Ok(())
}

// クライアントからの要求処理(受信メインループ)
fn handle_client<B: BusOps + Send + Sync>(
    mut stream: UnixStream,
    receivers: Arc<Vec<crossbeam_channel::Receiver<Vec<u8>>>>,
    px4_device: Arc<Mutex<Px4Device<B>>>, // ドライバの共有参照
) {
    println!("[client] Connected.");
    let mut reader = BufReader::new(stream.try_clone().unwrap());
    let mut line = String::new();

    // クライアントから送られてくる1行（JSON）を待つ
    if reader.read_line(&mut line).is_ok() {
        match serde_json::from_str::<DaemonCommand>(&line) {
            Ok(cmd) => {
                match cmd {
                    DaemonCommand::SetChannel {
                        port,
                        channel,
                        lnb_voltage,
                    } => {
                        // 周波数変換結果の Result をハンドリング
                        match channel_to_freq_khz(&channel) {
                            Ok(freq_khz) => {
                                let mut dev = px4_device.lock().unwrap();

                                // LNB電圧を設定する (BS/CSの場合)
                                // チャンネルの設定やチューニングの前に、アンテナ電源を確保する必要があるため
                                if let Some(mode) = lnb_voltage {
                                    // ドライバが 0 か 15 しか受け付けないなら、ここで明示的に変換する
                                    let target_voltage = match mode {
                                        LnbMode::Off => 0,
                                        LnbMode::Volt11 => 0, // 必要に応じて 15V 側に倒す
                                        LnbMode::Volt15 => 15,
                                    };

                                    if let Err(e) =
                                        dev.set_lnb_voltage(port as usize, target_voltage)
                                    {
                                        let _ = writeln!(stream, "{{\"status\":\"error\",\"message\":\"LNB voltage set failed: {:?}\"}}", e);
                                        return;
                                    }
                                }

                                match dev.tune(port, freq_khz) {
                                    Ok(_) => {
                                        let _ = writeln!(stream, "{{\"status\":\"ok\"}}");
                                    }
                                    Err(e) => {
                                        let _ = writeln!(
                                            stream,
                                            "{{\"status\":\"error\",\"message\":\"{:?}\"}}",
                                            e
                                        );
                                    }
                                }

                                // ストリームIDの設定（もしサブチャンネル指定があれば）
                                if let Some(tsid) = channel.sub_channel {
                                    // Tunerトレイトのメソッドとして直接呼ぶ
                                    // 衛星チューナーなら成功し、地上波チューナーならエラー(InvalidState)になる
                                    if let Err(e) = dev.set_stream_id(port, tsid as u16) {
                                        // もし「サポートしていないチューナー」でこのコマンドが来た場合は
                                        // エラーを返すか、あるいは無視するかの設計判断が必要です
                                        // ここではエラーとして報告する例
                                        let _ = writeln!(stream, "{{\"status\":\"error\",\"message\":\"TSID set failed: {:?}\"}}", e);
                                        return;
                                    }
                                }
                            }
                            Err(e) => {
                                // 不適切な設定（エラー）だった場合はクライアントに通知
                                let _ = writeln!(
                                    stream,
                                    "{{\"status\":\"error\",\"message\":\"{}\"}}",
                                    e
                                );
                            }
                        }
                    }

                    DaemonCommand::StartStream { port } => {
                        let mut dev = px4_device.lock().unwrap();
                        // キャプチャの有効化
                        if dev.set_capture(port, true).is_err() {
                            let _ = writeln!(
                                stream,
                                "{{\"status\":\"error\",\"message\":\"Failed to start capture\"}}"
                            );
                            return;
                        }

                        // 成功応答を返し、即座にストリーミング（バイナリ転送）モードに移行
                        let _ = writeln!(stream, "{{\"status\":\"ok\"}}");

                        if let Some(rx) = receivers.get(port) {
                            // crossbeam-channel からデータが届く限り、Unixソケットへ流し込み続ける
                            while let Ok(packet) = rx.recv() {
                                if stream.write_all(&packet).is_err() {
                                    break; // クライアントが切断（録画終了）したらループを抜ける
                                }
                            }
                        }

                        // クライアント切断時、またはエラー時は自動でキャプチャを安全に停止
                        let mut dev = px4_device.lock().unwrap();
                        let _ = dev.set_capture(port, false);
                    }

                    DaemonCommand::StopStream { port } => {
                        let mut dev = px4_device.lock().unwrap();
                        let _ = dev.set_capture(port, false);
                        let _ = writeln!(stream, "{{\"status\":\"ok\"}}");
                    }

                    DaemonCommand::GetSignal { port } => {
                        let mut dev = px4_device.lock().unwrap();
                        // ドライバから現在の C/N 比 (dB値) 等を取得してクライアントに返す
                        match dev.get_cnr(port) {
                            Ok(cnr_raw) => {
                                // 必要であればここで raw 値を dB に計算し直すか、そのまま返す
                                let _ =
                                    writeln!(stream, "{{\"status\":\"ok\",\"cnr\":{}}}", cnr_raw);
                            }
                            Err(e) => {
                                let _ = writeln!(stream, "{{\"status\":\"error\",\"message\":\"Failed to read CNR: {:?}\"}}", e);
                            }
                        }
                    }
                }
            }
            Err(e) => {
                let _ = writeln!(
                    stream,
                    "{{\"status\":\"error\",\"message\":\"Invalid JSON: {}\"}}",
                    e
                );
            }
        }
    }
}
