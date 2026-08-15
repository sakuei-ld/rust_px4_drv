//! Shim との TCP サーバー処理
//!
//! main.rs からサーバー関連のコードを分離し、
//! `run_server` で全体のサーバーループを担当する。

use bytes::Bytes;
use serde::Serialize;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::time::Duration;

use protocol::{DaemonCommand, SignalResponse, StatusResponse};

use crate::drivers::itedtv_bus::BusOps;
use crate::drivers::px4_device::Px4Device;
use crate::error::DaemonResult;

/// タイムアウト値（ミリ秒）
pub const TIMEOUT_MS: u128 = 5000;

/// シャットダウンフラグ
pub static SHUTDOWN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// TCP ソケットの入出力を扱う構造体
pub struct ClientIo {
    reader: BufReader<TcpStream>,
    writer: BufWriter<TcpStream>,

    // debug
    socket_sent_bytes: u64,
    socket_write_calls: u64,
}

impl ClientIo {
    pub fn new(stream: TcpStream) -> std::io::Result<Self> {
        Ok(Self {
            reader: BufReader::new(stream.try_clone()?),
            writer: BufWriter::with_capacity(188 * 1024, stream),
            socket_sent_bytes: 0,
            socket_write_calls: 0,
        })
    }

    pub fn read_line(&mut self, line: &mut String) -> std::io::Result<usize> {
        self.reader.read_line(line)
    }

    pub fn send_json<T: Serialize>(&mut self, value: &T) -> anyhow::Result<()> {
        serde_json::to_writer(&mut self.writer, value)?;
        self.writer.write_all(b"\n")?;
        self.writer.flush()?;
        Ok(())
    }

    pub fn send_ts(&mut self, packet: &[u8]) -> std::io::Result<()> {
        // debug
        self.writer.write_all(packet)?;
        self.socket_sent_bytes += packet.len() as u64;
        self.socket_write_calls += 1;
        Ok(())
    }
}

/// StatusResponse の生成へルパ
fn send_ok(io: &mut ClientIo) {
    let _ = io.send_json(&StatusResponse {
        status: "ok".into(),
        message: None,
    });
}

fn send_error(io: &mut ClientIo, msg: impl Into<String>) {
    let _ = io.send_json(&StatusResponse {
        status: "error".into(),
        message: Some(msg.into()),
    });
}

/// RAIIを利用した自動クリーンアップ用構造体 (接続単位で管理)
struct ClientGuard<'a, B: BusOps + Send + Sync> {
    device: Arc<std::sync::Mutex<Px4Device<'a, B>>>,
    port: Option<usize>,
}

impl<'a, B: BusOps + Send + Sync> Drop for ClientGuard<'a, B> {
    fn drop(&mut self) {
        // クライアント切断時(エラー終了含む)に必ずキャプチャを停止し、チューナーを閉じる
        if let Some(port) = self.port {
            if let Ok(mut dev) = self.device.lock() {
                tracing::info!("ClientGuard cleanup start");
                let _ = dev.set_capture(port, false);
                let _ = dev.close_tuner(port);
                tracing::info!("ClientGuard cleanup end");

                tracing::info!("ClientGuard: Cleaned up port {} on disconnect.", port);
            }
        }
    }
}

/// チャンネルから周波数(kHz)への変換ヘルパー
fn channel_to_freq_khz(config: &protocol::ChannelConfig) -> anyhow::Result<u32> {
    // sub_channel が None の場合は 0 (オフセットなし) として扱う
    let sub_ch_offset = config.sub_channel.unwrap_or(0) as u32;

    match config.space {
        protocol::ChannelSpace::Terrestrial => {
            // 地デジ: 13〜62ch
            if (13..=62).contains(&config.channel) {
                // Cコード: 95143 + freq_no * 6000 + slot
                // 例: 13ch => 95143 + 63 * 6000 = 473143 kHz
                Ok(473143 + (config.channel - 13) * 6000 + sub_ch_offset)
            } else {
                anyhow::bail!(
                    "Invalid Terrestrial channel: {}. Must be between 13 and 62.",
                    config.channel
                )
            }
        }
        protocol::ChannelSpace::CommunityAntennaTeleVision => {
            // CATV: C13〜C63ch
            if (13..=22).contains(&config.channel) {
                // Cコード: 93143 + freq_no * 6000 + slot
                // 例: C13ch => 93143 + 3 * 6000 = 111143 kHz
                let mut freq = 111143 + (config.channel - 13) * 6000;

                // C22ch (freq_no == 12) の特殊な 2MHz シフト
                if config.channel == 22 {
                    freq += 2000;
                }
                Ok(freq + sub_ch_offset)
            } else if (23..=63).contains(&config.channel) {
                // 例: C23ch => 93143 + 22 * 6000 = 225143 kHz
                Ok(225143 + (config.channel - 23) * 6000 + sub_ch_offset)
            } else {
                anyhow::bail!(
                    "Invalid CATV channel: {}. Must be between 13 and 63.",
                    config.channel
                )
            }
        }
        protocol::ChannelSpace::BroadcastingSatellite => {
            // BS: 1〜23ch (奇数のみ)
            if (1..=23).contains(&config.channel) && config.channel % 2 != 0 {
                let ch_idx = config.channel / 2;
                // 衛星波では sub_channel (slot) は周波数計算には使用しない (Cコードと等価)
                Ok(1049480 + (38360 * ch_idx))
            } else {
                anyhow::bail!(
                    "Invalid BS channel: {}. Must be an odd number between 1 and 23.",
                    config.channel
                )
            }
        }
        protocol::ChannelSpace::CommunicationSatellite => {
            // CS: 2〜24ch (偶数のみ)
            if (2..=24).contains(&config.channel) && config.channel % 2 == 0 {
                let ch_idx = config.channel / 2 - 1;
                // 衛星波では sub_channel (slot) は周波数計算には使用しない (Cコードと等価)
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

/// 単一クライアントの処理を行う内部関数
///
/// JSON-RPC ライクなプロトコルでコマンドを受け取り、
/// チューナー状態を操作してレスポンスを返す。
pub fn handle_client<B: BusOps + Send + Sync>(
    stream: TcpStream,
    receivers: Arc<Vec<crossbeam_channel::Receiver<Bytes>>>,
    px4_device: Arc<std::sync::Mutex<Px4Device<'static, B>>>,
) {
    tracing::info!("[client] Connected.");

    // 読み込みにタイムアウトを設定（0.5秒ごとにフラグを確認しにくる）
    let _ = stream.set_read_timeout(Some(Duration::from_millis(500)));

    let mut io = match ClientIo::new(stream) {
        Ok(io) => io,
        Err(e) => {
            tracing::error!("[client] Failed to create ClientIo: {}", e);
            return;
        }
    };

    let mut line = String::new();

    // クライアントの接続ライフサイクルを管理するガード
    let mut guard = ClientGuard {
        device: Arc::clone(&px4_device),
        port: None,
    };

    // debug
    let mut sent_packets = 0u64;

    while !SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
        line.clear();
        match io.read_line(&mut line) {
            Ok(0) => {
                tracing::info!("[client] Disconnected.");
                break;
            }
            Ok(_) => {
                tracing::debug!("recv raw line = {:?}", line);
                match serde_json::from_str::<DaemonCommand>(&line) {
                    Ok(cmd) => {
                        match cmd {
                            DaemonCommand::SetChannel {
                                port,
                                channel,
                                lnb_voltage,
                            } => {
                                match channel_to_freq_khz(&channel) {
                                    Ok(freq_khz) => {
                                        // 1. 古いポートの停止処理（必要な場合）
                                        if let Some(old_port) = guard.port {
                                            if let Ok(mut dev) = px4_device.lock() {
                                                let _ = dev.set_capture(old_port, false);
                                                if old_port != port {
                                                    let _ = dev.close_tuner(old_port);
                                                    guard.port = None;
                                                }
                                            }
                                        }

                                        // 2. チューナーのオープン処理（リトライ付き）
                                        tracing::info!("Attempting open tuner: {}", port);
                                        let mut retry = 0;
                                        let mut opened = false;

                                        while retry <= 20 {
                                            // スコープを作ってブロック終了時に即座に lock を解放（drop）させる
                                            let open_result = {
                                                match px4_device.lock() {
                                                    Ok(mut dev) => dev.open_tuner(port),
                                                    Err(e) => {
                                                        send_error(
                                                            &mut io,
                                                            format!("Lock error: {}", e),
                                                        );
                                                        break;
                                                    }
                                                }
                                            };

                                            match open_result {
                                                Ok(_) => {
                                                    opened = true;
                                                    break;
                                                }
                                                Err(e) => {
                                                    if retry >= 20 {
                                                        send_error(
                                                            &mut io,
                                                            format!("Tuner busy timeout: {}", e),
                                                        );
                                                        break;
                                                    }
                                                    std::thread::sleep(Duration::from_millis(100));
                                                    retry += 1;
                                                }
                                            }
                                        }

                                        if !opened {
                                            continue;
                                        }

                                        // クリーンアップ用にポートを記憶
                                        guard.port = Some(port);

                                        // 3. チューニングおよび各種設定（単一のロックでまとめて処理）
                                        let mut dev = match px4_device.lock() {
                                            Ok(g) => g,
                                            Err(e) => {
                                                send_error(&mut io, format!("Lock error: {}", e));
                                                continue;
                                            }
                                        };

                                        // ストリーミング中であれば、一度止める(安全のため)
                                        let _ = dev.set_capture(port, false);

                                        // ISDB-S かつ 「Tune前にStreamIDを設定する」タイプの場合
                                        if let Ok(true) = dev.is_set_stream_id_before_tune(port) {
                                            let tsid = channel.sub_channel.unwrap_or(0) as u16;
                                            if let Err(e) = dev.set_stream_id(port, tsid) {
                                                tracing::warn!("[warn] TSID set failed: {:?}", e);
                                                continue;
                                            }
                                        }

                                        // 周波数・帯域幅の設定を実行（チューナーLSIへの書き込みとPLLロック）
                                        tracing::info!("Attempting to tune to {} kHz", freq_khz);
                                        if let Err(e) = dev.tune(port, freq_khz) {
                                            send_error(&mut io, format!("{}", e));
                                            continue;
                                        }

                                        // 復調器（TC90522）のシグナルロック待ちループ
                                        let start_time = std::time::Instant::now();
                                        let mut locked = false;
                                        let mut loop_count = 0;

                                        loop {
                                            if let Ok(true) = dev.check_lock(port) {
                                                locked = true;
                                                break;
                                            }

                                            if start_time.elapsed().as_millis() >= TIMEOUT_MS {
                                                break;
                                            }

                                            std::thread::sleep(Duration::from_millis(20));
                                            loop_count += 1;
                                        }

                                        if !locked {
                                            tracing::warn!(
                                                "[warn] Tuner did not lock in time on port {}",
                                                port
                                            );
                                            send_error(&mut io, "Tuner did not lock");
                                            continue;
                                        }

                                        // ISDB-T かつ ロックが早すぎた場合のウェイト
                                        if let Ok(true) = dev.is_wait_after_check_lock(port) {
                                            if loop_count < 35 {
                                                let wait_time = (35 - loop_count) * 10;
                                                std::thread::sleep(Duration::from_millis(
                                                    wait_time as u64,
                                                ));
                                            }
                                        }

                                        // ISDB-S かつ 「Tune後にStreamIDを設定する」タイプの場合
                                        if let Ok(true) = dev.is_set_stream_id_after_tune(port) {
                                            let tsid = channel.sub_channel.unwrap_or(0) as u16;
                                            if let Err(e) = dev.set_stream_id(port, tsid) {
                                                tracing::warn!("[warn] TSID set failed: {:?}", e);
                                                continue;
                                            }
                                        }

                                        // ロック後の安定化ウェイト
                                        if let Ok(true) = dev.is_wait_after_lock(port) {
                                            std::thread::sleep(Duration::from_millis(200));
                                        }

                                        // LNB電源設定
                                        let is_satellite = matches!(
                                            channel.space,
                                            protocol::ChannelSpace::BroadcastingSatellite
                                                | protocol::ChannelSpace::CommunicationSatellite
                                        );

                                        if is_satellite {
                                            tracing::info!(
                                                "Attempting to set LNB voltage: {:?}",
                                                lnb_voltage
                                            );

                                            if let Some(mode) = lnb_voltage {
                                                let target_voltage = match mode {
                                                    protocol::LnbMode::Off => 0,
                                                    protocol::LnbMode::Volt11 => 15,
                                                    protocol::LnbMode::Volt15 => 15,
                                                };

                                                tracing::info!(
                                                    "Set LNB voltage: {}",
                                                    target_voltage
                                                );
                                                if let Err(e) = dev
                                                    .set_lnb_voltage(port as usize, target_voltage)
                                                {
                                                    send_error(
                                                        &mut io,
                                                        format!("LNB voltage set failed: {}", e),
                                                    );
                                                    continue;
                                                }
                                            }
                                        }

                                        // キャプチャ開始
                                        tracing::info!("Attempting set capture(status: true)");
                                        if let Err(e) = dev.set_capture(port, true) {
                                            send_error(&mut io, format!("Capture error: {}", e));
                                            continue;
                                        }

                                        // 最後まで進めたら OK
                                        send_ok(&mut io);
                                    }
                                    Err(e) => {
                                        send_error(&mut io, format!("{:?}", e));
                                    }
                                }
                            }

                            // SetChannel されてから呼ばれる前提
                            DaemonCommand::StartStream { port } => {
                                // 成功応答を返し、即座にストリーミング（バイナリ転送）モードに移行
                                send_ok(&mut io);

                                if let Some(rx) = receivers.get(port) {
                                    loop {
                                        match rx.recv_timeout(Duration::from_millis(500)) {
                                            Ok(packet) => {
                                                if io.send_ts(&packet).is_err() {
                                                    let _ = io.writer.flush();
                                                    tracing::info!("[client] stream closed.");
                                                    break;
                                                }

                                                // debug
                                                sent_packets += 1;
                                                if sent_packets % 1000 == 0 {
                                                    tracing::info!(
                                                        "stream sent_packets = {} queue_len = {}",
                                                        sent_packets,
                                                        rx.len()
                                                    );
                                                }
                                            }

                                            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                                                // データが途切れたタイミング（0.5秒）でソケットへ確実に押し出す
                                                if let Err(e) = io.writer.flush() {
                                                    tracing::error!("Flush error: {}", e);
                                                    break;
                                                }
                                                continue;
                                            }

                                            Err(
                                                crossbeam_channel::RecvTimeoutError::Disconnected,
                                            ) => {
                                                let _ = io.writer.flush();
                                                break;
                                            }
                                        }
                                    }
                                    // debug
                                    tracing::info!(
                                        "socket sent {} bytes, {} writes",
                                        io.socket_sent_bytes,
                                        io.socket_write_calls
                                    );
                                }

                                break;
                            }

                            // 基本は使わない。
                            DaemonCommand::StopStream { port } => {
                                let mut dev = match px4_device.lock() {
                                    Ok(g) => g,
                                    Err(e) => {
                                        send_error(&mut io, format!("Lock error: {}", e));
                                        continue;
                                    }
                                };
                                let _ = dev.set_capture(port, false);
                                let _ = dev.close_tuner(port);
                                send_ok(&mut io);
                            }

                            // SetChannel されてから呼ばれる前提
                            DaemonCommand::GetSignal { port } => {
                                // 監視中(StartStream前)は、バッファが溢れないように溜まったTSパケットを全破棄する
                                if let Some(rx) = receivers.get(port) {
                                    while let Ok(_) = rx.try_recv() {}
                                }

                                let mut dev = match px4_device.lock() {
                                    Ok(g) => g,
                                    Err(e) => {
                                        send_error(&mut io, format!("Lock error: {}", e));
                                        continue;
                                    }
                                };
                                // ドライバから現在の C/N 比 (dB値) 等を取得してクライアントに返す
                                let response = match dev.get_cnr(port) {
                                    Ok(cnr_raw) => SignalResponse {
                                        status: "ok".to_string(),
                                        cnr: Some(cnr_raw as f64),
                                        message: None,
                                    },
                                    Err(e) => SignalResponse {
                                        status: "error".to_string(),
                                        cnr: None,
                                        message: Some(format!("{:?}", e)),
                                    },
                                };

                                // 送信
                                if let Err(e) = io.send_json(&response) {
                                    tracing::error!("send_json failed: {}", e);
                                }
                            }
                        }
                    }
                    Err(e) => {
                        send_error(&mut io, format!("Invalid JSON: {}", e));
                    }
                }
            }
            // タイムアウト/WouldBlockは「データが来ていない」だけなので、
            // ループを継続して SHUTDOWN フラグを確認させる
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(e) => {
                tracing::error!("Read error: {}", e);
                break;
            }
        }
    }
}

/// TCP リスナーを起動し、着信クライアントを処理するループ
///
/// # Arguments
/// * `listener` - 既にバインド・リスン状態にある TcpListener（nonblocking）
/// * `receivers` - TSパケット受信チャネルのリスト
/// * `px4_device` - PX4デバイスへの共有アクセス
pub fn run_server<B: BusOps + Send + Sync>(
    listener: TcpListener,
    receivers: Arc<Vec<crossbeam_channel::Receiver<Bytes>>>,
    px4_device: Arc<std::sync::Mutex<Px4Device<'static, B>>>,
) -> DaemonResult<()> {
    tracing::info!(
        "[server] Daemon started. Waiting for connections on {}...",
        listener.local_addr()?
    );

    // クライアント接続待ちループ
    std::thread::scope(|s| {
        // メインストリーミング用Acceptループ
        loop {
            if SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
                tracing::info!("[server] Shutting down listener loop.");
                break;
            }

            match listener.accept() {
                Ok((stream, _)) => {
                    let _ = stream.set_nonblocking(false);

                    // 1. 低遅延モードの有効化
                    let _ = stream.set_nodelay(true);

                    // 2. OSの送信バッファを明示的に拡大 (例: 4MB)
                    let sock = socket2::SockRef::from(&stream);
                    let _ = sock.set_send_buffer_size(256 * 1024);

                    let rx_clone = Arc::clone(&receivers);
                    let dev_clone = Arc::clone(&px4_device);

                    // クライアントごとにスレッドを立てる
                    s.spawn(move || {
                        handle_client(stream, rx_clone, dev_clone);
                    });
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    std::thread::sleep(std::time::Duration::from_millis(100));
                    continue;
                }
                Err(e) => tracing::error!("[error] Connection failed: {}", e),
            }
        }
    });

    Ok(())
}
