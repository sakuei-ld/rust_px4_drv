use bytes::Bytes;
use crossbeam_channel::RecvTimeoutError;
use rusb::{Context, UsbContext};
use tracing::{error, info, instrument, warn};
use tracing_subscriber::{fmt, layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};

use std::io::{BufRead, BufReader, BufWriter, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::{Arc, Mutex};

// 既存のドライバをインポート
use rust_px4_drv_daemon::drivers::it930x::IT930x;
use rust_px4_drv_daemon::drivers::itedtv_bus::{BusOps, UsbBusRusb};
use rust_px4_drv_daemon::drivers::px4_device::Px4Device;

use protocol::{ChannelConfig, ChannelSpace, DaemonCommand, LnbMode};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Px4DeviceType {
    PX_W3U4,
    PX_Q3U4,
}

const TIMEOUT_MS: u128 = 5000;

static SHUTDOWN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

// チャンネルから周波数(kHz)への変換ヘルパー
fn channel_to_freq_khz(config: &ChannelConfig) -> anyhow::Result<u32> {
    // sub_channel が None の場合は 0 (オフセットなし) として扱う
    let sub_ch_offset = config.sub_channel.unwrap_or(0) as u32;

    match config.space {
        ChannelSpace::Terrestrial => {
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
        ChannelSpace::CommunityAntennaTeleVision => {
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
        ChannelSpace::BroadcastingSatellite => {
            // BS: 1〜23ch (奇数のみ)
            if (1..=23).contains(&config.channel) && config.channel % 2 != 0 {
                let ch_idx = config.channel / 2;
                // 💡 衛星波では sub_channel (slot) は周波数計算には使用しない (Cコードと等価)
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
                // 💡 衛星波では sub_channel (slot) は周波数計算には使用しない (Cコードと等価)
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

#[instrument(err)]
fn main() -> anyhow::Result<()> {
    // RUST_LOG 環境変数でレベルを制御可能にする (例: RUST_LOG=info ./daemon)
    tracing_subscriber::registry()
        .with(fmt::layer().with_writer(std::io::stdout))
        .with(EnvFilter::from_default_env())
        .init();

    // まず、USB関連の準備
    let context = match Context::new() {
        Ok(c) => c,
        Err(e) => {
            anyhow::bail!("Failed to create USB context: {}", e);
        }
    };

    // USBデバイスの検索
    const PX4_VID: u16 = 0x0511;
    const PX4_PID_W3U4: u16 = 0x083f;
    const PX4_PID_Q3U4: u16 = 0x004a;

    // USBデバイスの検索
    let devices = match context.devices() {
        Ok(d) => d,
        Err(e) => {
            anyhow::bail!("Failed to list USB devices: {}", e);
        }
    };

    // PX-W3U4 か PX-Q3U4 を検出
    let (device, device_type) = match devices.iter().find_map(|d| {
        let desc = d.device_descriptor().ok()?;

        if desc.vendor_id() != PX4_VID {
            return None;
        }

        match desc.product_id() {
            PX4_PID_W3U4 => Some((d, Px4DeviceType::PX_W3U4)),
            PX4_PID_Q3U4 => Some((d, Px4DeviceType::PX_Q3U4)),
            _ => None,
        }
    }) {
        Some(v) => v,
        None => anyhow::bail!("PX4 device not found."),
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
    // use_mldev は PX_Q3U4 かつ daemon実行時引数でdisable_multi_device_power_controlが立っていない ときに true とかにするのが良い
    let (device, receivers) =
        match Px4Device::new(&it930x, device_type == Px4DeviceType::PX_Q3U4, false) {
            Ok(v) => v,
            Err(e) => {
                anyhow::bail!("Failed to init Px4Device: {:?}", e);
            }
        };

    let shared_receivers = Arc::new(receivers);
    // これがダメっぽい？
    let shared_device = Arc::new(Mutex::new(device));

    // 2. Unix Domain Socket の準備 (シグナルハンドラ準備も含む)
    let socket_path = "/tmp/px4-tuner.sock";

    ctrlc::set_handler(move || {
        info!("\n[info] Received shutdown signal. Cleaning up...");

        // ソケットファイルの削除
        if let Err(e) = std::fs::remove_file(socket_path) {
            error!("[error] Failed to remove socket file: {}", e);
        } else {
            info!("[info] Socket file removed.");
        }

        SHUTDOWN.store(true, std::sync::atomic::Ordering::SeqCst)
    })
    .expect("Error setting Ctrl-C handler");

    if std::path::Path::new(socket_path).exists() {
        if let Err(e) = std::fs::remove_file(socket_path) {
            error!("Failed to remove socket file: {}", e);
        }
    }

    let listener = UnixListener::bind(socket_path)?;
    listener.set_nonblocking(true)?;

    info!(
        "[server] Daemon started. Waiting for connections on {}...",
        socket_path
    );

    // 3. クライアント接続待ちループ
    std::thread::scope(|s| {
        //for stream in listener.incoming() {
        loop {
            if SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
                info!("[server] Shutting down listener loop.");
                break;
            }

            match listener.accept() {
                //Ok(stream) => {
                Ok((stream, _)) => {
                    let rx_clone = Arc::clone(&shared_receivers);
                    let dev_clone = Arc::clone(&shared_device);

                    // クライアントごとにスレッドを立てる
                    s.spawn(move || {
                        handle_client(stream, rx_clone, dev_clone);
                    });
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // まだ接続がない場合は少し待機してループ先頭へ戻る
                    std::thread::sleep(std::time::Duration::from_millis(100));
                    continue;
                }
                Err(e) => error!("[error] Connection failed: {}", e),
            }
        }
    });

    Ok(())
}

// クライアントからの要求処理(受信メインループ)
fn handle_client<B: BusOps + Send + Sync>(
    mut stream: UnixStream,
    receivers: Arc<Vec<crossbeam_channel::Receiver<Bytes>>>,
    px4_device: Arc<Mutex<Px4Device<B>>>, // ドライバの共有参照
) {
    info!("[client] Connected.");

    // 読み込みにタイムアウトを設定（0.5秒ごとにフラグを確認しにくる）
    let _ = stream.set_read_timeout(Some(std::time::Duration::from_millis(500)));

    let mut reader = BufReader::new(stream.try_clone().unwrap());
    let mut line = String::new();

    let mut writer = BufWriter::with_capacity(128 * 1024, stream.try_clone().unwrap());

    // クライアントの接続ライフサイクルを管理するガード
    let mut guard = ClientGuard {
        device: Arc::clone(&px4_device),
        port: None,
    };

    while !SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
        line.clear();
        // クライアントから送られてくる1行（JSON）を待つ
        match reader.read_line(&mut line) {
            Ok(0) => {
                info!("[client] Disconnected.");
                break;
            }
            Ok(_) => {
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
                                        let mut dev = px4_device.lock().unwrap();

                                        // Px4Device 側に、多重 streaming 排除がついたので、それに合わせて新機能として足してみる
                                        // ポートの確保 (デバイスファイルの open() の相当)
                                        // 現在のクライアントが確保しているポートと違う場合のみオープン処理
                                        if guard.port != Some(port) {
                                            // もし、既に別のポートを開いていたなら、それを閉じて返却する
                                            if let Some(old_port) = guard.port {
                                                let _ = dev.set_capture(old_port, false);
                                                let _ = dev.close_tuner(old_port);
                                            }
                                        }

                                        info!("Attempting open tuner: {}", port);
                                        // 他のクライアントが使用中であれば、ここでエラー(InvalidState)になる
                                        if let Err(e) = dev.open_tuner(port) {
                                            let _ = writeln!(
                                                stream,
                                                "{{\"status\":\"error\",\"message\":\"{:?}\"}}",
                                                e
                                            );
                                            return;
                                        }

                                        // クリーンアップ用にポートを記憶
                                        guard.port = Some(port);

                                        // ストリーミング中であれば、一度止める(安全のため)
                                        // チャンネル切り替え時の割り込みを防ぎ、安全に Tune する
                                        let _ = dev.set_capture(port, false);

                                        // ptx_chrdev.c ptx_chrdev_unlock_ioctl(): switch PTX_SET_CHANNEL 相当
                                        // ISDB-S かつ 「Tune前にStreamIDを設定する」タイプの場合
                                        // C++: if ((params_.system == px4::SystemType::ISDB_S) && (options_ & RECEIVER_SAT_SET_STREAM_ID_BEFORE_TUNE))
                                        if let Ok(true) = dev.is_set_stream_id_before_tune(port) {
                                            let tsid = channel.sub_channel.unwrap_or(0) as u16;
                                            if let Err(e) = dev.set_stream_id(port, tsid) {
                                                warn!("[warn] TSID set failed: {:?}", e);
                                                continue;
                                            }
                                        }

                                        // 周波数・帯域幅の設定を実行（チューナーLSIへの書き込みとPLLロック）
                                        info!("Attempting to tune to {} kHz", freq_khz);
                                        if let Err(e) = dev.tune(port, freq_khz) {
                                            let _ = writeln!(
                                                stream,
                                                "{{\"status\":\"error\",\"message\":\"{:?}\"}}",
                                                e
                                            );
                                            continue;
                                        }

                                        // 復調器（TC90522）のシグナルロック待ちループ (C++ の CheckLock ループの再現)
                                        let start_time = std::time::Instant::now();
                                        let mut locked = false;
                                        let mut loop_count = 0;

                                        loop {
                                            // 復調器が信号をロックしたか確認
                                            if let Ok(true) = dev.check_lock(port) {
                                                locked = true;
                                                break;
                                            }

                                            // タイムアウトチェック
                                            if start_time.elapsed().as_millis() >= TIMEOUT_MS {
                                                break;
                                            }

                                            // C++: Sleep(20);
                                            std::thread::sleep(std::time::Duration::from_millis(
                                                20,
                                            ));
                                            loop_count += 1;
                                        }

                                        // lock が取れなかったら、ここで終わらす
                                        if !locked {
                                            warn!(
                                                "[warn] Tuner did not lock in time on port {}",
                                                port
                                            );

                                            // ロックに失敗した場合はエラー（または警告）を返す
                                            let _ = writeln!(stream, "{{\"status\":\"error\",\"message\":\"Tuner did not lock\"}}");
                                            continue;
                                        }

                                        // ISDB-T かつ ロックが早すぎた場合のウェイト
                                        // C++: if ((params_.system == px4::SystemType::ISDB_T) && (options_ & RECEIVER_WAIT_AFTER_LOCK_TC_T) && (i < 35))
                                        if let Ok(true) = dev.is_wait_after_check_lock(port) {
                                            if loop_count < 35 {
                                                let wait_time = (35 - loop_count) * 10;
                                                std::thread::sleep(
                                                    std::time::Duration::from_millis(
                                                        wait_time as u64,
                                                    ),
                                                );
                                            }
                                        }

                                        // ISDB-S かつ 「Tune後にStreamIDを設定する」タイプの場合（PX-W3U4のBS/CSは通常これ）
                                        // C++: if ((params_.system == px4::SystemType::ISDB_S) && !(options_ & RECEIVER_SAT_SET_STREAM_ID_BEFORE_TUNE))
                                        if let Ok(true) = dev.is_set_stream_id_after_tune(port) {
                                            let tsid = channel.sub_channel.unwrap_or(0) as u16;
                                            if let Err(e) = dev.set_stream_id(port, tsid) {
                                                warn!("[warn] TSID set failed: {:?}", e);
                                                continue;
                                            }
                                        }

                                        // ロック後の安定化ウェイト
                                        // C++: if (options_ & RECEIVER_WAIT_AFTER_LOCK) Sleep(200);
                                        if let Ok(true) = dev.is_wait_after_lock(port) {
                                            std::thread::sleep(std::time::Duration::from_millis(
                                                200,
                                            ));
                                        }

                                        // ptx_chrdev.c ptx_chrdev_unlock_ioctl(): switch PTX_ENABLE_LNB_POWER 相当
                                        // SetLnbVoltage
                                        let is_satellite = match channel.space {
                                            ChannelSpace::BroadcastingSatellite
                                            | ChannelSpace::CommunicationSatellite => true,
                                            _ => false,
                                        };

                                        if is_satellite {
                                            info!(
                                                "Attempting to set LNB voltage: {:?}",
                                                lnb_voltage
                                            );

                                            if let Some(mode) = lnb_voltage {
                                                // ドライバが 0 か 15 しか受け付けないなら、ここで明示的に変換する
                                                let target_voltage = match mode {
                                                    LnbMode::Off => 0,
                                                    LnbMode::Volt11 => 15, // 必要に応じて 15V 側に倒す
                                                    LnbMode::Volt15 => 15,
                                                };

                                                info!("Set LNB voltage: {}", target_voltage);
                                                if let Err(e) = dev
                                                    .set_lnb_voltage(port as usize, target_voltage)
                                                {
                                                    let _ = writeln!(stream, "{{\"status\":\"error\",\"message\":\"LNB voltage set failed: {:?}\"}}", e);
                                                    return;
                                                }
                                            }
                                        }

                                        // ptx_chrdev.c ptx_chrdev_unlock_ioctl(): switch PTX_START_STREAMING 相当
                                        // recisdb では check signal とかの前に処理しちゃうので。
                                        info!("Attempting set capture(status: true)");

                                        // SetCapture を true に
                                        if let Err(e) = dev.set_capture(port, true) {
                                            let _ = writeln!(stream, "{{\"status\":\"error\",\"message\":\"Capture error: {:?}\"}}", e);
                                            continue;
                                        }

                                        // 最後まで進めたら OK
                                        let _ = writeln!(stream, "{{\"status\":\"ok\"}}");
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

                            // SetChannel されてから呼ばれる前提
                            DaemonCommand::StartStream { port } => {
                                // 成功応答を返し、即座にストリーミング（バイナリ転送）モードに移行
                                let _ = writeln!(stream, "{{\"status\":\"ok\"}}");

                                if let Some(rx) = receivers.get(port) {
                                    loop {
                                        match rx.recv_timeout(std::time::Duration::from_millis(500))
                                        {
                                            Ok(packet) => {
                                                if writer.write_all(&packet).is_err() {
                                                    info!("[client] stream closed.");
                                                    break;
                                                }
                                            }

                                            Err(RecvTimeoutError::Timeout) => {
                                                continue;
                                            }

                                            Err(RecvTimeoutError::Disconnected) => {
                                                break;
                                            }
                                        }
                                    }
                                }

                                break;

                                // クライアント切断時、またはエラー時は自動でキャプチャを安全に停止
                                //let mut dev = px4_device.lock().unwrap();
                                //let _ = dev.set_capture(port, false);
                            }

                            // 基本は使わない。
                            DaemonCommand::StopStream { port } => {
                                let mut dev = px4_device.lock().unwrap();
                                let _ = dev.set_capture(port, false);
                                let _ = dev.close_tuner(port);
                                let _ = writeln!(stream, "{{\"status\":\"ok\"}}");
                            }

                            // SetChannel されてから呼ばれる前提
                            DaemonCommand::GetSignal { port } => {
                                // ptx_chrdev.c ptx_chrdev_unlock_ioctl(): switch PTX_GET_CNR 相当
                                // 監視中(StartStream前)は、バッファが溢れないように溜まったTSパケットを捨てる
                                if let Some(rx) = receivers.get(port) {
                                    while let Ok(_) = rx.try_recv() {}
                                }

                                let mut dev = px4_device.lock().unwrap();
                                // ドライバから現在の C/N 比 (dB値) 等を取得してクライアントに返す
                                match dev.get_cnr(port) {
                                    Ok(cnr_raw) => {
                                        // 必要であればここで raw 値を dB に計算し直すか、そのまま返す
                                        let _ = writeln!(
                                            stream,
                                            "{{\"status\":\"ok\",\"cnr\":{}}}",
                                            cnr_raw
                                        );
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
            // ★タイムアウト/WouldBlockは「データが来ていない」だけなので、
            // ループを継続して SHUTDOWN フラグを確認させる
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(e) => {
                error!("[error] Read error: {}", e);
                break;
            }
        }
    }
}

// RAIIを利用した自動クリーンアップ用構造体 (接続単位で管理)
struct ClientGuard<'a, B: BusOps + Send + Sync> {
    device: Arc<Mutex<Px4Device<'a, B>>>,
    port: Option<usize>,
}

impl<'a, B: BusOps + Send + Sync> Drop for ClientGuard<'a, B> {
    fn drop(&mut self) {
        // クライアント切断時(エラー終了含む)に必ずキャプチャを停止し、チューナーを閉じる
        if let Some(port) = self.port {
            if let Ok(mut dev) = self.device.lock() {
                let _ = dev.set_capture(port, false);
                let _ = dev.close_tuner(port);

                info!(
                    "[info] ClientGuard: Cleaned up port {} on disconnect.",
                    port
                );
            }
        }
    }
}
