use clap::Parser;
use rusb::UsbContext;
use time::UtcOffset;
use tracing::{debug, error, info, warn};
use tracing_subscriber::{
    fmt, fmt::time::OffsetTime, layer::SubscriberExt, util::SubscriberInitExt, EnvFilter,
};

use std::net::TcpListener;
use std::sync::{Arc, Mutex};

// 既存のドライバをインポート
use rust_px4_drv_daemon::drivers::it930x::IT930x;
use rust_px4_drv_daemon::drivers::itedtv_bus::{BusOps, UsbBusRusb};
use rust_px4_drv_daemon::drivers::px4_card::BcasCard;
use rust_px4_drv_daemon::drivers::px4_device::Px4Device;

// server モジュールから公開型・定数をインポート
use rust_px4_drv_daemon::server::{ClientIo, SHUTDOWN};

use protocol::{BCAS_RAW_SERVER_PORT, BCAS_SERVER_PORT};

/// B-CAS TCPサーバー用クライアントハンドラ
fn handle_bcas_client<B: BusOps + Send + Sync>(
    stream: std::net::TcpStream,
    card: Arc<Mutex<BcasCard<'_, B>>>,
) {
    info!("[bcas] Client connected.");

    // 読み込みにタイムアウトを設定
    let _ = stream.set_read_timeout(Some(std::time::Duration::from_secs(30)));

    let mut io = match ClientIo::new(stream) {
        Ok(io) => io,
        Err(e) => {
            error!("[bcas] Failed to create ClientIo: {}", e);
            return;
        }
    };

    // クライアントからのコマンドを待つループ
    let mut line = String::new();

    while !SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
        line.clear();
        match io.read_line(&mut line) {
            Ok(0) => {
                info!("[bcas] Client disconnected.");
                break;
            }
            Ok(_) => {
                tracing::debug!("[bcas] recv raw line = {:?}", line);

                // JSONコマンドをパース
                let parsed: serde_json::Value = match serde_json::from_str(&line) {
                    Ok(v) => v,
                    Err(e) => {
                        send_error_bcas(&mut io, format!("Invalid JSON: {}", e));
                        continue;
                    }
                };

                let command = parsed.get("command").and_then(|v| v.as_str());

                match command {
                    Some("initialize") => {
                        let mut bcas = match card.lock() {
                            Ok(g) => g,
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Lock error: {}", e));
                                continue;
                            }
                        };

                        match bcas.bcas_initialize() {
                            Ok(resp) => {
                                // 57バイトのレスポンスをhex文字列として返す
                                let hex_resp: String = resp
                                    .iter()
                                    .map(|b| format!("{:02X}", b))
                                    .collect::<String>();

                                // BonCasProxy形式でレスポンスを返す
                                let response = serde_json::json!({
                                    "status": "ok",
                                    "data": hex_resp,
                                    "length": resp.len(),
                                });
                                let _ = io.send_json(&response);
                            }
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Initialize failed: {}", e));
                            }
                        }
                    }
                    Some("get_info") => {
                        let mut bcas = match card.lock() {
                            Ok(g) => g,
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Lock error: {}", e));
                                continue;
                            }
                        };

                        match bcas.bcas_get_info() {
                            Ok(resp) => {
                                let hex_resp: String = resp
                                    .iter()
                                    .map(|b| format!("{:02X}", b))
                                    .collect::<String>();

                                let response = serde_json::json!({
                                    "status": "ok",
                                    "data": hex_resp,
                                    "length": resp.len(),
                                    "card_version": resp.get(8).copied(),
                                    "manufacturer_id": resp.get(7).copied(),
                                });
                                let _ = io.send_json(&response);
                            }
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Get info failed: {}", e));
                            }
                        }
                    }
                    Some("read_channel") => {
                        let mut bcas = match card.lock() {
                            Ok(g) => g,
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Lock error: {}", e));
                                continue;
                            }
                        };

                        match bcas.bcas_read_channel() {
                            Ok(resp) => {
                                let hex_resp: String = resp
                                    .iter()
                                    .map(|b| format!("{:02X}", b))
                                    .collect::<String>();

                                let response = serde_json::json!({
                                    "status": "ok",
                                    "data": hex_resp,
                                    "length": resp.len(),
                                });
                                let _ = io.send_json(&response);
                            }
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Read channel failed: {}", e));
                            }
                        }
                    }
                    Some("get_card_id") => {
                        let mut bcas = match card.lock() {
                            Ok(g) => g,
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Lock error: {}", e));
                                continue;
                            }
                        };

                        match bcas.bcas_format_card_id() {
                            Ok((card_id, card_version, manufacturer_id)) => {
                                let response = serde_json::json!({
                                    "status": "ok",
                                    "card_id": card_id,
                                    "card_version": card_version,
                                    "manufacturer_id": manufacturer_id,
                                });
                                let _ = io.send_json(&response);
                            }
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Get card ID failed: {}", e));
                            }
                        }
                    }
                    Some("detect") => {
                        let bcas = match card.lock() {
                            Ok(g) => g,
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Lock error: {}", e));
                                continue;
                            }
                        };

                        match bcas.check_card_present() {
                            Ok(present) => {
                                let response = serde_json::json!({
                                    "status": "ok",
                                    "card_present": present,
                                });
                                let _ = io.send_json(&response);
                            }
                            Err(e) => {
                                send_error_bcas(&mut io, format!("Detect failed: {}", e));
                            }
                        }
                    }
                    _ => {
                        send_error_bcas(
                            &mut io,
                            format!(
                                "Unknown command: {}. Valid commands: initialize, get_info, read_channel, get_card_id, detect",
                                command.unwrap_or("null")
                            ),
                        );
                    }
                }
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(e) => {
                error!("[bcas] Read error: {}", e);
                break;
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Px4DeviceType {
    PX_W3U4,
    PX_Q3U4,
}

#[derive(Parser, Debug)]
#[command(name = "rust_px4_drv_daemon", about = "rust_px4_drv daemon")]
struct Cli {
    /// 待ち受けるIPアドレス（コンテナ外からアクセスする場合は 0.0.0.0 を指定）
    #[arg(long, default_value = "127.0.0.1")]
    host: String,

    /// TSストリーミング用の待ち受けポート番号
    #[arg(short, long, default_value_t = 40771)]
    port: u16,

    /// B-CASカード用JSON制御プロトコルの待ち受けポート番号
    #[arg(long, default_value_t = BCAS_SERVER_PORT)]
    bcas_port: u16,

    /// B-CASカードを有効化するか
    #[arg(long, default_value = "true")]
    enable_bcas: bool,

    /// bcs-perl.pl 互換のバイナリプロトコルサーバーを有効化するか
    /// (--enable-bcas が false の場合は強制的に無効になる)
    #[arg(long, default_value = "true")]
    enable_bcas_raw: bool,

    /// bcs-perl.pl 互換のバイナリプロトコルの待ち受けポート番号
    #[arg(long, default_value_t = BCAS_RAW_SERVER_PORT)]
    bcas_raw_port: u16,
}

/// B-CAS用エラーレスポンス送信ヘルパー
fn send_error_bcas(io: &mut ClientIo, msg: impl Into<String>) {
    let _ = io.send_json(&protocol::StatusResponse {
        status: "error".into(),
        message: Some(msg.into()),
    });
}

/// メインエントリーポイント
fn main() -> anyhow::Result<()> {
    // RUST_LOG 環境変数でレベルを制御可能にする (例: RUST_LOG=info ./daemon)
    // log の表示時刻の設定
    let timer = OffsetTime::new(
        UtcOffset::current_local_offset().unwrap_or(UtcOffset::UTC),
        time::format_description::well_known::Rfc3339,
    );

    // with_writer(std::io::stderr) が一般的らしい
    tracing_subscriber::registry()
        .with(fmt::layer().with_writer(std::io::stdout).with_timer(timer))
        .with(EnvFilter::from_default_env())
        .init();

    // 引数のパース
    let cli = Cli::parse();

    // まず、USB関連の準備
    let context = match rusb::Context::new() {
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

    // デバイスの初期化と共有管理
    let (device, receivers) =
        match Px4Device::new(&it930x, device_type == Px4DeviceType::PX_Q3U4, false) {
            Ok(v) => v,
            Err(e) => {
                anyhow::bail!("Failed to init Px4Device: {:?}", e);
            }
        };

    let shared_receivers = Arc::new(receivers);
    let shared_device: Arc<Mutex<Px4Device<'static, UsbBusRusb>>> =
        unsafe { Arc::new(Mutex::new(std::mem::transmute(device))) };

    // BCAS 用の準備（BcasCard作成・初期化・リスナーbind）
    // enable_bcas_raw は enable_bcas が true のみ有効
    let enable_bcas_raw = cli.enable_bcas && cli.enable_bcas_raw;

    let bcas_setup: Option<(Arc<Mutex<BcasCard<UsbBusRusb>>>, TcpListener, TcpListener)> =
        if cli.enable_bcas {
            let mut bcas_card = BcasCard::new(&it930x);

            // B-CASカードの初期化
            if let Err(e) = bcas_card.bcas_init() {
                warn!("[bcas] init failed: {:?}", e);
                None
            } else {
                // カードを検出してリセット
                let detected = bcas_card.detect().unwrap_or(false);
                if detected {
                    match bcas_card.bcas_reset_card() {
                        Ok(_) => {
                            // ATR検証を実行 - B-CASカードであることを確認
                            if let Err(e) = bcas_card.verify_atr() {
                                warn!("[bcas] ATR verification failed after reset: {}", e);
                                warn!("[bcas] Card may not be a valid B-CAS card");
                            } else {
                                info!("[bcas] ATR verification passed");
                            }
                        }
                        Err(e) => {
                            warn!("[bcas] reset failed: {}", e);
                        }
                    }
                } else {
                    warn!("[bcas] No B-CAS card detected");
                }

                let bcas_bind_addr = format!("{}:{}", cli.host, cli.bcas_port);
                let bcas_raw_bind_addr = format!("{}:{}", cli.host, cli.bcas_raw_port);
                match (
                    TcpListener::bind(&bcas_bind_addr),
                    TcpListener::bind(&bcas_raw_bind_addr),
                ) {
                    (Ok(bcas_listener), Ok(bcas_raw_listener)) => {
                        bcas_listener.set_nonblocking(true).unwrap();
                        bcas_raw_listener.set_nonblocking(true).unwrap();
                        Some((
                            Arc::new(Mutex::new(bcas_card)),
                            bcas_listener,
                            bcas_raw_listener,
                        ))
                    }
                    (Ok(_), Err(e)) => {
                        warn!("[bcas] raw bind failed: {}", e);
                        None
                    }
                    (Err(e), _) => {
                        warn!("[bcas] bind failed: {}", e);
                        None
                    }
                }
            }
        } else {
            None
        };

    // TCP ソケットの準備
    let bind_addr = format!("{}:{}", cli.host, cli.port);

    // シグナルハンドラ(Ctrl+Cキャッチ)の準備
    ctrlc::set_handler(move || {
        info!("Received shutdown signal. Cleaning up...");
        SHUTDOWN.store(true, std::sync::atomic::Ordering::SeqCst)
    })
    .expect("Error setting Ctrl-C handler");

    let listener = TcpListener::bind(&bind_addr)?;
    listener.set_nonblocking(true)?;

    info!(
        "[server] Daemon started. Waiting for connections on {}...",
        bind_addr
    );

    // BCAS用ポートの情報をログ出力
    if let Some((_, bcas_listener, bcas_raw_listener)) = &bcas_setup {
        if let Ok(addr) = bcas_listener.local_addr() {
            info!("[bcas] JSON control protocol listening on {}", addr);
        }
        if let Ok(addr) = bcas_raw_listener.local_addr() {
            info!(
                "[bcas] Raw (BonCasServer-compatible) protocol listening on {}",
                addr
            );
        }
    }

    // 3. クライアント接続待ちループ
    std::thread::scope(|s| {
        // BCAS 側のAcceptループを scoped thread で起動
        if let Some((card, bcas_listener, bcas_raw_listener)) = &bcas_setup {
            // カード監視ループ（新規追加）
            let card_monitor = Arc::clone(card);
            s.spawn(move || {
                info!("[bcas] Card monitor loop started");
                rust_px4_drv_daemon::bcas_raw_server::card_monitor_loop(card_monitor);
            });

            let card_srv = Arc::clone(card);
            s.spawn(move || {
                info!("[bcas] B-CAS server started.");
                loop {
                    if SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
                        break;
                    }
                    match bcas_listener.accept() {
                        Ok((stream, _)) => {
                            let card_client = Arc::clone(&card_srv);
                            s.spawn(move || {
                                handle_bcas_client(stream, card_client);
                            });
                        }
                        Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                            std::thread::sleep(std::time::Duration::from_millis(100));
                            continue;
                        }
                        Err(e) => {
                            error!("[bcas] Accept error: {}", e);
                            break;
                        }
                    }
                }
            });

            // bcs-perl.pl 互換バイナリプロトコル用のAcceptループ
            if enable_bcas_raw {
                let card_raw = Arc::clone(card);
                let bcas_raw_listener_clone = bcas_raw_listener.try_clone().unwrap();
                s.spawn(move || {
                    use rust_px4_drv_daemon::bcas_raw_server::handle_raw_client_thread;

                    info!("[bcas] Raw server accept loop started.");
                    loop {
                        if SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
                            break;
                        }
                        match bcas_raw_listener_clone.accept() {
                            Ok((stream, _)) => {
                                let card_cli = Arc::clone(&card_raw);
                                s.spawn(move || {
                                    if let Err(e) = handle_raw_client_thread(stream, card_cli) {
                                        warn!("[bcas] Raw client error: {}", e);
                                    }
                                });
                            }
                            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                                std::thread::sleep(std::time::Duration::from_millis(100));
                                continue;
                            }
                            Err(e) => {
                                error!("[bcas] Raw accept error: {}", e);
                                break;
                            }
                        }
                    }
                });
            }
        }

        // TS配信用のサーバーを起動
        let receivers_clone = Arc::clone(&shared_receivers);
        let device_clone = Arc::clone(&shared_device);
        let server_handle = s.spawn(move || {
            rust_px4_drv_daemon::server::run_server(listener, receivers_clone, device_clone)
        });

        // サーバーが終了するのを待つ（エラーがあっても無視）
        let _ = server_handle;
    });

    Ok(())
}
