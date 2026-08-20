use clap::Parser;
use rusb::UsbContext;
use time::UtcOffset;
use tracing::{error, info, warn};
use tracing_subscriber::{
    fmt, fmt::time::OffsetTime, layer::SubscriberExt, util::SubscriberInitExt, EnvFilter,
};

use std::net::TcpListener;
use std::sync::{Arc, Mutex};

// 既存のドライバをインポート
use rust_px4_drv_daemon::drivers::it930x::IT930x;
use rust_px4_drv_daemon::drivers::itedtv_bus::UsbBusRusb;
use rust_px4_drv_daemon::drivers::px4_card::SmartCardError;
use rust_px4_drv_daemon::drivers::px4_device::Px4Device;

// server モジュールから公開型・定数をインポート
use rust_px4_drv_daemon::server::SHUTDOWN;

use protocol::BCAS_RAW_SERVER_PORT;

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

    /// B-CASカードを有効化するか
    /// (bcs-perl.pl 互換のバイナリプロトコルサーバーの有効化フラグ)
    #[arg(long)]
    enable_bcas: bool,

    /// bcs-perl.pl 互換のバイナリプロトコルの待ち受けポート番号
    #[arg(long, default_value_t = BCAS_RAW_SERVER_PORT)]
    bcas_proxy_port: u16,
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

    let it930x = Arc::new(IT930x::new(bus));

    // デバイスの初期化と共有管理
    let (device, receivers) = match Px4Device::new(
        Arc::clone(&it930x),
        device_type == Px4DeviceType::PX_Q3U4,
        false,
    ) {
        Ok(v) => v,
        Err(e) => {
            anyhow::bail!("Failed to init Px4Device: {:?}", e);
        }
    };

    let shared_receivers = Arc::new(receivers);
    let shared_device = Arc::new(Mutex::new(device));

    // BCAS 用の準備（BcasCard作成・初期化・リスナーbind）
    let bcas_setup: Option<TcpListener> = if cli.enable_bcas {
        let mut dev = shared_device.lock().unwrap();

        // BCASを使用するためにカードを取得（バックエンド電源ON）
        if let Err(e) = dev.card_acquire() {
            warn!("[bcas] card_acquire failed: {:?}", e);
            None
        } else {
            // 初期リセット や ATR確認
            let init_res = (|| -> Result<(), SmartCardError> {
                let detected = dev.card_detect()?;
                if detected {
                    match dev.card_full_reset() {
                        Ok(_) => info!("[bcas] ATR verification passed"),
                        Err(e) => warn!("[bcas] initial reset failed: {}", e),
                    }
                } else {
                    warn!("[bcas] No B-CAS card detected");
                }
                Ok(())
            })();

            if let Err(e) = init_res {
                warn!("[bcas] BCAS init failed: {:?}", e);
            }

            // 初期化が終わったら一旦リリース（電源カウントダウン）
            let _ = dev.card_release();

            // リスナーのバインド
            let bcas_raw_bind_addr = format!("{}:{}", cli.host, cli.bcas_proxy_port);
            match TcpListener::bind(&bcas_raw_bind_addr) {
                Ok(bcas_raw_listener) => {
                    bcas_raw_listener.set_nonblocking(true).unwrap();
                    Some(bcas_raw_listener)
                }
                Err(e) => {
                    warn!("[bcas] Bcas Proxy bind failed: {}", e);
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

    // BCAS用ポートの情報をログ出力
    if let Some(bcas_raw_listener) = &bcas_setup {
        if let Ok(addr) = bcas_raw_listener.local_addr() {
            info!(
                "[bcas] BonCasServer-compatible protocol listening on {}",
                addr
            );
        }
    }

    // 3. クライアント接続待ちループ
    std::thread::scope(|s| {
        // BCAS 側のAcceptループを scoped thread で起動
        if let Some(bcas_raw_listener) = &bcas_setup {
            // カード監視ループ（新規追加）
            //let dev_monitor = Arc::clone(&shared_device);
            //s.spawn(move || {
            //    info!("[bcas] Card monitor loop started");
            //    rust_px4_drv_daemon::bcas_raw_server::card_monitor_loop(dev_monitor);
            //});

            // bcs-perl.pl 互換バイナリプロトコル用のAcceptループ
            let dev = Arc::clone(&shared_device);
            let bcas_raw_listener_clone = bcas_raw_listener.try_clone().unwrap();
            s.spawn(move || {
                use rust_px4_drv_daemon::bcas_raw_server::handle_raw_client_thread;

                info!("[bcas] Bcas Proxy Server accept loop started.");
                loop {
                    if SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
                        break;
                    }
                    match bcas_raw_listener_clone.accept() {
                        Ok((stream, _)) => {
                            let dev_cli = Arc::clone(&dev);
                            s.spawn(move || {
                                if let Err(e) = handle_raw_client_thread(stream, dev_cli) {
                                    warn!("[bcas] Bcas client error: {}", e);
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
