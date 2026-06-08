use anyhow::{anyhow, Result};
use clap::{Parser, Subcommand};
use signal_hook::consts::{SIGINT, SIGPIPE, SIGTERM};
use signal_hook::flag;
use time::UtcOffset;
use tracing::{debug, error, info, instrument, warn};
use tracing_subscriber::{
    fmt, fmt::time::OffsetTime, layer::SubscriberExt, util::SubscriberInitExt, EnvFilter,
};

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
//use std::os::unix::net::UnixStream;
use std::net::TcpStream;

use protocol::{ChannelConfig, ChannelSpace, DaemonCommand, SignalResponse};

// --- CLI引数の定義 (clap を使用) ---
#[derive(Parser, Debug)]
#[command(name = "rust_px4_drv_shim", about = "rust_px4_drv shim client")]
struct Cli {
    #[command(subcommand)]
    command: Commands,

    /// 接続先のデーモンのIPアドレス
    #[arg(long, global = true, default_value = "127.0.0.1")]
    host: String,

    /// 接続先のデーモンのポート番号
    #[arg(short, long = "port", global = true, default_value_t = 40771)]
    tcp_port: u16,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// チューニングとストリームの受信を行います
    Tune {
        port: u8,
        space: String,
        channel: String,
        #[arg(long)]
        lnb_on: bool,
        /// 出力先ファイルパス。標準出力の場合は "-" を指定（デフォルト）
        #[arg(default_value = "-")]
        output: String,
    },
    /// シグナル強度を監視します（出力ファイルパス不要）
    Signal {
        port: u8,
        space: String,
        channel: String,
        #[arg(long)]
        lnb_on: bool,
    },
}

// 変換関数
fn calculate_db(raw: u64, space: &ChannelSpace) -> f64 {
    match space {
        ChannelSpace::Terrestrial => {
            // raw が 0 だとエラーになるのでガード
            if raw == 0 {
                return 0.0;
            }
            let p = (5505024.0 / (raw as f64)).log10() * 10.0;
            (0.000024 * p.powi(4)) - (0.0016 * p.powi(3))
                + (0.0398 * p.powi(2))
                + (0.5491 * p)
                + 3.0965
        }
        _ => {
            // Satellite (BS/CS)
            const AF_LEVEL_TABLE: [f64; 14] = [
                24.07, 24.07, 18.61, 15.21, 12.50, 10.19, 8.140, 6.270, 4.550, 3.730, 3.630, 2.940,
                1.420, 0.000,
            ];
            let sig = ((raw & 0xFF00) >> 8) as u8;
            if sig <= 0x10u8 {
                24.07
            } else if sig >= 0xB0u8 {
                0.0
            } else {
                let f_mix_rate = (((sig as u16 & 0x0F) << 8) | sig as u16) as f64 / 4096.0;
                AF_LEVEL_TABLE[(sig >> 4) as usize] * (1.0 - f_mix_rate)
                    + AF_LEVEL_TABLE[(sig >> 4) as usize + 0x01] * f_mix_rate
            }
        }
    }
}

/// 物理チャンネルとスロット番号から、デーモン（ハードウェア）が求める TSID を解決するヘルパー
fn resolve_tsid(space: &ChannelSpace, channel: u32, slot: u32) -> Option<u16> {
    match space {
        ChannelSpace::BroadcastingSatellite => {
            // 日本のBSトランスポンダのTSIDマッピング表 (近年の再編対応版)
            match (channel, slot) {
                (1, 0) => Some(16400),  // BS朝日
                (1, 1) => Some(16401),  // BS-TBS
                (1, 2) => Some(16402),  // BSテレ東
                (3, 0) => Some(16432),  // WOWOWプライム
                (5, 0) => Some(17488),  // WOWOWライブ
                (5, 1) => Some(17489),  // WOWOWシネマ
                (9, 0) => Some(16528),  // BS11
                (9, 1) => Some(16529),  // スターチャンネル等
                (13, 0) => Some(16592), // BS日テレ  👈 今回上手くいかなかったチャンネル
                (13, 1) => Some(16593), // BSフジ
                (19, 0) => Some(16689), // NHK BS 4K
                (21, 0) => Some(16721), // グリーンチャンネル
                (23, 0) => Some(16753), // BSよしもと
                (23, 1) => Some(16754), // BS松竹東急
                _ => {
                    // マッピングにない場合、一般的なISDB-Sの規則性から推測するフォールバック
                    // 規則: 0x4000 + (物理ch * 16) + slot
                    Some((0x4000 + (channel * 16) + slot) as u16)
                }
            }
        }
        ChannelSpace::CommunicationSatellite => {
            // CS (スカパー) の規則性: 0x6000 + (ND番号 * 16) + slot
            // ND2chなら (2 * 16) = 32 => 0x6020 (24608)
            Some((0x6000 + (channel * 16) + slot) as u16)
        }
        _ => None,
    }
}

fn main() -> Result<()> {
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

    // clap を用いて引数をパース
    let cli = Cli::parse();

    // コマンドに応じた変数の抽出
    let (mode, port, space_str, channel_arg, lnb_on, output_path) = match &cli.command {
        Commands::Tune {
            port,
            space,
            channel,
            lnb_on,
            output,
        } => ("tune", *port, space, channel, *lnb_on, Some(output.clone())),
        Commands::Signal {
            port,
            space,
            channel,
            lnb_on,
        } => ("signal", *port, space, channel, *lnb_on, None),
    };

    let space = match space_str.to_uppercase().as_str() {
        "GR" => ChannelSpace::Terrestrial,
        "BS" => ChannelSpace::BroadcastingSatellite,
        "CS" => ChannelSpace::CommunicationSatellite,
        _ => return Err(anyhow!("Invalid space: {}", space_str)),
    };

    // 引数(args[4])を「_」で分割してパースするロジックに変更
    let parts: Vec<&str> = channel_arg.split('_').collect();
    let channel: u32 = parts[0]
        .parse()
        .map_err(|_| anyhow!("Invalid channel number"))?;
    let slot: u32 = if parts.len() > 1 {
        parts[1]
            .parse()
            .map_err(|_| anyhow!("Invalid slot number"))?
    } else {
        0 // スロットの指定が省略された場合はデフォルト 0
    };

    // 衛星波（BS/CS）の場合、物理chとslotからTSID(u16)を自動導出
    let sub_channel = match space {
        ChannelSpace::BroadcastingSatellite | ChannelSpace::CommunicationSatellite => {
            resolve_tsid(&space, channel, slot)
        }
        _ => None, // 地デジ(GR)の場合はNone
    };

    let term = Arc::new(AtomicBool::new(false));
    flag::register(SIGINT, term.clone())?;
    flag::register(SIGTERM, term.clone())?;
    flag::register(SIGPIPE, term.clone())?;

    // Daemon のソケットに接続
    //let socket_path = "/tmp/px4-tuner.sock";
    //eprintln!("Connecting to daemon socket at {}...", socket_path);

    // Daemon Server に接続
    let bind_addr = format!("{}:{}", cli.host, cli.tcp_port);
    info!("Connecting to daemon at {}...", bind_addr);

    let mut stream = loop {
        //match UnixStream::connect(socket_path) {
        match TcpStream::connect(&bind_addr) {
            Ok(s) => {
                info!("Connected to daemon!");
                break s;
            }
            Err(e) => {
                // 接続失敗した場合は 500ms 待ってから再試行
                warn!("Failed to connect: {}. Retrying in 500ms...", e);
                std::thread::sleep(std::time::Duration::from_millis(500));
            }
        }
    };

    let mut reader = BufReader::new(stream.try_clone()?);

    // SetChannel コマンドを送信 (共通)
    let set_channel = DaemonCommand::SetChannel {
        port: port as usize,
        channel: ChannelConfig {
            space: space.clone(),
            channel: channel,
            sub_channel: sub_channel,
        },
        lnb_voltage: if lnb_on {
            Some(protocol::LnbMode::Volt15)
        } else {
            Some(protocol::LnbMode::Off)
        }, // あとでフラグ管理にする
    };

    serde_json::to_writer(&stream, &set_channel)?;
    // Daemon側の read_line のために改行が必要
    stream.write_all(b"\n")?;

    // ここで応答を待つ
    let mut response = String::new();
    match reader.read_line(&mut response) {
        Ok(n) => {
            info!("read_line ok n={} response={:?}", n, response);
        }
        Err(e) => {
            error!(
                "read_line err kind={:?} raw={:?}",
                e.kind(),
                e.raw_os_error()
            );
            return Err(e.into());
        }
    }

    // ここで "ok" かどうかチェックするロジックがあるとより堅牢です
    if !response.contains("\"status\":\"ok\"") {
        anyhow::bail!("SetChannel failed: {}", response);
    }

    //stream.set_read_timeout(Some(std::time::Duration::from_millis(200)))?;

    //reader
    //    .get_mut()
    //    .set_read_timeout(Some(std::time::Duration::from_millis(200)))?;

    // モード分岐
    match mode {
        "signal" => {
            stream.set_read_timeout(Some(std::time::Duration::from_millis(200)))?;
            reader
                .get_mut()
                .set_read_timeout(Some(std::time::Duration::from_millis(200)))?;

            eprintln!("Starting signal monitor (Ctrl+C to stop)...");
            while !term.load(Ordering::Acquire) {
                let get_signal = DaemonCommand::GetSignal {
                    port: port as usize,
                };

                if let Err(e) = serde_json::to_writer(&stream, &get_signal) {
                    error!("Failed to send command: {}", e);
                    break;
                }

                let _ = stream.write_all(b"\n");
                response.clear();

                match reader.read_line(&mut response) {
                    Ok(_) => {}
                    Err(ref e)
                        if e.kind() == std::io::ErrorKind::WouldBlock
                            || e.kind() == std::io::ErrorKind::TimedOut =>
                    {
                        eprintln!("signal timeout");
                        continue;
                    }
                    Err(e) => {
                        error!("Read error: {}", e);
                        break;
                    }
                }

                // JSONパースとdB変換
                if let Ok(res) = serde_json::from_str::<SignalResponse>(&response) {
                    if let Some(raw_cnr) = res.cnr {
                        let db = calculate_db(raw_cnr as u64, &space);
                        eprintln!("CNR: {:.2} dB (Raw: {})", db, raw_cnr);
                    } else {
                        error!(
                            "Error: {}",
                            res.message.unwrap_or_else(|| "Unknown".to_string())
                        );
                    }
                }

                // 1秒待機
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
            eprintln!("\nStopped.");
        }
        "tune" => {
            info!("[shim] Sending StartStream...");

            // StartStream コマンドを送信
            let start_stream = DaemonCommand::StartStream {
                port: port as usize,
            };

            serde_json::to_writer(&stream, &start_stream)?;
            stream.write_all(b"\n")?;

            info!("[shim] Waiting for StartStream response...");
            response.clear();
            reader.read_line(&mut response)?;

            info!("[shim] Received response: {}", response.trim());
            if !response.contains("\"status\":\"ok\"") {
                // リトライが大量に出ないよう追加
                std::thread::sleep(std::time::Duration::from_secs(1));
                anyhow::bail!("StartStream failed");
            }

            info!("[shim] Setting read timeouts...");
            stream.set_read_timeout(Some(std::time::Duration::from_millis(200)))?;
            reader
                .get_mut()
                .set_read_timeout(Some(std::time::Duration::from_millis(200)))?;

            info!(
                "[shim] Opening output path: {}",
                output_path.as_ref().unwrap()
            );
            // --- 出力先の抽象化 ---
            let out_path = output_path.unwrap();
            let mut writer: Box<dyn Write> = if out_path == "-" {
                //Box::new(BufWriter::with_capacity(128 * 1024, std::io::stdout()))
                Box::new(BufWriter::with_capacity(188 * 1024, std::io::stdout()))
            } else {
                info!("Recording to file: {}", out_path);
                Box::new(BufWriter::with_capacity(
                    //128 * 1024,
                    188 * 1024,
                    File::create(&out_path)?,
                ))
            };

            // Ctrl+C 等の終了要求を検知できるよう、タイムアウトを設定して読み込む
            // タイムアウトを設けることで、定期的に running フラグを確認できるようにする
            stream.set_read_timeout(Some(std::time::Duration::from_millis(200)))?;

            info!("[shim] Starting stream copy loop...");
            let mut buf = [0u8; 8192]; // 8KB 程度のバッファ

            // debug
            let mut file_bytes = 0u64;

            // ソケットから届く TS パケットを標準出力へコピーし続ける
            // running フラグを監視しながらループ
            while !term.load(Ordering::Acquire) {
                match reader.read(&mut buf) {
                    Ok(0) => {
                        info!("[shim] TCP Connection closed by Daemon (EOF).");
                        break; // EOF (Daemonが切断された)
                    }
                    Ok(n) => {
                        //if let Err(_) = stdout.write_all(&buf[..n]) {
                        if let Err(e) = writer.write_all(&buf[..n]) {
                            // mirakc側がパイプを閉じたら書き込みエラーになるので、それを検知して終了
                            error!("[shim] Write to output failed: {}", e);
                            break;
                        }
                        // debug
                        file_bytes += n as u64;
                    }
                    // タイムアウト時は何もせず、ループの先頭に戻ってフラグを再確認する
                    Err(ref e)
                        if e.kind() == std::io::ErrorKind::WouldBlock
                            || e.kind() == std::io::ErrorKind::TimedOut =>
                    {
                        continue;
                    }
                    Err(ref e) if e.raw_os_error() == Some(35) => {
                        continue;
                    }
                    Err(e) => {
                        error!("Socket read error: {}", e);
                        // 追加: 異常切断時は1秒待ってから終了し、無限ループ爆撃を防ぐ
                        std::thread::sleep(std::time::Duration::from_secs(1));
                        break;
                    }
                }
            }
            // 残りを書き出す
            writer.flush()?;
            info!("file bytes={}", file_bytes);
            info!("[shim] Exiting tune cleanly. file bytes={}", file_bytes);
        }
        _ => {
            anyhow::bail!("Invalid mode: {}", mode);
        }
    }

    Ok(())
}
