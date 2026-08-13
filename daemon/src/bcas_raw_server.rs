//! bcs-perl.pl 互換のバイナリTCPプロトコルサーバー
//!
//! プロトコルフォーマット:
//! - リクエスト: [1バイトの長さ L][Lバイトの生APDU]
//! - レスポンス: [1バイトの長さ][応答バイト列]
//! - 長さ `0` の受信は切断要求として扱う
//!
//! 参考: https://github.com/walkure/bcs-perl の bcs-perl.pl

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::{Arc, Mutex};

use tracing::{error, info, warn};

use crate::drivers::itedtv_bus::BusOps;
use crate::drivers::px4_card::{BcasCard, SmartCardInterface};
use crate::error::DaemonResult;
use crate::server::SHUTDOWN;

/// カードの抜き差しを監視し、必要なら再初期化する。
/// bcs-perl.pl のメインループにおける Status() チェック相当。
///
/// このループは、main.rs の外側の `std::thread::scope` の中で
/// `s.spawn(...)` により起動されることを前提とする（'static を要求しない）。
pub fn card_monitor_loop<'a, B: BusOps + Send + Sync>(card: Arc<Mutex<BcasCard<'a, B>>>) {
    loop {
        if SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
            tracing::info!("[bcas] Card monitor loop shutting down");
            break;
        }

        std::thread::sleep(std::time::Duration::from_millis(100));

        // SHUTDOWNが立った直後にsleepから復帰した場合に備えて再確認
        if SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
            break;
        }

        let card_guard = match card.lock() {
            Ok(g) => g,
            Err(e) => {
                tracing::error!("BCAS card mutex poisoned: {}", e);
                continue;
            }
        };

        // カードが検出されない場合、再初期化を試みる
        if !card_guard.detect().unwrap_or(false) {
            tracing::warn!("BCAS card not detected, attempting re-initialization");
            drop(card_guard); // ロックを解放してから再初期化

            std::thread::sleep(std::time::Duration::from_millis(100));

            let mut card_guard = match card.lock() {
                Ok(g) => g,
                Err(e) => {
                    tracing::error!("BCAS card mutex poisoned during re-init: {}", e);
                    continue;
                }
            };

            match card_guard.reset() {
                Ok(_card_info) => {
                    // ATR検証を実行 - B-CASカードであることを確認
                    if let Err(e) = card_guard.verify_atr() {
                        tracing::warn!("re-initialized card has unexpected ATR: {}", e);
                    } else {
                        tracing::info!("BCAS card re-initialized successfully");
                    }
                }
                Err(e) => {
                    tracing::warn!("BCAS card re-initialization failed: {}", e);
                }
            }
        }
    }
}

/// スレッド内で実行される単一クライアントの処理
///
/// main.rs の scoped thread から呼び出されることを想定している。
pub fn handle_raw_client_thread<B: BusOps + Send + Sync>(
    mut stream: TcpStream,
    card: Arc<Mutex<BcasCard<'_, B>>>,
) -> DaemonResult<()> {
    let peer = stream
        .peer_addr()
        .map(|a| a.to_string())
        .unwrap_or_default();
    info!("BCAS raw client connected: {}", peer);

    // 読み取りタイムアウトを設定し、定期的にSHUTDOWNを確認できるようにする
    let _ = stream.set_read_timeout(Some(std::time::Duration::from_millis(500)));

    while !SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
        // 1バイトの長さフィールドを読み込む
        let mut len_buf = [0u8; 1];
        match stream.read_exact(&mut len_buf) {
            Ok(()) => {}
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                // タイムアウトはデータが来ていないだけなので、SHUTDOWNを確認して継続
                continue;
            }
            Err(_) => break, // EOF/その他のエラーは切断
        }

        let len = len_buf[0] as usize;

        // 長さ0は明示的な切断要求
        if len == 0 {
            info!("BCAS raw client sent length=0, disconnecting: {}", peer);
            break;
        }

        // APDUデータを読み込む
        let mut req = vec![0u8; len];
        // データ部分もタイムアウトしうるが、長さが分かった以上は最後まで読み切りたいので
        // read_exact のままでよい（タイムアウト時はエラーとして切断）
        if stream.read_exact(&mut req).is_err() {
            warn!("failed to read {} bytes from client: {}", len, peer);
            break;
        }

        // カードにAPDUを送信し、応答を取得
        let res = {
            let mut card_guard = card.lock().unwrap();
            match card_guard.transceive_raw(&req) {
                Ok(res) => res,
                Err(e) => {
                    error!("card transceive failed: {}", e);
                    // エラー時は当該クライアント接続だけを切断
                    break;
                }
            }
        };

        // 応答が255バイトを超える場合はエラー（1バイト長プレフィックスで表現できない）
        if res.len() > 255 {
            error!(
                "response too large for 1-byte length prefix: {} bytes (client: {})",
                res.len(),
                peer
            );
            break;
        }

        // レスポンスを送信: [1バイトの長さ][応答バイト列]
        let mut out = Vec::with_capacity(res.len() + 1);
        out.push(res.len() as u8);
        out.extend_from_slice(&res);
        if stream.write_all(&out).is_err() {
            warn!("failed to send response to client: {}", peer);
            break;
        }
    }

    info!("BCAS raw client disconnected: {}", peer);
    Ok(())
}
