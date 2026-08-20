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
use crate::drivers::px4_card::SmartCardError;
use crate::drivers::px4_device::Px4Device;
use crate::error::{DaemonError, DaemonResult};
use crate::server::SHUTDOWN;

// 最大リトライ回数とインターバルを設定
const MAX_RETRIES: usize = 3;
const RETRY_INTERVAL_MS: u64 = 200;

/// スレッド内で実行される単一クライアントの処理
///
/// main.rs の scoped thread から呼び出されることを想定している。
pub fn handle_raw_client_thread<B: BusOps + Send + Sync>(
    mut stream: TcpStream,
    device: Arc<Mutex<Px4Device<B>>>,
) -> DaemonResult<()> {
    let peer = stream
        .peer_addr()
        .map(|a| a.to_string())
        .unwrap_or_default();
    info!("BCAS raw client connected: {}", peer);

    // 1. クライアント接続開始時に card_acquire を呼ぶ
    let need_reinit = {
        let mut dev = device
            .lock()
            .map_err(|e| DaemonError::Unknown(format!("Mutex poisoned: {}", e)))?;
        match dev.card_acquire() {
            Ok(v) => v,
            Err(e) => {
                error!("Failed to acquire card for client {}: {:?}", peer, e);
                return Ok(());
            }
        }
    };

    // 新規セッションの場合は、電源再投入直後のプロトコル状態(ボーレート/T=1シーケンス)を
    // 信用せず、必ず full_reset() でATRから取り直す
    if need_reinit {
        let mut dev = device
            .lock()
            .map_err(|e| DaemonError::Unknown(format!("Mutex poisoned: {}", e)))?;
        if let Err(e) = dev.card_full_reset() {
            error!(
                "[bcas] card_full_reset failed at session start for client {}: {:?}",
                peer, e
            );
            // カードが使える状態ではないため、リトライさせずここで切断する
            let _ = dev.card_release();
            return Ok(());
        }
        info!("[bcas] card re-initialized for new session: {}", peer);
    }

    // クライアント切断時に確実にな card_release されるようにスコープガードを仕込む
    // （または struct の Drop や defer パターン）
    struct CardGuard<B: BusOps + Send + Sync>(Arc<Mutex<Px4Device<B>>>);
    impl<B: BusOps + Send + Sync> Drop for CardGuard<B> {
        fn drop(&mut self) {
            if let Ok(mut dev) = self.0.lock() {
                let _ = dev.card_release();
            }
        }
    }

    let _guard = CardGuard(Arc::clone(&device));

    let _ = stream.set_read_timeout(Some(std::time::Duration::from_millis(500)));

    while !SHUTDOWN.load(std::sync::atomic::Ordering::SeqCst) {
        let mut len_buf = [0u8; 1];
        match stream.read_exact(&mut len_buf) {
            Ok(()) => {}
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(_) => break,
        }

        let len = len_buf[0] as usize;
        if len == 0 {
            info!("BCAS raw client sent length=0, disconnecting: {}", peer);
            break;
        }

        let mut req = vec![0u8; len];
        if stream.read_exact(&mut req).is_err() {
            warn!("failed to read {} bytes from client: {}", len, peer);
            break;
        }

        let res = {
            let mut attempt = 0;
            loop {
                // Drop 調査用(BCAS処理時間の計測)
                let lock_wait_start = std::time::Instant::now();

                let mut dev = match device.lock() {
                    Ok(g) => g,
                    Err(e) => {
                        error!("Px4Device mutex poisoned: {}", e);
                        break Err(e.to_string());
                    }
                };

                // Drop 調査用(BCAS処理時間の計測)
                let lock_wait_ms = lock_wait_start.elapsed().as_millis();

                // Drop 調査用(BCAS処理時間の計測)
                let xfer_start = std::time::Instant::now();

                // Px4Device 側の card_transceive を呼び出す
                match dev.card_transceive(&req) {
                    Ok(res) => break Ok(res),
                    Err(e) => {
                        attempt += 1;
                        if attempt > MAX_RETRIES {
                            error!(
                                "card transceive failed after {} attempts: {} (client: {})",
                                MAX_RETRIES, e, peer
                            );
                            break Err(e.to_string());
                        }

                        warn!(
                            "card transceive failed ({}), retrying ({}/{}) for client: {}",
                            e, attempt, MAX_RETRIES, peer
                        );

                        let needs_immediate_reset = matches!(&e, SmartCardError::Protocol(msg) if msg.contains("No protocol selected"));

                        if needs_immediate_reset || attempt >= MAX_RETRIES {
                            if let Err(reset_err) = dev.card_full_reset() {
                                warn!("failed to reset BCAS card during retry: {}", reset_err);
                            }
                        }

                        drop(dev);
                        std::thread::sleep(std::time::Duration::from_millis(RETRY_INTERVAL_MS));
                    }
                }

                let xfer_ms = xfer_start.elapsed().as_millis();

                if xfer_ms > 50 || lock_wait_ms > 20 {
                    warn!(
                        "card_transceive slow: xfer={}ms lock_wait={}ms client={}",
                        xfer_ms, lock_wait_ms, peer
                    );
                }
            }
        };

        let res = match res {
            Ok(r) => r,
            Err(_) => break,
        };

        if res.len() > 255 {
            error!(
                "response too large for 1-byte length prefix: {} bytes (client: {})",
                res.len(),
                peer
            );
            break;
        }

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
