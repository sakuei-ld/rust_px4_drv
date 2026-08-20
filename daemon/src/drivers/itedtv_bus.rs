// USBバスレイヤー
// USBデバイスと通信するための最小API
// 上位層の it930x から USB を隠蔽している
// → USB実装を差し替えやすくなる、らしい。

use rusb::{Context, DeviceHandle};
use tracing::{error, info, instrument, warn};

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

#[derive(Debug)]
pub enum BusError {
    Usb(rusb::Error),
    Timeout,
    Disconnected,
    Other(String),
}

impl From<rusb::Error> for BusError {
    fn from(e: rusb::Error) -> Self {
        BusError::Usb(e)
    }
}

// メモ: C の struct itedtv_bus_operations に該当 するらしい
pub trait BusOps {
    // Control転送(Out)
    fn ctrl_tx(&self, buf: &[u8]) -> Result<(), BusError>;
    // Control転送(In)
    fn ctrl_rx(&self, buf: &mut [u8]) -> Result<usize, BusError>;
    // ストリーム受信(Bulk In)
    fn stream_rx(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, BusError>;
    // ストリーミング開始
    fn start_streaming(
        &self,
        handler: Box<dyn Fn(&[u8]) + Send + Sync + 'static>,
    ) -> Result<(), BusError>;
    // ストリーミング停止
    fn stop_streaming(&self) -> Result<(), BusError>;

    // max_bulk_size の取得
    fn max_bulk_size(&self) -> u32;
}

// メモ: C の struct itedtv_bus に該当 するらしい
pub struct UsbBusRusb {
    handle: Arc<DeviceHandle<Context>>, // Mutex は要らない、らしい
    ctrl_tx_ep: u8,
    ctrl_rx_ep: u8,
    stream_ep: u8,
    ctrl_timeout: Duration,
    max_bulk_size: u32,
    is_streaming: Mutex<bool>,

    stop_flag: Arc<AtomicBool>,

    consumer_thread: Mutex<Option<thread::JoinHandle<()>>>,
    producer_thread: Mutex<Option<thread::JoinHandle<()>>>,

    // Drop 調査用
    // このデバイスインスタンス固有の ctrl bus (0x02/0x81) 使用量計測。
    // it930x側ではなくここで数えることで、複数IT930xチップ構成でも混線しない。
    ctrl_msg_count: Arc<std::sync::atomic::AtomicU64>,
    ctrl_msg_bytes: Arc<std::sync::atomic::AtomicU64>,
}

impl UsbBusRusb {
    pub fn new(handle: DeviceHandle<Context>) -> Result<Self, BusError> {
        // 一応、Linux向けに、OSの標準ドライバが掴んでいた場合に、切り離す
        //#[cfg(target_os = "linux")]
        //{
        //    if let Ok(true) = handle.kernel_driver_activate(0) {
        //        handle.detach_kernel_driver(0)?;
        //    }
        //}

        let device = handle.device();

        let config = device.active_config_descriptor()?;

        info!("=== USB CONFIG DUMP ===");

        for interface in config.interfaces() {
            info!("Interface {}", interface.number());

            for interface_desc in interface.descriptors() {
                info!(
                    "  Alt={} Class={:02x}",
                    interface_desc.setting_number(),
                    interface_desc.class_code()
                );

                for ep in interface_desc.endpoint_descriptors() {
                    info!(
                        "    EP=0x{:02x} attr=0x{:02x} max_packet={}",
                        ep.address(),
                        ep.transfer_type() as u8,
                        ep.max_packet_size()
                    );
                }
            }
        }

        info!("=======================");

        // interface を占有する。
        handle.claim_interface(0)?;
        handle.set_alternate_setting(0, 0)?;

        // 通信路のリセット (詰まりの解消)
        let _ = handle.clear_halt(0x02);
        let _ = handle.clear_halt(0x81);
        let _ = handle.clear_halt(0x84);

        // 下記なら出来るらしいので、取り敢えず実装してみる
        // handle から device記述子 を取得
        let device_desc = handle.device().device_descriptor()?;
        let usb_version = device_desc.usb_version();

        // USB 1.1 未満はエラーとする
        //if usb_version < rusb::Version(1, 1, 0) {
        //    return Err(BusError::Other(format!(
        //        "USB device requires at least USB 1.1"
        //    )));
        //}

        // USBバージョンに基づいて、バルク転送の最大サイズを決定
        // USB2.0 (0x0200) 以上なら 512、それ未満 (1.1など) なら 64
        //let max_bulk_size = if usb_version >= rusb::Version(2, 0, 0) {
        //    512
        //} else {
        //    64
        //};

        let mut max_bulk_size = None;

        if let Ok(config) = device.active_config_descriptor() {
            for interface in config.interfaces() {
                for desc in interface.descriptors() {
                    for ep in desc.endpoint_descriptors() {
                        if ep.transfer_type() == rusb::TransferType::Bulk {
                            max_bulk_size = Some(ep.max_packet_size());
                            break;
                        }
                    }
                }
            }
        }

        let max_bulk_size = max_bulk_size.unwrap_or(64);

        info!(
            "USB Version: {:04x}, Max Bulk Size: {}",
            usb_version.0, max_bulk_size
        );

        Ok(Self {
            handle: Arc::new(handle),
            ctrl_tx_ep: 0x02,
            ctrl_rx_ep: 0x81,
            stream_ep: 0x84,
            ctrl_timeout: Duration::from_millis(3000), // px4_usb_params.c px4_usb_params.ctrl_timeout から。
            max_bulk_size: max_bulk_size as u32,       //max_bulk_size,
            is_streaming: Mutex::new(false),
            stop_flag: Arc::new(AtomicBool::new(false)),
            consumer_thread: Mutex::new(None),
            producer_thread: Mutex::new(None),

            // Drop 調査用
            ctrl_msg_count: Arc::new(std::sync::atomic::AtomicU64::new(0)),
            ctrl_msg_bytes: Arc::new(std::sync::atomic::AtomicU64::new(0)),
        })
    }
}

impl BusOps for UsbBusRusb {
    // itedtv_bus.c の 47〜70 と思われる。
    fn ctrl_tx(&self, buf: &[u8]) -> Result<(), BusError> {
        self.handle
            .write_bulk(self.ctrl_tx_ep, buf, self.ctrl_timeout)?;

        // Drop 調査用
        // 実際に送信した全バイト数(ヘッダ+checksum込み)をそのまま計上。
        // 呼び出し回数は tx 側でのみ+1する(1回のctrl_msg呼び出し = tx1回+rx1回のペアなので、
        // "msgs" は tx 側だけでカウントすれば二重にならない)。
        self.ctrl_msg_count.fetch_add(1, Ordering::Relaxed);
        self.ctrl_msg_bytes
            .fetch_add(buf.len() as u64, Ordering::Relaxed);

        thread::sleep(Duration::from_millis(1));
        Ok(())
    }

    // itedtv_bus.c の 72〜97 と思われる。
    fn ctrl_rx(&self, buf: &mut [u8]) -> Result<usize, BusError> {
        let read_len = self
            .handle
            .read_bulk(self.ctrl_rx_ep, buf, self.ctrl_timeout)?;

        // Drop 調査用
        // RX側も実際に読めたバイト数を計上(以前は完全に漏れていた分)
        self.ctrl_msg_bytes
            .fetch_add(read_len as u64, Ordering::Relaxed);

        thread::sleep(Duration::from_millis(1));
        Ok(read_len)
    }

    // itedtv_bus.c の 99〜118 と思われる。
    fn stream_rx(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, BusError> {
        match self.handle.read_bulk(self.stream_ep, buf, timeout) {
            Ok(size) => {
                info!("stream_rx ok ep=0x{:02x} size={}", self.stream_ep, size);

                Ok(size)
            }
            Err(e) => {
                error!("stream_rx failed ep=0x{:02x} err={:?}", self.stream_ep, e);

                Err(e.into())
            }
        }
    }

    // itedtv_bus.c の 411〜509 と思われる。
    // mutex や メモリ確保、とかのように見える。
    #[instrument(skip(self, handler), fields(ep = self.stream_ep))]
    fn start_streaming(
        &self,
        handler: Box<dyn Fn(&[u8]) + Send + Sync + 'static>,
    ) -> Result<(), BusError> {
        let mut streaming = self.is_streaming.lock().unwrap();
        if *streaming {
            return Ok(());
        }

        *streaming = true;

        info!("Start stream...");

        // スレッドを停止させるためのセッター変数
        self.stop_flag.store(false, Ordering::SeqCst);
        let stop_flag_thread = self.stop_flag.clone();

        // stream ep
        let ep = self.stream_ep;

        // handle をクローンして移動させる
        // DeviceHandleのclone (rusb仕様)
        let handle_arc = self.handle.clone();

        // Producer と Consumer を繋ぐチャネル (容量が 128KB * 100 らしい)
        let (raw_tx, raw_rx) = crossbeam_channel::bounded::<bytes::Bytes>(2000); // ちょっと増やしてみた

        let mut buf = vec![0u8; 68 * 512];

        // macOSのUSBエンドポイント(Data Toggle)をクリーンにリセットする
        let _ = handle_arc.clear_halt(ep);

        // debug
        let packet_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let byte_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let packet_count_consumer = packet_count.clone();
        let byte_count_consumer = byte_count.clone();
        let total_packet_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let total_byte_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let total_packet_count_consumer = total_packet_count.clone();
        let total_byte_count_consumer = total_byte_count.clone();

        // drop数を共有するためのアトミックカウンタ
        let internal_drop_counter = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let internal_drop_consumer = internal_drop_counter.clone();
        let internal_drop_producer = internal_drop_counter.clone();

        // Drop 調査用
        // ctrl busの統計もコンシューマスレッドで一緒に読めるようにclone
        let ctrl_msg_count = self.ctrl_msg_count.clone();
        let ctrl_msg_bytes = self.ctrl_msg_bytes.clone();

        // Consumer スレッド (パースと分配の専任)
        let consumer_handle = thread::spawn(move || {
            // ここで span を生成すると、スレッド内の処理が「どのコンテキストか」明確になる
            let span = tracing::info_span!("usb_consumer_loop");
            let _enter = span.enter();

            // debug
            let mut last_log = std::time::Instant::now();

            // raw_rx にデータが届く限り、ひたすらハンドラを回す
            while let Ok(data) = raw_rx.recv() {
                // debug
                packet_count_consumer.fetch_add(1, Ordering::Relaxed);
                byte_count_consumer.fetch_add(data.len() as u64, Ordering::Relaxed);
                total_packet_count_consumer.fetch_add(1, Ordering::Relaxed);
                total_byte_count_consumer.fetch_add(data.len() as u64, Ordering::Relaxed);

                let elapsed = last_log.elapsed();
                if elapsed >= Duration::from_secs(5) {
                    // Drop 調査用
                    // 固定値(5.0)ではなく実測値を使う
                    let elapsed_secs = elapsed.as_secs_f64();

                    let usb_reads = packet_count_consumer.swap(0, Ordering::Relaxed);
                    let usb_bytes = byte_count_consumer.swap(0, Ordering::Relaxed);
                    let drops = internal_drop_consumer.swap(0, Ordering::Relaxed);
                    let ctrl_msgs = ctrl_msg_count.swap(0, Ordering::Relaxed);
                    let ctrl_bytes = ctrl_msg_bytes.swap(0, Ordering::Relaxed);

                    // "packets" ではなく "reads"(USBバルク読み取り回数)であることを明示
                    info!(
                        "usb bulk in: reads={} bytes={} rate={:.2}MiB/s (interval={:.3}s)",
                        usb_reads,
                        usb_bytes,
                        usb_bytes as f64 / 1024.0 / 1024.0 / elapsed_secs,
                        elapsed_secs
                    );

                    if drops > 0 {
                        warn!(
                            "{} USB reads dropped in the last interval (parser thread too slow)",
                            drops
                        );
                    }

                    info!("raw_rx queue depth: {}", raw_rx.len());
                    info!(
                        "ctrl bus: msgs={} bytes={} (interval={:.3}s)",
                        ctrl_msgs, ctrl_bytes, elapsed_secs
                    );

                    last_log = std::time::Instant::now();
                }

                // panic をキャッチしてログに出す
                if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    handler(&data);
                }))
                .is_err()
                {
                    // パニックしたらスレッドを終了させる
                    error!("Handler panicked! Thread will terminate.");
                    break;
                }
            }

            // debug
            info!(
                "consumer total: usb_reads={} bytes={}",
                total_packet_count_consumer.load(Ordering::Relaxed),
                total_byte_count_consumer.load(Ordering::Relaxed)
            );
            info!("Parser thread terminated.");
        });

        // Producer スレッド (USB受信の専任)
        let producer_handle = thread::spawn(move || {
            // ここで span を生成すると、スレッド内の処理が「どのコンテキストか」明確になる
            let span = tracing::info_span!("usb_producer_loop");
            let _enter = span.enter();

            // debug
            let mut usb_bytes = 0u64;
            let mut internal_drop = 0u64;

            while !stop_flag_thread.load(Ordering::Acquire) {
                let result = handle_arc.read_bulk(ep, &mut buf, Duration::from_millis(100));

                match result {
                    Ok(len) => {
                        // メモリコピー (128KBなら数usで終わる軽量処理、らしい)
                        let data = bytes::Bytes::copy_from_slice(&buf[..len]);
                        //info!("read_bulk size={}", len);

                        // try_send を使い、PC側の処理が遅れても USB の読み取りは止めない
                        if raw_tx.try_send(data).is_err() {
                            //debug
                            internal_drop_producer.fetch_add(1, Ordering::Relaxed);
                            internal_drop += 1;
                            //warn!("Warning: Parser thread is too slow! Internal drop occurred.");
                        }

                        // debug
                        usb_bytes += len as u64;
                    }
                    Err(rusb::Error::Timeout) => {
                        continue;
                    }
                    Err(e) => {
                        error!(
                            "stream read_bulk failed ep=0x{:02x} err={:?} buf_len={}",
                            ep,
                            e,
                            buf.len()
                        );
                        break;
                    }
                }
            }
            info!("Stream thread terminated.");
            // debug
            info!("USB received {} bytes", usb_bytes);
            info!("internal_drop={}", internal_drop);
        });

        *self.consumer_thread.lock().unwrap() = Some(consumer_handle);
        *self.producer_thread.lock().unwrap() = Some(producer_handle);
        Ok(())
    }

    // itedtv_bus.c の 511〜540 と思われる。
    // たぶん、streaming の開始で取った諸々を片付ける処理が入っている、と思われる。
    fn stop_streaming(&self) -> Result<(), BusError> {
        let mut streaming = self.is_streaming.lock().unwrap();

        if !*streaming {
            return Ok(());
        }

        self.stop_flag.store(true, Ordering::SeqCst);

        info!("Waiting stream thread...");

        if let Some(handle) = self.producer_thread.lock().unwrap().take() {
            let _ = handle.join();
        }

        if let Some(handle) = self.consumer_thread.lock().unwrap().take() {
            let _ = handle.join();
        }

        *streaming = false;

        info!("Stopped stream.");

        Ok(())
    }

    fn max_bulk_size(&self) -> u32 {
        self.max_bulk_size
    }
}

impl Drop for UsbBusRusb {
    fn drop(&mut self) {
        // デバイスが破棄されるときに自動で呼ばれる (C の itedtv_bus_term に相当)
        info!("Terminating bus...");

        // もしストリーミング中なら止める
        let streaming = self.is_streaming.lock().unwrap();
        if *streaming {
            // ここでデバイスへ停止コマンドを送る等の処理が必要なら呼ぶ
            // 今回はフラグを下ろすだけですが、実機に合わせて拡張可能、らしい
            println!("Auto-stopping stream in drop()");
        }

        // 占有していたインターフェースの解放
        let _ = self.handle.release_interface(0);

        // 一応、Linux 用に、OSのドライバに制御を戻しておく
        //#[cfg(target_os = "linux")]
        //let _ = handle.attach_kernel_driver(0);

        // Arc<Mutex<DeviceHandle>> なので、
        // ここでハンドルがスコープを抜ければ、USBデバイスは自動的にクローズされる、らしい。
    }
}
