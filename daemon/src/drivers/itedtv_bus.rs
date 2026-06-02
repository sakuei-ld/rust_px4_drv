// USBバスレイヤー
// USBデバイスと通信するための最小API
// 上位層の it930x から USB を隠蔽している
// → USB実装を差し替えやすくなる、らしい。

use rusb::{Context, DeviceHandle};
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

    stream_thread: Mutex<Option<thread::JoinHandle<()>>>,
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

        // interface を占有する。
        handle.claim_interface(0)?;

        // 通信路のリセット (詰まりの解消)
        let _ = handle.clear_halt(0x02);
        let _ = handle.clear_halt(0x81);
        let _ = handle.clear_halt(0x84);

        // 下記なら出来るらしいので、取り敢えず実装してみる
        // handle から device記述子 を取得
        let device_desc = handle.device().device_descriptor()?;
        let usb_version = device_desc.usb_version();

        // USB 1.1 未満はエラーとする
        if usb_version < rusb::Version(1, 1, 0) {
            return Err(BusError::Other(format!(
                "USB device requires at least USB 1.1"
            )));
        }

        // USBバージョンに基づいて、バルク転送の最大サイズを決定
        // USB2.0 (0x0200) 以上なら 512、それ未満 (1.1など) なら 64
        let max_bulk_size = if usb_version >= rusb::Version(2, 0, 0) {
            512
        } else {
            64
        };

        println!(
            "[usb_bus] USB Version: {:04x}, Max Bulk Size: {}",
            usb_version.0, max_bulk_size
        );

        Ok(Self {
            handle: Arc::new(handle),
            ctrl_tx_ep: 0x02,
            ctrl_rx_ep: 0x81,
            stream_ep: 0x84,
            ctrl_timeout: Duration::from_millis(3000), // px4_usb_params.c px4_usb_params.ctrl_timeout から。
            max_bulk_size,                             //max_bulk_size,
            is_streaming: Mutex::new(false),
            stop_flag: Arc::new(AtomicBool::new(false)),
            stream_thread: Mutex::new(None),
        })
    }
}

impl BusOps for UsbBusRusb {
    // itedtv_bus.c の 47〜70 と思われる。
    fn ctrl_tx(&self, buf: &[u8]) -> Result<(), BusError> {
        //let guarded_handle = self.handle.lock().unwrap();
        //guarded_handle.write_bulk(self.ctrl_tx_ep, buf, self.ctrl_timeout)?;
        self.handle
            .write_bulk(self.ctrl_tx_ep, buf, self.ctrl_timeout)?;

        thread::sleep(Duration::from_millis(1));
        Ok(())
    }

    // itedtv_bus.c の 72〜97 と思われる。
    fn ctrl_rx(&self, buf: &mut [u8]) -> Result<usize, BusError> {
        let read_len = self
            .handle
            .read_bulk(self.ctrl_rx_ep, buf, self.ctrl_timeout)?;

        thread::sleep(Duration::from_millis(1));
        Ok(read_len)
    }

    // itedtv_bus.c の 99〜118 と思われる。
    fn stream_rx(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, BusError> {
        let size = self.handle.read_bulk(self.stream_ep, buf, timeout)?;
        Ok(size)
    }

    // itedtv_bus.c の 411〜509 と思われる。
    // mutex や メモリ確保、とかのように見える。
    fn start_streaming(
        &self,
        handler: Box<dyn Fn(&[u8]) + Send + Sync + 'static>,
    ) -> Result<(), BusError> {
        let mut streaming = self.is_streaming.lock().unwrap();
        if *streaming {
            return Ok(());
        }

        *streaming = true;

        println!("[usb_bus] Start stream...");

        // スレッドを停止させるためのセッター変数
        self.stop_flag.store(false, Ordering::SeqCst);
        let stop_flag_thread = self.stop_flag.clone();

        // stream ep
        let ep = self.stream_ep;

        // handle をクローンして移動させる
        // DeviceHandleのclone (rusb仕様)
        let handle_arc = self.handle.clone();

        // Producer と Consumer を繋ぐチャネル (容量が 128KB * 100 らしい)
        let (raw_tx, raw_rx) = crossbeam_channel::bounded::<bytes::Bytes>(100);

        // Consumer スレッド (パースと分配の専任)
        thread::spawn(move || {
            // raw_rx にデータが届く限り、ひたすらハンドラを回す
            while let Ok(data) = raw_rx.recv() {
                handler(&data);
            }
            println!("[usb_bus] Parser thread terminated.");
        });

        // Producer スレッド (USB受信の専任)
        let join_handle = thread::spawn(move || {
            let mut buf = vec![0u8; 128 * 1024];

            while !stop_flag_thread.load(Ordering::Acquire) {
                let result = handle_arc.read_bulk(ep, &mut buf, Duration::from_millis(100));

                match result {
                    Ok(len) => {
                        // メモリコピー (128KBなら数usで終わる軽量処理、らしい)
                        let data = bytes::Bytes::copy_from_slice(&buf[..len]);

                        // try_send を使い、PC側の処理が遅れても USB の読み取りは止めない
                        if let Err(_) = raw_tx.try_send(data) {
                            println!("[usb_bus] Warning: Parser thread is too slow! Internal drop occurred.");
                        }
                    }
                    Err(rusb::Error::Timeout) => {
                        continue;
                    }
                    Err(e) => {
                        println!("[usb_bus] stream read error: {:?}", e);
                        break;
                    }
                }
            }
            println!("[usb_bus] Stream thread terminated.");
        });

        *self.stream_thread.lock().unwrap() = Some(join_handle);
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

        println!("[usb_bus] Waiting stream thread...");

        if let Some(handle) = self.stream_thread.lock().unwrap().take() {
            let _ = handle.join();
        }

        *streaming = false;

        println!("[usb_bus] Stopped stream.");

        Ok(())
    }

    fn max_bulk_size(&self) -> u32 {
        self.max_bulk_size
    }
}

impl Drop for UsbBusRusb {
    fn drop(&mut self) {
        // デバイスが破棄されるときに自動で呼ばれる (C の itedtv_bus_term に相当)
        println!("[usb_bus] Terminating bus...");

        // もしストリーミング中なら止める
        let streaming = self.is_streaming.lock().unwrap();
        if *streaming {
            // ここでデバイスへ停止コマンドを送る等の処理が必要なら呼ぶ
            // 今回はフラグを下ろすだけですが、実機に合わせて拡張可能、らしい
            println!("[usb_bus] Auto-stopping stream in drop()");
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
