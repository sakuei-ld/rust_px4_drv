// USBバスレイヤー
// USBデバイスと通信するための最小API
// 上位層の it930x から USB を隠蔽している
// → USB実装を差し替えやすくなる、らしい。

use std::time::Duration;
use rusb::{Context, DeviceHandle};
use std::sync::Mutex;
use std::thread;

#[derive(Debug)]
pub enum BusError 
{
    Usb(rusb::Error),
    Timeout,
    Disconnected,
    Other(String),   
}

impl From<rusb::Error> for BusError
{
    fn from(e: rusb::Error) -> Self
    {
        BusError::Usb(e)
    }
}

// メモ: C の struct itedtv_bus_operations に該当 するらしい
pub trait BusOps
{
    // Control転送(Out)
    fn ctrl_tx(&self, buf: &[u8]) -> Result<(), BusError>;
    // Control転送(In)
    fn ctrl_rx(&self, buf: &mut [u8]) -> Result<usize, BusError>;
    // ストリーム受信(Bulk In)
    fn stream_rx(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, BusError>;
    // ストリーミング開始
    fn start_streaming(&self) -> Result<(), BusError>;
    // ストリーミング停止
    fn stop_streaming(&self) -> Result<(), BusError>;

    // max_bulk_size の取得
    fn max_bulk_size(&self) -> u32;
}

// メモ: C の struct itedtv_bus に該当 するらしい
pub struct UsbBusRusb
{
    handle: Mutex<DeviceHandle<Context>>,
    ctrl_tx_ep: u8,
    ctrl_rx_ep: u8,
    stream_ep: u8,
    ctrl_timeout: Duration,
    max_bulk_size: u32,
    //streaming: // ... あとで足す
}

impl UsbBusRusb
{
    pub fn new(handle: DeviceHandle<Context>) -> Result<Self, BusError>
    {
        // ここだとダメ
        // main.rs で Device<Context> としている箇所じゃないといけないので、後で調整
        //let usb_version = handle.device_descriptor().usb_version();
        //if usb_version < 0x0110 
        //{
        //    return Err(BusError::Other(format!("USB device requires at least USB 1.1")));
        //}

        //let max_bulk_size = if usb_version == 0x0110 { 64 } else { 512 };

        Ok(Self 
        { 
            handle: Mutex::new(handle), // ここを Arc<Mutex<DeviceHandle<rusb::Context>>> として、参照カウントに対応するようにした方がいいと言われた
            ctrl_tx_ep: 0x02,
            ctrl_rx_ep: 0x81, 
            stream_ep: 0x84, 
            ctrl_timeout: Duration::from_millis(3000), // px4_usb_params.c px4_usb_params.ctrl_timeout から。
            max_bulk_size: 512, //max_bulk_size,
        })
    }
}

impl BusOps for UsbBusRusb
{
    // itedtv_bus.c の 47〜70 と思われる。
    fn ctrl_tx(&self, buf: &[u8]) -> Result<(), BusError> 
    {
        let guarded_handle = self.handle.lock().unwrap();
        //self.handle.write_bulk(self.ctrl_ep, buf, self.ctrl_timeout,)?;
        guarded_handle.write_bulk(self.ctrl_tx_ep, buf, self.ctrl_timeout,)?;

        thread::sleep(Duration::from_millis(1));
        Ok(())    
    }

    // itedtv_bus.c の 72〜97 と思われる。
    fn ctrl_rx(&self, buf: &mut [u8]) -> Result<usize, BusError>
    {
        let guarded_handle = self.handle.lock().unwrap();
        //let read_len = self.handle.read_bulk(self.ctrl_ep, buf, self.ctrl_timeout)?;
        let read_len = guarded_handle.read_bulk(self.ctrl_rx_ep, buf, self.ctrl_timeout)?;

        // あとで消す
        //if read_len != buf.len()
        //{
        //    return Err(BusError::Other(format!("short read: {} != {}", read_len, buf.len())));
        //}

        thread::sleep(Duration::from_millis(1));
        Ok(read_len)
    }

    // itedtv_bus.c の 99〜118 と思われる。
    fn stream_rx(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, BusError>
    {
        let guarded_handle = self.handle.lock().unwrap();
        //let size = self.handle.read_bulk(self.stream_ep, buf, self.stream_timeout)?;
        let size = guarded_handle.read_bulk(self.stream_ep, buf, timeout)?;
        Ok(size)
    }

    // itedtv_bus.c の 411〜509 と思われる。
    // mutex や メモリ確保、とかのように見える。
    fn start_streaming(&self) -> Result<(), BusError> 
    {
        // C では、フラグ管理らしいので、それらしきことをすれば良い？
        Ok(())    
    }

    // itedtv_bus.c の 511〜540 と思われる。
    // たぶん、streaming の開始で取った諸々を片付ける処理が入っている、と思われる。
    fn stop_streaming(&self) -> Result<(), BusError> 
    {
        // C では、フラグ管理らしいので、それらしきことをすれば良い？
        Ok(())
    }

    fn max_bulk_size(&self) -> u32 
    {
        self.max_bulk_size
    }
}

// ここまでが USBバスレイヤー