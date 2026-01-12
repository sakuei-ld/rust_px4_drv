// USBバスレイヤー
// USBデバイスと通信するための最小API
// 上位層の it930x から USB を隠蔽している
// → USB実装を差し替えやすくなる、らしい。

use std::time::Duration;
use rusb::{Context, DeviceHandle};
use std::sync::Mutex;

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
    fn ctrl_rx(&self, buf: &mut [u8]) -> Result<(), BusError>;
    // ストリーム受信(Bulk In)
    fn stream_rx(&self, buf: &mut [u8], timeout: Duration) -> Result<usize, BusError>;
    // ストリーミング開始
    fn start_streaming(&self) -> Result<(), BusError>;
    // ストリーミング停止
    fn stop_streaming(&self) -> Result<(), BusError>;
}

// メモ: C の struct itedtv_bus に該当 するらしい
pub struct UsbBusRusb
{
    handle: Mutex<DeviceHandle<Context>>,
    ctrl_ep: u8,
    stream_ep: u8,
    ctrl_timeout: Duration,
}

impl UsbBusRusb
{
    pub fn new(handle: DeviceHandle<Context>) -> Self
    {
        Self 
        { 
            handle: Mutex::new(handle), 
            ctrl_ep: 0x02, 
            stream_ep: 0x84, 
            ctrl_timeout: Duration::from_millis(3000), // px4_usb_params.c px4_usb_params.ctrl_timeout から。
        }
    }
}

impl BusOps for UsbBusRusb
{
    // itedtv_bus.c の 47〜70 と思われる。
    fn ctrl_tx(&self, buf: &[u8]) -> Result<(), BusError> 
    {
        let guarded_handle = self.handle.lock().unwrap();
        //self.handle.write_bulk(self.ctrl_ep, buf, self.ctrl_timeout,)?;
        guarded_handle.write_bulk(self.ctrl_ep, buf, self.ctrl_timeout,)?;
        Ok(())    
    }

    // itedtv_bus.c の 72〜97 と思われる。
    fn ctrl_rx(&self, buf: &mut [u8]) -> Result<(), BusError>
    {
        let guarded_handle = self.handle.lock().unwrap();
        //let read_len = self.handle.read_bulk(self.ctrl_ep, buf, self.ctrl_timeout)?;
        let read_len = guarded_handle.read_bulk(self.ctrl_ep, buf, self.ctrl_timeout)?;

        if read_len != buf.len()
        {
            return Err(BusError::Other(format!("short read: {} != {}", read_len, buf.len())));
        }

        Ok(())
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
}

// ここまでが USBバスレイヤー