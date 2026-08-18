// コントロールメッセージ層
// IT930xデバイスとやりとりする独自バイナリプロトコルの実装層
use thiserror::Error;
use tracing::{error, info};

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Mutex;

use crate::drivers::itedtv_bus::{BusError, BusOps};

// debug用
static CTRL_MSG_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static CTRL_MSG_BYTES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

// どこか定期実行される場所(例えばconsumerループと同じ5秒周期)から呼べる getter
pub fn take_ctrl_stats() -> (u64, u64) {
    (
        CTRL_MSG_COUNT.swap(0, Ordering::Relaxed),
        CTRL_MSG_BYTES.swap(0, Ordering::Relaxed),
    )
}


// これに関しては、いろんなところで使うので、実際はここじゃない方が良いかもしれない。
// 下記2つで i2c_comm.h 17 〜 26 の移植
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2CRequestType {
    Read,
    Write,
}

pub struct I2CCommRequest<'a> {
    pub addr: u8,
    pub data: &'a mut [u8],
    pub req: I2CRequestType,
}

// エラー型
#[derive(Debug, Error)]
//#[derive(Debug)]
pub enum CtrlMsgError {
    #[error("bus error")]
    Bus(BusError),
    #[error("invalid length")]
    InvalidLength,
    #[error("invalid argument")]
    InvalidArgument,
    #[error("invalid checksum")]
    InvalidChecksum,
    #[error("invalid sequence")]
    InvalidSequence,
    #[error("device returned error code {0:#02x}")]
    DeviceError(u8),
    #[error("EEPROM not responding or invalid")]
    EepromError,
    #[error("invalid device state: {0}")]
    InvalidDeviceState(String), // 何が不正なのかメッセージを入れられるようにする
    #[error("unsupported system")]
    UnsupportedSystem,
    #[error("file I/O error: {0}")]
    IO(#[from] std::io::Error),
}

#[derive(Debug, Clone)]
pub struct PidFilter {
    pub pids: Vec<u16>, // Cの filter->pid[] に相当
    pub block: bool,    // Cの filter->block に相当
}

// シーケンス管理
pub struct IT930x<B: BusOps> {
    bus: B,
    seq: AtomicU8,
    config: IT930xConfig,
    ctrl_lock: Mutex<()>,
    i2c_lock: Mutex<()>,

    gpio_status: Mutex<[GpioStatus; 16]>,
}

// Checksum ... it930x.c 58 〜 76 の移植
fn calc_checksum(buf: &[u8]) -> u16 {
    let mut sum: u16 = 0;
    let mut iter = buf.chunks(2);

    while let Some(chunk) = iter.next() {
        let word = match chunk {
            [a, b] => ((*a as u16) << 8) | (*b as u16),
            [a] => (*a as u16) << 8,
            _ => 0,
        };
        sum = sum.wrapping_add(word);
    }
    !sum
}

// debug用
/*
fn dump_hex(label: &str, data: &[u8]) {
    // 現在のミリ秒を取得
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis();

    // 文字列に変換してスライスで先頭4文字をカットする
    let ts_str = now.to_string();
    let ts_trimmed = &ts_str[4..];

    print!("    [{}] {label} ({}):", ts_trimmed, data.len());
    for b in data {
        print!(" {:02X}", b);
    }
    println!();
}
*/

impl<B: BusOps> IT930x<B> {
    // it930x.c 78〜176 の移植 ... おそらく Mutex が要るので、あとで調査する。
    pub fn ctrl_msg(&self, cmd: u16, wdata: &[u8], rdata: &mut [u8]) -> Result<(), CtrlMsgError> {
        // debug
        CTRL_MSG_COUNT.fetch_add(1, Ordering::Relaxed);
        CTRL_MSG_BYTES.fetch_add(wdata.len() as u64, Ordering::Relaxed);

        // Mutex
        let _lock = self.ctrl_lock.lock().unwrap();

        let seq = self.seq.fetch_add(1, Ordering::SeqCst);

        // TX packet 送信
        // TX packet の total size
        let tx_len = 1 + 2 + 1 + wdata.len() + 2;

        // 一応、表現可能サイズを超える場合はエラー
        if tx_len - 1 > u8::MAX as usize {
            return Err(CtrlMsgError::InvalidLength);
        }

        // 実際に送りつけるデータ
        let mut tx = Vec::with_capacity(tx_len);

        // LEN
        tx.push((tx_len - 1) as u8);

        // CMD
        tx.push((cmd >> 8) as u8);
        tx.push((cmd & 0xff) as u8);

        // SEQ
        tx.push(seq);

        // DATA
        tx.extend_from_slice(wdata);

        // Checksum
        let chk = calc_checksum(&tx[1..(tx_len - 2)]);
        tx.push((chk >> 8) as u8);
        tx.push((chk & 0xff) as u8);

        // USB 送信
        self.bus.ctrl_tx(&tx).map_err(CtrlMsgError::Bus)?;

        // RX packet
        //let rx_len = 1 + 1 + 1 + rdata.len() + 2; // C コード側は、256個固定で、内容チェックして rdate 側へ書き込んでいるが、実際に動くか？
        //let mut rx = vec![0u8; rx_len];
        let mut rx = [0u8; 256];
        let rlen = self.bus.ctrl_rx(&mut rx).map_err(CtrlMsgError::Bus)?;

        // debug
        //dump_hex("CTRL_MSG WB", &tx);
        //dump_hex("CTRL_MSG RB (expect)", &rx[0..rlen]);

        // packet size validate
        //let len = rx[0] as usize;
        //if len != rx_len - 1 // この辺も、想定通りに動くか？ (ctrl_rx の 読み込み buffer サイズは変わったりしないか？)
        //{
        //    return Err(CtrlMsgError::InvalidLength);
        //}
        if rlen < 5 {
            return Err(CtrlMsgError::InvalidLength);
        }

        let frame_len = rx[0] as usize + 1;
        //if frame_len < 5 || frame_len > rlen {
        if frame_len != rlen {
            return Err(CtrlMsgError::InvalidLength);
        }

        // checksum validate
        let recv_chk = ((rx[frame_len - 2] as u16) << 8) | (rx[frame_len - 1] as u16);
        if calc_checksum(&rx[1..frame_len - 2]) != recv_chk {
            return Err(CtrlMsgError::InvalidChecksum);
        }

        // packet seq validate
        let resp_seq = rx[1];
        if resp_seq != seq {
            return Err(CtrlMsgError::InvalidSequence);
        }

        // packet status check
        let status = rx[2];
        if status != 0 {
            return Err(CtrlMsgError::DeviceError(status));
        }

        // 一応、サイズチェック
        if frame_len - 5 < rdata.len() {
            return Err(CtrlMsgError::InvalidLength);
        }

        // rx packet data copy
        rdata.copy_from_slice(&rx[3..3 + rdata.len()]);

        Ok(())
    }
}

// レジスタアクセス層
// IT930x の 内部レジスタ を 読み書き するための 最小API
// (直接 ctrl_msg を使わず、意味のある操作のAPIとする箇所)
// 操作コマンドリスト
const IT930X_CMD_REG_READ: u16 = 0x0000; // u32じゃね？ ... ctrl_msg の cmd を u16 にしてるので、一旦、u16で……。
const IT930X_CMD_REG_WRITE: u16 = 0x0001;
const IT930X_CMD_QUERYINFO: u16 = 0x0022;
const IT930X_CMD_BOOT: u16 = 0x0023;
const IT930X_CMD_FW_SCATTER_WRITE: u16 = 0x0029;
const IT930X_CMD_I2C_READ: u16 = 0x002a;
const IT930X_CMD_I2C_WRITE: u16 = 0x002b;

// UART/Smart Card commands (it930x.h)
const IT930X_CMD_UART_READ: u16 = 0x0033;
const IT930X_CMD_UART_WRITE: u16 = 0x0034;
const IT930X_CMD_UART_SET_BAUDRATE: u16 = 0x0035;
const IT930X_CMD_UART_SET_MODE: u16 = 0x0037;

// UART/Smart Card registers (it930x.c)
const IT930X_REG_UART_RX_READY: u32 = 0x496a;
const IT930X_REG_UART_RX_LENGTH: u32 = 0x496b;
const IT930X_REG_UART_REALSEND: u32 = 0x4965;

// it930x.c 44 〜 56 の移植
fn reg_length(reg: u32) -> u8 {
    match reg {
        r if r & 0xff000000 != 0 => 4,
        r if r & 0x00ff0000 != 0 => 3,
        r if r & 0x0000ff00 != 0 => 2,
        _ => 1,
    }
}

impl<B: BusOps> IT930x<B> {
    // it930x.c 178 〜 203 の移植 ... read_reg は実装しない。(要素数1の配列を送り付ければいいので)
    pub fn read_regs(&self, reg: u32, data: &mut [u8]) -> Result<(), CtrlMsgError> {
        if data.len() > 251 {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut buf = [0u8; 6];
        buf[0] = data.len() as u8;
        buf[1] = reg_length(reg);
        buf[2] = ((reg >> 24) & 0xff) as u8;
        buf[3] = ((reg >> 16) & 0xff) as u8;
        buf[4] = ((reg >> 8) & 0xff) as u8;
        buf[5] = (reg & 0xff) as u8;

        self.ctrl_msg(IT930X_CMD_REG_READ, &buf, data)
    }

    // it930x.c 210〜233 の移植 ... write_reg は実装しない。(要素数1の配列を送り付ければいいので)
    pub fn write_regs(&self, reg: u32, data: &[u8]) -> Result<(), CtrlMsgError> {
        if data.len() > 244 {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut buf = Vec::with_capacity(6 + data.len());
        buf.push(data.len() as u8);
        buf.push(reg_length(reg));
        buf.push(((reg >> 24) & 0xff) as u8);
        buf.push(((reg >> 16) & 0xff) as u8);
        buf.push(((reg >> 8) & 0xff) as u8);
        buf.push((reg & 0xff) as u8);
        buf.extend_from_slice(data);

        self.ctrl_msg(IT930X_CMD_REG_WRITE, &buf, &mut [])
    }

    pub fn write_reg_mask(&self, reg: u32, val: u8, mask: u8) -> Result<(), CtrlMsgError> {
        // mask が 0 なら、何もできないので終了
        if mask == 0 {
            return Err(CtrlMsgError::InvalidLength);
        }

        // mask が ff なら そのまま使うので、そのまま処理
        if mask == 0xff {
            return self.write_regs(reg, &[val]);
        }

        // 1byte 読み込み
        let mut cur = [0u8; 1];
        self.read_regs(reg, &mut cur)?;
        let old = cur[0];

        // チェック
        let new_val = (old & !mask) | (val & mask);

        // 1byte 書き込み
        self.write_regs(reg, &[new_val])?;
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct StreamInput {
    pub enable: bool,
    pub is_parallel: bool,
    pub port_number: u8,
    pub slave_number: u8,
    pub i2c_bus: u8,
    pub i2c_addr: u8,
    pub packet_len: u8,
    pub sync_byte: u8,
}

pub struct IT930xConfig {
    pub i2c_speed: u8,
    pub xfer_size: u32,
    pub inputs: [StreamInput; 5],
}

// メモ: sync_byte のアクセスが必要になるかも？
impl Default for IT930xConfig {
    fn default() -> Self {
        let inputs = [
            // index 0: 衛星1 (ISDB-S)
            StreamInput {
                enable: true,
                is_parallel: false,
                port_number: 1,
                slave_number: 0,
                i2c_bus: 2,
                i2c_addr: 0x11,
                packet_len: 188,
                sync_byte: 0x17,
            },
            // index 1: 衛星2 (ISDB-S)
            StreamInput {
                enable: true,
                is_parallel: false,
                port_number: 2,
                slave_number: 1,
                i2c_bus: 2,
                i2c_addr: 0x13,
                packet_len: 188,
                sync_byte: 0x27,
            },
            // index 2: 地上1 (ISDB-T)
            StreamInput {
                enable: true,
                is_parallel: false,
                port_number: 3,
                slave_number: 2,
                i2c_bus: 2,
                i2c_addr: 0x10,
                packet_len: 188,
                sync_byte: 0x37,
            },
            // index 3: 地上2 (ISDB-T)
            StreamInput {
                enable: true,
                is_parallel: false,
                port_number: 4,
                slave_number: 3,
                i2c_bus: 2,
                i2c_addr: 0x12,
                packet_len: 188,
                sync_byte: 0x47,
            },
            // index 4: 未使用ポート
            StreamInput {
                enable: false,
                is_parallel: false,
                port_number: 0,
                slave_number: 0,
                i2c_bus: 0,
                i2c_addr: 0,
                packet_len: 0,
                sync_byte: 0,
            },
        ];

        Self {
            i2c_speed: 0x07,
            xfer_size: 188 * 816, // px4_usb.c の it930x->config.xfer_size = 188 * px4_usb_params.xfer_packets; と px4_usb_params.c の .xfer_packets = 816, から
            inputs,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GpioMode {
    Undefined,
    In,
    Out,
}

// UART baudrate settings (it930x.h: enum it930x_uart_baudrate)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UartBaudrate {
    Baudrate9600 = 0,
    Baudrate19200 = 1,
    Baudrate38400 = 2,
    Baudrate57600 = 245,
    Baudrate115200 = 250,
}

impl UartBaudrate {
    pub fn as_value(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, Debug)]
struct GpioStatus {
    mode: GpioMode,
    enable: bool,
}

impl Default for GpioStatus {
    fn default() -> Self {
        Self {
            mode: GpioMode::Undefined,
            enable: false,
        }
    }
}

use std::path::Path;
//use anyhow::{Ok, Result};
// px4_usb_probe 相当の処理
impl<B: BusOps> IT930x<B> {
    // it930x.c 354 〜 378 の移植
    pub fn read_firmware_version(&self) -> Result<u32, CtrlMsgError> {
        let mut wbuf = [0u8; 1];
        let mut rbuf = [0u8; 4];

        wbuf[0] = 1;
        //rbuf[0] = 1;

        self.ctrl_msg(IT930X_CMD_QUERYINFO, &wbuf, &mut rbuf)?;
        let fw_version = ((rbuf[0] as u32) << 24)
            | ((rbuf[1] as u32) << 16)
            | ((rbuf[2] as u32) << 8)
            | (rbuf[3] as u32);

        Ok(fw_version)
    }

    // it930x.c 619〜630 をそのまま移植
    pub fn raise(&self) -> Result<(), CtrlMsgError> {
        // debug
        info!("raise()");
        let mut last_err = None;

        for _ in 0..5 {
            // readチェックのみ
            match self.read_firmware_version() {
                Ok(_) => return Ok(()),
                Err(e) => last_err = Some(e),
            }
        }

        Err(last_err.unwrap())
    }

    pub fn check_epprom(&self) -> Result<(), CtrlMsgError> {
        let mut buf = [0u8; 1];
        self.read_regs(0x4979, &mut buf)?;

        if buf[0] == 0 {
            return Err(CtrlMsgError::EepromError);
        }

        Ok(())
    }

    // it930x.c 632 〜 752 の移植
    pub fn load_firmware<P: AsRef<Path>>(&self, path: P) -> Result<(), CtrlMsgError> {
        // debug
        info!("load_firmware()");

        // 1. firmware がロード済みか確認
        let fw_version = self.read_firmware_version()?;
        if fw_version != 0 {
            info!(
                "Firmware is already loaded. version: {}.{}.{}.{}",
                (fw_version >> 24) & 0xff,
                (fw_version >> 16) & 0xff,
                (fw_version >> 8) & 0xff,
                fw_version & 0xff
            );
            return Ok(());
        }

        // 2. I2Cスピード設定
        self.write_regs(0xf103, &[self.config.i2c_speed])?;

        // 3. firmware file 読み込み
        let fw_data = std::fs::read(path)?;
        let fw_len = fw_data.len();
        let mut i = 0;

        // 4. scatter-write
        while i < fw_len {
            let p = &fw_data[i..];
            if p[0] != 0x03 {
                error!("Invalid firmware block at offset {}", i);
                return Err(CtrlMsgError::Bus(rusb::Error::Other.into()));
            }

            let m = p[3] as usize;
            let mut len = 0;
            for j in 0..m {
                len += p[6 + j * 3] as usize;
            }

            if len == 0 {
                error!("No data in firmware block at offset {}", i);
                len += 4 + m * 3;
                i += len;
                continue;
            }

            len += 4 + m * 3;
            let wb = &p[0..len];

            self.ctrl_msg(IT930X_CMD_FW_SCATTER_WRITE, wb, &mut [])?;

            i += len;
        }

        // 5. Boot command
        self.ctrl_msg(IT930X_CMD_BOOT, &[], &mut [])?;

        // 6. firmware version 確認
        let fw_version = self.read_firmware_version()?;
        if fw_version == 0 {
            error!("Firmware failed to load (version = 0)");
            return Err(CtrlMsgError::InvalidDeviceState(
                "Firmware failed to boot (version 0)".to_string(),
            ));
        }

        info!(
            "Firmware is loaded. version: {}.{}.{}.{}",
            (fw_version >> 24) & 0xff,
            (fw_version >> 16) & 0xff,
            (fw_version >> 8) & 0xff,
            fw_version & 0xff
        );
        Ok(())
    }

    pub fn config_i2c(&self) -> Result<(), CtrlMsgError> {
        const I2C_REGS: [[u32; 2]; 5] = [
            [0x4975, 0x4971],
            [0x4974, 0x4970],
            [0x4973, 0x496f],
            [0x4972, 0x496e],
            [0x4964, 0x4963],
        ];

        self.write_regs(0xf6a7, &[self.config.i2c_speed])?;
        self.write_regs(0xf103, &[self.config.i2c_speed])?;

        for input in self.config.inputs.iter().filter(|i| i.enable) {
            let sn = input.slave_number as usize;
            if sn >= I2C_REGS.len() {
                return Err(CtrlMsgError::InvalidLength);
            }

            let regs = &I2C_REGS[sn];
            self.write_regs(regs[0], &[input.i2c_addr << 1])?;
            self.write_regs(regs[1], &[input.i2c_bus])?;
        }

        Ok(())
    }

    pub fn config_stream_input(&self) -> Result<(), CtrlMsgError> {
        for input in &self.config.inputs {
            let port = input.port_number as u32;

            if !input.enable {
                // input port が disable の場合
                self.write_regs(0xda4c + port, &[0])?;
                continue;
            }

            if input.port_number < 2 {
                let v = if input.is_parallel { 1 } else { 0 };
                self.write_regs(0xda58 + port, &[v])?;
            }

            // aggregation mode: sync byte
            self.write_regs(0xda73 + port, &[1])?;

            // set sync byte
            self.write_regs(0xda78 + port, &[input.sync_byte])?;

            // enable input port
            self.write_regs(0xda4c + port, &[1])?;
        }

        Ok(())
    }

    pub fn config_stream_output(&self) -> Result<(), CtrlMsgError> {
        self.write_reg_mask(0xda1d, 0x01, 0x01)?;

        let ret: Result<(), CtrlMsgError> = (|| {
            // disable ep4
            self.write_reg_mask(0xdd11, 0x00, 0x20)?;

            // disable nak of ep4
            self.write_reg_mask(0xdd13, 0x00, 0x20)?;

            // enable ep4
            self.write_reg_mask(0xdd11, 0x20, 0x20)?;

            // threshold of transfer size
            let x = (self.config.xfer_size / 4) as u16;
            self.write_regs(0xdd88, &x.to_le_bytes())?;

            // max bulk packet size
            let v = (self.bus.max_bulk_size() / 4) as u8;
            self.write_regs(0xdd0c, &[v])?;

            self.write_reg_mask(0xda05, 0x00, 0x01)?;
            self.write_reg_mask(0xda06, 0x00, 0x01)?;

            Ok(())
        })();

        // 必ず実行したい exit
        let ret2 = self.write_reg_mask(0xda1d, 0x00, 0x01);
        let ret3 = self.write_regs(0xd920, &[0]);

        if ret.is_err() {
            return ret;
        }
        ret2?;
        ret3?;

        Ok(())
    }

    pub fn init_warm(&self) -> Result<(), CtrlMsgError> {
        // debug
        info!("init_warm()");

        self.write_regs(0x4976, &[0])?;
        self.write_regs(0x4bfb, &[0])?;
        self.write_regs(0x4978, &[0])?;
        self.write_regs(0x4977, &[0])?;

        // ignore sync byte: no
        self.write_regs(0xda1a, &[0])?;

        // dvb-t interrupt: enable
        self.write_reg_mask(0xf41f, 0x04, 0x04)?;

        // mpeg full speed
        self.write_reg_mask(0xda10, 0x00, 0x01)?;

        // dvb-t mode: enable
        self.write_reg_mask(0xf41a, 0x01, 0x01)?;

        // stream output
        self.config_stream_output()?;

        // power config
        self.write_regs(0xd833, &[1])?;
        self.write_regs(0xd830, &[0])?;
        self.write_regs(0xd831, &[1])?;
        self.write_regs(0xd832, &[0])?;

        // i2c
        self.config_i2c()?;

        // stream input
        self.config_stream_input()?;

        Ok(())
    }

    pub fn set_gpio_mode(
        &self,
        gpio: i32,
        mode: GpioMode,
        enable: bool,
    ) -> Result<(), CtrlMsgError> {
        // debug
        //info!("set_gpio_mode()");

        const GPIO_EN_REGS: [u32; 16] = [
            0xd8b0, 0xd8b8, 0xd8b4, 0xd8c0, 0xd8bc, 0xd8c8, 0xd8c4, 0xd8d0, 0xd8cc, 0xd8d8, 0xd8d4,
            0xd8e0, 0xd8dc, 0xd8e4, 0xd8e8, 0xd8ec,
        ];

        if gpio <= 0 || gpio > 16 {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let val = match mode {
            GpioMode::In => 0u8,
            GpioMode::Out => 1u8,
            GpioMode::Undefined => return Err(CtrlMsgError::InvalidArgument),
        };

        let idx = (gpio - 1) as usize;

        let mut status = self.gpio_status.lock().unwrap();

        if status[idx].mode == mode {
            return Ok(());
        }

        // debug (モードが変わった場合だけ表示)
        info!("set_gpio_mode(gpio={}, mode={:?})", gpio, mode);

        status[idx].mode = mode;
        self.write_regs(GPIO_EN_REGS[idx], &[val])?;

        if enable && !status[idx].enable {
            status[idx].enable = true;
            self.write_regs(GPIO_EN_REGS[idx] + 1, &[1])?;
        }

        Ok(())
    }

    // GPIOを制御する (itedtv_bus_enable_gpio の移植)
    pub fn enable_gpio(&self, gpio: u8, enable: bool) -> Result<(), CtrlMsgError> {
        // Cコードの gpio_on_regs
        const GPIO_ON_REGS: [u16; 16] = [
            0xd8b1, 0xd8b9, 0xd8b5, 0xd8c1, 0xd8bd, 0xd8c9, 0xd8c5, 0xd8d1, 0xd8cd, 0xd8d9, 0xd8d5,
            0xd8e1, 0xd8dd, 0xd8e5, 0xd8e9, 0xd8ed,
        ];

        // 引数のチェック (Cコード: gpio <= 0 || gpio > ARRAY_SIZE)
        // 引数が 1-based なので 0 や 17以上はエラー
        if gpio == 0 || gpio > GPIO_ON_REGS.len() as u8 {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let index = (gpio - 1) as usize;

        // Cコードの mutex_lock(&priv->gpio_lock) に相当
        let mut status = self.gpio_status.lock().unwrap();

        // 現在の状態と同じなら何もしない (Cコードの最適化ロジック)
        if status[index].enable == enable {
            return Ok(());
        }

        // レジスタ書き込み (Cコード: it930x_write_reg)
        let addr = GPIO_ON_REGS[index] as u32;
        let val = if enable { 1 } else { 0 };
        self.write_regs(addr, &[val])?;

        // 状態を更新
        status[index].enable = enable;

        Ok(())
    }

    pub fn read_gpio(&self, gpio: u8) -> Result<bool, CtrlMsgError> {
        // Cコードの gpio_i_regs
        const GPIO_I_REGS: [u16; 16] = [
            0xd8ae, 0xd8b6, 0xd8b2, 0xd8be, 0xd8ba, 0xd8c6, 0xd8c2, 0xd8ce, 0xd8ca, 0xd8d6, 0xd8d2,
            0xd8de, 0xd8da, 0xd8e2, 0xd8e6, 0xd8ea,
        ];

        if gpio == 0 || gpio > GPIO_I_REGS.len() as u8 {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let index = (gpio - 1) as usize;
        let status = self.gpio_status.lock().unwrap();

        // Cコードのモードチェックを再現
        // 入力モードでない場合はエラーにする
        if status[index].mode != GpioMode::In {
            return Err(CtrlMsgError::InvalidArgument);
        }

        // レジスタ読み出し
        let mut tmp = [0u8; 1];
        self.read_regs(GPIO_I_REGS[index] as u32, &mut tmp)?;

        Ok(tmp[0] != 0)
    }

    pub fn write_gpio(&self, gpio: i32, high: bool) -> Result<(), CtrlMsgError> {
        // debug
        info!("write_gpio()");

        const GPIO_O_REGS: [u32; 16] = [
            0xd8af, 0xd8b7, 0xd8b3, 0xd8bf, 0xd8bb, 0xd8c7, 0xd8c3, 0xd8cf, 0xd8cb, 0xd8d7, 0xd8d3,
            0xd8df, 0xd8db, 0xd8e3, 0xd8e7, 0xd8eb,
        ];

        if gpio <= 0 || gpio > 16 {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let idx = (gpio - 1) as usize;

        let status = self.gpio_status.lock().unwrap();

        if status[idx].mode != GpioMode::Out {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let v = if high { 1u8 } else { 0u8 };
        self.write_regs(GPIO_O_REGS[idx], &[v])?;

        Ok(())
    }

    pub fn set_pid_filter(
        &self,
        input_idx: usize,
        filter: Option<&PidFilter>,
    ) -> Result<(), CtrlMsgError> {
        // debug
        info!("set_pid_filter()");

        // 各ポートに対応するレジスタ配列
        const REMAP_MODE_REGS: [u32; 5] = [0xda13, 0xda25, 0xda29, 0xda2d, 0xda7f];
        const PID_INDEX_REGS: [u32; 5] = [0xda15, 0xda26, 0xda2a, 0xda2e, 0xda80];

        // 境界チェック (Cの input_idx < 0 || input_idx > 4 に相当)
        if input_idx >= 5 {
            return Err(CtrlMsgError::InvalidArgument);
        }

        // ポート番号の取得 (Cの it930x->config.input[input_idx].port_number)
        // it930x.rsの定義に合わせて inputs を使用
        let port = self.config.inputs[input_idx].port_number as usize;
        if port >= 5 {
            return Err(CtrlMsgError::InvalidDeviceState(format!(
                "Invalid port number: {}",
                port
            )));
        }

        // フィルターが無効、またはPIDリストが空の場合 (Cの !filter || !filter->num に相当)
        if filter.is_none() || filter.unwrap().pids.is_empty() {
            /* disable pid filter */
            self.write_regs(REMAP_MODE_REGS[port], &[0])?;

            /* sync_byte only */
            self.write_regs(0xda73 + port as u32, &[1])?;

            return Ok(());
        }

        let filter = filter.unwrap();

        // 各PIDをハードウェアフィルタに登録
        for (i, &pid) in filter.pids.iter().enumerate() {
            let data = [(pid & 0xff) as u8, ((pid >> 8) & 0xff) as u8];

            /* target pid */
            self.write_regs(0xda16, &data)?;

            /* enable */
            self.write_regs(0xda14, &[1])?;

            /* index */
            // ハードウェア側のインデックスレジスタは1バイト書き込みのため u8 にキャスト
            self.write_regs(PID_INDEX_REGS[port], &[i as u8])?;
        }

        /* block or pass */
        let remap_mode = if filter.block { 0 } else { 2 };
        self.write_regs(REMAP_MODE_REGS[port], &[remap_mode])?;

        /* sync_byte and remap */
        self.write_regs(0xda73 + port as u32, &[3])?;

        /* pid offset */
        self.write_regs(0xda81 + (port as u32 * 2), &[0, 0])?;

        Ok(())
    }

    pub fn purge_psb(&self, timeout: std::time::Duration) -> Result<(), CtrlMsgError> {
        // USB接続であるか確認 (BusOpsトレイトでチェック可能にするのがベストです)
        // ここでは便宜上、条件を満たしている前提とします
        // debug
        info!("call purge_psb()");

        // 1. レジスタ操作によるPSBパージの有効化
        self.write_reg_mask(0xda1d, 0x01, 0x01)?;

        // 2. 受信用バッファの確保 (Rustではこれで自動的にメモリ管理されます)
        let mut buf = vec![0u8; 1024];

        // 3. ストリーム受信 (ITEDTV_BUS_USB の呼び出し)
        // bus.stream_rx が [u8] を受け取り、実際に読み込んだ長さを返す設計にします
        //let read_len = self
        //    .bus
        //    .stream_rx(&mut buf, timeout)
        //    .map_err(CtrlMsgError::Bus)?;

        let result = self.bus.stream_rx(&mut buf, timeout);

        // 4. パージの無効化
        // エラーハンドリングについては、パージ後の状態を優先させるため、
        // 処理の成功/失敗に関わらずレジスタを戻すのが安全です
        let _ = self.write_reg_mask(0xda1d, 0x00, 0x01);

        let read_len = match result {
            Ok(len) => {
                info!("purge_psb: stream_rx len={}", len);
                len
            }

            Err(e) => {
                error!("purge_psb: stream_rx error={:?}", e);

                let _ = self.write_reg_mask(0xda1d, 0x00, 0x01);

                return Err(CtrlMsgError::Bus(e));
            }
        };

        // 5. 判定処理 (Cの if (len == 512) に相当)
        if read_len == 512 {
            Ok(())
        } else {
            // 必要に応じてエラーを返すか、デバッグログを出力
            Ok(())
        }
    }

    pub fn start_streaming<F>(&self, handler: F) -> Result<(), CtrlMsgError>
    where
        F: Fn(&[u8]) + Send + Sync + 'static,
    {
        info!("Start streaming via bus.");
        self.bus
            .start_streaming(Box::new(handler))
            .map_err(CtrlMsgError::Bus)
    }

    /// USBバスのストリーミングを停止する
    pub fn stop_streaming(&self) -> Result<(), CtrlMsgError> {
        // BusOps の stop_streaming を呼び出すだけ
        self.bus.stop_streaming().map_err(CtrlMsgError::Bus)?;

        info!("Stopped streaming via bus.");
        Ok(())
    }
}

impl<B: BusOps> IT930x<B> {
    pub fn i2c_master_request(
        &self,
        bus: u8,
        requests: &mut [I2CCommRequest],
    ) -> Result<(), CtrlMsgError> {
        // Mutex を掛ける 多分。
        let _lock = self.i2c_lock.lock().unwrap();

        for req in requests.iter_mut() {
            // データ長の取得
            let len = req.data.len();
            if len == 0 {
                return Err(CtrlMsgError::InvalidArgument);
            }

            match req.req {
                I2CRequestType::Read => {
                    if len > 251 {
                        return Err(CtrlMsgError::InvalidLength);
                    }

                    let buf = [len as u8, bus, req.addr << 1];
                    self.ctrl_msg(IT930X_CMD_I2C_READ, &buf, req.data)?;
                }

                I2CRequestType::Write => {
                    if len > (250 - 3) {
                        return Err(CtrlMsgError::InvalidLength);
                    }

                    let mut buf = Vec::with_capacity(3 + len);
                    buf.push(len as u8);
                    buf.push(bus);
                    buf.push(req.addr << 1);
                    buf.extend_from_slice(req.data);

                    self.ctrl_msg(IT930X_CMD_I2C_WRITE, &buf, &mut [])?;
                }
            }
        }
        Ok(())
    }
}

impl<B: BusOps> IT930x<B> {
    pub fn new(bus: B) -> Self {
        // 多分、IT930xConfig::default() は、xfer_size の設定もした方がいいと思う。
        Self {
            bus,
            seq: AtomicU8::new(0),
            config: IT930xConfig::default(),
            ctrl_lock: Mutex::new(()),
            i2c_lock: Mutex::new(()),
            gpio_status: Mutex::new([GpioStatus::default(); 16]),
        }
    }
}

// UART/Smart Card functions (it930x.c: UART/Smart Card functions)
impl<B: BusOps> IT930x<B> {
    /// Set UART baudrate (it930x_set_uart_baudrate)
    pub fn set_uart_baudrate(&self, baudrate: UartBaudrate) -> Result<(), CtrlMsgError> {
        let val = baudrate.as_value();
        self.ctrl_msg(IT930X_CMD_UART_SET_BAUDRATE, &[val], &mut [])
    }

    /// Send UART data in chunks of 48 bytes (it930x_send_uart_data)
    pub fn send_uart_data(&self, data: &[u8]) -> Result<(), CtrlMsgError> {
        if data.is_empty() {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let mut buf_idx = 0;
        let mut write_len = data.len();

        while write_len > 0 {
            let mut write_buf = [0u8; 49];

            if write_len > 48 {
                write_buf[0] = 48;
                for i in 0..48 {
                    write_buf[i + 1] = data[buf_idx + i];
                }

                self.ctrl_msg(IT930X_CMD_UART_WRITE, &write_buf, &mut [])?;
                buf_idx += 48;
                write_len -= 48;
            } else {
                write_buf[0] = write_len as u8;
                for i in 0..write_len {
                    write_buf[i + 1] = data[buf_idx + i];
                }

                self.ctrl_msg(IT930X_CMD_UART_WRITE, &write_buf[..write_len + 1], &mut [])?;
                buf_idx += write_len;
                write_len = 0;
            }
        }

        Ok(())
    }

    // ---- B-CAS/Smart Card functions (it930x_bcas_*) ----

    /// Initialize B-CAS card mode (it930x_bcas_init)
    pub fn bcas_init(&self) -> Result<(), CtrlMsgError> {
        let val: u8 = 1;
        self.ctrl_msg(IT930X_CMD_UART_SET_MODE, &[val], &mut [])
    }

    /// Reset B-CAS card (it930x_bcas_reset_card)
    pub fn bcas_reset_card(&self) -> Result<(), CtrlMsgError> {
        info!("bcas_reset_card().");
        // Enable GPIO H14 as output
        self.set_gpio_mode(14, GpioMode::Out, true)?;

        // Set GPIO H14 low (assert reset)
        self.write_gpio(14, false)?;

        // Set UART status register
        self.write_regs(0x7904, &[2])?;

        // Set UART baudrate to 9600
        self.set_uart_baudrate(UartBaudrate::Baudrate9600)?;

        // Wait 5ms (Linux kernel: msleep(5))
        // カードと IC が安定するまでしっかり待つ (5ms -> 100ms に変更)
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Set GPIO H14 high (release reset)
        self.write_gpio(14, true)?;

        Ok(())
    }

    /// Check if UART RX is ready (it930x_bcas_check_ready)
    pub fn bcas_check_ready(&self) -> Result<bool, CtrlMsgError> {
        let mut val = [0u8; 1];
        self.read_regs(IT930X_REG_UART_RX_READY, &mut val)?;
        Ok(val[0] != 0)
    }

    /// Get data from B-CAS card (it930x_bcas_get_data)
    /// Get data from B-CAS card (it930x_bcas_get_data)
    pub fn bcas_get_data(&self, buf: &mut [u8]) -> Result<usize, CtrlMsgError> {
        if buf.is_empty() {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let mut index = 0usize;

        if buf.len() > 32 {
            // 32バイトずつ分割して読み込む
            while index < buf.len() {
                let mut temp = [0u8; 1];
                self.read_regs(IT930X_REG_UART_RX_LENGTH, &mut temp)?;

                if temp[0] == 0 {
                    break;
                }

                // デバイス側の残量・呼び出し側バッファの残量、両方でクランプする
                let remaining = buf.len() - index;
                let read_len = (temp[0] as usize).min(32).min(remaining) as u8;

                if read_len == 0 {
                    break;
                }

                let wb = [read_len];
                let mut rb = [0u8; 32];
                self.ctrl_msg(IT930X_CMD_UART_READ, &wb, &mut rb[..read_len as usize])?;

                buf[index..index + read_len as usize].copy_from_slice(&rb[..read_len as usize]);
                index += read_len as usize;
            }
            Ok(index)
        } else {
            // 1回で読み込む
            let mut rx_len = [0u8; 1];
            self.read_regs(IT930X_REG_UART_RX_LENGTH, &mut rx_len)?;

            if rx_len[0] == 0 {
                return Ok(0);
            }

            // デバイス報告長がバッファより大きい場合はクランプする
            let read_len = (rx_len[0] as usize).min(buf.len()) as u8;

            let wb = [read_len];
            self.ctrl_msg(IT930X_CMD_UART_READ, &wb, &mut buf[..read_len as usize])?;

            Ok(read_len as usize)
        }
    }

    /// Send data to B-CAS card (it930x_bcas_send_data)
    pub fn bcas_send_data(&self, data: &[u8]) -> Result<(), CtrlMsgError> {
        if data.is_empty() || data.len() > 255 {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let mut buf_idx = 0;
        let mut write_len = data.len();

        while write_len > 0 {
            let mut write_buf = [0u8; 49];

            // Copy data to buffer (skip first byte which is length)
            for i in 0..48 {
                if buf_idx + i < data.len() {
                    write_buf[i + 1] = data[buf_idx + i];
                }
            }

            if write_len > 48 {
                write_buf[0] = 48;
                self.ctrl_msg(IT930X_CMD_UART_WRITE, &write_buf, &mut [])?;
                buf_idx += 48;
                write_len -= 48;
            } else {
                // Set real send flag before last chunk
                self.write_regs(IT930X_REG_UART_REALSEND, &[1])?;

                write_buf[0] = write_len as u8;
                self.ctrl_msg(IT930X_CMD_UART_WRITE, &write_buf[..write_len + 1], &mut [])?;
                buf_idx += write_len;
                write_len = 0;
            }
        }

        Ok(())
    }

    /// Detect if B-CAS card is present (it930x_bcas_detect_card)
    pub fn bcas_detect_card(&self) -> Result<bool, CtrlMsgError> {
        // Configure GPIO H6 as input
        self.set_gpio_mode(6, GpioMode::In, true)?;

        // Read GPIO H6 input
        let detected = self.read_gpio(6)?;

        // Card is detected when GPIO is low (active low)
        Ok(!detected)
    }

    /// Set B-CAS specific baudrate (it930x_bcas_set_baudrate)
    pub fn bcas_set_baudrate(&self, baudrate: UartBaudrate) -> Result<(), CtrlMsgError> {
        match baudrate {
            UartBaudrate::Baudrate9600 | UartBaudrate::Baudrate19200 => {
                let val = baudrate.as_value();
                self.ctrl_msg(IT930X_CMD_UART_SET_BAUDRATE, &[val], &mut [])
            }
            _ => Err(CtrlMsgError::InvalidArgument),
        }
    }
}

impl<B: BusOps> Drop for IT930x<B> {
    fn drop(&mut self) {
        info!("Terminating IT930x device...");

        // ストリーミングを停止させる
        // Cコードの itedtv_bus_stop_streaming 相当
        if let Err(e) = self.bus.stop_streaming() {
            error!("Failed to stop streaming during drop: {:?}", e);
        }
    }
}
