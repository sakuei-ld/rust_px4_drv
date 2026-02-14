// コントロールメッセージ層
// IT930xデバイスとやりとりする独自バイナリプロトコルの実装層

// これに関しては、いろんなところで使うので、実際はここじゃない方が良いかもしれない。
// 下記2つで i2c_comm.h 17 〜 26 の移植
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2CRequestType
{
    Read,
    Write,
}

pub struct I2CCommRequest<'a>
{
    pub addr: u8,
    pub data: &'a mut [u8],
    pub req: I2CRequestType,
}

use crate::itedtv_bus::{BusError, BusOps};

// エラー型
// 
use thiserror::Error;
#[derive(Debug, Error)]
//#[derive(Debug)]
pub enum CtrlMsgError
{
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
    #[error("file I/O error: {0}")]
    IO(#[from] std::io::Error),
}

use std::os::macos::raw::stat;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Mutex;
// シーケンス管理
pub struct IT930x<B: BusOps>
{
    bus: B,
    seq: AtomicU8,
    config: IT930xConfig,
    ctrl_lock: Mutex<()>,
    i2c_lock: Mutex<()>,

    gpio_lock: Mutex<()>,
    gpio_status: Mutex<[GpioStatus; 16]>,
}

// Checksum ... it930x.c 58 〜 76 の移植
fn checksum(buf: &[u8]) -> u16
{
    let mut sum: u16 = 0;
    let mut iter = buf.chunks(2);

    while let Some(chunk) = iter.next()
    {
        let word = match chunk
        {
            [a, b] => ((*a as u16) << 8) | (*b as u16),
            [a] => (*a as u16) << 8,
            _ => 0,
        };
        sum = sum.wrapping_add(word);
    }
    !sum
}


// debug用
fn dump_hex(label: &str, data: &[u8]) {
    print!("{label} ({}):", data.len());
    for b in data {
        print!(" {:02X}", b);
    }
    println!();
}


impl<B: BusOps> IT930x<B>
{
    // it930x.c 78〜176 の移植 ... おそらく Mutex が要るので、あとで調査する。
    pub fn ctrl_msg(&self, cmd: u16, wdata: &[u8], rdata: &mut [u8],) -> Result<(), CtrlMsgError>
    {        
        // Mutex
        let _lock = self.ctrl_lock.lock().unwrap();

        let seq = self.seq.fetch_add(1, Ordering::SeqCst);

        // TX packet 送信
        // TX packet の total size
        let tx_len =  1 + 2 + 1 + wdata.len() + 2;

        // 一応、表現可能サイズを超える場合はエラー
        if tx_len - 1 > u8::MAX as usize
        {
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
        let chk = checksum(&tx[1..(tx_len - 2)]);
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
        dump_hex("CTRL_MSG WB", &tx);
        dump_hex("CTRL_MSG RB (expect)", &rx[0..rlen]);

        // packet size validate
        //let len = rx[0] as usize;
        //if len != rx_len - 1 // この辺も、想定通りに動くか？ (ctrl_rx の 読み込み buffer サイズは変わったりしないか？)
        //{
        //    return Err(CtrlMsgError::InvalidLength);
        //}
        if rlen < 5
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let frame_len = rx[0] as usize + 1;
        if frame_len < 5 || frame_len > rlen
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        // checksum validate
        let recv_chk = ((rx[frame_len - 2] as u16) << 8) | (rx[frame_len - 1] as u16);
        if checksum(&rx[1..frame_len - 2]) != recv_chk
        {
            return Err(CtrlMsgError::InvalidChecksum);
        }

        // packet seq validate
        let resp_seq = rx[1];
        if resp_seq != seq
        {
            return Err(CtrlMsgError::InvalidSequence);
        }

        // packet status check
        let status = rx[2];
        if status != 0
        {
            return Err(CtrlMsgError::DeviceError(status));
        }

        // 一応、サイズチェック
        if frame_len - 5 < rdata.len()
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        // rx packet data copy
        rdata.copy_from_slice(&rx[3..3 + rdata.len()]);

        // 必要なら、ここで Mutex を解除 (Rust で要るのかは、わからん)

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

// it930x.c 44 〜 56 の移植
fn it930x_reg_length(reg: u32) -> u8
{
    match reg
    {
        r if r & 0xff000000 != 0 => 4,
        r if r & 0x00ff0000 != 0 => 3,
        r if r & 0x0000ff00 != 0 => 2,
        _ => 1, 
    }
}

impl<B: BusOps> IT930x<B>
{
    // it930x.c 178 〜 203 の移植 ... read_reg は実装しない。(要素数1の配列を送り付ければいいので)
    pub fn read_regs(&self, reg: u32, data: &mut [u8],) -> Result<(), CtrlMsgError>
    {
        if data.len() > 251
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut buf = [0u8; 6];
        buf[0] = data.len() as u8;
        buf[1] = it930x_reg_length(reg);
        buf[2] = ((reg >> 24) & 0xff) as u8;
        buf[3] = ((reg >> 16) & 0xff) as u8;
        buf[4] = ((reg >> 8) & 0xff) as u8;
        buf[5] = (reg & 0xff) as u8;

        self.ctrl_msg(IT930X_CMD_REG_READ, &buf, data)
    }

    // it930x.c 210〜233 の移植 ... write_reg は実装しない。(要素数1の配列を送り付ければいいので)
    pub fn write_regs(&self, reg: u32, data: &[u8],) -> Result<(), CtrlMsgError>
    {
        if data.len() > 244
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut buf= Vec::with_capacity(6 + data.len());
        buf.push(data.len() as u8);
        buf.push(it930x_reg_length(reg));
        buf.push(((reg >> 24) & 0xff) as u8);
        buf.push(((reg >> 16) & 0xff) as u8);
        buf.push(((reg >> 8) & 0xff) as u8);
        buf.push((reg & 0xff) as u8);
        buf.extend_from_slice(data);

        self.ctrl_msg(IT930X_CMD_REG_WRITE, &buf, &mut [])
    }

    pub fn write_reg_mask(&self, reg: u32, val: u8, mask: u8) -> Result<(), CtrlMsgError>
    {
        // mask が 0 なら、何もできないので終了
        if mask == 0{return Err(CtrlMsgError::InvalidLength);}
        
        // mask が ff なら そのまま使うので、そのまま処理
        if mask == 0xff{return self.write_regs(reg, &[val]);}

        // 1byte 読み込み
        let mut cur = [0u8; 1];
        self.read_regs(reg, &mut cur)?;
        let old = cur[0];

        // チェック
        let new_val = (old & !mask) | (val & mask);
        
        // 変化がない場合は書き込まない (USB負荷軽減)
        if new_val == old{return Ok(());}

        // 1byte 書き込み
        self.write_regs(reg, &[new_val])?;
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct StreamInput
{
    pub enable: bool,
    pub is_parallel: bool,
    pub port_number: u8,
    pub slave_number: u8,
    pub i2c_bus: u8,
    pub i2c_addr: u8,
    pub packet_len: u8,
    pub sync_byte: u8,
}

pub struct IT930xConfig
{
    pub i2c_speed: u8,
    pub xfer_size: u32,
    pub inputs: [StreamInput; 5],
}

// 
impl Default for IT930xConfig
{
    fn default() -> Self 
    {
        let inputs =
        [
            StreamInput
            {
                enable: true,
                is_parallel: false,
                port_number: 1,
                slave_number: 0,
                i2c_bus: 2,
                i2c_addr: 0x11,
                packet_len: 188,
                sync_byte: 0x17,
            },
            StreamInput
            {
                enable: true,
                is_parallel: false,
                port_number: 2,
                slave_number: 1,
                i2c_bus: 2,
                i2c_addr: 0x13,
                packet_len: 188,
                sync_byte: 0x27,
            },
            StreamInput
            {
                enable: true,
                is_parallel: false,
                port_number: 3,
                slave_number: 2,
                i2c_bus: 2,
                i2c_addr: 0x10,
                packet_len: 188,
                sync_byte: 0x37,
            },
            StreamInput
            {
                enable: true,
                is_parallel: false,
                port_number: 4,
                slave_number: 3,
                i2c_bus: 2,
                i2c_addr: 0x12,
                packet_len: 188,
                sync_byte: 0x47,
            },
            StreamInput
            {
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

        Self
        {
            i2c_speed: 0x07,
            xfer_size: 188 * 816, // px4_usb.c の it930x->config.xfer_size = 188 * px4_usb_params.xfer_packets; と px4_usb_params.c の .xfer_packets = 816, から
            inputs,
        }
    }
}


#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GpioMode
{
    In,
    Out,
}

#[derive(Clone, Copy, Debug)]
struct GpioStatus
{
    mode: GpioMode,
    enable: bool,
}

impl Default for GpioStatus
{
    fn default() -> Self {
        Self { mode:GpioMode::In, enable: false }
    }
}

use std::fs::File;
use std::io::Read;
use std::path::Path;
//use anyhow::{Ok, Result};
// px4_usb_probe 相当の処理
impl<B: BusOps> IT930x<B>
{
    // it930x.c 354 〜 378 の移植
    pub fn read_firmware_version(&self) -> Result<u32, CtrlMsgError>
    {
        let mut wbuf = [0u8; 1];
        let mut rbuf = [0u8; 4];

        wbuf[0] = 1;
        //rbuf[0] = 1;

        self.ctrl_msg(IT930X_CMD_QUERYINFO, &wbuf,&mut rbuf)?;
        let fw_version = ((rbuf[0] as u32) << 24) | ((rbuf[1] as u32) << 16) | ((rbuf[2] as u32) << 8) | (rbuf[3] as u32);

        return Ok(fw_version)
    }

    // it930x.c 619〜630 をそのまま移植
    pub fn raise(&self) -> Result<(), CtrlMsgError>
    {
        let mut last_err = None;

        for i in 0..5
        {
            // readチェックのみ
            match self.read_firmware_version()
            {
                Ok(u32) => return Ok(()),
                Err(e) => last_err = Some(e),
            }
        }

        Err(last_err.unwrap())
    }

    pub fn check_epprom(&self) -> Result<(), CtrlMsgError>
    {
        let mut buf = [0u8; 1];
        self.read_regs(0x4979, &mut buf)?;

        if buf[0] == 0
        {
            return Err(CtrlMsgError::EepromError);
        }

        Ok(())
    }

    // it930x.c 632 〜 752 の移植
    pub fn load_firmware<P: AsRef<Path>>(&self, path: P) -> Result<(), CtrlMsgError>
    {
        // 1. firmware がロード済みか確認
        let fw_version = self.read_firmware_version()?;
        if fw_version != 0
        {
            println!("Firmware is already loaded. version: {}.{}.{}.{}", (fw_version >> 24) & 0xff, (fw_version >> 24) & 0xff, (fw_version >> 24) & 0xff, fw_version & 0xff);
            return Ok(());
        }

        println!("[debug] passed read_firmware_version()");

        // 2. I2Cスピード設定
        self.write_regs(0xf103, &[self.config.i2c_speed])?;

        // 3. firmware file 読み込み
        let mut fw_file = File::open(path)?;
        let mut fw_data = Vec::new();
        fw_file.read_to_end(&mut fw_data)?;

        let fw_len = fw_data.len();
        let mut i = 0;

        // 4. scatter-write
        while i < fw_len
        {
            let p = &fw_data[i..];
            if p[0] != 0x03
            {
                eprintln!("Invalid firmware block at offset {}", i);
                //return Err(rusb::Error::Other.into());
                return Err(CtrlMsgError::Bus(rusb::Error::Other.into()));
            }

            let m = p[3] as usize;
            let mut len = 0;
            for j in 0..m
            {
                len += p[6 + j * 3] as usize;
            }

            if len == 0
            {
                eprintln!("No data in firmware block at offset {}", i);
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
        if fw_version == 0
        {
            eprintln!("Firmware failed to load (version = 0)");
            //return Err(rusb::Error::Other.into());
            return Err(CtrlMsgError::Bus(rusb::Error::Other.into()));
        }

        println!("Firmware is loaded. version: {}.{}.{}.{}", (fw_version >> 24) & 0xff, (fw_version >> 24) & 0xff, (fw_version >> 24) & 0xff, fw_version & 0xff);
        return Ok(());
    }

    pub fn config_i2c(&self) -> Result<(), CtrlMsgError>
    {
        const I2C_REGS: [[u32; 2]; 5] =
        [
            [0x4975, 0x4971],
            [0x4974, 0x4970],
            [0x4973, 0x496f],
            [0x4972, 0x496e],
            [0x4964, 0x4963],
        ];

        self.write_regs(0xf6a7, &[self.config.i2c_speed])?;
        self.write_regs(0xf103, &[self.config.i2c_speed])?;

        for input in self.config.inputs.iter().filter(|i| i.enable)
        {
            let sn = input.slave_number as usize;
            if sn >= I2C_REGS.len()
            {
                return Err(CtrlMsgError::InvalidLength);
            }

            let regs = &I2C_REGS[sn];
            self.write_regs(regs[0], &[input.i2c_addr << 1])?;
            self.write_regs(regs[1], &[input.i2c_bus])?;
        }

        Ok(())
    }

    pub fn config_stream_input(&self) -> Result<(), CtrlMsgError>
    {
        for input in &self.config.inputs
        {
            let port = input.port_number as u32;

            if !input.enable
            {
                // input port が disable の場合
                self.write_regs(0xda4c + port, &[0])?;
                continue;
            }

            if input.port_number < 2
            {
                let v = if input.is_parallel {1} else {0};
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

    pub fn config_stream_output(&self) -> Result<(), CtrlMsgError>
    {
        self.write_reg_mask(0xda1d, 0x01, 0x01)?;

        let mut ret: Result<(), CtrlMsgError> = (||
        {
            // disable ep4
            self.write_reg_mask(0xdd11, 0x00, 0x20)?;

            // disable nak of ep4
            self.write_reg_mask(0xdd13, 0x00, 0x20)?;

            // enable ep4
            self.write_reg_mask(0xdd11, 0x20, 0x20)?;

            // threshold of transfer size
            let x = ((self.config.xfer_size / 4) & 0xffff) as u16;
            let buf = [(x & 0xff) as u8, ((x >> 8) & 0xff) as u8];
            self.write_regs(0xdd88, &buf)?;

            // max bulk packet size
            let v = ((self.bus.max_bulk_size() / 4) & 0xff) as u8;
            self.write_regs(0xdd0c, &[v])?;

            self.write_reg_mask(0xda05, 0x00, 0x01)?;
            self.write_reg_mask(0xda06, 0x00, 0x01)?;

            Ok(())
        })();

        // 必ず実行したい exit
        let ret2 = self.write_reg_mask(0xda1d, 0x00, 0x01);
        let ret3 = self.write_regs(0xd920, &[0]);

        if ret.is_err() { return ret; }
        ret2?;
        ret3?;

        Ok(())
    }

    pub fn init_warm(&self) -> Result<(), CtrlMsgError>
    {
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

    pub fn set_gpio_mode(&self, gpio: i32, mode: GpioMode, enable: bool) -> Result<(), CtrlMsgError>
    {
        const GPIO_EN_REGS: [u32; 16] =
        [
            0xd8b0, 0xd8b8, 0xd8b4, 0xd8c0,
            0xd8bc, 0xd8c8, 0xd8c4, 0xd8d0,
            0xd8cc, 0xd8d8, 0xd8d4, 0xd8e0,
            0xd8dc, 0xd8e4, 0xd8e8, 0xd8ec,
        ];

        if gpio <= 0 || gpio > 16
        {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let val = match mode {
            GpioMode::In => 0u8,
            GpioMode::Out => 1u8,            
        };

        let idx = (gpio - 1) as usize;

        let _lock = self.gpio_lock.lock().unwrap();
        let mut status = self.gpio_status.lock().unwrap();

        if status[idx].mode != mode
        {
            status[idx].mode = mode;
            self.write_regs(GPIO_EN_REGS[idx], &[val])?;
        }

        if enable && !status[idx].enable
        {
            status[idx].enable = true;
            self.write_regs(GPIO_EN_REGS[idx] + 1, &[1])?;
        }

        Ok(())
    }

    pub fn write_gpio(&self, gpio: i32, high: bool) -> Result<(), CtrlMsgError>
    {
        const GPIO_O_REGS: [u32; 16] = 
        [
            0xd8af, 0xd8b7, 0xd8b3, 0xd8bf, 
            0xd8bb, 0xd8c7, 0xd8c3, 0xd8cf, 
            0xd8cb, 0xd8d7, 0xd8d3, 0xd8df, 
            0xd8db, 0xd8e3, 0xd8e7, 0xd8eb, 
        ];

        if gpio <= 0 || gpio > 16
        {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let idx = (gpio - 1) as usize;

        let _lock = self.gpio_lock.lock().unwrap();
        let status = self.gpio_status.lock().unwrap();

        if status[idx].mode != GpioMode::Out
        {
            return Err(CtrlMsgError::InvalidArgument);
        }

        let v = if high {1u8} else {0u8};
        self.write_regs(GPIO_O_REGS[idx], &[v])?;

        Ok(())
    }

}

impl<B: BusOps> IT930x<B>
{
    pub fn i2c_master_request(&self, bus: u8, requests: &mut [I2CCommRequest],) -> Result<(), CtrlMsgError>
    {
        // Mutex を掛ける 多分。
        let _lock = self.i2c_lock.lock().unwrap();

        for req in requests.iter_mut()
        {
            match req.req 
            {
                I2CRequestType::Read =>
                {
                    let len = req.data.len();
                    if len > 251
                    {
                        return Err(CtrlMsgError::InvalidLength);
                    }

                    let buf = [len as u8, bus, req.addr << 1,];

                    // debug
                    println!(
                        "[i2c_read] bus={} addr=0x{:02x} len={}",
                        bus, req.addr, len
                    );
                    dump_hex("wb", &buf);

                    self.ctrl_msg(IT930X_CMD_I2C_READ, &buf, req.data,)?;
                }
                
                I2CRequestType::Write =>
                {
                    let len = req.data.len();
                    if len > (250 - 3)
                    {
                        return Err(CtrlMsgError::InvalidLength);
                    }

                    let mut buf = Vec::with_capacity(3 + len);
                    buf.push(len as u8);
                    buf.push(bus);
                    buf.push(req.addr << 1);
                    buf.extend_from_slice(req.data);

                    // debug
                    println!(
                        "[i2c_write] bus={} addr=0x{:02x} len={}",
                        bus, req.addr, len
                    );
                    dump_hex("wb", &buf);

                    self.ctrl_msg(IT930X_CMD_I2C_WRITE, &buf, &mut [])?;
                }
            }
        }
        Ok(())
    }
}


impl<B: BusOps> IT930x<B>
{
    pub fn new(bus: B) -> Self
    {
        // 多分、IT930xConfig::default() は、xfer_size の設定もした方がいいと思う。
        Self { bus, seq: AtomicU8::new(0), config: IT930xConfig::default(), ctrl_lock: Mutex::new(()), i2c_lock: Mutex::new(()), gpio_lock: Mutex::new(()), gpio_status: Mutex::new([GpioStatus::default(); 16]), }
    }
}