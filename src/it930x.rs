// コントロールメッセージ層
// IT930xデバイスとやりとりする独自バイナリプロトコルの実装層

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
    #[error("invalid checksum")]
    InvalidChecksum,
    #[error("invalid sequence")]
    InvalidSequence,
    #[error("device returned error code {0:#02x}")]
    DeviceError(u8),
    #[error("file I/O error: {0}")]
    IO(#[from] std::io::Error),
}

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
        let chk = checksum(&tx[1..]);
        tx.push((chk >> 8) as u8);
        tx.push((chk & 0xff) as u8);
        
        // USB 送信
        self.bus.ctrl_tx(&tx).map_err(CtrlMsgError::Bus)?;

        // RX packet
        let rx_len = 1 + 1 + 1 + rdata.len() + 2; // C コード側は、256個固定で、内容チェックして rdate 側へ書き込んでいるが、実際に動くか？
        let mut rx = vec![0u8; rx_len];

        self.bus.ctrl_rx(&mut rx).map_err(CtrlMsgError::Bus)?;

        // packet size validate
        let len = rx[0] as usize;
        if len != rx_len - 1 // この辺も、想定通りに動くか？ (ctrl_rx の 読み込み buffer サイズは変わったりしないか？)
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        // checksum validate
        let recv_chk = ((rx[rx_len - 2] as u16) << 8) | (rx[rx_len - 1] as u16);
        if checksum(&rx[1..rx_len - 2]) != recv_chk
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
            return  Err(CtrlMsgError::DeviceError(status));
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

        let mut buf= Vec::with_capacity(3 * data.len());
        buf.push(data.len() as u8);
        buf.push(it930x_reg_length(reg));
        buf.push(((reg >> 24) & 0xff) as u8);
        buf.push(((reg >> 16) & 0xff) as u8);
        buf.push(((reg >> 8) & 0xff) as u8);
        buf.push((reg & 0xff) as u8);
        buf.extend_from_slice(data);

        self.ctrl_msg(IT930X_CMD_REG_WRITE, &buf, &mut [])
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
        rbuf[0] = 1;

        self.ctrl_msg(IT930X_CMD_QUERYINFO, &wbuf,&mut rbuf)?;
        let fw_version = ((rbuf[0] as u32) << 24) | ((rbuf[1] as u32) << 16) | ((rbuf[2] as u32) << 8) | (rbuf[3] as u32);

        return Ok(fw_version)
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

        // 2. I2Cスピード設定
        let conf = [0u8]; // あとで変える必要がある。 // config.i2c_spped が必要
        self.write_regs(0xf103, &conf)?;

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

        println!("Firmware is already loaded. version: {}.{}.{}.{}", (fw_version >> 24) & 0xff, (fw_version >> 24) & 0xff, (fw_version >> 24) & 0xff, fw_version & 0xff);
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
            let regs = &I2C_REGS[input.slave_number as usize];
            self.write_regs(regs[0], &[input.i2c_addr << 1])?;
            self.write_regs(regs[1], &[input.i2c_bus])?;
        }

        Ok(())
    }

}


// 下記2つで i2c_comm.h 17 〜 26 の移植
#[derive(Clone, Copy)]
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
        Self { bus, seq: AtomicU8::new(0), config: IT930xConfig::default(), ctrl_lock: Mutex::new(()), i2c_lock: Mutex::new(()), }
    }
}