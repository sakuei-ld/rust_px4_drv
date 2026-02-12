// ここからは RT710の話

use std::sync::Mutex;

use crate::it930x::{CtrlMsgError, I2CCommRequest, I2CRequestType, IT930x};
use crate::itedtv_bus::BusOps;
use crate::tc90522::TC90522;

use crate::px4_device::TunerError;

const NUM_REGS: usize = 0x10; // 実際の値に合わせて調整

#[derive(Default, Clone, Copy)]
struct BandwidthParam {
    coarse: u8,
    fine: u8,
}

#[derive(Debug, Clone, Copy)]
pub enum RT710ChipType
{
    UNKNOWN = 0,
    RT710,
    RT720,
}

#[derive(Clone, Copy)]
pub enum SignalOutputMode {
    Single = 0,
    Differential,
}

#[derive(Clone, Copy)]
pub enum AgcMode {
    Negative = 0,
    Positive,
}

#[derive(Clone, Copy)]
pub enum VgaAttenuateMode {
    Off = 0,
    On,
}

#[derive(Clone, Copy)]
pub enum FineGain {
    FineGain3DB = 0,
    FineGain2DB,
    FineGain1DB,
    FineGain0DB,
}

#[derive(Clone, Copy)]
pub enum ScanMode {
    Manual = 0,
    Auto,
}

pub struct RT710Config
{
    pub xtal: u32,
    pub loop_through: bool,
    pub clock_out: bool,
    pub signal_output_mode: SignalOutputMode,
    pub agc_mode: AgcMode,
    pub vga_atten_mode: VgaAttenuateMode,
    pub fine_gain: FineGain,
    pub scan_mode: ScanMode,
}

pub struct  RT710Priv
{
    lock: Mutex<()>,
    init: bool,
    freq: u32,
    chip: RT710ChipType,
}

pub struct RT710<'a, B: BusOps>
{
    tc90522: TC90522<'a, B>,
    //pub i2c_bus: u8,
    pub i2c_addr: u8,
    config: RT710Config,
    priv_: RT710Priv,
}

impl<'a, B: BusOps> RT710<'a, B>
{
    pub fn reverse_bit(val: u8) -> u8
    {
        let mut t = val;

        t = ((t & 0x55) << 1) | ((t & 0xAA) >> 1);
        t = ((t & 0x33) << 2) | ((t & 0xCC) >> 2);
        ((t & 0x0F) << 4) | ((t & 0xF0) >> 4)
    }

    pub fn read_regs(&self, reg: u8, buf: &mut [u8]) -> Result<(), CtrlMsgError>
    {
        if (buf.len() == 0) || (buf.len() > NUM_REGS - reg as usize)
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut write_buf = [0x00];
        let mut read_buf = vec![0u8; reg as usize + buf.len()];

        let mut reqs = 
        [
            I2CCommRequest
            {
                addr: self.i2c_addr,
                data: &mut write_buf,
                req: I2CRequestType::Write,
            },
            I2CCommRequest
            {
                addr: self.i2c_addr,
                data: &mut read_buf,
                req: I2CRequestType::Read,
            }
        ];

        self.tc90522.i2c_master_request(&mut reqs)?;

        // ここで buf へ値を出す
        // 逆イテレータで reg のサイズ前まで取りつつ、reverse_bit()
        for i in 0..buf.len()
        {
            buf[i] = Self::reverse_bit(read_buf[reg as usize + i]);
        }

        Ok(())
    }

    pub fn write_regs(&self, reg: u8, buf: &[u8]) -> Result<(), CtrlMsgError>
    {
        if (buf.len() == 0) || (buf.len() > (NUM_REGS - reg as usize))
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut wbuf = Vec::with_capacity(1 + buf.len());
        wbuf.push(reg);
        wbuf.extend_from_slice(buf);

        let mut reqs = 
        [
            I2CCommRequest
            {
                addr: self.i2c_addr,
                data: &mut wbuf,
                req: I2CRequestType::Write,
            }
        ];

        self.tc90522.i2c_master_request(&mut reqs)
    }

    pub fn new(it930x: &'a IT930x<B>, tc90522_bus: u8, tc90522_addr: u8) -> Self
    {
        Self 
        {
            tc90522: TC90522::new(it930x, tc90522_bus, tc90522_addr), 
            //i2c_addr: 0x7a, // 決まっているので 
            i2c_addr: 0x3d, // bit数が違うらしい？
            // px4_device.c の 1134〜1144行目
            config: RT710Config { xtal: 24000, loop_through: false, clock_out: false, signal_output_mode: SignalOutputMode::Differential, agc_mode: AgcMode::Positive, vga_atten_mode: VgaAttenuateMode::Off, fine_gain: FineGain::FineGain3DB, scan_mode: ScanMode::Manual, },
            priv_: RT710Priv { lock: Mutex::new(()), init: false, freq: 0, chip: RT710ChipType::RT710, }
        }
    }

    pub fn init(&mut self) -> Result<(), TunerError>
    {
        let mut tmp = [0u8; 1];
        {
            let _lock = self.priv_.lock.lock().unwrap();

            self.priv_.init = false;
            self.priv_.freq = 0;

            self.read_regs(0x03, &mut tmp)?;

            self.priv_.chip = 
            if (tmp[0] & 0xf0) == 0x70
            {
                RT710ChipType::RT710
            }
            else 
            {
                RT710ChipType::RT720    
            };

            self.priv_.init = true;
        }

        // いらないのでは？
        println!("RT710 init done. chip: {:?}, reg03=0x{:02x}", self.priv_.chip, tmp[0]);
        Ok(())
    }
}