use std::sync::Mutex;

use crate::it930x::{CtrlMsgError, I2CCommRequest, I2CRequestType, IT930x};
use crate::itedtv_bus::BusOps;
use crate::tc90522::TC90522;

use crate::px4_device::TunerError;

const R850_NUM_REGS: usize = 0x30;

// C の init_regs 配列を Rust にコピー
pub const INIT_REGS: [u8; R850_NUM_REGS] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xca, 0xc0, 0x72, 0x50, 0x00, 0xe0, 0x00, 0x30,
    0x86, 0xbb, 0xf8, 0xb0, 0xd2, 0x81, 0xcd, 0x46,
    0x37, 0x40, 0x89, 0x8c, 0x55, 0x95, 0x07, 0x23,
    0x21, 0xf1, 0x4c, 0x5f, 0xc4, 0x20, 0xa9, 0x6c,
    0x53, 0xab, 0x5b, 0x46, 0xb3, 0x93, 0x6e, 0x41,
];

// 設定
#[derive(Debug, Clone, Copy)]
pub struct R850Config {
    pub xtal: u32,
    pub loop_through: bool,
    pub clock_out: bool,
    pub no_imr_calibration: bool,
    pub no_lpf_calibration: bool,
}

// システム定義
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum R850System {
    Undefined = 0,
    DvbT,
    DvbT2,
    DvbT2_1,
    DvbC,
    J83B,
    IsdbT,
    Dtmb,
    Atsc,
    Fm,
}

// 帯域幅
#[derive(Debug, Clone, Copy)]
pub enum R850Bandwidth {
    B6M = 0,
    B7M,
    B8M,
}

// システム設定
#[derive(Debug, Clone, Copy)]
pub struct R850SystemConfig {
    pub system: R850System,
    pub bandwidth: R850Bandwidth,
    pub if_freq: u32,
}

// IMR 構造体
#[derive(Debug, Clone, Copy)]
pub struct R850Imr {
    pub gain: u8,
    pub phase: u8,
    pub iqcap: u8,
    pub value: u8,
}

// 内部状態
#[derive(Debug)]
pub struct R850Priv {
    pub lock: Mutex<()>,
    pub init: bool,
    pub chip: i32,
    pub xtal_pwr: u8,
    pub regs: [u8; R850_NUM_REGS],
    pub sleep: bool,
    pub sys: R850SystemConfig,
    pub mixer_mode: u8,
    pub mixer_amp_lpf_imr_cal: u8,
    pub imr_cal: [R850ImrCal; 2],
    pub sys_curr: R850SystemConfig,
}

#[derive(Debug)]
pub struct R850ImrCal {
    pub imr: [R850Imr; 5],
    pub done: bool,
    pub result: [bool; 5],
    pub mixer_amp_lpf: u8,
}

pub struct R850<'a, B: BusOps>
{
    tc90522: TC90522<'a, B>,

    // 設定パラメータ
    pub xtal: u32,
    pub loop_through: bool,
    pub clock_out: bool,
    pub no_imr_calibration: bool,
    pub no_lpf_calibration: bool,

    pub i2c_addr: u8,
    priv_: R850Priv,
}

impl<'a, B: BusOps> R850<'a, B>
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
        if (buf.len() == 0) || (buf.len() > (R850_NUM_REGS - reg as usize))
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut write_buf = [0];
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
        if (buf.len() == 0) || (buf.len() > (R850_NUM_REGS - reg as usize))
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

    // メモ: 初期値(デフォルト値)に戻すイメージ
    pub fn init_regs(&mut self)
    {
        self.priv_.regs.copy_from_slice(&INIT_REGS);

        // ここで各種微調整も可能らしい。
        // 微調整のコードは、r850.c 619〜633行目を参照
    }

    pub fn check_xtal_power(&mut self) -> Result<(), CtrlMsgError>
    {
        let bank = 55u8;
        let mut pwr = 3u8; // xtal 24MHz

        // 保持するレジスタ状態を初期化し、xtal power 確認のために変更後のレジスタ状態を保持
        self.init_regs();
        {
            let r = &mut self.priv_.regs;
            if self.priv_.chip != 0 {r[0x2f] &= 0xfd;}
            else {r[0x2f] &= 0xfc;}

            r[0x1b] &= 0x80;
            r[0x1b] |= 0x12;

            r[0x1e] &= 0xe0;
            r[0x1e] |= 0x08;

            r[0x22] &= 0x27;

            r[0x1d] &= 0x0f;

            r[0x21] |= 0xf8;

            r[0x22] &= 0x77;
            r[0x22] |= 0x80;
            
            r[0x1f] &= 0x80;
            r[0x1f] |= 0x40;
            r[0x1f] &= 0xbf;
        }

        // 本体のレジスタに書き込み
        self.write_regs(0x08, &self.priv_.regs[0x08..R850_NUM_REGS]);

        // ループで xtal_power を探す
        for i in 0..=3 {
            self.priv_.regs[0x22] &= 0xcf;
            self.priv_.regs[0x22] |= i << 4;

            self.write_regs(0x22, &[self.priv_.regs[0x22]])?;

            let mut tmp = [0u8; 1];
            self.read_regs(0x02, &mut tmp)?;

            if (tmp[0] & 0x40) != 0 && ((tmp[0] & 0x3f).wrapping_sub(bank - 6) <= 12) {
                pwr = i;
                break;
            }
        }

        if pwr < 3 
        {
            pwr += 1;
        }
        
        self.priv_.xtal_pwr = pwr;

        Ok(())
    }

    pub fn new(it930x: &'a IT930x<B>, tc90522_bus: u8, tc90522_addr: u8) -> Self
    {
        Self 
        { 
            tc90522: TC90522::new(it930x, tc90522_bus, tc90522_addr), 
            i2c_addr: 0x7c, 
            //i2c_addr: 0x3e,

            xtal: 0,
            loop_through: false,
            clock_out: false,
            no_imr_calibration: false,
            no_lpf_calibration: false,

            priv_: R850Priv
            {
                lock: Mutex::new(()),
                init: false,
                chip: 0,
                xtal_pwr: 0,
                regs: [0u8; R850_NUM_REGS],
                sleep: false,
                sys: R850SystemConfig { system: R850System::Undefined, bandwidth: R850Bandwidth::B6M, if_freq: 0 },
                sys_curr: R850SystemConfig { system: R850System::Undefined, bandwidth: R850Bandwidth::B6M, if_freq: 0 },
                imr_cal: [
                    R850ImrCal{imr: [R850Imr{gain: 0, phase: 0, iqcap: 0, value: 0}; 5], done: false, result: [false; 5], mixer_amp_lpf: 0,}, 
                    R850ImrCal{imr: [R850Imr{gain: 0, phase: 0, iqcap: 0, value: 0}; 5], done: false, result: [false; 5], mixer_amp_lpf: 0,},
                    ],
                mixer_mode: 0,
                mixer_amp_lpf_imr_cal: 0
            },
        }
    }

    pub fn init(&mut self) -> Result<(), TunerError>
    {
        // 初期状態の設定
        {
            let _lock = self.priv_.lock.lock().unwrap();

            self.priv_.init = false;

            self.priv_.chip = 0;
            self.priv_.sleep = false;

            self.priv_.sys.system = R850System::Undefined;
            self.priv_.sys_curr.system = R850System::Undefined;

            for cal in self.priv_.imr_cal.iter_mut()
            {
                cal.done = false;
                cal.result = [false; 5];
                cal.mixer_amp_lpf = 0;
                for imr in cal.imr.iter_mut()
                {
                    *imr = R850Imr { gain: 0, phase: 0, iqcap: 0, value: 0 };
                }
            }

            // チップ判定
            let mut detected = false;
            for _ in 0..4
            {
                let mut tmp = [0u8];
                if self.read_regs(0x00, &mut tmp).is_ok()
                {
                    if (tmp[0] & 0x98) != 0
                    {
                        self.priv_.chip = 0;
                        detected = true;
                        break;
                    }
                }
            }

            if !detected
            {
                // なんか、新しいエラーを生やせばいいかな
                return Err(TunerError::ChipNotDetected);
            }

            // レジスタ初期化
            let mut regs = [0u8; R850_NUM_REGS - 0x08];
            self.read_regs(0x08, &mut regs)?;

            // check xtal power

            self.write_regs(0x08, &regs)?;

            // init regs
        }

        Ok(())
    }
}