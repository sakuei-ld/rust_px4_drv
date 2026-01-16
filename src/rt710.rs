// ここからは RT710の話

use std::sync::Mutex;
use crate::itedtv_bus::BusOps;
use crate::it930x::{IT930x, CtrlMsgError, I2CCommRequest, I2CRequestType};

#[derive(Debug, Clone, Copy)]
pub enum RT710ChipType
{
    RT710,
    RT720,
}

pub struct  RT710Priv
{
    lock: Mutex<()>,
    init: bool,
    freq: u32,
    chip: RT710ChipType,
}

pub struct RT710<'a, B:BusOps>
{
    it930x: &'a IT930x<B>,
    pub i2c_bus: u8,
    pub i2c_addr: u8,
    priv_: RT710Priv,
}

impl<'a, B: BusOps> RT710<'a, B>
{
    const NUM_REGS: u8 = 0x10;
    pub fn read_regs(&self, reg: u8, buf: &mut [u8]) -> Result<(), CtrlMsgError>
    {
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

        self.it930x.i2c_master_request(self.i2c_bus, &mut reqs)?;

        // ここで buf へ値を出す
        // 逆イテレータで reg のサイズ前まで取りつつ、reverse_bit()

        Ok(())
    }

    pub fn new(it930x: &'a IT930x<B>, bus: u8, addr: u8) -> Self
    {
        Self 
        { 
            it930x, 
            i2c_bus: bus, 
            i2c_addr: addr, 
            priv_: RT710Priv
            {
                lock: Mutex::new(()),
                init: false,
                freq: 0,
                chip: RT710ChipType::RT710, // init をここに含めていいのでは？
            }
        }
    }

    pub fn init(&mut self) -> Result<(), CtrlMsgError>
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