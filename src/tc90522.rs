// TC90522 の制御用

use std::sync::{Mutex, atomic::{AtomicU8, Ordering}};

// 多分、これで大丈夫だと思う。
use crate::{it930x::IT930x, itedtv_bus::BusOps};

// 同じ定義を使うだけ
use crate::it930x::{I2CRequestType, I2CCommRequest, CtrlMsgError};

pub struct TC90522<'a, B: BusOps>
{
    it930x: &'a IT930x<B>,

    // I2C バスアクセス用
    pub bus: u8,

    // TC90522 の I2Cアドレス
    pub i2c_addr: u8,

    // 内部制御用
    ctrl_lock: Mutex<()>,
    i2c_lock: Mutex<()>,
}

impl<'a, B: BusOps> TC90522<'a, B>
{
    pub fn new(it930x: &'a IT930x<B>, bus: u8, i2c_addr: u8) -> Self
    {
        TC90522
        {
            it930x,
            bus,
            i2c_addr,
            ctrl_lock: Mutex::new(()),
            i2c_lock: Mutex::new(()),
        }
    }

    pub fn read_regs(&self, reg: u8, buf: &mut [u8]) -> Result<(), CtrlMsgError>
    {
        if buf.is_empty()
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let _lock = self.ctrl_lock.lock().unwrap();
        let mut write_buf = [reg];

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
                data: buf,
                req: I2CRequestType::Read,
            }
        ];

        self.it930x.i2c_master_request(self.bus, &mut reqs)
    }

    pub fn write_regs(&self, reg: u8, buf: &[u8]) -> Result<(), CtrlMsgError>
    {
        if buf.is_empty() || (buf.len() > 254)
        {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut wbuf = Vec::with_capacity(1 + buf.len());
        wbuf.push(reg);
        wbuf.extend_from_slice(buf);
        
        let mut req = 
        [
            I2CCommRequest
            {
                addr: self.i2c_addr,
                data: &mut wbuf,
                req: I2CRequestType::Write,
            }
        ];

        let _lock = self.ctrl_lock.lock().unwrap();
        self.it930x.i2c_master_request(self.bus, &mut req)
    }

    pub fn i2c_master_request(&self, requests: &mut [I2CCommRequest]) -> Result<(), CtrlMsgError>
    {
        let _lock = self.i2c_lock.lock().unwrap();
        let mut master_reqs: Vec<I2CCommRequest> = Vec::new();

        // for文内での借用における、データ保持箇所
        let mut buffers: Vec<Vec<u8>> = vec![Vec::new(); requests.len()];

        for (req, buf) in requests.iter_mut().zip(buffers.iter_mut())
        {
            match req.req
            {
                I2CRequestType::Read =>
                {
                    buf.clear();
                    buf.push(0xFE);
                    buf.push(req.addr << 1);

                    master_reqs.push(
                        I2CCommRequest 
                        { 
                            addr: self.i2c_addr, 
                            data: buf.as_mut_slice(), 
                            req: I2CRequestType::Write, 
                        }
                    );

                    master_reqs.push(
                        I2CCommRequest 
                        { 
                            addr: self.i2c_addr, 
                            data: req.data, 
                            req: I2CRequestType::Read, 
                        }
                    )
                }

                I2CRequestType::Write =>
                {
                    buf.clear();
                    buf.push(0xFE); // tc90522.c 327行目
                    buf.push(req.addr << 1);
                    buf.extend_from_slice(req.data);

                    master_reqs.push(
                        I2CCommRequest 
                        { 
                            addr: self.i2c_addr, 
                            data: buf.as_mut_slice(),
                            req: I2CRequestType::Write,
                        }
                    );
                }
            }
        }

        self.it930x.i2c_master_request(self.bus, &mut master_reqs)
    }

}