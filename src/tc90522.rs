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
        println!(
            "[tc90522.new] bus={} tc90522_addr=0x{:02X}",
            bus, i2c_addr
        );

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

    /*
    // Cの特別扱い分岐の再現
    if requests.len() == 2
        && requests[0].req == I2CRequestType::Write
        && requests[1].req == I2CRequestType::Read
    {
        // 1) [0xFE, addr<<1, payload...]
        let mut b0 = Vec::with_capacity(2 + requests[0].data.len());
        b0.push(0xFE);
        b0.push(requests[0].addr << 1);
        b0.extend_from_slice(requests[0].data);

        // 2) [0xFE, addr<<1|1]
        let mut b1 = [0xFE, (requests[1].addr << 1) | 0x01];

        // 3本を「1回の呼び出し」で投げる
        let mut master = [
            I2CCommRequest {
                addr: self.i2c_addr,
                data: b0.as_mut_slice(),
                req: I2CRequestType::Write,
            },
            I2CCommRequest {
                addr: self.i2c_addr,
                data: &mut b1,
                req: I2CRequestType::Write,
            },
            I2CCommRequest {
                addr: self.i2c_addr,
                data: requests[1].data,
                req: I2CRequestType::Read,
            },
        ];

        return self.it930x.i2c_master_request(self.bus, &mut master);
    }
    */

        for req in requests.iter_mut()
        {
            println!(
                "[tc90522.req] target_addr=0x{:02X} {:?} len={} first={:02X?}",
                req.addr, req.req, req.data.len(), &req.data.get(..req.data.len().min(8)).unwrap_or(&[])
            );

            match req.req
            {
                I2CRequestType::Read =>
                {
                    let mut write_buf = [0xFE, (req.addr << 1) | 0x01];

                    // debug
                    println!(
                        "[tc90522->it930x] READ-SET bus={} it930x_addr=0x{:02X} data={:02X?}",
                        self.bus, self.i2c_addr, &write_buf
                    );
                    println!(
                        "[tc90522->it930x] READ-DATA bus={} it930x_addr=0x{:02X} len={}",
                        self.bus, self.i2c_addr, req.data.len()
                    );

                    let mut master = 
                    [
                        I2CCommRequest{
                            addr: self.i2c_addr,
                            data: &mut write_buf,
                            req: I2CRequestType::Write,
                        },
                        I2CCommRequest{
                            addr: self.i2c_addr,
                            data: req.data,
                            req: I2CRequestType::Read,
                        }
                    ];

                    self.it930x.i2c_master_request(self.bus, &mut master)?;
                }
                
                I2CRequestType::Write =>
                {
                    if req.data.is_empty() || req.data.len() > 253
                    {
                        return Err(CtrlMsgError::InvalidLength);
                    }

                    let mut buf = Vec::with_capacity(2 + req.data.len());
                    buf.push(0xFE);
                    buf.push(req.addr << 1);
                    buf.extend_from_slice(req.data);

                    // debug
                    println!(
                        "[tc90522->it930x] WRITE bus={} it930x_addr=0x{:02X} data={:02X?}", 
                        self.bus, self.i2c_addr, &buf
                    );

                    let mut master = [
                        I2CCommRequest{
                            addr: self.i2c_addr,
                            data: buf.as_mut_slice(),
                            req: I2CRequestType::Write,
                        }
                    ];

                    self.it930x.i2c_master_request(self.bus, &mut master)?;
                }
            }
        }
        Ok(())
    }
}