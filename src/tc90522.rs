// TC90522 の制御用

use std::sync::{Mutex, atomic::{AtomicU8, Ordering}};

// 多分、これで大丈夫だと思う。
use crate::itedtv_bus::BusOps;

// 同じ定義を使うだけ
use crate::it930x::{I2CRequestType, I2CCommRequest, CtrlMsgError};

pub struct TC90522<'a, B: BusOps>
{
    // I2C バスアクセス用
    pub bus: &'a B,

    // TC90522 の I2Cアドレス
    pub ic2_addr: u8,

    // 内部制御用
    ctrl_lock: Mutex<()>,
    i2c_lock: Mutex<()>,
}

impl<'a, B: BusOps> TC90522<'a, B>
{
    pub fn new(bus: &'a B, i2c_addr: u8) -> Self
    {
        TC90522
        {
            bus,
            i2c_addr,
            ctrl_lock: Mutex::new(()),
            i2c_lock: Mutex::new(()),
        }
    }

    pub fn init(&self) -> Result<(), i32>
    {
        // ここに tc90522_init() 相当の処理を入れる
        // ……いらないのでは？
        Ok(())
    }

    pub fn i2c_master_request(&self, requests: &mut [I2CCommRequest]) -> Result<(), i32>
    {
        let _lock = self.i2c_lock.lock().unwrap();

        for req in requests.iter_mut()
        {
            match req.req
            {
                I2CRequestType::Read =>
                {

                }

                I2CRequestType::Write =>
                {
                    
                }
            }
        }
    }

}