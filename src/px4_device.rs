use std::time::Duration;

use crate::itedtv_bus::BusOps;
use crate::rt710::RT710;
use crate::r850::R850;

use crate::it930x::{CtrlMsgError, IT930x};

// エラー関連
use thiserror::Error;
#[derive(Debug, Error)]
pub enum TunerError
{
    #[error("control message error: {0}")]
    CtrlMsg(#[from] CtrlMsgError),  // CtrlMsgError をラップ
    #[error("R850 chip not detected.")]
    ChipNotDetected,
}

#[derive(Debug, Clone, Copy)]
pub enum System
{
    ISDB_S,
    ISDB_T,
}

// チューナーデバイスの必要なパラメータ
// System, TC90522 bus の順
// これは、W3U4 の場合だけ
// S1UR とか Q3U4 のときは知らないが、Q3U4 は多分、これで良い。(これの外側で2つ持つイメージだと思う)
const PX4_CHRDEV_CONFIGS: [(System, u8); 4] = [
    (System::ISDB_S, 0x11),
    (System::ISDB_S, 0x13),
    (System::ISDB_T, 0x10),
    (System::ISDB_T, 0x12),
];


pub enum Tuner<'a, B: BusOps>
{
    RT710(RT710<'a, B>),
    R850(R850<'a, B>),
}

pub struct Px4Chrdev<'a, B: BusOps>
{
    pub system: System,

    // ここ3つは要らない気がする。
    // → IT930x内部に記載して良さげ。
    pub port_number: u8,
    pub slave_number: u8,
    pub sync_byte: u8,

    //pub tc90522: &'a TC90522<'a, B>,
    pub tuner: Tuner<'a, B>,
}

pub struct Px4Device<'a, B: BusOps>
{
    it930x: &'a IT930x<B>,
    px4chrdev: Vec<Px4Chrdev<'a, B>>,
}

impl<'a, B: BusOps> Px4Device<'a, B>
{
    pub fn new(it930x: &'a IT930x<B>) -> Self
    {
        Self 
        {
            it930x: it930x,
            px4chrdev: Vec::new(),
        }
    }

    pub fn set_power(&mut self, state: bool) -> Result<(), CtrlMsgError>
    {
        println!(
            "[px4] backend_set_power: {}",
            if state { "true" } else { "false" }
        );

        if state
        {
            // gpio7 = low
            self.it930x.write_gpio(7, false)?;
            std::thread::sleep(Duration::from_millis(80));

            // gpio2 = high
            self.it930x.write_gpio(2, true)?;
            std::thread::sleep(Duration::from_millis(20));
        }
        else
        {
            // off は失敗しても無視
            let _ = self.it930x.write_gpio(2, false);
            let _ = self.it930x.write_gpio(7, true);            
        }

        Ok(())
    }

    pub fn init(&mut self) -> Result<(), TunerError>
    {
        for (i, (system, addr)) in PX4_CHRDEV_CONFIGS.iter().enumerate()
        {
            // px4_device.c 1128 行目に chrdev4->tc90522.i2c = &it930x->i2c_master[1]; とあり
            // it930x.c の 571 行目で、priv->i2c[i].bus = i + 1; で、
            // it930x.c の 575 行目で、it930x->i2c_master[i].priv = &priv->i2c[i] とあるので、
            // bus 番号は 2 で固定。
            // -> px4 device の場合の話っぽい。
            //  -> pxmlt device の場合は、&it930x->i2c_master[input->i2c_bus - 1]; みたいになってる。
            //  -> s1ur や m1ur は [2] なので bus 番号は 3 らしい。
            // あと、CHRDEV ごとにアドレスが違くて、0x10〜0x13。
            //let tc90522 = TC90522::new(&it930x, 2, *addr);
            //tc90522s.push(tc90522);
        
            let tuner = match system
            {
                System::ISDB_S => Tuner::RT710(RT710::new(&self.it930x, 2, *addr)),
                System::ISDB_T => Tuner::R850(R850::new(&self.it930x, 2, *addr)),
            };

            self.px4chrdev.push(
                Px4Chrdev
                {
                    system: *system,
                    port_number: i as u8 + 1,
                    slave_number: i as u8,
                    sync_byte: ((i as u8 + 1) << 4) | 0x07,
                    tuner: tuner,
                }
            );
        }

        for chrdev in &mut self.px4chrdev
        {
            //chrdev.tc90522.init();

            let result = match &mut chrdev.tuner
            {
                Tuner::RT710(t) => t.init()?,
                Tuner::R850(t) => t.init()?,
            };
        }
        Ok(())
    }

}