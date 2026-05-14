use std::time::Duration;

use crate::itedtv_bus::BusOps;
use crate::r850::R850;
use crate::rt710::RT710;
use crate::tc90522::{System, TunerError};

use crate::it930x::{CtrlMsgError, GpioMode, IT930x};

const PX4_DEVICE_TS_SYNC_COUNT: usize = 4;
const PX4_DEVICE_TS_SYNC_SIZE: usize = 188 * PX4_DEVICE_TS_SYNC_COUNT;

/// px4_device_params.c に相当する設定群
#[derive(Debug, Clone)]
pub struct Px4DeviceConfig {
    pub tsdev_max_packets: u32,
    pub psb_purge_timeout: i32,
    pub disable_multi_device_power_control: bool,
    pub multi_device_power_control_mode: Px4MldevMode,
    pub s_tuner_no_sleep: bool,
    pub discard_null_packets: bool,
}

impl Default for Px4DeviceConfig {
    fn default() -> Self {
        Self {
            tsdev_max_packets: 2048,
            psb_purge_timeout: 2000,
            disable_multi_device_power_control: false,
            multi_device_power_control_mode: Px4MldevMode::All,
            s_tuner_no_sleep: false,
            discard_null_packets: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Px4MldevMode {
    All,
    SOnly,
    S0Only,
    S1Only,
}

// ストリームコンテキスト
// TSパケットの同期を管理する。
struct Px4StreamContext {
    // 各チャンネルの参照（またはID）を保持
    // Cの struct ptx_chrdev *chrdev[PX4_CHRDEV_NUM] に相当
    remain_buf: [u8; PX4_DEVICE_TS_SYNC_SIZE],
    remain_len: usize,
}

impl Px4StreamContext {
    fn new() -> Self {
        Self {
            remain_buf: [0u8; PX4_DEVICE_TS_SYNC_SIZE],
            remain_len: 0,
        }
    }
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

// チューナーが開かれる際に、それぞれの復調器に書き込むパラメータ(らしい)
const TC_INIT_T: [(u8, u8); 10] = [
    (0xb0, 0xa0),
    (0xb2, 0x3d),
    (0xb3, 0x25),
    (0xb4, 0x8b),
    (0xb5, 0x4b),
    (0xb6, 0x3f),
    (0xb7, 0xff),
    (0xb8, 0xc0),
    (0x1f, 0x00),
    (0x75, 0x00),
];
const TC_INIT_S: [(u8, u8); 3] = [(0x15, 0x00), (0x1d, 0x00), (0x04, 0x02)];

// デバイス全体の初期化用
const TC_INIT_S0: [(u8, u8); 2] = [(0x07, 0x31), (0x08, 0x77)];
const TC_INIT_T0: [(u8, u8); 2] = [(0x0e, 0x77), (0x0f, 0x13)];

pub enum Tuner<'a, B: BusOps> {
    RT710(RT710<'a, B>),
    R850(R850<'a, B>),
}

impl<'a, B: BusOps> Tuner<'a, B> {
    pub fn term(&mut self) -> Result<(), TunerError> {
        match self {
            Tuner::RT710(t) => t.term(),
            Tuner::R850(t) => t.term(),
        }
    }
}

pub struct Px4Chrdev<'a, B: BusOps> {
    pub system: System,

    // ここ3つは要らない気がする。
    // → IT930x内部に記載して良さげ。
    pub port_number: u8,
    pub slave_number: u8,
    pub sync_byte: u8,

    //pub tc90522: &'a TC90522<'a, B>,
    pub tuner: Tuner<'a, B>,
}

pub struct Px4Device<'a, B: BusOps> {
    it930x: &'a IT930x<B>,
    px4chrdev: Vec<Px4Chrdev<'a, B>>,
}

impl<'a, B: BusOps> Px4Device<'a, B> {
    pub fn new(it930x: &'a IT930x<B>) -> Self {
        Self {
            it930x: it930x,
            px4chrdev: Vec::new(),
        }
    }

    pub fn set_power(&mut self, state: bool) -> Result<(), CtrlMsgError> {
        println!(
            "[px4] backend_set_power: {}",
            if state { "true" } else { "false" }
        );

        if state {
            // gpio7 = low
            self.it930x.write_gpio(7, false)?;
            std::thread::sleep(Duration::from_millis(80));

            // gpio2 = high
            self.it930x.write_gpio(2, true)?;
            std::thread::sleep(Duration::from_millis(20));
        } else {
            // off は失敗しても無視
            let _ = self.it930x.write_gpio(2, false);
            let _ = self.it930x.write_gpio(7, true);
        }

        Ok(())
    }

    pub fn init(&mut self) -> Result<(), TunerError> {
        // 各種初期化なども、ここに入れてしまう。
        // ブリッジ自体の起動
        self.it930x.raise()?;
        self.it930x.load_firmware("it930x-firmware.bin")?;
        self.it930x.init_warm()?;

        // 電源投入
        self.it930x.set_gpio_mode(7, GpioMode::Out, true)?;
        self.it930x.set_gpio_mode(2, GpioMode::Out, true)?;

        self.it930x.write_gpio(7, true)?;
        self.it930x.write_gpio(2, false)?;

        self.it930x.set_gpio_mode(11, GpioMode::Out, true)?;
        self.it930x.write_gpio(11, false)?;

        for (i, (system, addr)) in PX4_CHRDEV_CONFIGS.iter().enumerate() {
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

            let tuner = match system {
                System::ISDB_S => Tuner::RT710(RT710::new(&self.it930x, 2, *addr, i % 2 == 1)),
                System::ISDB_T => Tuner::R850(R850::new(&self.it930x, 2, *addr, i % 2 == 1)),
            };

            self.px4chrdev.push(Px4Chrdev {
                system: *system,
                port_number: i as u8 + 1,
                slave_number: i as u8,
                sync_byte: ((i as u8 + 1) << 4) | 0x07,
                tuner: tuner,
            });
        }

        for chrdev in &mut self.px4chrdev {
            //chrdev.tc90522.init();

            let result = match &mut chrdev.tuner {
                Tuner::RT710(t) => t.init()?,
                Tuner::R850(t) => t.init()?,
            };
        }
        Ok(())
    }

    // memo: await 消したので、tokio いらないかも？
    // バックエンドの電源状態を制御します (px4_backend_set_power)
    fn backend_set_power(&self, state: bool) -> Result<(), CtrlMsgError> {
        // デバッグ出力相当（必要に応じてログライブラリを使用）
        println!(
            "px4_backend_set_power: {}",
            if state { "on" } else { "off" }
        );

        if state {
            // 電源 ON シーケンス
            // GPIO 0 を Low にしてリセット解除 (?)
            self.it930x.write_regs(0xd8b4, &[0x01])?; // GPIO 0 output enable
            self.it930x.write_regs(0xd8b3, &[0x00])?; // GPIO 0 output low

            // 10ms 待機
            std::thread::sleep(std::time::Duration::from_millis(10));

            // GPIO 0 を High に
            self.it930x.write_regs(0xd8b3, &[0x01])?; // GPIO 0 output high

            // 10ms 待機
            std::thread::sleep(std::time::Duration::from_millis(10));
        } else {
            // 電源 OFF シーケンス
            // GPIO 0 を Low にして保持
            self.it930x.write_regs(0xd8b4, &[0x01])?;
            self.it930x.write_regs(0xd8b3, &[0x00])?;
        }

        Ok(())
    }
}
