use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

use crate::drivers::it930x::{CtrlMsgError, I2CCommRequest, I2CRequestType, IT930x};
use crate::drivers::itedtv_bus::BusOps;
use crate::drivers::px4_device::{SatelliteTuner, Tuner};
use crate::drivers::tc90522::{System, TunerError, TC90522};

const NUM_REGS: usize = 0x10;

const RT710_INIT_REGS: [u8; NUM_REGS] = [
    0x03, 0x5c, 0x08, 0x30, 0x41, 0x48, 0xed, 0x25, 0x47, 0xfc, 0x48, 0x22, 0x08, 0x0f, 0xf3, 0x59,
];

const RT720_INIT_REGS: [u8; NUM_REGS] = [
    0xff, 0x5c, 0x88, 0x30, 0x41, 0xc8, 0xed, 0x25, 0x47, 0xfc, 0x48, 0xa2, 0x08, 0x0f, 0xf3, 0x59,
];

const SLEEP_REGS: [u8; NUM_REGS] = [
    0xff, 0x5c, 0x88, 0x30, 0x41, 0xc8, 0xed, 0x25, 0x47, 0xfc, 0x48, 0xa2, 0x08, 0x0f, 0xf3, 0x59,
];

const RT710_LNA_ACC_GAIN: [i32; 19] = [
    0, 26, 42, 74, 103, 129, 158, 181, 188, 200, 220, 248, 280, 312, 341, 352, 366, 389, 409,
];

const RT720_LNA_ACC_GAIN: [i32; 32] = [
    0, 27, 53, 81, 109, 134, 156, 176, 194, 202, 211, 221, 232, 245, 258, 271, 285, 307, 326, 341,
    357, 374, 393, 410, 428, 439, 445, 470, 476, 479, 495, 507,
];

// px4_device.c の定義を、こっちでしか使わないので移動
// BS/CS用 (ISDB-S) 初期化パラメータ
const TC_INIT_S: [(u8, &'static [u8]); 3] = [(0x15, &[0x00]), (0x1d, &[0x00]), (0x04, &[0x02])];

// px4_device.c の定義を、こっちでしか使わないので移動
// デバイス全体の初期化用 (BS/CS)
const TC_INIT_S0: [(u8, &'static [u8]); 2] = [(0x07, &[0x31]), (0x08, &[0x77])];

#[derive(Default, Clone, Copy)]
struct BandwidthParam {
    coarse: u8,
    fine: u8,
}

// Cコードの bandwidth_params 構造体相当
struct BandwidthLookup {
    bandwidth: u32,
    param: BandwidthParam,
}

const BANDWIDTH_PARAMS: [BandwidthLookup; 12] = [
    BandwidthLookup {
        bandwidth: 220000,
        param: BandwidthParam {
            coarse: 0x00,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 236000,
        param: BandwidthParam {
            coarse: 0x01,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 252000,
        param: BandwidthParam {
            coarse: 0x02,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 268000,
        param: BandwidthParam {
            coarse: 0x03,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 284000,
        param: BandwidthParam {
            coarse: 0x04,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 300000,
        param: BandwidthParam {
            coarse: 0x05,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 316000,
        param: BandwidthParam {
            coarse: 0x06,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 332000,
        param: BandwidthParam {
            coarse: 0x07,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 348000,
        param: BandwidthParam {
            coarse: 0x08,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 364000,
        param: BandwidthParam {
            coarse: 0x09,
            fine: 0,
        },
    },
    BandwidthLookup {
        bandwidth: 380000,
        param: BandwidthParam {
            coarse: 0x0a,
            fine: 1,
        },
    },
    BandwidthLookup {
        bandwidth: 396000,
        param: BandwidthParam {
            coarse: 0x0b,
            fine: 1,
        },
    },
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RT710ChipType {
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

pub struct RT710Config {
    pub xtal: u32,
    pub loop_through: bool,
    pub clock_out: bool,
    pub signal_output_mode: SignalOutputMode,
    pub agc_mode: AgcMode,
    pub vga_atten_mode: VgaAttenuateMode,
    pub fine_gain: FineGain,
    pub scan_mode: ScanMode,
}

pub struct RT710Priv {
    //lock: Mutex<()>,
    init: bool,
    freq: u32,
    chip: RT710ChipType,
}

pub struct RT710<'a, B: BusOps> {
    tc90522: TC90522<'a, B>,
    //pub i2c_bus: u8,
    pub i2c_addr: u8,
    config: RT710Config,
    priv_: Mutex<RT710Priv>,
    is_initialized: AtomicBool,
}

impl<'a, B: BusOps> RT710<'a, B> {
    // bit反転
    fn reverse_bit(val: u8) -> u8 {
        let mut t = val;

        t = ((t & 0x55) << 1) | ((t & 0xAA) >> 1);
        t = ((t & 0x33) << 2) | ((t & 0xCC) >> 2);
        ((t & 0x0F) << 4) | ((t & 0xF0) >> 4)
    }

    // レジストリ読み取り
    fn read_regs(&self, reg: u8, buf: &mut [u8]) -> Result<(), CtrlMsgError> {
        if (buf.len() == 0) || (buf.len() > NUM_REGS - reg as usize) {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut write_buf = [0x00];
        let mut read_buf = vec![0u8; reg as usize + buf.len()];

        let mut reqs = [
            I2CCommRequest {
                addr: self.i2c_addr,
                data: &mut write_buf,
                req: I2CRequestType::Write,
            },
            I2CCommRequest {
                addr: self.i2c_addr,
                data: &mut read_buf,
                req: I2CRequestType::Read,
            },
        ];

        self.tc90522.i2c_master_request(&mut reqs)?;

        // ここで buf へ値を出す
        // 逆イテレータで reg のサイズ前まで取りつつ、reverse_bit()
        for i in 0..buf.len() {
            buf[i] = Self::reverse_bit(read_buf[reg as usize + i]);
        }

        Ok(())
    }

    // レジストリ書き込み
    fn write_regs(&self, reg: u8, buf: &[u8]) -> Result<(), CtrlMsgError> {
        if (buf.len() == 0) || (buf.len() > (NUM_REGS - reg as usize)) {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut wbuf = Vec::with_capacity(1 + buf.len());
        wbuf.push(reg);
        wbuf.extend_from_slice(buf);

        let mut reqs = [I2CCommRequest {
            addr: self.i2c_addr,
            data: &mut wbuf,
            req: I2CRequestType::Write,
        }];

        self.tc90522.i2c_master_request(&mut reqs)
    }

    // チューナー制御 および 計算関数 (内部処理用)
    // 内部レジスタ配列と周波数を引数にとる
    // 目標とする周波数に合わせ、PLL（位相同期回路）の分周比、VCO（電圧制御発振器）周波数、およびSDM（シグマ・デルタ変調）のパラメータを計算し、レジスタ配列に適用・書き込みを行う
    fn set_pll(&self, regs: &mut [u8; 16], freq: u32) -> Result<(), CtrlMsgError> {
        let xtal = self.config.xtal; // Cコード: u32 (ex: 24000)
        let vco_min: u32 = 2350000;
        let vco_max: u32 = vco_min * 2;
        let mut mix_div: u32 = 2;
        let mut vco_freq = freq * mix_div;

        let mut priv_data = self.priv_.lock().unwrap();

        // Cコード: t->priv.freq = 0;
        priv_data.freq = 0;

        // VCO周波数が min と max の間に収まるように分周比を計算
        while mix_div <= 16 {
            if vco_freq >= vco_min && vco_freq <= vco_max {
                break;
            }
            mix_div *= 2;
            vco_freq = freq * mix_div;
        }

        let div_num: u8 = match mix_div {
            2 => 1,
            4 => 0,
            8 => 2,
            16 => 3,
            _ => 0,
        };

        regs[0x04] &= 0xfe;
        regs[0x04] |= div_num & 0x01;

        self.write_regs(0x04, &[regs[0x04]])?;

        // チップ種別による分岐 (RT710 か RT720 か)
        if priv_data.chip == RT710ChipType::RT720 {
            regs[0x08] &= 0xef;
            regs[0x08] |= (div_num << 3) & 0x10;

            self.write_regs(0x08, &[regs[0x08]])?;

            regs[0x04] &= 0x3f;

            if div_num <= 1 {
                regs[0x04] |= 0x40;
                regs[0x0c] |= 0x10;
            } else {
                regs[0x04] |= 0x80;
                regs[0x0c] &= 0xef;
            }

            self.write_regs(0x04, &[regs[0x04]])?;
            self.write_regs(0x0c, &[regs[0x0c]])?;
        }

        // PLL 整数部 (nint) と分数部 (vco_fra) の計算
        let mut nint = (vco_freq / 2) / xtal;
        let mut vco_fra = vco_freq - (xtal * 2 * nint);

        // 分数部の補正
        if vco_fra < (xtal / 64) {
            vco_fra = 0;
        } else if vco_fra > (xtal * 127 / 64) {
            vco_fra = 0;
            nint += 1;
        } else if vco_fra > (xtal * 127 / 128) && vco_fra < xtal {
            vco_fra = xtal * 127 / 128;
        } else if vco_fra > xtal && vco_fra < (xtal * 129 / 128) {
            vco_fra = xtal * 129 / 128;
        }

        let ni = (nint - 13) / 4;
        let si = nint - (ni * 4) - 13;

        //regs[0x05] = (ni as u8 & 0x3f) | ((si as u8 << 6) & 0xc0);
        regs[0x05] = (ni as u8 & 0x3f) | ((si << 6) as u8 & 0xc0);

        self.write_regs(0x05, &[regs[0x05]])?;

        if vco_fra == 0 {
            regs[0x04] |= 0x02;
        }

        self.write_regs(0x04, &[regs[0x04]])?;

        // SDM (Sigma-Delta Modulator) の計算
        let mut nsdm: u32 = 2;
        let mut sdm: u32 = 0;

        while vco_fra > 1 {
            let t = (xtal * 2) / nsdm;
            if vco_fra > t {
                sdm += 0x8000 / (nsdm / 2);
                vco_fra -= t;

                if nsdm >= 0x8000 {
                    break;
                }
            }
            nsdm *= 2;
        }

        regs[0x07] = ((sdm >> 8) & 0xff) as u8;
        regs[0x06] = (sdm & 0xff) as u8;

        self.write_regs(0x07, &[regs[0x07]])?;
        self.write_regs(0x06, &[regs[0x06]])?;

        // 設定が成功したら周波数を記録
        priv_data.freq = freq;

        Ok(())
    }

    // インスタンス生成
    pub fn new(
        it930x: &'a IT930x<B>,
        tc90522_bus: u8,
        tc90522_addr: u8,
        is_secondary: bool,
    ) -> Result<Self, TunerError> {
        let tc90522 = TC90522::new(
            it930x,
            tc90522_bus,
            tc90522_addr,
            System::ISDB_S,
            is_secondary,
        );

        // 生成された直後に、この論理コアをスリープ状態にする
        //tc90522.sleep(true)?;
        // まだ、ちゃんと立ち上がってなくて、送れないっぽい

        Ok(Self {
            tc90522,
            i2c_addr: 0x7a, // 決まっているので
            // px4_device.c の 1134〜1144行目
            config: RT710Config {
                xtal: 24000,
                loop_through: false,
                clock_out: false,
                signal_output_mode: SignalOutputMode::Differential,
                agc_mode: AgcMode::Positive,
                vga_atten_mode: VgaAttenuateMode::Off,
                fine_gain: FineGain::FineGain3DB,
                scan_mode: ScanMode::Manual,
            },
            priv_: Mutex::new(RT710Priv {
                //lock: Mutex::new(()),
                init: false,
                freq: 0,
                chip: RT710ChipType::RT710,
            }),
            is_initialized: AtomicBool::new(false),
        })
    }

    // チューナーをスリープ状態に移行
    fn sleep(&self, priv_data: &RT710Priv) -> Result<(), TunerError> {
        //let mut priv_data = self.priv_.lock().unwrap();

        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        let mut regs = SLEEP_REGS;

        //let _lock = self.priv_.lock.lock().unwrap();

        match priv_data.chip {
            RT710ChipType::RT720 => {
                regs[0x01] = 0x5e;
                regs[0x03] |= 0x20;
            }
            _ => {
                if self.config.clock_out {
                    regs[0x03] = 0x20;
                }
            }
        }

        // debug
        println!(
            "[debug] rt710.sleep chip={:?} clock_out={} r01=0x{:02x} r03=0x{:02x}",
            priv_data.chip, self.config.clock_out, regs[0x01], regs[0x03]
        );

        self.write_regs(0x00, &regs)?;

        Ok(())
    }

    // 選局処理 らしい
    // 各種設定を反映し、set_pll() で周波数を合わせる。
    // その後、シンボルレートとロールオフ率から適切な帯域幅（Bandwidth）パラメータを計算し、フィルターを設定する。
    pub fn set_params(
        &self,
        freq: u32,
        mut symbol_rate: u32,
        rolloff: u32,
    ) -> Result<(), TunerError> {
        // 1. 基本チェック
        if rolloff > 5 {
            return Err(TunerError::InvalidArgument);
        }

        let mut regs = [0u8; NUM_REGS];
        let mut bw_param = BandwidthParam::default();

        // ロックを取得して内部状態を参照・変更する
        {
            let priv_data = self.priv_.lock().unwrap();
            if !priv_data.init {
                return Err(TunerError::InvalidState);
            }

            // 2. 初期レジスタ値のコピー (memcpy相当)
            regs.copy_from_slice(if priv_data.chip == RT710ChipType::RT710 {
                &RT710_INIT_REGS
            } else {
                &RT720_INIT_REGS
            });

            // 3. コンフィグに基づくビット操作
            if self.config.loop_through {
                regs[0x01] &= 0xfb;
            } else {
                regs[0x01] |= 0x04;
            }

            if self.config.clock_out {
                regs[0x03] &= 0xef;
            } else {
                regs[0x03] |= 0x10;
            }

            match self.config.signal_output_mode {
                SignalOutputMode::Differential => regs[0x0b] &= 0xef,
                _ => regs[0x0b] |= 0x10,
            }

            match self.config.agc_mode {
                AgcMode::Positive => regs[0x0d] |= 0x10,
                _ => regs[0x0d] &= 0xef,
            }

            match self.config.vga_atten_mode {
                VgaAttenuateMode::On => regs[0x0b] |= 0x08,
                _ => regs[0x0b] &= 0xf7,
            }

            // 4. Fine Gain の設定
            if priv_data.chip == RT710ChipType::RT710 {
                let fg = self.config.fine_gain as u8;
                if FineGain::FineGain3DB as u8 <= fg && fg <= FineGain::FineGain0DB as u8 {
                    regs[0x0e] &= 0xfc;
                    regs[0x0e] |= fg & 0x03;
                }
            } else {
                match self.config.fine_gain {
                    FineGain::FineGain3DB | FineGain::FineGain2DB => regs[0x0e] &= 0xfe,
                    _ => regs[0x0e] |= 0x01,
                }
                regs[0x03] &= 0xf0;
            }
        } // ここで、self.priv_ の Mutex を外している

        // 5. レジスタ一括書き込みと PLL 設定
        self.write_regs(0x00, &regs)?;
        self.set_pll(&mut regs, freq)?;

        // 10ms 待機
        std::thread::sleep(std::time::Duration::from_millis(10));

        // 再度ロック取得
        let priv_data = self.priv_.lock().unwrap();

        // 6. チップごとの事後調整
        if priv_data.chip == RT710ChipType::RT710 {
            if (freq.saturating_sub(1600000)) >= 350000 {
                regs[0x02] &= 0xbf;
                regs[0x08] &= 0x7f;
                if freq >= 1950000 {
                    regs[0x0a] = 0x38;
                }
            } else {
                regs[0x02] |= 0x40;
                regs[0x08] |= 0x80;
            }
            self.write_regs(0x0a, &[regs[0x0a]])?;
            self.write_regs(0x02, &[regs[0x02]])?;
            self.write_regs(0x08, &[regs[0x08]])?;

            regs[0x0e] &= 0xf3;
            if freq >= 2000000 {
                regs[0x0e] |= 0x08;
            }
            self.write_regs(0x0e, &[regs[0x0e]])?;
        } else {
            // RT720 用スキャンモード調整
            match self.config.scan_mode {
                ScanMode::Auto => {
                    regs[0x0b] |= 0x02;
                    symbol_rate += 10000;
                }
                _ => {
                    regs[0x0b] &= 0xfc;
                    if symbol_rate >= 15000 {
                        symbol_rate += 6000;
                    }
                }
            }
            self.write_regs(0x0b, &[regs[0x0b]])?;
        }

        // 7. 帯域幅（Bandwidth）の計算
        let bandwidth = (symbol_rate * (115 + (rolloff * 5))) / 10;
        if bandwidth == 0 {
            return Err(TunerError::InvalidState); // C: -ECANCELED
        }

        // 8. bw_param (coarse / fine) の決定
        if priv_data.chip == RT710ChipType::RT710 {
            if bandwidth >= 380000 {
                let diff = bandwidth - 380000;
                bw_param.coarse = 0x10 + ((diff / 17400) as u8 & 0xff);
                if (diff % 17400) != 0 {
                    bw_param.coarse += 1;
                }
                bw_param.fine = 1;
            } else {
                for entry in BANDWIDTH_PARAMS.iter() {
                    if bandwidth <= entry.bandwidth {
                        bw_param = entry.param;
                        break;
                    }
                }
            }
        } else {
            // RT720 専用の複雑な計算ロジック
            bw_param.fine = if rolloff > 1 { 1 } else { 0 };
            let range = (bw_param.fine as u32) * 20000;

            // Cコード上では順序が逆で、意味が無いので、コメントアウト
            // Cコードの実装ミスの可能性があり、場合によっては、この処理を入れた方が良い可能性あり
            // 受信幅のマージンを増やすための処理っぽい？
            //if symbol_rate <= 15000 {
            //    symbol_rate += 3000;
            //} else if symbol_rate <= 20000 {
            //    symbol_rate += 2000;
            //} else if symbol_rate <= 30000 {
            //    symbol_rate += 1000;
            //}

            //let s = sr_adj * 12;
            let s = symbol_rate * 12;

            if s <= (88000 + range) {
                bw_param.coarse = 0;
            } else if s <= (368000 + range) {
                let val = s - 88000 - range;
                bw_param.coarse = (val / 20000) as u8;
                if (val % 20000) != 0 {
                    bw_param.coarse += 1;
                }
                if bw_param.coarse > 6 {
                    bw_param.coarse += 1;
                }
            } else if s <= (764000 + range) {
                let val = s - 368000 - range;
                bw_param.coarse = ((val / 20000) + 15) as u8;
                // Cコードの特殊な剰余判定: (s + 25216 - range) % 20000
                if ((s + 25216).wrapping_sub(range) % 20000) != 0 {
                    bw_param.coarse += 1;
                }

                // オフセット調整(内容不明)
                if bw_param.coarse >= 33 {
                    bw_param.coarse += 3;
                } else if bw_param.coarse >= 29 {
                    bw_param.coarse += 2;
                } else if bw_param.coarse >= 27 {
                    bw_param.coarse += 3;
                } else if bw_param.coarse >= 24 {
                    bw_param.coarse += 2;
                } else if bw_param.coarse >= 19 {
                    bw_param.coarse += 1;
                }
            } else {
                bw_param.coarse = 42;
            }
        }

        // 9. 最終レジスタ書き込み
        regs[0x0f] = ((bw_param.coarse << 2) & 0xfc) | (bw_param.fine & 0x03);
        self.write_regs(0x0f, &[regs[0x0f]])?;

        Ok(())
    }

    // 指定した周波数でPLLが正常にロック（同調）したかを判定
    fn is_pll_locked(&self) -> Result<bool, TunerError> {
        let priv_data = self.priv_.lock().unwrap();
        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        let mut tmp = [0u8; 1];
        // 0x02 レジスタを読み取る
        self.read_regs(0x02, &mut tmp)?;

        // bit 7 (0x80) が 1 ならロック
        Ok((tmp[0] & 0x80) != 0)
    }

    // 現在チューナーに設定されている RF ゲインのインデックス値（0〜18程度）を計算、取得する。
    pub fn get_rf_gain(&self) -> Result<u8, TunerError> {
        let priv_data = self.priv_.lock().unwrap();
        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        let mut tmp = [0u8; 1];
        self.read_regs(0x01, &mut tmp)?;
        let val = tmp[0];

        // ビット操作でゲイン値(g)を算出
        let g = ((val & 0xf0) >> 4) | ((val & 0x01) << 4);

        if priv_data.chip == RT710ChipType::RT710 {
            let gain = if g <= 2 {
                0
            } else if g <= 9 {
                g - 2
            } else if g <= 12 {
                7
            } else if g <= 22 {
                g - 5
            } else {
                18
            };
            Ok(gain)
        } else {
            // RT720などの場合はgをそのまま返すか、別の変換が必要
            // Cコードでは RT710 以外の場合 gain がセットされないため g をそのまま返すと想定
            Ok(g)
        }
    }

    // get_rf_gain で取得したゲイン値と、現在の周波数帯、およびルックアップテーブル（RT710_LNA_ACC_GAIN / RT720_LNA_ACC_GAIN）を用いて、RF信号強度（シグナルレベル）を計算する。
    pub fn get_rf_signal_strength(&self) -> Result<i32, TunerError> {
        // 内部で get_rf_gain を呼ぶ
        let gain_idx = self.get_rf_gain()? as usize;

        let priv_data = self.priv_.lock().unwrap();
        let mut strength: i32;

        if priv_data.chip == RT710ChipType::RT710 {
            strength = if priv_data.freq < 1200000 {
                190
            } else if priv_data.freq < 1800000 {
                170
            } else {
                140
            };

            // テーブル参照（境界チェック付き）
            if gain_idx < RT710_LNA_ACC_GAIN.len() {
                strength += RT710_LNA_ACC_GAIN[gain_idx];
            }
        } else {
            // RT720用
            strength = 70;
            if gain_idx < RT720_LNA_ACC_GAIN.len() {
                strength += RT720_LNA_ACC_GAIN[gain_idx];
            }
        }

        // 最終的な計算値を返す (Cコード: tmp * -100)
        Ok(strength * -100)
    }
}

impl<'a, B: BusOps> Tuner for RT710<'a, B> {
    // チューナー初期化
    fn init(&mut self) -> Result<(), TunerError> {
        let mut chip_type = RT710ChipType::UNKNOWN;
        let mut tmp = [0u8; 1];
        {
            //let _lock = self.priv_.lock.lock().unwrap();
            let mut priv_data = self.priv_.lock().unwrap();

            priv_data.init = false;
            priv_data.freq = 0;

            self.read_regs(0x03, &mut tmp)?;

            priv_data.chip = if (tmp[0] & 0xf0) == 0x70 {
                RT710ChipType::RT710
            } else {
                RT710ChipType::RT720
            };

            priv_data.init = true;

            // debug用
            chip_type = priv_data.chip
        }

        // いらないのでは？
        println!(
            "RT710 init done. chip: {:?}, reg03=0x{:02x}",
            chip_type, tmp[0]
        );
        Ok(())
    }

    // デバイスの利用を開始する
    // px4_device.c の一部の機能を切り出し
    fn open(&self) -> Result<(), TunerError> {
        // 1. 個別ウェイクアップレジスタ (tc_init_s) の書き込み
        self.tc90522.write_multiple_regs(&TC_INIT_S)?;

        // 2. 復調器の TS出力 を無効化
        self.tc90522.enable_ts_pins(false)?;

        // 3. 復調器のスリープを解除
        self.tc90522.sleep(false)?;

        println!("[RT710] Device opened and awakened successfully.");
        Ok(())
    }

    // デバイスの利用を終了する
    // px4_device.c の一部の機能を切り出し
    fn close(&self) -> Result<(), TunerError> {
        let priv_data = self.priv_.lock().unwrap();

        // 逆の順序で終了させる
        // 1. チューナー自身をスリープ
        self.sleep(&*priv_data)?;

        // 2. 復調器の TS出力 を無効化
        self.tc90522.enable_ts_pins(false)?;

        // 3. 復調器をスリープ
        self.tc90522.sleep(true)?;

        println!("[RT710] Device closed and put to sleep.");
        Ok(())
    }

    fn init_0(&self) -> Result<(), TunerError> {
        // px4_device.c のコードの一部を切り出して、RT710の役割として貼り付け
        // 481行目の処理で、Tuner の オープン1個目のときに走らせる。
        println!("[RT710] Performing global demodulator initialization (S0)...");
        self.tc90522.write_multiple_regs(&TC_INIT_S0)?;
        Ok(())
    }

    fn tune(&mut self, freq: u32) -> Result<(), TunerError> {
        // px4_device.c のコードの一部を切り出して、RT710の役割として貼り付け
        // px4_chrdev_tune_s() の移植
        // 1. AGC設定（false）
        self.tc90522.set_agc(false)?;
        self.tc90522.write_regs(0x8e, &[0x06])?;
        self.tc90522.write_regs(0xa3, &[0xf7])?;

        // 2. 周波数設定 (RT710はシンボルレート等のパラメータが必要)
        self.set_params(freq, 28860, 4)?;

        // 3. PLLロック待ち
        let mut locked = false;
        for _ in 0..50 {
            if self.is_pll_locked()? {
                locked = true;
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        if !locked {
            return Err(TunerError::InvalidState); // EAGAIN相当
        }

        // 信号強度の取得（デバッグ用）
        if let Ok(ss) = self.get_rf_signal_strength() {
            println!(
                "[RT710] Locked. Strength: {}.{:03} dBm",
                ss / 1000,
                -ss % 1000
            );
        }

        // 4. AGC設定（true）
        self.tc90522.set_agc(true)?;

        Ok(())
    }

    fn is_locked(&self) -> Result<bool, TunerError> {
        // px4_device.c のコードの一部を切り出して、RT710の役割として貼り付け
        // px4_chrdev_check_lock_s() の移植
        // 地デジ側と全く同じコードで、内部の tc90522 が自動的に ISDB-S 用のレジスタ(0xc3)を見てくれます
        let locked = self.tc90522.is_signal_locked()?;
        Ok(locked)
    }

    fn enable_ts_pins(&mut self, enable: bool) -> Result<(), TunerError> {
        // 内部の tc90522 に対して enable_ts_pins を呼ぶ
        self.tc90522.enable_ts_pins(enable)?;
        Ok(())
    }

    fn read_cnr_raw(&self) -> Result<u32, TunerError> {
        self.tc90522.get_cn().map_err(|e| TunerError::CtrlMsg(e))
    }

    fn term(&mut self) -> Result<(), TunerError> {
        {
            let mut priv_data = self.priv_.lock().unwrap();

            if !priv_data.init {
                return Ok(()); // 既に終了済みなら何もしない
            }

            println!("[rt710] terminating tuner...");

            // 1. 自身のハードウェアをスリープ
            let _ = self.sleep(&*priv_data);

            // 2. 状態をクリア
            priv_data.init = false;
            priv_data.freq = 0;
        }

        // 3. 内包する復調器 (TC90522) の終了処理を連鎖させる
        self.tc90522.term()?;

        Ok(())
    }
}

impl<'a, B: BusOps> SatelliteTuner for RT710<'a, B> {
    fn set_stream_id(&mut self, stream_id: u16) -> Result<(), TunerError> {
        let tsid = if stream_id < 12 {
            // TMCC から TSID を取得するループ (100回 * 10ms = 1秒)
            let mut found_tsid = 0;
            for _ in 0..100 {
                if let Ok(id) = self.tc90522.tmcc_get_tsid(stream_id as u8) {
                    if id != 0 {
                        found_tsid = id;
                        break;
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            if found_tsid == 0 {
                return Err(TunerError::InvalidState);
            } // EAGAIN
            found_tsid
        } else {
            stream_id
        };

        // 設定の反映
        self.tc90522.set_tsid(tsid)?;

        // 設定確認のループ
        for i in 0..100 {
            if let Ok(current_tsid) = self.tc90522.get_tsid() {
                if i % 10 == 0 {
                    println!(
                        "[debug] TSID retry {}: expected=0x{:04X}, got=0x{:04X}",
                        i, tsid, current_tsid
                    );
                }

                if current_tsid == tsid {
                    return Ok(());
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        Err(TunerError::InvalidState) // 設定失敗
    }
}

impl<'a, B: BusOps> Drop for RT710<'a, B> {
    // インスタンス破棄時に、初期化フラグをリセット
    // (保険としての終了処理で、エラーは無視)
    fn drop(&mut self) {
        // ロックが取れるなら取ってフラグを下ろす（パニック時はロックが取れない可能性があるので注意）
        if let Ok(mut priv_data) = self.priv_.lock() {
            if priv_data.init {
                priv_data.init = false;
                priv_data.freq = 0;
                let _ = self.sleep(&*priv_data);
                println!("RT710 dropped and terminated.");

                // メモ: tc90522 は構造体のメンバなので、この後自動的に tc90522 の drop() が呼ばれる。
            }
        }
    }
}
