// TC90522 の制御用

use std::sync::Mutex;

// 多分、これで大丈夫だと思う。
use crate::drivers::it930x::{CtrlMsgError, I2CCommRequest, I2CRequestType, IT930x};
use crate::drivers::itedtv_bus::BusOps;

// エラー関連 (RT710 および R850 の TunerError は共通のはずなので、共通でアクセスするこちらに保持)
use thiserror::Error;
#[derive(Debug, Error)]
pub enum TunerError {
    #[error("control message error: {0}")]
    CtrlMsg(#[from] CtrlMsgError), // CtrlMsgError をラップ
    #[error("R850 chip not detected.")]
    ChipNotDetected,
    #[error("Invalid state.")]
    InvalidState,
    #[error("Invalid argument.")]
    InvalidArgument,
    #[error("Calibration failed.")]
    CalibrationFailed,
}

// いらないのでは？
#[derive(Clone, Copy, Debug)]
pub struct I2CAddr(pub u8);

#[derive(Clone, Copy, Debug)]
pub struct Reg(pub u8);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum System {
    ISDB_S,
    ISDB_T,
}

pub struct TC90522<'a, B: BusOps> {
    it930x: &'a IT930x<B>,

    // I2C バスアクセス用
    pub bus: u8,

    // TC90522 の I2Cアドレス
    pub i2c_addr: u8,

    // 内部の排他制御用
    lock: Mutex<()>,

    // sleep を drop で呼ぶために、自身がコントロールするチューナータイプを保持
    system: System,

    // cにあるらしいので、一旦保持
    is_secondary: bool,
    // デバイスの状態を保持しておく方が良いかも？
}

impl<'a, B: BusOps> TC90522<'a, B> {
    // インスタンス生成(各チューナーごとの個別設定)
    pub fn new(
        it930x: &'a IT930x<B>,
        bus: u8,
        i2c_addr: u8,
        system: System,
        is_secondary: bool,
    ) -> Self {
        println!("[tc90522.new] bus={} tc90522_addr=0x{:02X}", bus, i2c_addr);

        TC90522 {
            it930x,
            bus,
            i2c_addr,
            lock: Mutex::new(()),
            system,
            is_secondary,
        }
    }

    // 明示的な終了処理 (tc90522_term()相当)
    pub fn term(&mut self) -> Result<(), TunerError> {
        println!("[info] TC90522 terminating: powering down...");

        // デバイスをスリープ状態にする
        self.sleep(true)?;
        Ok(())
    }

    // I2C経由のTC90522レジスタの読み込み (排他制御なし)
    fn read_regs_nolock(&self, reg: u8, buf: &mut [u8]) -> Result<(), CtrlMsgError> {
        // debug
        //println!("[tc90522] read_regs_nolock(): reg={:02X}", reg);

        if buf.is_empty() {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut write_buf = [reg];

        let mut reqs = [
            I2CCommRequest {
                addr: self.i2c_addr,
                data: &mut write_buf,
                req: I2CRequestType::Write,
            },
            I2CCommRequest {
                addr: self.i2c_addr,
                data: buf,
                req: I2CRequestType::Read,
            },
        ];

        self.it930x.i2c_master_request(self.bus, &mut reqs)
    }

    // I2C経由のTC90522レジスタの読み込み
    pub fn read_regs(&self, reg: u8, buf: &mut [u8]) -> Result<(), CtrlMsgError> {
        //println!("[tc90522] call read_regs");
        let _lock = self.lock.lock().unwrap();
        self.read_regs_nolock(reg, buf)
    }

    // I2C経由のTC90522レジスタの多重読み込み
    pub fn read_multiple_regs(&self, regs: &mut [(u8, &mut [u8])]) -> Result<(), CtrlMsgError> {
        //println!("[tc90522] call read_multiple_regs");
        let _lock = self.lock.lock().unwrap();

        for (reg, data) in regs.iter_mut() {
            self.read_regs_nolock(*reg, data)?;
        }

        Ok(())
    }

    // I2C経由のTC90522レジスタの書き込み (排他制御なし)
    fn write_regs_nolock(&self, reg: u8, buf: &[u8]) -> Result<(), CtrlMsgError> {
        // debug
        /*
            println!(
                "[tc90522] write_regs_nolock(): reg = {:02X}, buf = {}",
                reg,
                buf.iter()
                    .map(|b| format!("{:02X}", b))
                    .collect::<Vec<_>>()
                    .join(" ")
            );
        */

        if buf.is_empty() || (buf.len() > 254) {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut wbuf = Vec::with_capacity(1 + buf.len());
        wbuf.push(reg);
        wbuf.extend_from_slice(buf);

        let mut req = [I2CCommRequest {
            addr: self.i2c_addr,
            data: &mut wbuf,
            req: I2CRequestType::Write,
        }];

        self.it930x.i2c_master_request(self.bus, &mut req)
    }

    // I2C経由のTC90522レジスタの書き込み
    pub fn write_regs(&self, reg: u8, buf: &[u8]) -> Result<(), CtrlMsgError> {
        //println!("[tc90522] call write_regs");
        let _lock = self.lock.lock().unwrap();
        self.write_regs_nolock(reg, buf)
    }

    // I2C経由のTC90522レジスタの多重書き込み
    pub fn write_multiple_regs(&self, regs: &[(u8, &[u8])]) -> Result<(), CtrlMsgError> {
        //println!("[tc90522] call write_multiple_regs");
        let _lock = self.lock.lock().unwrap();
        for &(reg, data) in regs {
            self.write_regs_nolock(reg, data)?;
        }

        Ok(())
    }

    // チューナーチップへのI2Cブリッジ通信
    // TC90522特有のパケット整形を行う。
    pub fn i2c_master_request(&self, requests: &mut [I2CCommRequest]) -> Result<(), CtrlMsgError> {
        //println!("[tc90522] call i2c_master_request");
        let _lock = self.lock.lock().unwrap();

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
        for req in requests.iter_mut() {
            /*
            println!(
                "[tc90522.req] target_addr=0x{:02X} {:?} len={} first={:02X?}",
                req.addr,
                req.req,
                req.data.len(),
                &req.data.get(..req.data.len().min(8)).unwrap_or(&[])
            );
            */

            match req.req {
                I2CRequestType::Read => {
                    let mut write_buf = [0xFE, (req.addr << 1) | 0x01];

                    // debug
                    /*
                    println!(
                        "[tc90522->it930x] READ-SET bus={} it930x_addr=0x{:02X} data={:02X?}",
                        self.bus, self.i2c_addr, &write_buf
                    );
                    println!(
                        "[tc90522->it930x] READ-DATA bus={} it930x_addr=0x{:02X} len={}",
                        self.bus,
                        self.i2c_addr,
                        req.data.len()
                    );
                    */

                    let mut master = [
                        I2CCommRequest {
                            addr: self.i2c_addr,
                            data: &mut write_buf,
                            req: I2CRequestType::Write,
                        },
                        I2CCommRequest {
                            addr: self.i2c_addr,
                            data: req.data,
                            req: I2CRequestType::Read,
                        },
                    ];

                    self.it930x.i2c_master_request(self.bus, &mut master)?;
                }

                I2CRequestType::Write => {
                    if req.data.is_empty() || req.data.len() > 253 {
                        return Err(CtrlMsgError::InvalidLength);
                    }

                    let mut buf = Vec::with_capacity(2 + req.data.len());
                    buf.push(0xFE);
                    buf.push(req.addr << 1);
                    buf.extend_from_slice(req.data);

                    // debug
                    /*
                    println!(
                        "[tc90522->it930x] WRITE bus={} it930x_addr=0x{:02X} data={:02X?}",
                        self.bus, self.i2c_addr, &buf
                    );
                    */

                    let mut master = [I2CCommRequest {
                        addr: self.i2c_addr,
                        data: buf.as_mut_slice(),
                        req: I2CRequestType::Write,
                    }];

                    self.it930x.i2c_master_request(self.bus, &mut master)?;
                }
            }
        }
        Ok(())
    }

    // PD操作(各方式(インスタンスが保持するモード)のデジタル回路を低電力状態にする)
    pub fn sleep(&self, sleep: bool) -> Result<(), CtrlMsgError> {
        // debug
        println!("[tc90522] sleep(): sleep = {}", sleep);

        match self.system {
            System::ISDB_S => {
                if sleep {
                    self.write_multiple_regs(&[(0x13, &[0x80]), (0x17, &[0xFF])])
                } else {
                    self.write_multiple_regs(&[(0x13, &[0x00]), (0x17, &[0x00])])
                }
            }
            System::ISDB_T => self.write_regs(0x03, if sleep { &[0xF0] } else { &[0x00] }),
        }
    }

    // AGC設定(自動利得制御(信号増幅)の設定)
    pub fn set_agc(&self, on: bool) -> Result<(), CtrlMsgError> {
        // debug
        println!(
            "[debug] set_agc(): on = {}, is_secondary = {}",
            on, self.is_secondary
        );

        match self.system {
            System::ISDB_S => {
                let mut r10 = if self.is_secondary { 0x30 } else { 0xB0 };
                let mut r0a = 0x00;
                let mut r11 = 0x02;

                if on {
                    r0a = 0xFF;
                    r10 |= 0x02;
                    r11 = 0x00;
                }

                self.write_multiple_regs(&[
                    (0x0A, &[r0a]),
                    (0x10, &[r10]),
                    (0x11, &[r11]),
                    (0x03, &[0x01]),
                ])
            }
            System::ISDB_T => {
                let r23 = if on { 0x4D & !0x01 } else { 0x4D };
                self.write_multiple_regs(&[
                    (0x25, &[0x00]),
                    (0x20, &[0x00]),
                    (0x23, &[r23]),
                    (0x01, &[0x50]),
                ])
            }
        }
    }

    // TMCC(制御信号) から TSID(ストリーム識別子) を取得 (ISDB-S専用)
    pub fn tmcc_get_tsid(&self, idx: u8) -> Result<u16, CtrlMsgError> {
        // debug
        //println!("[tc90522] tmcc_get_tsid(): idx = {}", idx);

        match self.system {
            System::ISDB_S => {
                if idx >= 12 {
                    return Err(CtrlMsgError::InvalidLength);
                }

                let mut buf = [0u8; 2];
                self.read_regs(0xce + (idx * 2), &mut buf)?;

                // 2バイトを結合して 16bit の TSID を作成
                // buf[0] が上位 8bit (MSB)、buf[1] が下位 8bit (LSB)
                let tsid = ((buf[0] as u16) << 8) | (buf[1] as u16);

                Ok(tsid)
            }
            System::ISDB_T => {
                println!("[debug] TC90522 tmcc_get_tsid is not used for ISDB-T.");
                Ok(0)
            }
        }
    }

    /// TSID (Transport Stream ID) を取得 (ISDB-S専用)
    pub fn get_tsid(&self) -> Result<u16, CtrlMsgError> {
        // debug
        //println!("[tc90522] get_tsid() called");

        match self.system {
            System::ISDB_S => {
                let mut b = [0u8; 2];
                // 移植元 0xe6 レジスタから読み出し
                self.read_regs(0xe6, &mut b)?;
                Ok(u16::from_be_bytes(b))
            }
            System::ISDB_T => {
                // 地上波では使用されないため 0 を返す
                println!("[debug] TC90522 get_tsid is not used for ISDB-T.");
                Ok(0)
            }
        }
    }

    /// TSID (Transport Stream ID) を設定 (ISDB-S専用)
    pub fn set_tsid(&self, tsid: u16) -> Result<(), CtrlMsgError> {
        //println!("[tc90522] set_tsid(): tsid = {}", tsid);
        match self.system {
            System::ISDB_S => {
                // 移植元 0x8f レジスタへ書き込み
                self.write_regs(0x8f, &tsid.to_be_bytes())
            }
            System::ISDB_T => {
                // 地上波では何もしない
                println!("[debug] TC90522 set_tsid is not used for ISDB-T.");
                Ok(())
            }
        }
    }

    /// C/N比（信号品質）に関連する生データを取得
    pub fn get_cn(&self) -> Result<u32, CtrlMsgError> {
        //println!("[tc90522] call get_cn");
        match self.system {
            System::ISDB_S => {
                // tc90522_get_cn_s 相当 (16bit)
                let mut b = [0u8; 2];
                self.read_regs(0xbc, &mut b)?;
                // tc90522_get_cn_s だと 16bit で返していたが、関数を統一化するため、32bit へ変換
                Ok(u16::from_be_bytes(b) as u32)
            }
            System::ISDB_T => {
                // tc90522_get_cndat_t 相当 (24bit)
                let mut b = [0u8; 3];
                self.read_regs(0x8b, &mut b)?;
                // [b0, b1, b2] -> (b0 << 16) | (b1 << 8) | b2
                // u32にしつつ C のコード通り変換する。
                Ok(((b[0] as u32) << 16) | ((b[1] as u32) << 8) | (b[2] as u32))
            }
        }
    }

    // TSデータ出力ピン(IT930xへのデータ出力物理ピン)の有効/無効化
    pub fn enable_ts_pins(&self, enable: bool) -> Result<(), CtrlMsgError> {
        // debug
        println!("[tc90522] enable_ts_pins(): enable = {}", enable);
        match self.system {
            System::ISDB_S => {
                // tc90522_enable_ts_pins_s 相当
                if enable {
                    self.write_multiple_regs(&[(0x1c, &[0x00]), (0x1f, &[0x00])])
                } else {
                    self.write_multiple_regs(&[(0x1c, &[0x80]), (0x1f, &[0x22])])
                }
            }
            System::ISDB_T => {
                // tc90522_enable_ts_pins_t 相当
                let val = if enable { 0x00 } else { 0xa8 };
                self.write_regs(0x1d, &[val])
            }
        }
    }

    // signal lock のチェック(ロック状態が、電波を掴めて、デジタル復元が出来ている状態、らしい)
    pub fn is_signal_locked(&self) -> Result<bool, CtrlMsgError> {
        // debug
        println!("[tc90522] call is_signal_locked");
        match self.system {
            System::ISDB_S => {
                // tc90522_is_signal_locked_s 相当
                // レジスタ 0xc3 の 0x10 ビットが 0 ならロック
                let mut b = [0u8];
                self.read_regs(0xc3, &mut b)?;

                // ★ここに追加：どんな値が読み出されているか確認する
                //println!(
                //    "[debug] TC90522::is_signal_locked System::ISDB_S reg 0xc3 = 0x{:02X}",
                //    b[0]
                //);

                Ok((b[0] & 0x10) == 0)
            }
            System::ISDB_T => {
                // tc90522_is_signal_locked_t 相当
                // 0x80 のチェックと 0xb0 のチェックを連続で行う
                let _lock = self.lock.lock().unwrap();

                let mut b80 = [0u8];
                self.read_regs_nolock(0x80, &mut b80)?;
                if (b80[0] & 0x28) != 0 {
                    return Ok(false);
                }

                let mut bb0 = [0u8];
                self.read_regs_nolock(0xb0, &mut bb0)?;
                // 0xb0 の下位4bitが 8 以上ならロックと判定
                Ok((bb0[0] & 0x0f) >= 8)
            }
        }
    }

    // debug用
    /*
    pub fn dump_regs(&self, start: u8, end: u8) -> Result<(), CtrlMsgError> {
        for reg in start..=end {
            let mut buf = [0u8];
            self.read_regs(reg, &mut buf)?;

            println!("[dump] TC90522 reg[0x{:02X}] = 0x{:02X}", reg, buf[0]);
        }

        Ok(())
    }
    */
}

// term() の代わり および Rust として、終了時に適切にデバイスを止めるための実装をここで行う。
impl<'a, B: BusOps> Drop for TC90522<'a, B> {
    fn drop(&mut self) {
        // Cコードの tc90522_term() 相当の処理
        // デバイスをスリープ状態にする
        // 保険としての終了処理とし、エラーは無視。

        println!("[info] TC90522 dropping: powering down...");

        let _ = self.sleep(true);
        // 最後に LNA (Low Noise Amplifier) などの電源を切る処理があればここに追加
    }
}
