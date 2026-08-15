//! スマートカードIFD (Interface Device Driver) 層
//!
//! IT930xデバイスを介してスマートカード（B-CASカード等）と通信するための抽象化レイヤー。
//! Cコードの ifdhandler.c (px4_drv) の T=1 プロトコル実装に準拠。

use crate::drivers::it930x::{CtrlMsgError, GpioMode, IT930x};
use crate::drivers::itedtv_bus::BusOps;
use ::tracing::warn;
use std::time::{Duration, Instant};
use thiserror::Error;

// B-CASカード固有コマンド
pub const BCAS_CMD_INITIALIZE: u8 = 0x90; // Initialize (get card info)
pub const BCAS_CMD_INIT_SUB: u8 = 0x30; // Initialize sub
pub const BCAS_CMD_GET_INFO: u8 = 0x90; // Get card info
pub const BCAS_CMD_GET_INFO_SUB: u8 = 0x32; // Get info sub
pub const BCAS_CMD_READ_CH: u8 = 0x90; // Read channel
pub const BCAS_CMD_READ_CH_SUB: u8 = 0x34; // Read channel sub

// B-CAS ATR (Expected)
pub const BCAS_ATR: &[u8] = &[
    0x3B, 0xF0, 0x12, 0x00, 0xFF, 0x91, 0x81, 0xB1, 0x7C, 0x45, 0x1F, 0x03, 0x99,
];

// B-CAS response sizes
pub const BCAS_INIT_RESP_SIZE: usize = 57;
pub const BCAS_INFO_RESP_SIZE: usize = 17;

// Timeout settings
pub const BCAS_CMD_TIMEOUT_MS: u64 = 2000;
pub const BCAS_READ_CH_TIMEOUT_MS: u64 = 1000;

// スマートカード関連定数
const SC_RESET_ON: u8 = 1;
const SC_RESET_OFF: u8 = 0;
const SC_ENABLE_POWER: u8 = 1;
const SC_DISABLE_POWER: u8 = 0;

// T=1プロトコル定数 (ISO/IEC 7816-3準拠)
const T1_NAD_IFD_ICC: u8 = 0x00; // NAD: IFD=0, ICC=0
const T1_PCB_I_BLOCK: u8 = 0x00; // I-block (bit7=0)
const T1_PCB_I_SEQ: u8 = 0x40; // I-block sequence number (bit6)
const T1_PCB_I_CHAIN: u8 = 0x20; // I-block more-data chain flag (bit5)
const T1_PCB_R_BLOCK: u8 = 0x80; // R-block (bits7:6=10)
const T1_PCB_R_SEQ: u8 = 0x10; // R-block next-expected seq (bit4)
const T1_PCB_R_NO_ERROR: u8 = 0x00; // R-block: no error
const T1_PCB_S_RESYNCH_REQ: u8 = 0xC0; // S-block RESYNCH request
const T1_PCB_S_RESYNCH_RSP: u8 = 0xE0; // S-block RESYNCH response
const T1_PCB_S_IFS_REQ: u8 = 0xC1; // S-block IFS request
const T1_PCB_S_IFS_RSP: u8 = 0xE1; // S-block IFS response
const T1_IFS_IFSD: u8 = 254; // IFD max INF size to advertise to card

const DEFAULT_T1_IFSC: u8 = 32;
const MAX_ATR_LENGTH: usize = 33;
const MAX_BLOCK_SIZE: usize = 254; // T=1 max INF size
const T1_RX_TIMEOUT_MS: u64 = 200;
const T1_RX_POLL_MS: u64 = 10;
const T1_GUARD_INTERVAL_US: u64 = 0;
const RX_ZERO_LENGTH_RETRY_MAX: u8 = 3;

const T1_GUARD_INTERVAL_MS: u64 = 10;

/// B-CASカードIDを算出する（bcs-perl.pl の formatCardId と同一アルゴリズム）。
///
/// `BcasCard::bcas_format_card_id()` とユニットテストの両方から呼ばれる、
/// このロジックの唯一の実装（Single Source of Truth）。
pub fn compute_bcas_card_id(uid: [u8; 6], check: u16) -> String {
    let uid_hex: u128 = ((uid[0] as u128) << 40)
        | ((uid[1] as u128) << 32)
        | ((uid[2] as u128) << 24)
        | ((uid[3] as u128) << 16)
        | ((uid[4] as u128) << 8)
        | (uid[5] as u128);

    let id: u128 = uid_hex * 100_000 + check as u128;

    let part0 = format!(
        "{}{:03}",
        uid[0] >> 5,
        (id / 10_000_000_000_000_000) % 10_000
    );
    let part1 = format!("{:04}", (id / 1_000_000_000_000) % 10_000);
    let part2 = format!("{:04}", (id / 100_000_000) % 10_000);
    let part3 = format!("{:04}", (id / 10_000) % 10_000);
    let part4 = format!("{:04}", id % 10_000);

    format!("{} {} {} {} {}", part0, part1, part2, part3, part4)
}

/// スマートカードエラー型
#[derive(Debug, Error)]
pub enum SmartCardError {
    #[error("IT930x control error: {0}")]
    Control(#[from] CtrlMsgError),
    #[error("card not present")]
    CardNotPresent,
    #[error("card reset failed")]
    ResetFailed,
    #[error("communication timeout")]
    Timeout,
    #[error("invalid ATR (Answer To Reset)")]
    InvalidAtr,
    #[error("unexpected ATR: expected {expected:?}, got {actual:?}")]
    UnexpectedAtr { expected: Vec<u8>, actual: Vec<u8> },
    #[error("protocol error: {0}")]
    Protocol(String),
    #[error("buffer too small")]
    BufferTooSmall,
    #[error("NACK received from card")]
    Nack,
}

/// スマートカードのATR情報
#[derive(Debug, Clone)]
pub struct CardInfo {
    /// ATRデータ
    pub atr: Vec<u8>,
    /// TS (Initial character in ATR)
    pub ts: u8,
    /// T0 (Second character in ATR)
    pub t0: u8,
    /// 支持されたプロトコル (T=0 or T=1)
    pub protocol: Protocol,
    /// IFSC (Interface Character)
    pub ifsc: Option<u8>,
    /// EDC種別: true=CRC, false=LRC
    pub edc_crc: bool,
}

/// スマートカードプロトコル
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Protocol {
    T0,
    T1,
    None,
}

/// スマートカードインターフェーストレイト
///
/// スマートカードとの基本的な操作（リセット、データ送受信、検出）を定義します。
pub trait SmartCardInterface {
    /// カードのリセットを実行し、ATR情報を取得する
    fn reset(&mut self) -> Result<CardInfo, SmartCardError>;

    /// カードにデータを送信し、レスポンスを受信する
    /// T=1プロトコルに基づくデータ送受信を行う
    fn transceive(&mut self, data: &[u8]) -> Result<Vec<u8>, SmartCardError>;

    /// カードが挿入されているか検出する
    fn detect(&self) -> Result<bool, SmartCardError>;

    /// カードに電力を供給する（イネーブル/ディセーブル）
    fn set_power(&mut self, enable: bool) -> Result<(), SmartCardError>;

    /// カードのリセット状態を設定する
    fn set_reset(&mut self, reset: bool) -> Result<(), SmartCardError>;
}

/// T=1プロトコル状態
struct T1State {
    /// IFSC (Interface Character) - デフォルト32
    ifsc: u8,
    /// EDC種別: true=CRC, false=LRC
    edc_crc: bool,
    /// IFD側送信I-blockのシーケンス番号 (0 or 1)
    seq: u8,
}

impl Default for T1State {
    fn default() -> Self {
        Self {
            ifsc: DEFAULT_T1_IFSC,
            edc_crc: false, // デフォルトLRC
            seq: 0,
        }
    }
}

/// B-CASカード用スマートカードIFD実装
///
/// IT930xデバイスを介してB-CASカードと通信します。
pub struct BcasCard<'a, B: BusOps> {
    it930x: &'a IT930x<B>,
    card_present: bool,
    current_protocol: Protocol,
    /// T=1プロトコル状態
    t1: T1State,
    /// 直近のATR (Answer To Reset)
    pub atr: Vec<u8>,
    // 前回の通信完了時刻
    last_io_time: Option<Instant>,
}

impl<'a, B: BusOps> BcasCard<'a, B> {
    pub fn new(it930x: &'a IT930x<B>) -> Self {
        Self {
            it930x,
            card_present: false,
            current_protocol: Protocol::None,
            t1: T1State::default(),
            atr: Vec::new(),
            last_io_time: None,
        }
    }

    // ---- T=1 low-level helpers (based on ifdhandler.c) ----

    /// LRC: XOR of all bytes (px4_t1_lrc)
    pub fn t1_lrc(data: &[u8]) -> u8 {
        let mut lrc: u8 = 0;
        for &byte in data {
            lrc ^= byte;
        }
        lrc
    }

    /// CRC-CCITT: polynomial 0x1021, init 0xFFFF (px4_t1_crc)
    pub fn t1_crc(data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    /// Build a T=1 frame: [NAD][PCB][LEN][INF...][EDC...]
    /// Returns total frame length, or 0 if frame_max is too small. (px4_t1_make_frame)
    pub fn make_t1_frame(frame: &mut [u8], pcb: u8, inf: &[u8], use_crc: bool) -> usize {
        let edc_len: usize = if use_crc { 2 } else { 1 };
        let total_needed: usize = 3 + inf.len() + edc_len;

        if frame.len() < total_needed {
            return 0;
        }

        frame[0] = T1_NAD_IFD_ICC;
        frame[1] = pcb;
        frame[2] = inf.len() as u8;
        if !inf.is_empty() {
            frame[3..3 + inf.len()].copy_from_slice(inf);
        }

        let hdr_inf = 3 + inf.len();
        if use_crc {
            let crc = Self::t1_crc(&frame[..hdr_inf]);
            frame[hdr_inf] = ((crc >> 8) & 0xFF) as u8;
            frame[hdr_inf + 1] = (crc & 0xFF) as u8;
        } else {
            frame[hdr_inf] = Self::t1_lrc(&frame[..hdr_inf]);
        }
        total_needed
    }

    /// カードの検出状態を更新する
    fn update_detection(&mut self) -> Result<bool, SmartCardError> {
        let detected = self.it930x.bcas_detect_card()?;
        self.card_present = detected;
        Ok(detected)
    }

    /// カードを検出する（SmartCardInterface traitのためpublic）
    pub fn detect(&self) -> Result<bool, SmartCardError> {
        self.it930x
            .bcas_detect_card()
            .map_err(SmartCardError::Control)
    }

    /// B-CASカードをリセットする（main.rsから呼び出し用のpublicメソッド）
    pub fn bcas_reset_card(&mut self) -> Result<(), SmartCardError> {
        self.it930x
            .bcas_reset_card()
            .map_err(SmartCardError::Control)?;

        // T=1プロトコルに設定
        self.current_protocol = Protocol::T1;
        Ok(())
    }

    /// カードをリセットし、ATRを取得して保存する（SmartCardInterface trait実装）
    /// 親クラスの `reset()` を呼び出してATRを取得し、`self.atr` に保存する。

    /// T=1プロトコルに基づくブロックを送信する
    fn send_t1_block(&mut self, block: &[u8]) -> Result<(), SmartCardError> {
        // フレーム送信の直前で wait_guard_interval() を呼び出し、送信後に update_io_time() を呼ぶ
        self.wait_guard_interval();

        // ブロックデータをそのままUARTで送信
        self.it930x
            .bcas_send_data(block)
            .map_err(SmartCardError::Control)?;

        self.update_io_time();

        Ok(())
    }

    /// T=1プロトコルに基づくブロックを受信する
    fn recv_t1_block(&mut self, buf: &mut [u8]) -> Result<usize, SmartCardError> {
        // レディ状態を待つ（タイムアウト付き）
        let timeout = std::time::Duration::from_millis(100);
        let start = std::time::Instant::now();

        while start.elapsed() < timeout {
            if self.it930x.bcas_check_ready()? {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        // データを受信
        let len = self
            .it930x
            .bcas_get_data(buf)
            .map_err(SmartCardError::Control)?;

        if len == 0 {
            return Err(SmartCardError::Timeout);
        }

        Ok(len)
    }

    /// T=1 I-blockを送信する
    fn send_t1_i_block(&mut self, seq: u8, data: &[u8]) -> Result<(), SmartCardError> {
        let mut block = Vec::with_capacity(3 + data.len());

        // INS byte (T=1 Information Block)
        block.push(0x60 | (seq << 4));

        // L.C (Length)
        if data.len() < 256 {
            block.push(data.len() as u8);
        } else {
            return Err(SmartCardError::Protocol("Data too long".to_string()));
        }

        // Data
        block.extend_from_slice(data);

        // CRCはIT930xハードウェアが処理する可能性あり
        // 必要に応じて実装

        self.send_t1_block(&block)?;
        Ok(())
    }

    /// T=1 R-block（レスポンス要求）を送信する
    fn send_t1_r_block(&mut self, seq: u8) -> Result<(), SmartCardError> {
        let block = [0xA0 | (seq << 4)];
        self.send_t1_block(&block)?;
        Ok(())
    }

    /// T=1 S-block（インターブロック制御）を送信する
    fn send_t1_s_block(&mut self, func: u8, block_num: u8) -> Result<(), SmartCardError> {
        let block = [0xC0 | (func << 4) | block_num];
        self.send_t1_block(&block)?;
        Ok(())
    }

    /// ATR解析 (ISO/IEC 7816-3準拠)
    ///
    /// TS, TA1, TB1, TC1, TD1, ... とインターフェースバイトを辿り、
    /// T=1用のパラメータ（IFSC、EDC種別）を抽出する。
    /*
    fn parse_atr(atr: &[u8]) -> Result<CardInfo, SmartCardError> {
        if atr.is_empty() {
            return Err(SmartCardError::InvalidAtr);
        }

        let ts = atr[0];
        // TS: 標準的には 0x3B (direct) または 0x3F (inverse)
        if ts != 0x3B && ts != 0x3F {
            return Err(SmartCardError::InvalidAtr);
        }

        let t0 = if atr.len() > 1 { atr[1] } else { 0 };

        // T0ビット解析: TA/TB/TC count are per TD chain, not just initial T0
        let _ta_count_init = ((t0 & 0xF0) >> 4) as usize; // Initial TAx count
        let _tb_count_init = ((t0 & 0x0F) >> 0) as usize; // Initial TBx count
        let _protocol_t = (t0 & 0x10) >> 4; // Tバイト (16=T=1, 0=T=0)

        // プロトコル判定（最初のTバイト）
        let mut current_protocol = if t0 & 0x10 != 0 {
            Protocol::T1
        } else if t0 & 0x01 != 0 {
            Protocol::T0
        } else {
            Protocol::None
        };

        // インターフェースバイトを順に辿る (ISO/IEC 7816-3準拠)
        // 位置: TS(0) T0(1) TA1(..) TB1(..) TC1(..) TD1(..) TA2(..) ...
        let mut idx = 2; // T0の次から開始

        // IFSCとEDC種別（T=1用）
        let mut ifsc: Option<u8> = None;
        let mut edc_crc = false; // デフォルトLRC

        // TDバイトがない場合はTA/TB/TCは指定されない
        // 正しい実装: TD0 があれば TA1/TB1/TC1/TD2 ... の順で続く
        if atr.len() > 2 {
            let has_interface = (t0 & 0x0F) != 0 || ((t0 >> 4) & 0x0F) != 0;

            if has_interface && idx < atr.len() {
                // TD0 at idx=2
                let td0 = atr[idx];
                idx += 1;

                // TA1 exists if bit4 of T0 is set
                for _ in 0..((t0 >> 4) & 0x0F) {
                    if idx < atr.len() {
                        idx += 1; // Skip TA bytes
                    }
                }

                // TB1 exists if bit0 of T0 is set
                for _ in 0..(t0 & 0x0F) {
                    if idx < atr.len() {
                        idx += 1; // Skip TB bytes
                    }
                }

                // TC1 exists if bit1 of T0 is set
                for _ in 0..((t0 >> 1) & 0x0F) {
                    if idx < atr.len() {
                        idx += 1; // Skip TC bytes (EDC type per TD chain, not here)
                    }
                }

                // TD1 at idx (if TD0 bit 4 = 1, meaning T=1 is selected)
                if td0 & 0x10 != 0 {
                    // T=1 selected
                    current_protocol = Protocol::T1;

                    // Check for more interface bytes (TD1)
                    if idx < atr.len() && (td0 & 0x0F) != 0 {
                        let td1 = atr[idx];
                        idx += 1;

                        // TA2 exists if bit4 of TD0 is set
                        if (td0 >> 4) & 0x01 != 0 {
                            if idx < atr.len() {
                                idx += 1; // TA2
                            }
                        }

                        // TB2 exists if bit0 of TD0 is set
                        if td0 & 0x01 != 0 {
                            if idx < atr.len() {
                                idx += 1; // TB2
                            }
                        }

                        // TC2 exists if bit1 of TD0 is set
                        if (td0 >> 1) & 0x01 != 0 {
                            if idx < atr.len() {
                                idx += 1; // TC2
                            }
                        }

                        // TD2 - check for T=1 continuation with more interface bytes
                        if (td1 & 0x10) != 0 && idx < atr.len() && (td1 & 0x0F) != 0 {
                            // TA3 exists if bit4 of TD1 is set
                            if (td1 >> 4) & 0x01 != 0 {
                                if idx < atr.len() {
                                    // TA3 contains IFSC in bits 0-3
                                    ifsc = Some(atr[idx] & 0x0F);
                                    idx += 1;
                                }
                            }

                            // TC3 exists if bit1 of TD1 is set — EDC type
                            if (td1 >> 1) & 0x01 != 0 {
                                if idx < atr.len() {
                                    let tc3 = atr[idx];
                                    edc_crc = (tc3 & 0x01) != 0; // bit0: 1=CRC, 0=LRC
                                    idx += 1;
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(CardInfo {
            atr: atr.to_vec(),
            ts,
            t0,
            protocol: current_protocol,
            ifsc,
            edc_crc,
        })
    }
    */

    /// ATR (Answer To Reset) のパース処理
    pub fn parse_atr(atr: &[u8]) -> Result<CardInfo, SmartCardError> {
        if atr.len() < 2 {
            return Err(SmartCardError::Protocol("ATR too short".to_string()));
        }

        let ts = atr[0];
        if ts != 0x3B && ts != 0x3F {
            return Err(SmartCardError::Protocol(format!(
                "Invalid TS byte in ATR: 0x{:02X}",
                ts
            )));
        }

        let t0 = atr[1];
        let mut idx = 2;

        let mut td = t0;
        let mut generation = 1;

        let mut ifsc: Option<u8> = None; // 明示的な TA3 がない場合は None（またはデフォルト32）
        let mut edc_crc = false; // デフォルトは LRC

        while (td & 0xF0) != 0 {
            let presence = (td & 0xF0) >> 4;
            let has_ta = (presence & 0x01) != 0;
            let has_tb = (presence & 0x02) != 0;
            let has_tc = (presence & 0x04) != 0;
            let has_td = (presence & 0x08) != 0;

            if has_ta {
                let ta = *atr.get(idx).ok_or_else(|| {
                    SmartCardError::Protocol(format!("Truncated TA{}", generation))
                })?;
                idx += 1;

                if generation == 3 {
                    ifsc = Some(ta); // TA3 = IFSC
                }
            }

            if has_tb {
                let _tb = *atr.get(idx).ok_or_else(|| {
                    SmartCardError::Protocol(format!("Truncated TB{}", generation))
                })?;
                idx += 1;
            }

            if has_tc {
                let tc = *atr.get(idx).ok_or_else(|| {
                    SmartCardError::Protocol(format!("Truncated TC{}", generation))
                })?;
                idx += 1;

                if generation == 3 {
                    edc_crc = (tc & 0x01) != 0; // TC3 bit0 = EDC (0:LRC, 1:CRC)
                }
            }

            if has_td {
                td = *atr.get(idx).ok_or_else(|| {
                    SmartCardError::Protocol(format!("Truncated TD{}", generation))
                })?;
                idx += 1;
            } else {
                break;
            }

            generation += 1;
        }

        Ok(CardInfo {
            atr: atr.to_vec(),
            ts,
            t0,
            protocol: Protocol::T1, // B-CASは T=1 固定
            ifsc,
            edc_crc,
        })
    }

    /// カードとの通信プロトコルが確立済みかどうか
    pub fn protocol_ready(&self) -> bool {
        !matches!(self.current_protocol, Protocol::None)
    }
}

impl<'a, B: BusOps> SmartCardInterface for BcasCard<'a, B> {
    /// カードのリセットを実行
    fn reset(&mut self) -> Result<CardInfo, SmartCardError> {
        // UART再初期化込みの完全リセットを使う
        self.it930x
            .bcas_reset_card()
            .map_err(SmartCardError::Control)?;

        // ATR受信
        let mut atr_buf = [0u8; MAX_ATR_LENGTH];
        let mut offset = 0;
        let timeout = std::time::Duration::from_secs(2);
        let start = std::time::Instant::now();

        loop {
            if start.elapsed() > timeout {
                return Err(SmartCardError::Timeout);
            }

            match self.it930x.bcas_check_ready() {
                Ok(true) => {}
                _ => {
                    std::thread::sleep(std::time::Duration::from_millis(1));
                    continue;
                }
            }

            let len = match self.it930x.bcas_get_data(&mut atr_buf[offset..]) {
                Ok(n) if n > 0 => n,
                _ => {
                    std::thread::sleep(std::time::Duration::from_millis(1));
                    continue;
                }
            };

            offset += len;

            // ATRが完全に受信されたか判定（簡易実装）
            if offset >= 3 && offset < MAX_ATR_LENGTH {
                // カード依存だが、一定サイズ以上取得できたら完了とみなす
                break;
            }
        }

        if offset == 0 {
            return Err(SmartCardError::ResetFailed);
        }

        let card_info = Self::parse_atr(&atr_buf[..offset])?;

        // 取得した CardInfo の情報で内部状態を更新
        self.t1.ifsc = card_info.ifsc.unwrap_or(32); // TA3指定がなければ規定値 32
        self.t1.edc_crc = card_info.edc_crc;
        self.current_protocol = card_info.protocol.clone();
        self.atr = card_info.atr.clone();

        // ATRを内部状態に保存（monitor_loop や verify_atr で使用）
        self.atr = atr_buf[..offset].to_vec();

        Ok(card_info)
    }

    /// データ送受信（T=1プロトコルベース）
    fn transceive(&mut self, data: &[u8]) -> Result<Vec<u8>, SmartCardError> {
        match self.current_protocol {
            Protocol::T1 => self.transceive_t1(data),
            Protocol::T0 => self.transceive_t0(data),
            Protocol::None => Err(SmartCardError::Protocol("No protocol selected".to_string())),
        }
    }

    /// カード検出
    fn detect(&self) -> Result<bool, SmartCardError> {
        self.it930x
            .bcas_detect_card()
            .map_err(SmartCardError::Control)
    }

    /// 電力供給制御
    fn set_power(&mut self, enable: bool) -> Result<(), SmartCardError> {
        // B-CASカードの電力制御はGPIO経由で行う
        // 実際のハードウェア実装に応じて調整が必要
        if enable {
            self.it930x.bcas_init()?;
        }
        Ok(())
    }

    /// リセット状態制御
    fn set_reset(&mut self, reset: bool) -> Result<(), SmartCardError> {
        // GPIO H14でリセット制御
        self.it930x.set_gpio_mode(14, GpioMode::Out, true)?;
        self.it930x
            .write_gpio(14, reset)
            .map_err(|e| SmartCardError::Control(e))?;
        Ok(())
    }
}

impl<'a, B: BusOps> BcasCard<'a, B> {
    // ---- Generic APDU transparent relay API ----

    /// 任意のAPDUバイト列をカードへ送信し、応答をそのまま返す。
    ///
    /// カード側プロトコル（T=0/T=1）は `current_protocol` に従って自動選択される。
    /// 呼び出し側は APDU の中身を解釈せず、そのまま透過的に中継すること。
    ///
    /// # 制約
    /// - 送信APDUは255バイト以内（`it930x.bcas_send_data` の制約）
    /// - 受信側フレームバッファは256バイト（`frame_buf: [u8; 256]` 由来）
    /// - 呼び出し前に必ず `reset()` または `bcas_reset_card()` を呼び、
    ///   `current_protocol` を `Protocol::T1` または `Protocol::T0` に確定させること。
    ///   プロトコル未確定（`Protocol::None`）の場合はエラーを返す。
    ///
    /// # 将来的な拡張について
    /// TODO: 256バイトを超えるAPDU（Case 4でLe=256等）や、複数チャンクへの分割送信に対応する場合は、
    /// `transceive_t1` のバッファ管理を動的サイズ対応へ変更する必要がある。
    // ---- B-CAS specific commands (BonCasServer style) ----

    /// B-CASカードを初期化する (Initialize command: 90 30 00 00 00)
    ///
    /// これは `transceive_raw()` を使った固定APDU送信の一例である。
    /// Returns the 57-byte response containing card UID and other info
    pub fn bcas_initialize(&mut self) -> Result<Vec<u8>, SmartCardError> {
        // Initialize command: 90 30 00 00 00
        let cmd = [BCAS_CMD_INITIALIZE, BCAS_CMD_INIT_SUB, 0x00, 0x00, 0x00];
        let resp = self.transceive(&cmd)?;

        if resp.len() < BCAS_INIT_RESP_SIZE {
            return Err(SmartCardError::Protocol(format!(
                "Initialize response too small: {} bytes (expected {})",
                resp.len(),
                BCAS_INIT_RESP_SIZE
            )));
        }

        Ok(resp)
    }

    /// B-CASカードの情報を取得する (Get Info command: 90 32 00 00 00)
    ///
    /// これは `transceive_raw()` を使った固定APDU送信の一例である。
    /// Returns the 17-byte response containing card version and manufacturer ID
    pub fn bcas_get_info(&mut self) -> Result<Vec<u8>, SmartCardError> {
        // Get info command: 90 32 00 00 00
        let cmd = [BCAS_CMD_GET_INFO, BCAS_CMD_GET_INFO_SUB, 0x00, 0x00, 0x00];
        let resp = self.transceive(&cmd)?;

        if resp.len() < BCAS_INFO_RESP_SIZE {
            return Err(SmartCardError::Protocol(format!(
                "Info response too small: {} bytes (expected {})",
                resp.len(),
                BCAS_INFO_RESP_SIZE
            )));
        }

        Ok(resp)
    }

    /// B-CASカードのチャンネル情報を取得する (Read Channel command: 90 34 00 00 00)
    ///
    /// これは `transceive_raw()` を使った固定APDU送信の一例である。
    /// Returns the channel data (typically 16 bytes + SW1 SW2)
    pub fn bcas_read_channel(&mut self) -> Result<Vec<u8>, SmartCardError> {
        // Read channel command: 90 34 00 00 00
        let cmd = [BCAS_CMD_READ_CH, BCAS_CMD_READ_CH_SUB, 0x00, 0x00, 0x00];
        let resp = self.transceive(&cmd)?;
        Ok(resp)
    }

    /// 任意のAPDUバイト列をカードへ送信し、応答をそのまま返す（汎用APDU中継）。
    ///
    /// 既存の `transceive()` (SmartCardInterfaceトレイトメソッド) のpublicエイリアスとして追加。
    /// C版の `PX4CARD_READ`/`PX4CARD_WRITE` ioctl が生バイト列を透過的にやり取りするのと同様、
    /// Rust側でもAPDUの中身を一切解釈せずそのまま中継する。
    ///
    /// # 制約
    /// - 送信APDUは255バイト以内（`it930x.bcas_send_data` の制約）
    /// - 受信側フレームバッファは256バイト
    /// - 呼び出し前に `reset()` または `bcas_reset_card()` でプロトコルを確定させること
    pub fn transceive_raw(&mut self, apdu: &[u8]) -> Result<Vec<u8>, SmartCardError> {
        self.transceive(apdu)
    }

    /// B-CASカードのIDをフォーマットして取得する
    ///
    /// BonCasServerと同じ形式: `"XXXXXXXX XXXX XXXX XXXX XXXX"`
    ///
    /// Internal: `bcas_initialize()` → UID取得、`bcas_get_info()` → check値・バージョン・メーカID取得。
    /// 実際のカードID文字列の生成は [`compute_bcas_card_id`](Self::compute_bcas_card_id) に委ねる。
    pub fn bcas_format_card_id(&mut self) -> Result<(String, u8, u8), SmartCardError> {
        let init_resp = self.bcas_initialize()?;
        let info_resp = self.bcas_get_info()?;

        // Extract UID bytes (init_resp[8..14])
        let uid_bytes = [
            init_resp[8],
            init_resp[9],
            init_resp[10],
            init_resp[11],
            init_resp[12],
            init_resp[13],
        ];

        // Check value: GetInfo応答の byte[15] と byte[16] から 16bit値として算出
        // Perl版: $info_res_array->[15] << 8 | $info_res_array->[16]
        let check = if info_resp.len() > 16 {
            ((info_resp[15] as u16) << 8) | (info_resp[16] as u16)
        } else {
            return Err(SmartCardError::Protocol(
                "GetInfo response too short for check value".to_string(),
            ));
        };

        // Delegate card ID calculation to the pure function (SSoT)
        let card_id = compute_bcas_card_id(uid_bytes, check);

        let card_version = info_resp[8];
        let manufacturer_id = info_resp[7];

        Ok((card_id, card_version, manufacturer_id))
    }

    /// T=0プロトコルでAPDUを送信・受信する
    fn transceive_t0(&mut self, data: &[u8]) -> Result<Vec<u8>, SmartCardError> {
        if data.is_empty() {
            return Err(SmartCardError::Protocol("Empty data".to_string()));
        }

        // フレーム送信の直前で wait_guard_interval() を呼び出し、送信後に update_io_time() を呼ぶ
        self.wait_guard_interval();

        // T=0: コマンドを送信
        self.it930x.bcas_send_data(data)?;

        self.update_io_time();

        // レスポンスを受信（簡易実装）
        let mut resp_buf = [0u8; 256];
        let timeout = std::time::Duration::from_secs(1);
        let start = std::time::Instant::now();

        while start.elapsed() < timeout {
            if self.it930x.bcas_check_ready()? {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        let len = self.it930x.bcas_get_data(&mut resp_buf)?;
        if len == 0 {
            return Err(SmartCardError::Timeout);
        }

        Ok(resp_buf[..len].to_vec())
    }

    /// T=1初期化: RESYNCH + IFSネゴシエーション (px4_ifd_t1_init)
    ///
    /// bcas_reset_card()の直後に呼び出し、通常のI-block通信の前に実行する。
    //*
    pub fn t1_init(&mut self) -> Result<(), SmartCardError> {
        const MAX_RETRIES: u8 = 3;

        // Step 1: RESYNCH (S-block with R=1, bit3 set)
        for _ in 0..MAX_RETRIES {
            // Send RESYNCH S-block request from IFD
            let mut frame_buf = [0u8; 256];
            let len =
                Self::make_t1_frame(&mut frame_buf, T1_PCB_S_RESYNCH_REQ, &[], self.t1.edc_crc);
            if len == 0 {
                return Err(SmartCardError::Protocol(
                    "Frame buffer too small".to_string(),
                ));
            }

            // フレーム送信の直前で wait_guard_interval() を呼び出し、送信後に update_io_time() を呼ぶ
            self.wait_guard_interval();

            self.it930x.bcas_send_data(&frame_buf[..len])?;

            self.update_io_time();

            // Wait for RESYNCH response from ICC
            let mut resp = [0u8; 256];
            //if !self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS)? {
            //    continue;
            //}
            let recv_len = self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS)?;
            if recv_len == 0 {
                continue;
            }

            // Parse response
            if resp[0] == T1_NAD_IFD_ICC && (resp[1] & 0xFC) == T1_PCB_S_RESYNCH_RSP {
                // Valid RESYNCH response received
                break;
            }
        }

        // Step 2: IFS negotiation (send S-block IFS request with IFSD=254)
        let mut ifs_inf = [0u8; 1];
        ifs_inf[0] = T1_IFS_IFSD;

        for _ in 0..MAX_RETRIES {
            let mut frame_buf = [0u8; 256];
            let len =
                Self::make_t1_frame(&mut frame_buf, T1_PCB_S_IFS_REQ, &ifs_inf, self.t1.edc_crc);
            if len == 0 {
                return Err(SmartCardError::Protocol(
                    "Frame buffer too small".to_string(),
                ));
            }

            // フレーム送信の直前で wait_guard_interval() を呼び出し、送信後に update_io_time() を呼ぶ
            self.wait_guard_interval();

            self.it930x.bcas_send_data(&frame_buf[..len])?;

            self.update_io_time();

            let mut resp = [0u8; 256];
            //if !self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS)? {
            //    continue;
            //}
            let recv_len = self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS)?;
            if recv_len == 0 {
                continue;
            }

            // Check for S-block IFS response
            if resp[0] == T1_NAD_IFD_ICC && (resp[1] & 0xFC) == T1_PCB_S_IFS_RSP {
                // IFSD from card is in INF (index 3)
                if resp.len() > 3 {
                    // Card accepted our IFS request; we don't necessarily adopt their IFSD,
                    // but we note it. Our max send size is still limited by our buffer.
                    let _card_ifsd = resp[3];
                }
                return Ok(());
            }
        }

        Err(SmartCardError::Protocol(
            "T=1 init failed: RESYNCH/IFS negotiation timeout".to_string(),
        ))
    }
    //*/
    /// T=1プロトコルでAPDUを送信・受信する (px4_ifd_t1_transmit)
    /*
    fn transceive_t1(&mut self, data: &[u8]) -> Result<Vec<u8>, SmartCardError> {
        if data.is_empty() {
            return Err(SmartCardError::Protocol("Empty data".to_string()));
        }

        let mut inf_buf = Vec::new(); // 蓄積したINFデータ
        let mut offset: usize = 0;
        let mut last_recv_has_chain: bool = false;
        let mut zero_len_retry: u8 = 0;

        while offset < data.len() || inf_buf.is_empty() {
            // --- IFD → ICC: Send I-block ---
            let is_first = offset == 0;
            let remaining = data.len() - offset;
            let block_len = std::cmp::min(remaining, MAX_BLOCK_SIZE);
            let chunk = &data[offset..offset + block_len];

            // PCB bits for I-block: bit7=0(I), bit6=seq, bit5=chain(M)
            let mut pcb: u8 = T1_PCB_I_BLOCK | self.t1.seq; // base I-block with seq
            if !is_first || offset >= MAX_BLOCK_SIZE {
                pcb |= T1_PCB_I_CHAIN; // set M bit if chained
            }

            let mut frame_buf = [0u8; 256];
            let frame_len = Self::make_t1_frame(&mut frame_buf, pcb, chunk, self.t1.edc_crc);
            if frame_len == 0 {
                return Err(SmartCardError::Protocol(
                    "Frame buffer too small".to_string(),
                ));
            }

            // Send frame
            self.it930x.bcas_send_data(&frame_buf[..frame_len])?;

            // Advance offset only after successful send
            offset += block_len;

            // If this is a chained block (M=1), we need to receive R-block ACK from ICC
            if pcb & T1_PCB_I_CHAIN != 0 {
                // Wait for R-block ACK
                let mut resp = [0u8; 256];
                //if !self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS)? {
                //    return Err(SmartCardError::Timeout);
                //}
                let recv_len = self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS)?;
                if recv_len == 0 {
                    return Err(SmartCardError::Timeout);
                }

                // Parse R-block: NAD=0x00, PCB=1xxx0xxX (R-block)
                if resp.len() < 2 || resp[0] != T1_NAD_IFD_ICC {
                    return Err(SmartCardError::Protocol("Invalid R-block NAD".to_string()));
                }
                let pcb_resp = resp[1];
                if (pcb_resp & 0xC0) != T1_PCB_R_BLOCK {
                    return Err(SmartCardError::Protocol(
                        "Expected R-block, got different type".to_string(),
                    ));
                }

                // Update our seq to the next expected from card's R-block
                self.t1.seq = (pcb_resp & T1_PCB_R_SEQ) >> 4;
                continue;
            }

            // --- ICC → IFD: Receive I/R/S-block ---
            let mut resp = [0u8; 256];
            let recv_len = match self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS) {
                Ok(0) | Err(SmartCardError::Timeout) => {
                    // Zero-length response retry logic (per ifdhandler.c)
                    zero_len_retry += 1;
                    if zero_len_retry < RX_ZERO_LENGTH_RETRY_MAX {
                        continue;
                    }
                    return Err(SmartCardError::Timeout);
                }
                Ok(n) => n,
                Err(e) => return Err(e),
            };

            zero_len_retry = 0;

            if recv_len < 2 {
                return Err(SmartCardError::Protocol("Frame too short".to_string()));
            }

            // Parse response frame
            if resp[0] != T1_NAD_IFD_ICC {
                return Err(SmartCardError::Protocol(
                    "Invalid NAD in response".to_string(),
                ));
            }

            let pcb_resp = resp[1];
            let block_type = pcb_resp & 0xC0;

            match block_type {
                T1_PCB_I_BLOCK => {
                    // I-block: extract INF
                    if recv_len < 3 {
                        return Err(SmartCardError::Protocol("I-block too short".to_string()));
                    }
                    let inf_len = resp[2] as usize;
                    if recv_len < 3 + inf_len + if self.t1.edc_crc { 2 } else { 1 } {
                        return Err(SmartCardError::Protocol(
                            "I-block length mismatch".to_string(),
                        ));
                    }

                    // Check EDC
                    let edc_start = 3 + inf_len;
                    if self.t1.edc_crc {
                        let expected_crc = Self::t1_crc(&resp[..edc_start]);
                        let actual_crc =
                            ((resp[edc_start] as u16) << 8) | (resp[edc_start + 1] as u16);
                        if expected_crc != actual_crc {
                            return Err(SmartCardError::Protocol("CRC mismatch".to_string()));
                        }
                    } else {
                        let expected_lrc = Self::t1_lrc(&resp[..edc_start]);
                        if resp[edc_start] != expected_lrc {
                            return Err(SmartCardError::Protocol("LRC mismatch".to_string()));
                        }
                    }

                    let inf_data = &resp[3..3 + inf_len];
                    inf_buf.extend_from_slice(inf_data);

                    // Check chain bit in received I-block
                    last_recv_has_chain = (pcb_resp & T1_PCB_I_CHAIN) != 0;

                    if !last_recv_has_chain {
                        // Final block received — done
                        break;
                    }

                    // Card has more data: send R-block ACK
                    let r_pcb = T1_PCB_R_BLOCK | ((self.t1.seq & 0x01) << 4);
                    let mut r_frame = [0u8; 256];
                    let r_len = Self::make_t1_frame(&mut r_frame, r_pcb, &[], self.t1.edc_crc);
                    if r_len == 0 {
                        return Err(SmartCardError::Protocol(
                            "R-frame buffer too small".to_string(),
                        ));
                    }
                    self.it930x.bcas_send_data(&r_frame[..r_len])?;

                    // Flip our seq for next exchange
                    self.t1.seq = 1 - self.t1.seq;
                }
                T1_PCB_R_BLOCK => {
                    // R-block: ACK from card (shouldn't happen when we're not expecting)
                    self.t1.seq = (pcb_resp & T1_PCB_R_SEQ) >> 4;
                }
                _ => {
                    // S-block or unknown — may need handling (NACK, etc.)
                    // For now, treat as error
                    return Err(SmartCardError::Protocol(format!(
                        "Unexpected block type: 0x{:02X}",
                        block_type
                    )));
                }
            }
        }

        Ok(inf_buf)
    }
    */
    fn transceive_t1(&mut self, data: &[u8]) -> Result<Vec<u8>, SmartCardError> {
        if data.is_empty() {
            return Err(SmartCardError::Protocol("Empty data".to_string()));
        }

        let mut inf_buf = Vec::new();
        let mut offset: usize = 0;
        let mut zero_len_retry: u8 = 0;

        // --- 1. 送信フェーズ (IFD -> ICC) ---
        while offset < data.len() {
            let remaining = data.len() - offset;
            // 固定値 MAX_BLOCK_SIZE ではなく self.t1.ifsc を使用する
            let block_len = std::cmp::min(remaining, self.t1.ifsc as usize);
            let chunk = &data[offset..offset + block_len];

            // 送信後にまだ残りのデータがあれば連鎖 (M bit = 1)
            let is_chained = (offset + block_len) < data.len();

            let mut pcb: u8 = T1_PCB_I_BLOCK | (self.t1.seq << 6);
            if is_chained {
                pcb |= T1_PCB_I_CHAIN;
            }

            let mut frame_buf = [0u8; 256];
            let frame_len = Self::make_t1_frame(&mut frame_buf, pcb, chunk, self.t1.edc_crc);
            if frame_len == 0 {
                return Err(SmartCardError::Protocol(
                    "Frame buffer too small".to_string(),
                ));
            }

            // フレーム送信の直前で wait_guard_interval() を呼び出し、送信後に update_io_time() を呼ぶ
            self.wait_guard_interval();

            self.it930x.bcas_send_data(&frame_buf[..frame_len])?;

            self.update_io_time();

            offset += block_len;
            // I-block送信ごとに自身のseqを反転
            self.t1.seq ^= 1;

            // 連鎖送信(M=1)の場合は、カードからの R-block ACK を待つ
            if is_chained {
                let mut resp = [0u8; 256];
                let recv_len = self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS)?;
                if recv_len < 3 || (resp[1] & 0xC0) != T1_PCB_R_BLOCK {
                    return Err(SmartCardError::Protocol("Expected R-block ACK".to_string()));
                }
                // R-blockの要求seqに更新
                self.t1.seq = (resp[1] & T1_PCB_R_SEQ) >> 4;
            }
        }

        // --- 2. 受信フェーズ (ICC -> IFD) ---
        loop {
            let mut resp = [0u8; 256];
            let recv_len = match self.t1_wait_response(&mut resp, T1_RX_TIMEOUT_MS) {
                Ok(0) | Err(SmartCardError::Timeout) => {
                    zero_len_retry += 1;
                    if zero_len_retry < RX_ZERO_LENGTH_RETRY_MAX {
                        continue;
                    }
                    return Err(SmartCardError::Timeout);
                }
                Ok(n) => n,
                Err(e) => return Err(e),
            };

            zero_len_retry = 0;

            if recv_len < 3 || resp[0] != T1_NAD_IFD_ICC {
                return Err(SmartCardError::Protocol(
                    "Invalid response frame".to_string(),
                ));
            }

            let pcb_resp = resp[1];
            if (pcb_resp & 0x80) != 0 {
                return Err(SmartCardError::Protocol(format!(
                    "Expected I-block, got PCB: 0x{:02X}",
                    pcb_resp
                )));
            }

            let inf_len = resp[2] as usize;
            let edc_len = if self.t1.edc_crc { 2 } else { 1 };
            if recv_len < 3 + inf_len + edc_len {
                return Err(SmartCardError::Protocol(
                    "Frame length mismatch".to_string(),
                ));
            }

            // EDC (CRC/LRC) のチェック
            let edc_start = 3 + inf_len;
            if self.t1.edc_crc {
                let expected_crc = Self::t1_crc(&resp[..edc_start]);
                let actual_crc = ((resp[edc_start] as u16) << 8) | (resp[edc_start + 1] as u16);
                if expected_crc != actual_crc {
                    return Err(SmartCardError::Protocol("CRC mismatch".to_string()));
                }
            } else {
                let expected_lrc = Self::t1_lrc(&resp[..edc_start]);
                if resp[edc_start] != expected_lrc {
                    return Err(SmartCardError::Protocol("LRC mismatch".to_string()));
                }
            }

            inf_buf.extend_from_slice(&resp[3..3 + inf_len]);

            // カードからの連鎖フラグ(M bit)を確認
            let has_chain = (pcb_resp & T1_PCB_I_CHAIN) != 0;
            if !has_chain {
                break; // 最終ブロックを受信完了
            }

            // カード側が次のブロックを持っていれば、R-block ACK を返す
            let card_seq = if (pcb_resp & T1_PCB_I_SEQ) != 0 { 1 } else { 0 };
            let next_expected_seq = 1 - card_seq; // カードに要求する次のseq
            let r_pcb = T1_PCB_R_BLOCK | T1_PCB_R_NO_ERROR | (next_expected_seq << 4);

            let mut r_frame = [0u8; 256];
            let r_len = Self::make_t1_frame(&mut r_frame, r_pcb, &[], self.t1.edc_crc);

            // フレーム送信の直前で wait_guard_interval() を呼び出し、送信後に update_io_time() を呼ぶ
            self.wait_guard_interval();

            self.it930x.bcas_send_data(&r_frame[..r_len])?;

            self.update_io_time();
        }

        Ok(inf_buf)
    }

    /// Wait for a response frame from the card (non-blocking poll)
    fn t1_wait_response(&self, buf: &mut [u8], timeout_ms: u64) -> Result<usize, SmartCardError> {
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        let mut total = 0usize;

        loop {
            //if self.it930x.bcas_check_ready().unwrap_or(false) {
            if self.it930x.bcas_check_ready().unwrap_or_else(|e| {
                warn!("check_ready error: {e}");
                false
            }) {
                let n = self.it930x.bcas_get_data(&mut buf[total..])?;
                total += n;

                // NAD+PCB+LENの3バイトが揃って初めてフレーム全長が分かる
                if total >= 3 {
                    let inf_len = buf[2] as usize;
                    let edc_len = if self.t1.edc_crc { 2 } else { 1 };
                    let frame_total = 3 + inf_len + edc_len;

                    if total >= frame_total {
                        // 完全なフレームが揃ったら終了
                        return Ok(frame_total);
                    }
                    // 全部届いていない場合はポーリング継続
                }
            }
            if Instant::now() >= deadline {
                // タイムアウトの場合
                // 0 or 中途半端なバイト数 を返す
                // 0 に落としても良いかも
                return Ok(total);
            }
            std::thread::sleep(Duration::from_millis(T1_RX_POLL_MS));
        }
    }

    /// 取得済みのATRが既知のB-CAS ATRと一致するか検証する。
    /// 不一致の場合、Errを返す（bcs-perl.pl の `initCard()` と同様の挙動）。
    /// このメソッドは内部で保持している `self.atr` を使って検証を行う。
    pub fn verify_atr(&self) -> Result<(), SmartCardError> {
        if self.atr.as_slice() != BCAS_ATR {
            return Err(SmartCardError::UnexpectedAtr {
                expected: BCAS_ATR.to_vec(),
                actual: self.atr.clone(),
            });
        }
        Ok(())
    }

    /// 直近に取得・保存したATRを返す。
    pub fn atr(&self) -> &[u8] {
        &self.atr
    }

    /// カードが挿入されているか確認する（非ブロッキング）
    pub fn check_card_present(&self) -> Result<bool, SmartCardError> {
        self.it930x
            .bcas_detect_card()
            .map_err(SmartCardError::Control)
    }

    /// GPIO H6 の読み込み（カード検出用）
    /// H6 = 1: カード未挿入, H6 = 0: カード挿入
    pub fn read_gpio_h6(&self) -> Result<bool, SmartCardError> {
        self.it930x.read_gpio(6).map_err(SmartCardError::Control)
    }

    /// B-CASカードの初期化（UARTモード設定 + GPIO制御）
    /// BonCasServerスタイル: UARTボーレート設定 → モード切替 → リセット
    pub fn bcas_init(&mut self) -> Result<(), SmartCardError> {
        // UART ボーレート設定 (9600bps)
        self.it930x
            .set_uart_baudrate(crate::drivers::it930x::UartBaudrate::Baudrate9600)
            .map_err(|e| SmartCardError::Control(e))?;

        // UART モード切替 (BCASカード通信モード)
        self.it930x
            .bcas_init()
            .map_err(|e| SmartCardError::Control(e))?;

        // GPIO H14 を出力モードに設定（リセット制御用）
        self.it930x
            .set_gpio_mode(14, crate::drivers::it930x::GpioMode::Out, true)
            .map_err(|e| SmartCardError::Control(e))?;

        // GPIO H6 を入力モードに設定（カード検出用）
        self.it930x
            .set_gpio_mode(6, crate::drivers::it930x::GpioMode::In, true)
            .map_err(|e| SmartCardError::Control(e))?;

        Ok(())
    }

    /// カードを初期化してリセットする
    pub fn initialize(&mut self) -> Result<CardInfo, SmartCardError> {
        // 既存の実装を維持
        self.bcas_init_internal()?;
        let card_info = self.reset()?;
        Ok(card_info)
    }

    /// B-CASカードのUART初期化を内部実行
    fn bcas_init_internal(&mut self) -> Result<(), SmartCardError> {
        self.it930x.bcas_init()?;
        self.it930x.bcas_reset_card()?;
        Ok(())
    }

    /// カードの完全な初期化・再初期化を行う。
    /// ハードウェアリセット → ATR受信・パース → (T=1の場合) RESYNCH+IFSネゴシエーション、
    /// までを一括で行う。通常はこのメソッドだけを呼べばよい。
    pub fn full_reset(&mut self) -> Result<CardInfo, SmartCardError> {
        let card_info = self.reset()?; // HWリセット + ATR受信 + current_protocol確定

        if card_info.protocol == Protocol::T1 {
            // ATRから解析したT=1パラメータをセッション状態へ反映する
            self.t1.ifsc = card_info.ifsc.unwrap_or(DEFAULT_T1_IFSC);
            self.t1.edc_crc = card_info.edc_crc;
            self.t1.seq = 0;

            // ATR受信後、19200bpsへ切り替える
            // (ifdhander.c の IFDHPowerICC と同じ手順)
            self.it930x
                .bcas_set_baudrate(crate::drivers::it930x::UartBaudrate::Baudrate19200)
                .map_err(SmartCardError::Control)?;

            self.t1_init()?; // RESYNCH + IFSネゴシエーション
        }

        Ok(card_info)
    }

    /// 直前の通信から十分なガードタイムが経過しているか確認し、必要ならウェイトを入れる
    fn wait_guard_interval(&mut self) {
        if let Some(last_time) = self.last_io_time {
            let elapsed = last_time.elapsed();
            let guard_duration = Duration::from_millis(T1_GUARD_INTERVAL_MS);

            if elapsed < guard_duration {
                std::thread::sleep(guard_duration - elapsed);
            }
        }
    }

    /// 通信成功時にタイムスタンプを更新する
    fn update_io_time(&mut self) {
        self.last_io_time = Some(Instant::now());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_card_id_known_vector() {
        // UID = 01:23:45:67:89:AB, check = 0x00CD (205)
        // uid_hex = 0x0123456789AB = 1,250,999,896,491
        // id = 1,250,999,896,491 * 100_000 + 205 = 125,099,989,649,100,205
        let result = compute_bcas_card_id([0x01, 0x23, 0x45, 0x67, 0x89, 0xAB], 0x00CD);
        assert_eq!(result, "0012 5099 9896 4910 0205");
    }

    #[test]
    fn test_card_id_all_ff() {
        let result = compute_bcas_card_id([0xFF; 6], 0xFFFF);
        let parts: Vec<&str> = result.split_whitespace().collect();
        assert_eq!(parts.len(), 5, "Should have exactly 5 groups");
        assert_eq!(result, "72814 7497 6710 6556 5535");
    }

    #[test]
    fn test_card_id_uid_high_bits() {
        // 0x80 = 0b1000_0000 → uid[0] >> 5 = 4 (1 ではない)
        let result = compute_bcas_card_id([0x80, 0x00, 0x00, 0x00, 0x00, 0x00], 0x0000);
        assert!(result.starts_with('4'), "got {}", result);
    }

    #[test]
    fn test_card_id_min_uid() {
        let result = compute_bcas_card_id([0x00; 6], 0x0001);
        assert_eq!(result, "0000 0000 0000 0000 0001");
    }
}
