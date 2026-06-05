use bytes::{Buf, Bytes, BytesMut};
use crossbeam_channel::{unbounded, Receiver, Sender};
use tracing::{error, info, warn};

use std::sync::Mutex;
use std::time::Duration;

use crate::drivers::it930x::{CtrlMsgError, GpioMode, IT930x, PidFilter};
use crate::drivers::itedtv_bus::BusOps;
use crate::drivers::r850::R850;
use crate::drivers::rt710::RT710;
use crate::drivers::tc90522::{System, TunerError};

const PX4_DEVICE_TS_SYNC_COUNT: usize = 4;
const PX4_DEVICE_TS_SYNC_SIZE: usize = 188 * PX4_DEVICE_TS_SYNC_COUNT;

// Q3U4 で必要そう
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Px4MldevMode {
    All,
    SOnly,
    S0Only,
    S1Only,
}

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

// チューナーデバイスの必要なパラメータ
struct ChrdevConfig {
    system: System,
    addr: u8,
    is_secondary: bool,
    options: u32,
}
// System, TC90522 bus の順
// これは、W3U4 の場合だけ
// S1UR とか Q3U4 のときは知らないが、Q3U4 は多分、これで良い。(これの外側で2つ持つイメージだと思う)
const PX4_CHRDEV_CONFIGS: [ChrdevConfig; 4] = [
    ChrdevConfig {
        system: System::IsdbS,
        addr: 0x11,
        is_secondary: false,
        options: 0, // px4_device.c 1263行目参照
    },
    ChrdevConfig {
        system: System::IsdbS,
        addr: 0x13,
        is_secondary: true,
        options: 0,
    },
    ChrdevConfig {
        system: System::IsdbT,
        addr: 0x10,
        is_secondary: false,
        options: 0x00000080, // px4_device.c 1258行目参照
    },
    ChrdevConfig {
        system: System::IsdbT,
        addr: 0x12,
        is_secondary: true,
        options: 0x00000080,
    },
];

pub enum TunerInstance<'a, B: BusOps> {
    Satellite(RT710<'a, B>),
    Terrestrial(R850<'a, B>),
}

pub trait Tuner {
    // チューナー初期化
    fn init(&mut self) -> Result<(), TunerError>;
    // S0/T0初期化用
    fn init_0(&self) -> Result<(), TunerError>;
    fn open(&self) -> Result<(), TunerError>;
    fn close(&self) -> Result<(), TunerError>;

    // 録画
    fn tune(&mut self, freq: u32) -> Result<(), TunerError>;

    // lock状態の確認
    fn is_locked(&self) -> Result<bool, TunerError>;

    fn enable_ts_pins(&mut self, enable: bool) -> Result<(), TunerError>;

    // CNR(raw) を読み取るメソッド
    fn read_cnr_raw(&self) -> Result<u32, TunerError>;

    // BS用
    // デフォルト実装により、全チューナーで強制させない
    fn set_stream_id(&mut self, _stream_id: u16) -> Result<(), TunerError> {
        // 地上波チューナーなどはここでエラーを返せば良い
        Err(TunerError::InvalidState)
    }

    // 論理的な終了処理
    // 各チューナーチップが「電源が落とされた」ことを認識し、内部状態をリセットするためのメソッド
    fn term(&mut self) -> Result<(), TunerError>;
}

impl<'a, B: BusOps> Tuner for TunerInstance<'a, B> {
    fn init(&mut self) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.init(),
            TunerInstance::Terrestrial(t) => t.init(),
        }
    }

    fn init_0(&self) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.init_0(),
            TunerInstance::Terrestrial(t) => t.init_0(),
        }
    }

    fn open(&self) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.open(),
            TunerInstance::Terrestrial(t) => t.open(),
        }
    }

    fn close(&self) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.close(),
            TunerInstance::Terrestrial(t) => t.close(),
        }
    }

    fn tune(&mut self, freq: u32) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.tune(freq),
            TunerInstance::Terrestrial(t) => t.tune(freq),
        }
    }

    fn is_locked(&self) -> Result<bool, TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.is_locked(),
            TunerInstance::Terrestrial(t) => t.is_locked(),
        }
    }

    fn enable_ts_pins(&mut self, enable: bool) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.enable_ts_pins(enable),
            TunerInstance::Terrestrial(t) => t.enable_ts_pins(enable),
        }
    }

    fn read_cnr_raw(&self) -> Result<u32, TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.read_cnr_raw(),
            TunerInstance::Terrestrial(t) => t.read_cnr_raw(),
        }
    }

    fn set_stream_id(&mut self, stream_id: u16) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.set_stream_id(stream_id),
            TunerInstance::Terrestrial(t) => t.set_stream_id(stream_id),
        }
    }

    fn term(&mut self) -> Result<(), TunerError> {
        match self {
            TunerInstance::Satellite(t) => t.term(),
            TunerInstance::Terrestrial(t) => t.term(),
        }
    }
}

pub struct Px4Chrdev<T: Tuner> {
    // BS/GR
    pub system: System,

    // チューナー idx
    pub port_number: u8,

    // アプリケーション層へデータを送るためのキュー
    pub tx: Sender<Bytes>,

    // チューナー
    pub tuner: T,

    // オープン状態かのフラグ
    pub is_opened: bool,

    // streaming中かのフラグ
    pub is_streaming: bool,

    // LNB給電中かどうかのフラグ
    pub lnb_power: bool,
}

// 各ポートの情報と、一括送信用の一時バッファ（バケツ）を管理する構造体
struct TargetPort {
    port_number: u8,
    tx: Sender<Bytes>,
    // ポートごとの一括送信用バッファ
    buffer: BytesMut,
    // debug
    sent_batches: u64,
    sent_bytes: u64,
}

// debug
static DISPATCH_BYTES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

// ストリーム処理部分
struct Px4StreamContext {
    pub targets: Vec<TargetPort>,
    // バッチ送信用のアロケーション使い回しバッファ
    pub stream_buffer: BytesMut,
    // debug
    dispatch_bytes: u64,
    dispatch_batches: u64,
    debug_last_log: std::time::Instant,
}

impl Px4StreamContext {
    pub fn new<T: Tuner>(chrdevs: &[Px4Chrdev<T>]) -> Self {
        let targets = chrdevs
            .iter()
            .map(|c| TargetPort {
                port_number: c.port_number,
                tx: c.tx.clone(),
                buffer: BytesMut::with_capacity(32 * 1024),
                sent_batches: 0, // debug
                sent_bytes: 0,
            })
            .collect();

        Self {
            targets,
            // 64KB分を事前に確保。以降、キャパシティを超えない限りアロケーションは発生しない
            stream_buffer: BytesMut::with_capacity(64 * 1024),
            dispatch_bytes: 0,
            dispatch_batches: 0,
            debug_last_log: std::time::Instant::now(),
        }
    }

    /// USBから読んだバルクデータ(bulk_buf)を処理する
    pub fn process_stream(&mut self, data: &[u8]) {
        // 各ポートの一時バッファをクリア（確保済みのメモリ枠は維持）
        for t in &mut self.targets {
            t.buffer.clear();
        }

        // 新しいデータをバッファに結合
        self.stream_buffer.extend_from_slice(data);

        // 752 byte (4パケット分) 以上ある間、同期チェックと切り出しをループ
        let mut offset = 0;
        while offset + PX4_DEVICE_TS_SYNC_SIZE <= self.stream_buffer.len() {
            let mut is_synced = true;
            for i in 0..PX4_DEVICE_TS_SYNC_COUNT {
                if (self.stream_buffer[offset + i * 188] & 0x8f) != 0x07 {
                    is_synced = false;
                    break;
                }
            }

            if !is_synced {
                offset += 1;

                // 試行中
                // 同期が崩れた場合の高速スキャン
                while offset + PX4_DEVICE_TS_SYNC_SIZE <= self.stream_buffer.len() {
                    if (self.stream_buffer[offset] & 0x8f) == 0x07 {
                        break;
                    }
                    offset += 1;
                }
                continue;
            }

            // 同期が取れたら、188 byte 単位でパケットを切り出し
            while offset + 188 <= self.stream_buffer.len()
                && (self.stream_buffer[offset] & 0x8f) == 0x07
            {
                let id = (self.stream_buffer[offset] & 0x70) >> 4;

                if id > 0 && id < 5 {
                    // 同期バイトを 0x47 に書き換え
                    self.stream_buffer[offset] = 0x47;
                    let packet = &self.stream_buffer[offset..offset + 188];

                    // 【最適化】クロージャの find をやめ、単純なループで比較 (O(N)の高速化)
                    for t in &mut self.targets {
                        if t.port_number == id {
                            t.buffer.extend_from_slice(packet);
                            break;
                        }
                    }
                }
                offset += 188;
            }
        }

        // データの削除（drainはメモリ枠を維持するため高速）
        //self.stream_buffer.drain(0..offset);
        // 読み取り開始位置(ポインタ)を offset 分進める
        self.stream_buffer.advance(offset);

        // 溜まったパケットをポートごとに一括送信
        for t in &mut self.targets {
            if !t.buffer.is_empty() {
                // split().freeze() によるゼロコピー送信
                // t.buffer の中身を切り出し、データのコピーを一切行わずに送信用のイミュータブルな Bytes に変換
                let batch = t.buffer.split().freeze();
                let batch_len = batch.len();

                // try_send() で、受信側が詰まっていても USB受信スレッドをブロックさせない
                match t.tx.try_send(batch) {
                    Ok(_) => {
                        // debug
                        t.sent_batches += 1;
                        t.sent_bytes += batch_len as u64;
                        self.dispatch_batches += 1;
                        self.dispatch_bytes += batch_len as u64;
                    }
                    Err(crossbeam_channel::TrySendError::Full(_)) => {
                        error!(
                            "[Warning] Port {} channel full. Dropped batch.",
                            t.port_number
                        );
                    }
                    Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                        error!("Port {} receiver disconnected", t.port_number);
                    }
                }

                DISPATCH_BYTES.fetch_add(batch_len as u64, std::sync::atomic::Ordering::Relaxed);
            }
        }

        if self.debug_last_log.elapsed() >= Duration::from_secs(5) {
            info!(
                "dispatch batches={} bytes={}",
                self.dispatch_batches, self.dispatch_bytes
            );
            self.debug_last_log = std::time::Instant::now();
        }
    }
}

impl<T: Tuner> Px4Chrdev<T> {
    // 複数のTSパケット(188 byte x N)がまとまったバッファを一括送信する
    pub fn put_stream_batch(&self, batch: &[u8]) {
        if batch.is_empty() {
            return;
        }

        // try_send() で、受信側が詰まっていても USB受信スレッドをブロックさせずに、古いパケットを破棄
        match self.tx.try_send(Bytes::copy_from_slice(batch)) {
            Ok(_) => {}
            Err(crossbeam_channel::TrySendError::Full(_)) => {
                // ここで警告ログを出すと、バッファサイズ不足かI/O遅延かを可視化できる
                warn!("[Warning] Channel is full, dropping {} bytes", batch.len());
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                // クライアントが切断された場合の処理
            }
        }
    }

    pub fn open(&mut self) -> Result<(), TunerError> {
        info!("Opening port {}...", self.port_number);

        // トレイトオブジェクト経由でチューナーを開く
        self.tuner.open()?;

        Ok(())
    }

    pub fn tune(&mut self, freq: u32) -> Result<(), TunerError> {
        // Tuner トレイトで統一されているため、SかTかを意識せず実行できる
        self.tuner.tune(freq)
    }

    pub fn set_stream_id(&mut self, streamd_id: u16) -> Result<(), TunerError> {
        self.tuner.set_stream_id(streamd_id)
    }
}

pub struct Px4Device<'a, B: BusOps + Sync> {
    it930x: &'a IT930x<B>,
    px4chrdev: Vec<Px4Chrdev<TunerInstance<'a, B>>>,

    open_count: usize,
    lnb_power_count: usize,

    streaming_count: usize,

    // PX-Q3U4 かつ multi device として使う場合に true をセットする。
    use_mldev: bool,
}

impl<'a, B: BusOps + Sync> Px4Device<'a, B> {
    pub fn new(
        it930x: &'a IT930x<B>,
        use_mldev: bool,
        discard_null_packets: bool,
    ) -> Result<(Self, Vec<Receiver<Bytes>>), TunerError> {
        // px4_device_init() の処理
        // itedtv_bus_init() と it930x_init() は事前に走らせる。

        // 一応、オリジナルでは、ここで parse_serial_number() を走らせる。
        // 実装してはいるが、ここでは動かしにくい。
        // 不要なので削除？

        // ブリッジ自体の起動
        it930x.raise()?;

        // オリジナルでは、ここで px4_device_load_config()
        // 中で、EEPROM をチェックしてる、っぽい？
        let mut buf = [0u8; 1];
        it930x.read_regs(0x4979, &mut buf)?;
        if buf[0] == 0 {
            error!("[Px4Device::new] EEPROM error.");
            return Err(TunerError::InvalidState);
        }

        // オリジナルは、ここで chrdev ごとの ops と options の設定
        // および、ringbuf とかの設定

        it930x.load_firmware("it930x-firmware.bin")?;
        it930x.init_warm()?;

        // 電源投入
        it930x.set_gpio_mode(7, GpioMode::Out, true)?;
        it930x.set_gpio_mode(2, GpioMode::Out, true)?;

        if use_mldev {
            // ここで、PX-Q3U4 用の処理
            // px4_mldev_add()
            // px4_mldev_alloc()

            // failったときは、px4_mldev_remove()
        } else {
            it930x.write_gpio(7, true)?;
            it930x.write_gpio(2, false)?;
        }

        it930x.set_gpio_mode(11, GpioMode::Out, true)?;
        it930x.write_gpio(11, false)?;

        // NULLパケット (PID: 0x1fff) の破棄設定
        if discard_null_packets {
            let filter = PidFilter {
                pids: vec![0x1fff],
                block: true,
            };

            // PX4_CHRDEV_CONFIGS の数（通常4つ）だけループしてフィルターを適用
            for i in 0..PX4_CHRDEV_CONFIGS.len() {
                it930x
                    .set_pid_filter(i, Some(&filter))
                    .map_err(|e| TunerError::from(e))?;
            }
        }

        // px4_backend_init() の処理 (tc90522 の init の部分) + Rust 向けに拡張
        // Tuner をラップした chrdev の Vec
        let mut px4chrdev = Vec::new();

        // アプリケーション側に引き渡すレシーバーを格納するVec
        let mut receivers = Vec::new();

        for (i, config) in PX4_CHRDEV_CONFIGS.iter().enumerate() {
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

            // 1. Tuner構造体を作成
            let tuner = match config.system {
                System::IsdbS => TunerInstance::Satellite(RT710::new(
                    &it930x,
                    2,
                    config.addr,
                    config.is_secondary,
                )?),
                System::IsdbT => TunerInstance::Terrestrial(R850::new(
                    &it930x,
                    2,
                    config.addr,
                    config.is_secondary,
                )?),
            };

            // ここでチャンネル(送信・受信のペア)を生成
            //let (tx, rx) = channel();
            let (tx, rx) = unbounded::<Bytes>();
            // 受信側 (rx) はリストに保存して最後に init の戻り値として返す
            receivers.push(rx);

            px4chrdev.push(Px4Chrdev {
                system: config.system,
                port_number: i as u8 + 1,
                tx,
                tuner: tuner,
                is_opened: false,
                is_streaming: false,
                lnb_power: false,
            });
        }

        let device = Self {
            it930x,
            px4chrdev,
            open_count: 0,
            lnb_power_count: 0,
            streaming_count: 0,
            use_mldev: use_mldev,
        };

        Ok((device, receivers))
    }

    // px4_device.c px4_backend_set_power() の移植
    fn backend_set_power(&mut self, state: bool) -> Result<(), CtrlMsgError> {
        info!(
            "backend_set_power: {}",
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

    // BS/CSアンテナ用のLNB給電設定 ... チューナー外なので、Px4Device で管理。
    pub fn set_lnb_voltage(&mut self, target_idx: usize, voltage: i32) -> Result<(), TunerError> {
        // debug
        info!(
            "set_lnb_voltage(): target = {}, voltage = {}",
            target_idx, voltage
        );

        // 一応、ISDB-T はスルーするようにしておく
        if self.px4chrdev[target_idx].system == System::IsdbT {
            return Ok(());
        }

        // バリデーション (0V か 15V 以外は無効)
        if voltage != 0 && voltage != 15 {
            return Err(TunerError::InvalidArgument);
        }

        let is_on = voltage == 15;

        // 既に要求された状態と同じなら何もしない
        if self.px4chrdev[target_idx].lnb_power == is_on {
            return Ok(());
        }

        // Cコードの !voltage && !atomic_read(&px4->available) の条件は、
        // ユーザスペースドライバとしてデバイスオブジェクトが生きていれば不要なため省略

        if !is_on {
            // OFF にする処理
            if self.lnb_power_count > 0 {
                self.lnb_power_count -= 1;
            }

            // 誰もLNB電源を必要としなくなった場合、GPIO 11 を LOW に落とす
            if self.lnb_power_count == 0 {
                // Cコードの挙動を再現：OFFにする際のエラーはログ出力に留め、状態の更新を優先する
                if let Err(e) = self.it930x.write_gpio(11, false) {
                    warn!("[warn] Failed to turn off LNB GPIO 11: {:?}", e);
                }
            }

            // フラグをOFFに更新
            self.px4chrdev[target_idx].lnb_power = false;
        } else {
            // ON にする処理(最初の1基目がONになるタイミングで、GPIO 11 を HIGH に引き上げる)
            if self.lnb_power_count == 0 {
                // ONにする際のエラーは致命的なため、? で即座にエラーを返して状態を更新しない
                self.it930x.write_gpio(11, true)?;
            }

            self.lnb_power_count += 1;
            // フラグをONに更新
            self.px4chrdev[target_idx].lnb_power = true;
        }

        Ok(())
    }

    // ハードウェアの一斉初期化用関数(open_count が 0 のときに、内部から呼ぶ関数)
    // px4_chrdev_open の中で、open_count が 0 のときの処理を抽出
    fn init(&mut self) -> Result<(), TunerError> {
        info!("First tuner open requested. Powering on backend hardware...");
        // 295行目
        self.backend_set_power(true)?;

        // 310行目 (px4_backend_init の中身で、r850 および rt710 の init を呼ぶ)
        for chrdev in self.px4chrdev.iter_mut() {
            // チューナー側のレジスタ初期化（元の C の r850_init など）をここで叩く
            chrdev.tuner.init()?;
        }

        Ok(())
    }

    pub fn open_tuner(&mut self, target_idx: usize) -> Result<(), TunerError> {
        // debug
        info!("open_tuner(): target = {}", target_idx);

        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        // すでにオープンされている場合は、何もせず成功を返す
        if self.px4chrdev[target_idx].is_opened {
            //return Ok(());
            return Err(TunerError::InvalidState);
        }

        // memo: 今の所、mldev を想定していないので無視

        if self.open_count == 0 {
            // 295行目
            self.init()?;

            // 初期化直後は全チップが起きているので、自分以外を即座に眠らせる（前回のclose代用案）
            for (i, chrdev) in self.px4chrdev.iter_mut().enumerate() {
                if i != target_idx {
                    chrdev.tuner.close()?;
                }
            }
        }

        // 目的のターゲットチャンネルをウェイクアップ（起動）させる
        self.px4chrdev[target_idx].tuner.open()?;
        self.px4chrdev[target_idx].is_opened = true;

        if self.open_count == 0 {
            // 一旦、0 と 2 が primary として固定のはずなので、強制で。
            // PX-Q3U4 のことを考えると、何か必要かも？
            info!("Performing global demodulator initialization (S0/T0)...");
            self.px4chrdev[0].tuner.init_0()?;
            self.px4chrdev[2].tuner.init_0()?;
        }

        self.open_count += 1;

        Ok(())
    }

    // px4_chrdev_release に相当
    // 若干、順序が違う気がしないでも無いけど、多分大丈夫
    pub fn close_tuner(&mut self, target_idx: usize) -> Result<(), TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        info!("Closing channel index: {}", target_idx);

        // すでに閉じている場合は何もしない
        if !self.px4chrdev[target_idx].is_opened {
            return Ok(());
        }

        // 閉じる前にストリーミング中なら強制停止
        if self.px4chrdev[target_idx].is_streaming {
            let _ = self.stop_capture(target_idx);
            self.px4chrdev[target_idx].is_streaming = false;
        }

        // IsdbS（BS/CS）の場合は、チューナーを閉じる前にLNB電源を確実に落とす
        if self.px4chrdev[target_idx].system == System::IsdbS {
            if let Err(e) = self.set_lnb_voltage(target_idx, 0) {
                error!("[error] Failed to stop LNB voltage during close: {:?}", e);
            }
        }

        // まずは対象のチャンネルのハードウェアをスリープさせる
        self.px4chrdev[target_idx].tuner.close()?;
        self.px4chrdev[target_idx].is_opened = false;

        self.open_count -= 1;

        // もし最後のチャンネルが閉じられたなら、エコモード（電源OFF）に移行する
        if self.open_count == 0 {
            info!("All tuners closed. Powering off backend hardware...");

            // 全チップの論理的な終了処理（フラグクリア等）
            for chrdev in self.px4chrdev.iter_mut() {
                chrdev.tuner.term()?;
            }

            // 基板全体の電源を落とす
            self.backend_set_power(false)?;
        }

        Ok(())
    }

    /// 指定したチャンネルが現在放送波をロックしているか確認する
    /// Bon Driver では、lock チェックしてないので、要らないかも？
    pub fn check_lock(&self, target_idx: usize) -> Result<bool, TunerError> {
        // debug
        info!("check_lock(): target = {}", target_idx);

        if !self.px4chrdev[target_idx].is_opened {
            return Ok(false);
        }
        // SかTかを意識することなく、ポリモーフィズムで一発取得
        self.px4chrdev[target_idx].tuner.is_locked()
    }

    pub fn start_capture(&mut self, target_idx: usize) -> Result<(), TunerError> {
        // ストリーミングがまだ始まっていなければ、ブリッジ全体の準備をする
        if self.streaming_count == 0 {
            // purge
            if let Err(e) = self.it930x.purge_psb(Duration::from_millis(2000)) {
                error!("start_capture(): failed it930x.purge_psb()");
                // purge 失敗時はピン出力を戻す
                let _ = self.px4chrdev[target_idx].tuner.enable_ts_pins(false);
                return Err(e.into());
            }
        }

        // TS出力ピン有効化
        self.px4chrdev[target_idx].tuner.enable_ts_pins(true)?;

        if self.streaming_count == 0 {
            // Px4StreamContext を生成し、1つの Mutex で包む
            let stream_ctx = Px4StreamContext::new(&self.px4chrdev);
            let ctx_mutex = Mutex::new(stream_ctx);

            // ストリームハンドラの開始
            if let Err(e) = self.it930x.start_streaming(move |data| {
                // USB受信1回につき、ロックの取得はこれ1回だけ
                let mut ctx = ctx_mutex.lock().unwrap();
                ctx.process_stream(data);
            }) {
                let _ = self.px4chrdev[target_idx].tuner.enable_ts_pins(false);
                return Err(e.into());
            }
        }

        info!("port {} start capture.", target_idx);
        self.streaming_count += 1;
        Ok(())
    }

    pub fn stop_capture(&mut self, target_idx: usize) -> Result<(), TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        if self.streaming_count == 0 {
            // EALREADY相当
            return Err(TunerError::InvalidState);
        }

        self.streaming_count -= 1;

        // 誰もストリーミングしていないなら、ブリッジ全体を停止
        if self.streaming_count == 0 {
            self.it930x.stop_streaming()?;
        }

        // チューナー固有のTS出力ピンを無効化
        self.px4chrdev[target_idx].tuner.enable_ts_pins(false)?;

        Ok(())
    }

    pub fn set_capture(&mut self, target_idx: usize, status: bool) -> Result<(), TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        // ストリーミング状態の保護を追加
        // PTX_START_STREAMING相当
        let is_streaming = self.px4chrdev[target_idx].is_streaming;

        if status {
            if is_streaming {
                return Err(TunerError::InvalidState);
            }
            self.start_capture(target_idx)?;
            self.px4chrdev[target_idx].is_streaming = true;
        } else {
            if !is_streaming {
                // すでに停止している場合は無視
                return Ok(());
            }
            self.stop_capture(target_idx)?;
            self.px4chrdev[target_idx].is_streaming = false;
        }

        Ok(())
    }

    // 使わなくていいかも？
    pub fn parse_serial_number(serial_str: &str) -> Result<(u64, u8), CtrlMsgError> {
        if serial_str.len() != 15 {
            return Err(CtrlMsgError::InvalidArgument);
        }
        let full_val = serial_str
            .parse::<u64>()
            .map_err(|_| CtrlMsgError::InvalidArgument)?;

        let serial_number = full_val / 10;
        let dev_id = (full_val % 10) as u8;

        Ok((serial_number, dev_id))
    }

    pub fn tune(&mut self, target_idx: usize, freq: u32) -> Result<(), TunerError> {
        // debug
        info!("tune(): target = {}, freq = {}.", target_idx, freq);

        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        // チャンネルがオープンされているか確認（またはオープンされていればチューニング可能とする設計）
        if !self.px4chrdev[target_idx].is_opened {
            return Err(TunerError::InvalidState);
        }

        // 該当する chrdev の tune メソッドを呼ぶ
        self.px4chrdev[target_idx].tune(freq)
    }

    pub fn set_stream_id(&mut self, target_idx: usize, stream_id: u16) -> Result<(), TunerError> {
        info!(
            "set_stream_id(): target = {}, stream_id ={}.",
            target_idx, stream_id
        );

        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        self.px4chrdev[target_idx].set_stream_id(stream_id)
    }

    pub fn get_cnr(&mut self, target_idx: usize) -> Result<u32, TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        // tuner.read_cnr_raw() を呼び出す
        self.px4chrdev[target_idx].tuner.read_cnr_raw()
    }

    pub fn is_set_stream_id_before_tune(&mut self, target_idx: usize) -> Result<bool, TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        Ok(((PX4_CHRDEV_CONFIGS[target_idx].options & 0x00000010) != 0)
            && (PX4_CHRDEV_CONFIGS[target_idx].system == System::IsdbS))
    }

    pub fn is_wait_after_check_lock(&mut self, target_idx: usize) -> Result<bool, TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        Ok(((PX4_CHRDEV_CONFIGS[target_idx].options & 0x00000080) != 0)
            && (PX4_CHRDEV_CONFIGS[target_idx].system == System::IsdbT))
    }

    pub fn is_set_stream_id_after_tune(&mut self, target_idx: usize) -> Result<bool, TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        Ok(((PX4_CHRDEV_CONFIGS[target_idx].options & 0x00000010) == 0)
            && (PX4_CHRDEV_CONFIGS[target_idx].system == System::IsdbS))
    }

    pub fn is_wait_after_lock(&mut self, target_idx: usize) -> Result<bool, TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        Ok((PX4_CHRDEV_CONFIGS[target_idx].options & 0x00000040) != 0)
    }
}

impl<'a, B: BusOps + Sync> Drop for Px4Device<'a, B> {
    fn drop(&mut self) {
        info!("Dropping device, ensuring power is off...");
        // 電源が入ったままなら強制的に切る
        if self.open_count > 0 {
            let _ = self.backend_set_power(false);
        }

        // debug
        info!(
            "dispatch bytes={}",
            DISPATCH_BYTES.load(std::sync::atomic::Ordering::Relaxed)
        );
    }
}
