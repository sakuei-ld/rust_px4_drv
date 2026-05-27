//use std::sync::mpsc::{channel, Receiver, Sender};
use crossbeam_channel::{unbounded, Receiver, Sender};
use std::sync::Mutex;
use std::time::Duration;

use crate::drivers::itedtv_bus::BusOps;
use crate::drivers::r850::R850;
use crate::drivers::rt710::RT710;
use crate::drivers::tc90522::{System, TunerError};

use crate::drivers::it930x::{CtrlMsgError, GpioMode, IT930x};

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

/// remain_len などの複雑なフラグ制御を完全に排除し、Rust 向けに最適化したプロセス関数
fn px4_device_stream_process(chrdevs: &mut [Px4Chrdev], buf: &mut [u8]) -> usize {
    let mut offset = 0;

    // 4パケット分の同期チェックを行うため、最低でも 752 バイト残っている間ループする
    while offset + PX4_DEVICE_TS_SYNC_SIZE <= buf.len() {
        let mut is_synced = true;

        // 4パケット連続で同期パターン (xxxx 0111) が一致するかチェック
        for i in 0..PX4_DEVICE_TS_SYNC_COUNT {
            if (buf[offset + i * 188] & 0x8f) != 0x07 {
                is_synced = false;
                break;
            }
        }

        // 同期が確認できなかった場合、1バイト進めてリトライ (Cコードの p++ 相当)
        if !is_synced {
            offset += 1;
            continue;
        }

        // 同期が取れたあと、ここから188バイト単位で処理可能な限りパケットを切り出す
        while offset + 188 <= buf.len() && (buf[offset] & 0x8f) == 0x07 {
            // パケットヘッダからポートID (1〜4) を抽出
            let id = (buf[offset] & 0x70) >> 4;

            if id > 0 && id < 5 {
                // 同期バイトを通常の MPEG-2 TS 規格である 0x47 に書き換える
                buf[offset] = 0x47;

                let packet = &buf[offset..offset + 188];

                // 該当するポートIDを持つ Px4Chrdev を探してデータを配信
                if let Some(chrdev) = chrdevs.iter().find(|c| c.port_number == id) {
                    // 既存のメソッドを呼び出すだけで安全に送信されます
                    chrdev.put_stream(packet);
                }
            }

            offset += 188;
        }
    }

    // 消費した（次回への持ち越しが不要な）バイト数を親に返す
    offset
}

struct Px4StreamContext<'a> {
    pub chrdevs: Vec<Px4Chrdev<'a>>,
    // 柔軟に伸縮する動的バッファ
    pub buffer: Vec<u8>,
}

impl<'a> Px4StreamContext<'a> {
    pub fn new(chrdevs: Vec<Px4Chrdev<'a>>) -> Self {
        Self {
            chrdevs,
            // 予めある程度の容量を確保しておくことで、再アロケーションのオーバーヘッドをゼロにできます
            buffer: Vec::with_capacity(1024 * 64),
        }
    }

    /// ユーザー空間向けの非常にシンプルなストリームハンドラ
    pub fn handle_stream(&mut self, new_data: &[u8]) {
        // 1. 新しく届いたデータをバッファの後方にそのまま結合
        self.buffer.extend_from_slice(new_data);

        // 2. 結合されたバッファ全体に対してスキャンを実行
        let consumed = px4_device_stream_process(&mut self.chrdevs, &mut self.buffer);

        // 3. 処理が完了した分のデータをバッファの先頭から削除し、残差を前方に詰める
        self.buffer.drain(0..consumed);
    }
}

// チューナーデバイスの必要なパラメータ
struct ChrdevConfig {
    system: System,
    addr: u8,
    is_secondary: bool,
}
// System, TC90522 bus の順
// これは、W3U4 の場合だけ
// S1UR とか Q3U4 のときは知らないが、Q3U4 は多分、これで良い。(これの外側で2つ持つイメージだと思う)
const PX4_CHRDEV_CONFIGS: [ChrdevConfig; 4] = [
    ChrdevConfig {
        system: System::ISDB_S,
        addr: 0x11,
        is_secondary: false,
    },
    ChrdevConfig {
        system: System::ISDB_S,
        addr: 0x13,
        is_secondary: true,
    },
    ChrdevConfig {
        system: System::ISDB_T,
        addr: 0x10,
        is_secondary: false,
    },
    ChrdevConfig {
        system: System::ISDB_T,
        addr: 0x12,
        is_secondary: true,
    },
];

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
pub struct Px4Chrdev<'a> {
    pub system: System,

    // ここ3つは要らない気がする。
    // → IT930x内部に記載して良さげ。
    pub port_number: u8,
    pub slave_number: u8,
    pub sync_byte: u8,

    // アプリケーション層へデータを送るためのキュー
    pub tx: Sender<Vec<u8>>,

    //pub tc90522: &'a TC90522<'a, B>,
    pub tuner: Box<dyn Tuner + Send + Sync + 'a>,

    // オープン状態かのフラグ
    pub is_opened: bool,

    // LNB給電中かどうかのフラグ
    pub lnb_power: bool,
}

impl<'a> Px4Chrdev<'a> {
    /// 188バイトのTSパケットを配信する
    pub fn put_stream(&self, packet: &[u8]) {
        // チャンネルが生きている場合のみ送信（エラーは無視、またはログ出力）
        let _ = self.tx.send(packet.to_vec());
    }

    pub fn open(&mut self) -> Result<(), TunerError> {
        println!("[px4] Opening port {}...", self.port_number);

        // 2. トレイトオブジェクト経由でチューナーを開く
        self.tuner.open()?;

        // 3. USBブリッジ（IT930x）側のストリーミングを開始（必要に応じて実装）
        // self.it930x.start_stream(self.port_number)?;

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
    px4chrdev: Vec<Px4Chrdev<'a>>,

    open_count: usize,
    lnb_power_count: usize,

    streaming_count: usize,
}

impl<'a, B: BusOps + Sync> Px4Device<'a, B> {
    pub fn new(it930x: &'a IT930x<B>) -> Result<(Self, Vec<Receiver<Vec<u8>>>), TunerError> {
        // px4_device_init() の処理
        // it930x.raise 以前は、it930x 生成時に走る(ハズ)
        // ブリッジ自体の起動
        it930x.raise()?;
        it930x.load_firmware("it930x-firmware.bin")?;
        it930x.init_warm()?;

        // 電源投入
        it930x.set_gpio_mode(7, GpioMode::Out, true)?;
        it930x.set_gpio_mode(2, GpioMode::Out, true)?;

        it930x.write_gpio(7, true)?;
        it930x.write_gpio(2, false)?;

        it930x.set_gpio_mode(11, GpioMode::Out, true)?;
        it930x.write_gpio(11, false)?;

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
            let mut tuner_box: Box<dyn Tuner + Send + Sync> = match config.system {
                System::ISDB_S => {
                    Box::new(RT710::new(&it930x, 2, config.addr, config.is_secondary)?)
                }
                System::ISDB_T => {
                    Box::new(R850::new(&it930x, 2, config.addr, config.is_secondary)?)
                }
            };

            // ここでチャンネル(送信・受信のペア)を生成
            //let (tx, rx) = channel();
            let (tx, rx) = unbounded::<Vec<u8>>();
            // 受信側 (rx) はリストに保存して最後に init の戻り値として返す
            receivers.push(rx);

            px4chrdev.push(Px4Chrdev {
                system: config.system,
                port_number: i as u8 + 1,
                slave_number: i as u8,
                sync_byte: ((i as u8 + 1) << 4) | 0x07,
                tx,
                tuner: tuner_box,
                is_opened: false,
                lnb_power: false,
            });
        }

        let device = Self {
            it930x,
            px4chrdev,
            open_count: 0,
            lnb_power_count: 0,
            streaming_count: 0,
        };

        Ok((device, receivers))
    }

    // px4_device.c px4_backend_set_power() の移植
    fn backend_set_power(&mut self, state: bool) -> Result<(), CtrlMsgError> {
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

    // BS/CSアンテナ用のLNB給電設定 ... チューナー外なので、Px4Device で管理。
    pub fn set_lnb_voltage(&mut self, target_idx: usize, voltage: i32) -> Result<(), TunerError> {
        // 一応、ISDB-T はスルーするようにしておく
        if self.px4chrdev[target_idx].system == System::ISDB_T {
            return Ok(());
        }

        // 1. バリデーション (0V か 15V 以外は無効)
        if voltage != 0 && voltage != 15 {
            return Err(TunerError::InvalidArgument);
        }

        let is_on = voltage == 15;

        // 2. 既に要求された状態と同じなら何もしない
        if self.px4chrdev[target_idx].lnb_power == is_on {
            return Ok(());
        }

        // Cコードの !voltage && !atomic_read(&px4->available) の条件は、
        // ユーザスペースドライバとしてデバイスオブジェクトが生きていれば不要なため省略します。

        if !is_on {
            // ---- OFF にする処理 ----
            if self.lnb_power_count > 0 {
                self.lnb_power_count -= 1;
            }

            // 誰もLNB電源を必要としなくなった場合、GPIO 11 を LOW に落とす
            if self.lnb_power_count == 0 {
                // Cコードの挙動を再現：OFFにする際のエラーはログ出力に留め、状態の更新を優先する
                if let Err(e) = self.it930x.write_gpio(11, false) {
                    println!("[warn] Failed to turn off LNB GPIO 11: {:?}", e);
                }
            }

            // フラグをOFFに更新
            self.px4chrdev[target_idx].lnb_power = false;
        } else {
            // ---- ON にする処理 ----
            // 最初の1基目がONになるタイミングで、GPIO 11 を HIGH に引き上げる
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
        println!("First tuner open requested. Powering on backend hardware...");
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
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        println!("[Px4Device] Opening channel index: {}", target_idx);
        // すでにオープンされている場合は、何もせず成功を返す
        if self.px4chrdev[target_idx].is_opened {
            return Ok(());
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
            println!("Performing global demodulator initialization (S0/T0)...");
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

        println!("[Px4Device] Closing channel index: {}", target_idx);

        // すでに閉じている場合は何もしない
        if !self.px4chrdev[target_idx].is_opened {
            return Ok(());
        }

        // ISDB_S（BS/CS）の場合は、チューナーを閉じる前にLNB電源を確実に落とす
        if self.px4chrdev[target_idx].system == System::ISDB_S {
            if let Err(e) = self.set_lnb_voltage(target_idx, 0) {
                println!("[error] Failed to stop LNB voltage during close: {:?}", e);
            }
        }

        // 1. まずは対象のチャンネルのハードウェアをスリープさせる
        self.px4chrdev[target_idx].tuner.close()?;
        self.px4chrdev[target_idx].is_opened = false;

        self.open_count -= 1;

        // 2. もし最後のチャンネルが閉じられたなら、エコモード（電源OFF）に移行する
        if self.open_count == 0 {
            println!("All tuners closed. Powering off backend hardware...");

            // 全チップの論理的な終了処理（フラグクリア等）
            for chrdev in self.px4chrdev.iter_mut() {
                chrdev.tuner.term()?;
            }

            // 【最重要】基板全体の電源を落とす
            self.backend_set_power(false)?;
        }

        Ok(())
    }

    /// 指定したチャンネルが現在放送波をロックしているか確認する
    pub fn check_lock(&self, target_idx: usize) -> Result<bool, TunerError> {
        if !self.px4chrdev[target_idx].is_opened {
            return Ok(false);
        }
        // SかTかを意識することなく、ポリモーフィズムで一発取得
        self.px4chrdev[target_idx].tuner.is_locked()
    }

    pub fn start_capture(&mut self, target_idx: usize) -> Result<(), TunerError> {
        // 1. チューナー固有のTS出力ピンを有効化 (成功すれば後で失敗時に無効化する)
        self.px4chrdev[target_idx].tuner.enable_ts_pins(true)?;

        // 2. ストリーミングがまだ始まっていなければ、ブリッジ全体の準備をする
        if self.streaming_count == 0 {
            // purge
            if let Err(e) = self.it930x.purge_psb(Duration::from_millis(2000)) {
                // purge 失敗時はピン出力を戻す
                let _ = self.px4chrdev[target_idx].tuner.enable_ts_pins(false);
                return Err(e.into());
            }

            // 全ポートの「Sender(tx)」と「port_number」のリストだけをクローン
            // クロージャの中に move
            let mut dispatch_targets: Vec<(u8, Sender<Vec<u8>>)> = self
                .px4chrdev
                .iter()
                .map(|c| (c.port_number, c.tx.clone()))
                .collect();

            // 柔軟な残差処理のため、バッファもスレッド側へ
            let stream_buffer = Mutex::new(Vec::with_capacity(1024 * 64));

            // ストリームハンドラの開始
            if let Err(e) = self.it930x.start_streaming(move |data| {
                // Mutex のロックを取得して、内部バッファを可変として取り出す
                let mut stream_buffer = stream_buffer.lock().unwrap();

                // 1. 新しいデータをバッファに結合
                stream_buffer.extend_from_slice(data);

                // 2. 752 byte (4パケット分) 以上ある間、同期チェックと切り出しをループ
                let mut offset = 0;
                while offset + PX4_DEVICE_TS_SYNC_SIZE <= stream_buffer.len() {
                    let mut is_synced = true;
                    for i in 0..PX4_DEVICE_TS_SYNC_COUNT {
                        if (stream_buffer[offset + i * 188] & 0x8f) != 0x07 {
                            is_synced = false;
                            break;
                        }
                    }

                    if !is_synced {
                        offset += 1;
                        continue;
                    }

                    // 同期が取れたら、188 byte 単位でパケットを切り出して Sender に分配
                    while offset + 188 <= stream_buffer.len()
                        && (stream_buffer[offset] & 0x8f) == 0x07
                    {
                        let id = (stream_buffer[offset] & 0x70) >> 4;

                        if id > 0 && id < 5 {
                            // 同期バイトを 0x47 に書き換え
                            stream_buffer[offset] = 0x47;
                            let packet = &stream_buffer[offset..offset + 188];

                            // 対応するポートにデータを送信 (C の dispatch_packet の役割も内包)
                            if let Some((_, tx)) =
                                dispatch_targets.iter().find(|(port, _)| *port == id)
                            {
                                let _ = tx.send(packet.to_vec());
                            }
                        }
                        offset += 188;
                    }
                }

                stream_buffer.drain(0..offset);
            }) {
                let _ = self.px4chrdev[target_idx].tuner.enable_ts_pins(false);
                return Err(e.into());
            }
        }

        self.streaming_count += 1;
        Ok(())
    }

    pub fn stop_capture(&mut self, target_idx: usize) -> Result<(), TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        if self.streaming_count == 0 {
            return Err(TunerError::InvalidState); // EALREADY相当
        }

        self.streaming_count -= 1;

        // 1. 誰もストリーミングしていないなら、ブリッジ全体を停止
        if self.streaming_count == 0 {
            self.it930x.stop_streaming()?;
        }

        // 2. チューナー固有のTS出力ピンを無効化
        self.px4chrdev[target_idx].tuner.enable_ts_pins(false)?;

        Ok(())
    }

    pub fn set_capture(&mut self, target_idx: usize, status: bool) -> Result<(), TunerError> {
        // インデックスの範囲外アクセスを防ぐチェック
        if target_idx >= self.px4chrdev.len() {
            return Err(TunerError::InvalidArgument);
        }

        if status {
            self.start_capture(target_idx)
        } else {
            self.stop_capture(target_idx)
        }
    }

    /*
    fn dispatch_packet(&self, packet: &[u8]) {
        // パケットヘッダからPIDを読み取り、対応するチャンネルへ流す処理
        // C言語の px4_device_stream_handler 相当
        let pid = ((packet[1] as u16 & 0x1f) << 8) | (packet[2] as u16);

        // チャンネルごとのフィルタ設定に基づいて処理を分岐
        for chrdev in &self.px4chrdev {
            // もしこのチャンネルがこのPIDを要求していればバッファへ追記
            // chrdev.write_ts_packet(packet);
        }
    }

    pub fn stream_handler(&self, data: &[u8]) {
        // デバッグ用: 受信データの確認
        println!("[Px4Device] Received {} bytes of data", data.len());

        // C言語のロジックに基づき、TSパケット(通常188バイト)単位で解析・分配します
        // データには複数のパケットが含まれている可能性があるため、188バイトずつ処理
        let packet_size = 188;
        let num_packets = data.len() / packet_size;

        for i in 0..num_packets {
            let offset = i * packet_size;
            let packet = &data[offset..offset + packet_size];

            // 1. 各チャンネル(px4chrdev)がストリーミング中か確認
            // 2. チャンネルが受信している特定のPIDやIDに基づいてパケットを分配
            // 3. 各チャンネルのバッファへ書き込み

            // 同期バイト 0x47 を確認
            if packet[0] == 0x47 {
                self.dispatch_packet(packet);
            }
        }
    }
    */

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
}

impl<'a, B: BusOps + Sync> Drop for Px4Device<'a, B> {
    fn drop(&mut self) {
        println!("[Px4Device] Dropping device, ensuring power is off...");
        // 電源が入ったままなら強制的に切る
        if self.open_count > 0 {
            let _ = self.backend_set_power(false);
        }
        // ここで it930x の終了処理 (Cコードの it930x_term) などを呼べれば呼びます
    }
}
