mod it930x;
mod itedtv_bus;
mod px4_device;
mod r850;
mod rt710;
mod tc90522;

use rusb::{Context, UsbContext};

use it930x::IT930x;
use itedtv_bus::UsbBusRusb;
use px4_device::Px4Device;

use std::thread;

use crate::px4_device::Tuner;

fn main() {
    // まず、USB関連の準備
    let context = match Context::new() {
        Ok(c) => c,
        Err(e) => {
            println!("Failed to create USB context: {}", e);
            return;
        }
    };

    // USBデバイスの検索
    const PX4_VID: u16 = 0x0511;
    const PX4_PID: u16 = 0x083f;

    let devices = match context.devices() {
        Ok(d) => d,
        Err(e) => {
            println!("Failed to list USB devices: {}", e);
            return;
        }
    };

    let device = match devices.iter().find(|d| {
        d.device_descriptor()
            .map(|desc| desc.vendor_id() == PX4_VID && desc.product_id() == PX4_PID)
            .unwrap_or(false)
    }) {
        Some(d) => d,
        None => {
            println!("PX4 device not found.");
            return;
        }
    };

    // USBデバイスを開く
    let handle = match device.open() {
        Ok(h) => h,
        Err(e) => {
            println!("Failed to open device: {}", e);
            return;
        }
    };

    // 各種、デバイス操作用の準備
    let bus = match UsbBusRusb::new(handle) {
        Ok(b) => b,
        Err(e) => {
            println!("Failed to UsbBusRusb::new().");
            return;
        }
    };

    let it930x = IT930x::new(bus);

    let (mut px4_device, receivers) = match Px4Device::new(&it930x) {
        Ok(v) => v,
        Err(e) => {
            println!("Failed to init Px4Device: {:?}", e);
            return;
        }
    };

    println!("[debug] Px4Device initialized successfully!");

    // 受信スレッドの準備 (各ポートごとにスレッドを立てる)
    let mut handles = Vec::new();
    for (i, rx) in receivers.into_iter().enumerate() {
        handles.push(thread::spawn(move || {
            println!("[receiver] Port {} listening...", i + 1);
            let mut packet_count = 0;

            // 受信ループ
            while let Ok(packet) = rx.recv() {
                packet_count += 1;
                if packet_count % 1000 == 0 {
                    println!(
                        "[receiver] Port {} received {} packets",
                        i + 1,
                        packet_count
                    );
                }

                // ここで b25 等にパケットを渡してデコード処理を行う
                // 実際には、recisdb に投げれば良いので、デコードはここの仕事にしない
            }
        }));
    }

    // 4ch 同時動作確認用

    struct TunerConfig {
        port: usize,
        name: &'static str,
        freq_khz: u32,
    }

    let target_configs = vec![
        TunerConfig {
            port: 0,
            name: "ISDB-S(1)",
            //freq_khz: 1235000,
            freq_khz: 1049480,
        }, // BS15 (NHK BS)
        TunerConfig {
            port: 1,
            name: "ISDB-S(2)",
            //freq_khz: 1049480,
            freq_khz: 1235000,
        }, // BS1 (BS朝日/TBS)
        TunerConfig {
            port: 2,
            name: "ISDB-T(1)",
            freq_khz: 473143 + (27 - 13) * 6000,
        }, // 物理27ch (例: NHK総合)
        TunerConfig {
            port: 3,
            name: "ISDB-T(2)",
            freq_khz: 473143 + (25 - 13) * 6000,
        }, // 物理25ch (例: 日本テレビ)
    ];

    // 1. 全ポートのオープンと選局 (Tune) を行う
    for config in &target_configs {
        println!("[debug] Opening {} on Port {}...", config.name, config.port);
        if let Err(e) = px4_device.open_tuner(config.port) {
            println!("  ➔ Failed to open port {}: {:?}", config.port, e);
            return;
        }

        println!(
            "[debug] Tuning {} to {} kHz...",
            config.name, config.freq_khz
        );

        if let Err(e) = px4_device.tune(config.port, config.freq_khz) {
            println!("  ➔ Failed to tune port {}: {:?}", config.port, e);
            // エラーになっても他のポートのテストを続ける場合は continue に変更してください
            return;
        }

        thread::sleep(std::time::Duration::from_millis(500))
    }

    println!("[debug] All tuners initialized and tuned. Starting capture...");

    // 2. 全ポートのキャプチャを一斉に開始
    for config in &target_configs {
        if let Err(e) = px4_device.set_capture(config.port, true) {
            println!(
                "  ➔ Failed to start capture on port {}: {:?}",
                config.port, e
            );
        } else {
            println!(
                "  ➔ Capture started on Port {} ({})",
                config.port, config.name
            );
        }
    }

    println!("[debug] 4-Channel Capturing... Press Enter to STOP.");
    let mut input = String::new();
    std::io::stdin().read_line(&mut input).unwrap();

    // 3. 終了処理 (全ポートのキャプチャ停止とクローズ)
    println!("\nStopping all channels...");
    for config in &target_configs {
        let _ = px4_device.set_capture(config.port, false);
        let _ = px4_device.close_tuner(config.port);
        println!("  ➔ Closed Port {} ({})", config.port, config.name);
    }

    println!("[debug] Passed!")
}
