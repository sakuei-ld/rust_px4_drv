mod itedtv_bus;
mod it930x;
mod rt710;
mod r850;
mod tc90522;
mod px4_device;

use rusb::{Context, UsbContext};

use itedtv_bus::UsbBusRusb;
use it930x::IT930x;
use px4_device::Px4Device;

fn main()
{
    // まず、USB関連の準備
    let context = match Context::new()
    {
        Ok(c) => c,
        Err(e) =>
        {
            println!("Failed to create USB context: {}", e);
            return;
        }
    };

    // USBデバイスの検索
    const PX4_VID: u16 = 0x0511;
    const PX4_PID: u16 = 0x083f;

    let devices = match context.devices()
    {
        Ok(d) => d,
        Err(e) => 
        {
            println!("Failed to list USB devices: {}", e);
            return;
        }
    };

    let device = match devices.iter().find(|d|
    {
        d.device_descriptor().map(|desc| desc.vendor_id() == PX4_VID && desc.product_id() == PX4_PID).unwrap_or(false)
    })
    {
        Some(d) => d,
        None => 
        {
            println!("PX4 device not found.");
            return;
        }
    };

    // USBデバイスを開く
    let handle = match device.open()
    {
        Ok(h) => h,
        Err(e) =>
        {
            println!("Failed to open device: {}", e);
            return;
        }
    };

    // USBデバイスを占有する
    if let Err(e) = handle.claim_interface(0)
    {
        println!("Failed to claim interface 0: {}", e);
    }

    // 各種、デバイス操作用の準備
    let bus = match UsbBusRusb::new(handle)
    {
        Ok(b) => b,
        Err(e) =>
        {
            println!("Failed to UsbBusRusb::new().");
            return;
        }
    };

    let it930x = IT930x::new(bus);

    // 疎通チェック
    if let Err(e) = it930x.raise()
    {
        println!("Failed to raise.: {}", e);
        return;
    }

    if let Err(e) = it930x.load_firmware("it930x-firmware.bin")
    {
        println!("Failed to load firmware.: {}", e);
        return;
    }

    if let Err(e) = it930x.init_warm()
    {
        println!("Failed to initial warm.: {}", e);
        return;
    }

    it930x.set_gpio_mode(7, it930x::GpioMode::Out, true).expect("gpio7 mode failed");
    it930x.set_gpio_mode(2, it930x::GpioMode::Out, true).expect("gpio2 mode failed");

    it930x.write_gpio(7, true).expect("gpio7 write failed");
    it930x.write_gpio(2, false).expect("gpio2 write failed");

    it930x.set_gpio_mode(11, it930x::GpioMode::Out, true).expect("gpio11 mode failed");
    it930x.write_gpio(11, false).expect("gpio11 write failed.");

    // Px4Device の init() で、R850 や RT710 の read_regs が走るので、いつ init() すべきかは、ちゃんと考える必要がある。
    let mut px4dev = Px4Device::new(&it930x);

    println!("[debug] px4_dev.set_power() start => ");
    if let Err(e) = px4dev.set_power(true)
    {
        println!("Failed to TunerError: {}", e);
    }

    println!("[debug] px4_dev.init() start => ");
    if let Err(e) = px4dev.init()
    {
        println!("Failed to TunerError: {}", e);
        return;
    }

    println!("[debug] Passed!")

}
