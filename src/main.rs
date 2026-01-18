mod itedtv_bus;
mod it930x;
mod rt710;
mod tc90522;

use rusb::{Context, DeviceHandle, UsbContext};

use itedtv_bus::UsbBusRusb;
use it930x::IT930x;
use rt710::RT710;
use tc90522::TC90522;

fn main()
{
    let context = match rusb::Context::new()
    {
        Ok(c) => c,
        Err(e) =>
        {
            println!("Failed to create USB context: {}", e);
            return;
        }
    };

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

    let handle = match device.open()
    {
        Ok(h) => h,
        Err(e) =>
        {
            println!("Failed to open device: {}", e);
            return;
        }
    };

    if let Err(e) = handle.claim_interface(0)
    {
        println!("Failed to claim interface 0: {}", e);
    }

    let bus = UsbBusRusb::new(handle);
    let it930x = IT930x::new(bus);

    if let Err(e) = it930x.load_firmware("it930x-firmware.bin")
    {
        println!("Failed to load firmware.: {}", e);
        return;
    }

    if let Err(e) = it930x.config_i2c()
    {
        println!("Failed to configure I2C.: {}", e);
        return;
    }

    // px4_device.c 1128 行目に chrdev4->tc90522.i2c = &it930x->i2c_master[1]; とあり
    // it930x.c の 571 行目で、priv->i2c[i].bus = i + 1; で、
    // it930x.c の 575 行目で、it930x->i2c_master[i].priv = &priv->i2c[i] とあるので、
    // bus 番号は 2 で固定。
    // -> px4 device の場合の話っぽい。
    //  -> pxmlt device の場合は、&it930x->i2c_master[input->i2c_bus - 1]; みたいになってる。
    //  -> s1ur や m1ur は [2] なので bus 番号は 3 らしい。
    // あと、CHRDEV ごとにアドレスが違くて、0x10〜0x13。
    let tc90522 = TC90522::new(&it930x, 2, 0x1f);

    // addr は、要確認。
    // 4つあるうちの2つを選ぶ、感じのはず。
    let mut rt710 = RT710::new(&tc90522, 0x7a);
    if let Err(e) = rt710.init()
    {
        println!("Failed to initialize RT710.: {}", e);
        return;
    }

    println!("RT710 Initialization successful.");

}
