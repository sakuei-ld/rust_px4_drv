mod itedtv_bus;
mod it930x;
mod rt710;

use rusb::{Context, DeviceHandle, UsbContext};

use itedtv_bus::UsbBusRusb;
use it930x::IT930x;
use rt710::RT710;

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

    if let Err(e) = it930x.load_firmware("firmware.bin")
    {
        println!("Failed to load firmware.: {}", e);
        return;
    }

    if let Err(e) = it930x.config_i2c()
    {
        println!("Failed to configure I2C.: {}", e);
        return;
    }

    let mut rt710 = RT710::new(&it930x, 2, 0x79);
    if let Err(e) = rt710.init()
    {
        println!("Failed to initialize RT710.: {}", e);
        return;
    }

    println!("RT710 Initialization successful.");

}
