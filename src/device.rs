use constants::{
    CAM_FPS,
    WIN_HEIGHT,
    WIN_WIDTH
};
use uvc::{Context, Device, DeviceHandle, FrameFormat, StreamHandle};

lazy_static! {
    static ref DEV_CTX: Context<'static> = Context::new().expect("Could not create context");
    static ref DEV: Device<'static> = DEV_CTX
        .find_device(None, None, None)
        .expect("Could not find device");
    static ref DEVH: DeviceHandle<'static> = DEV.open().expect("Could not open device");
}

pub fn get_stream <'a> () -> StreamHandle<'a, 'a> {
    let format = DEVH
        .get_preferred_format(|x, y| {
            if
                x.width == WIN_WIDTH &&
                x.height == WIN_HEIGHT &&
                x.fps == CAM_FPS &&
                x.format == FrameFormat::Uncompressed
            {
                x
            } else {
                y
            }
        }).unwrap();

    let streamh = DEVH
        .get_stream_handle_with_format(format)
        .unwrap();

    let description = DEV.description().unwrap();

    println!(
        "Found device: Bus {:03} Device {:03} : ID {:04x}:{:04x} {} ({})",
        DEV.bus_number(),
        DEV.device_address(),
        description.vendor_id,
        description.product_id,
        description.product.unwrap_or_else(|| "Unknown".to_owned()),
        description
            .manufacturer
            .unwrap_or_else(|| "Unknown".to_owned())
    );

    println!("Best format found: {:?}", format);

    println!(
        "Scanning mode: {:?}\nAuto-exposure mode: {:?}\nAuto-exposure priority: {:?}\nAbsolute exposure: {:?}\nRelative exposure: {:?}\nAboslute focus: {:?}\nRelative focus: {:?}",
        DEVH.scanning_mode(),
        DEVH.ae_mode(),
        DEVH.ae_priority(),
        DEVH.exposure_abs(),
        DEVH.exposure_rel(),
        DEVH.focus_abs(),
        DEVH.focus_rel(),
    );

    streamh
}
