use im::{ConvertBuffer, RgbImage, RgbaImage};
use std::error::Error;
use std::sync::Mutex;
use std::sync::mpsc::Sender;
use uvc::{Context, Device, DeviceHandle, Frame, FrameFormat, StreamHandle};
use video_stream::base::VideoStream;
use video_stream::standard_camera::StandardCamera;

lazy_static! {
    static ref DEV_CTX: Context<'static> = Context::new().expect("Could not create context");
    static ref DEV: Device<'static> = DEV_CTX
        .find_device(None, None, None)
        .expect("Could not find device");
    static ref DEVH: DeviceHandle<'static> = DEV.open().expect("Could not open device");
}

pub struct UVCStream {}

impl UVCStream {
    pub fn new () -> UVCStream {
        UVCStream {}
    }

    fn frame_to_image (frame: &Frame) -> Result<RgbaImage, Box<dyn Error>> {
        let width = frame.width();
        let height = frame.height();

        let new_frame = frame.to_rgb()?;
        let data = new_frame.to_bytes();
        let image: RgbaImage = RgbImage::from_raw(
            width,
            height,
            data.to_vec(),
        ).ok_or("This shouldn't happen")?.convert();

        Ok(image)
    }

    fn get_stream <'a> () -> StreamHandle<'a, 'a> {
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
}

impl VideoStream for UVCStream {
    type C = StandardCamera;

    fn get_camera (&self) -> StandardCamera {
        let camera_params = get_camera_params_from_json("./data/cameras/uvc.json");
        StandardCamera::new(camera_params)
    }

    fn consume_image_transmitter (&self, sender: Mutex<Sender<RgbaImage>>) {
        let mut streamh = Self::get_stream();

        let _stream = streamh
            .start_stream(|frame, received_sync_tx| {
                let image = Self::frame_to_image(frame).unwrap();
                let int_tx = Mutex::lock(received_sync_tx).unwrap();
                int_tx.send(image).unwrap();
            }, sender)
            .unwrap();
    }
}
