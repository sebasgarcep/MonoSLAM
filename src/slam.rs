extern crate image as im;
#[macro_use]
extern crate ndarray;
extern crate piston_window;
extern crate uvc;
#[macro_use]
extern crate lazy_static;

mod analysis;
mod constants;
mod init;

use analysis::{AnalysisResult, Analyzer, Feature};
use constants::{
    BLOCKSIZE,
    CAM_FPS,
    WIN_HEIGHT,
    WIN_WIDTH
};
use im::{ConvertBuffer, RgbaImage, RgbImage};
use piston_window::{
    image as render_image,
    rectangle as render_rectangle,
    PistonWindow,
    Texture,
    TextureSettings,
    WindowSettings,
};
use std::error::Error;
use std::sync::{Arc, Mutex};
use uvc::{Context, Frame, FrameFormat};

type UserDataType = Arc<Mutex<Option<AnalysisResult>>>;

lazy_static! {
    static ref analyzer: Analyzer = Analyzer::new();
}

// Shi-Tomasi insipired by: https://github.com/onkursen/corner-detection
fn frame_to_raw_image(
    frame: &Frame,
) -> Result<AnalysisResult, Box<dyn Error>> {
    let width = frame.width();
    let height = frame.height();

    let new_frame = frame.to_rgb()?;
    let data = new_frame.to_bytes();
    let image: RgbaImage = RgbImage::from_raw(
        width,
        height,
        data.to_vec(),
    ).ok_or("This shouldn't happen")?.convert();

    let result = analyzer.process(image);

    Ok(result)
}

fn callback_frame_to_image(
    frame: &Frame,
    user_data: &mut UserDataType,
) {
    let image = frame_to_raw_image(frame);

    match image {
        Err(x) => println!("{:#?}", x),
        Ok(x) => {
            let mut data = Mutex::lock(&user_data).unwrap();
            *data = Some(x);
        }
    }
}

fn main() {
    let ctx = Context::new().expect("Could not create context");
    let dev = ctx
        .find_device(None, None, None)
        .expect("Could not find device");

    let description = dev.description().unwrap();
    println!(
        "Found device: Bus {:03} Device {:03} : ID {:04x}:{:04x} {} ({})",
        dev.bus_number(),
        dev.device_address(),
        description.vendor_id,
        description.product_id,
        description.product.unwrap_or_else(|| "Unknown".to_owned()),
        description
            .manufacturer
            .unwrap_or_else(|| "Unknown".to_owned())
    );

    let devh = dev.open().expect("Could not open device");

    let format = devh
        .get_preferred_format(|x, y| {
            if x.width == WIN_WIDTH &&
                x.height == WIN_HEIGHT &&
                x.fps == CAM_FPS &&
                x.format == FrameFormat::Uncompressed {
                x
            } else {
                y
            }
        }).unwrap();

    println!("Best format found: {:?}", format);

    let mut streamh = devh.get_stream_handle_with_format(format).unwrap();

    println!(
        "Scanning mode: {:?}\nAuto-exposure mode: {:?}\nAuto-exposure priority: {:?}\nAbsolute exposure: {:?}\nRelative exposure: {:?}\nAboslute focus: {:?}\nRelative focus: {:?}",
        devh.scanning_mode(),
        devh.ae_mode(),
        devh.ae_priority(),
        devh.exposure_abs(),
        devh.exposure_rel(),
        devh.focus_abs(),
        devh.focus_rel(),
    );

    let mut window: PistonWindow =
        WindowSettings::new("Hello Piston!", [WIN_WIDTH, WIN_HEIGHT])
        .exit_on_esc(true).build().unwrap();

    let user_data = Arc::new(Mutex::new(None));
    let _stream = streamh
        .start_stream(callback_frame_to_image, user_data.clone())
        .unwrap();

    let mut texture: Option<Texture<_>> = None;
    let mut opt_corner_list: Option<Vec<Feature>> = None;

    while let Some(event) = window.next() {
        let mut mutex = Mutex::lock(&user_data).unwrap();
        let lock_results = mutex.take();

        if let Some(AnalysisResult { image, features }) = lock_results {
            opt_corner_list = Some(features);
            if let Some(ref mut texture_content) = texture {
                let _ = texture_content.update(&mut window.encoder, &image);
            } else {
                texture = Texture::from_image(
                    &mut window.factory,
                    &image,
                    &TextureSettings::new(),
                ).ok();
            }
        }

        window.draw_2d(&event, |context, graphics| {
            if let Some(ref texture_content) = texture {
                render_image(texture_content, context.transform, graphics);
            }

            if let Some(good_corners) = &opt_corner_list {
                let half_len = (BLOCKSIZE as f64) / 2.0;
                for Feature { x, y, .. } in good_corners {
                    render_rectangle(
                        [0.0, 0.0, 0.0, 1.0],
                        [(*x as f64) - half_len, (*y as f64) - half_len, half_len, half_len],
                        context.transform,
                        graphics,
                    );
                }
            }
        });
    }
}
