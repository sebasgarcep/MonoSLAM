extern crate image as im;
#[macro_use]
extern crate ndarray;
extern crate piston_window;
extern crate uvc;
#[macro_use]
extern crate lazy_static;

use im::{ConvertBuffer, RgbaImage, RgbImage};
use ndarray::{arr2, Array2, ShapeBuilder};
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

type CornerType = (u32, u32, f64);
type UserDataContentType = (RgbaImage, Vec<CornerType>);
type UserDataType = Arc<Mutex<Option<UserDataContentType>>>;

const WIN_WIDTH: u32 = 640;
const WIN_HEIGHT: u32 = 480;
const BLOCKSIZE: u32 = 11;
const QUALITY_LEVEL: f64 = 1.0;
const CAM_FPS: u32 = 30;
const NUM_FEATURES: usize = 12;
const MIN_DISTANCE_SQ: u32 = 1000;
const PADDING: u32 = 10;

lazy_static! {
    static ref SOBEL_GX: Array2<f64> = arr2(&[
        [1.0, 0.0, -1.0],
        [2.0, 0.0, -2.0],
        [1.0, 0.0, -1.0],
    ]);

    static ref SOBEL_GY: Array2<f64> = arr2(&[
        [1.0, 2.0, 1.0],
        [0.0, 0.0, 0.0],
        [-1.0, -2.0, -1.0]
    ]);
}

fn find_eigenvalues(m_xx: f64, m_xy: f64, m_yy: f64) -> (f64, f64) {
    let bb = ((m_xx + m_yy).powi(2) - 4.0 * (m_xx * m_yy - m_xy * m_xy)).sqrt();
    ((m_xx + m_yy + bb) / 2.0, (m_xx + m_yy - bb) / 2.0)
}

// Shi-Tomasi insipired by: https://github.com/onkursen/corner-detection
fn frame_to_raw_image(
    frame: &Frame,
) -> Result<UserDataContentType, Box<dyn Error>> {
    let width = frame.width();
    let height = frame.height();

    let new_frame = frame.to_rgb()?;
    let data = new_frame.to_bytes();
    let image: RgbaImage = RgbImage::from_raw(
        width,
        height,
        data.to_vec(),
    ).ok_or("This shouldn't happen")?.convert();

    let shape = (width as usize, height as usize).f();
    let mut image_mat = Array2::<f64>::zeros(shape);
    let mut image_dx = Array2::<f64>::zeros(shape);
    let mut image_dy = Array2::<f64>::zeros(shape);

    for x in 0..width {
        for y in 0..height {
            let pixel = image.get_pixel(x, y);
            let r = pixel.data[0] as f64;
            let g = pixel.data[1] as f64;
            let b = pixel.data[2] as f64;
            image_mat[[x as usize, y as usize]] = (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0;
        }
    }

    for x in 0..width {
        for y in 0..height {
            let image_win = image_mat.slice(s![
                ((if x == 0 { x } else { x - 1 }) as usize)..(((if x == width - 1 { x } else { x + 1 }) + 1) as usize),
                ((if y == 0 { y } else { y - 1 }) as usize)..(((if y == height - 1 { y } else { y + 1 }) + 1) as usize),
            ]);

            let gx_win = SOBEL_GX.slice(s![
                (if x == 0 { 1 } else { 0 })..((if x == width - 1 { 1 } else { 2 }) + 1),
                (if y == 0 { 1 } else { 0 })..((if y == height - 1 { 1 } else { 2 }) + 1),
            ]);

            let gy_win = SOBEL_GY.slice(s![
                (if x == 0 { 1 } else { 0 })..((if x == width - 1 { 1 } else { 2 }) + 1),
                (if y == 0 { 1 } else { 0 })..((if y == height - 1 { 1 } else { 2 }) + 1),
            ]);

            image_dx[[x as usize, y as usize]] = (&gx_win * &image_win).scalar_sum();
            image_dy[[x as usize, y as usize]] = (&gy_win * &image_win).scalar_sum();
        }
    }

    let image_dxx = &image_dx * &image_dx;
    let image_dxy = &image_dx * &image_dy;
    let image_dyy = &image_dy * &image_dy;
    let mut good_corners: Vec<CornerType> = vec![];

    for x in PADDING..(width - PADDING) {
        for y in PADDING..(height - PADDING) {
            let mut m_xx = 0.0;
            let mut m_xy = 0.0;
            let mut m_yy = 0.0;

            let half_len = BLOCKSIZE / 2;
            let x_min = if x < half_len { 0 } else { x - half_len };
            let y_min = if y < half_len { 0 } else { y - half_len };
            let x_max = if x + half_len > width - 1 { width - 1 } else { x + half_len };
            let y_max = if y + half_len > height - 1 { height - 1 } else { y + half_len };

            for u in x_min..(x_max + 1) {
                for v in y_min..(y_max + 1) {
                    m_xx += image_dxx[[u as usize, v as usize]];
                    m_xy += image_dxy[[u as usize, v as usize]];
                    m_yy += image_dyy[[u as usize, v as usize]];
                }
            }

            let (ev1, ev2) = find_eigenvalues(m_xx, m_xy, m_yy);

            let grad = if ev1 < ev2 { ev1 } else { ev2 };

            if grad < QUALITY_LEVEL {
                continue;
            }

            let corner = (x, y, grad);
            let mut should_push = good_corners.len() < NUM_FEATURES;

            for idx in 0..good_corners.len() {
                let test_corner = good_corners[idx];
                let distance = (corner.0 - test_corner.0).pow(2) + (corner.1 - test_corner.1).pow(2);

                if distance < MIN_DISTANCE_SQ {
                    if test_corner.2 < corner.2 {
                        good_corners[idx] = corner;
                    }
                    should_push = false;
                    break;
                } else if !should_push {
                    if test_corner.2 < corner.2 {
                        good_corners[idx] = corner;
                        break;
                    }
                }
            }

            if should_push {
                good_corners.push(corner);
            }
        }
    }

    Ok((image, good_corners))
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
    let mut opt_corner_list: Option<Vec<CornerType>> = None;

    while let Some(event) = window.next() {
        let mut mutex = Mutex::lock(&user_data).unwrap();
        let lock_results = mutex.take();

        if let Some((image, good_corners)) = lock_results {
            opt_corner_list = Some(good_corners);
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
                for (x, y, _) in good_corners {
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
