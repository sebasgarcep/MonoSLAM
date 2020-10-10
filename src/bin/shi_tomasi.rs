extern crate image;
extern crate piston_window;
extern crate mono_slam;

use mono_slam::detection::Detection;
use image::DynamicImage;
use piston_window::{EventLoop, ellipse};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;

fn main() {
    let mut window: piston_window::PistonWindow =
        piston_window::WindowSettings::new("Raytracer", [WIDTH, HEIGHT])
            .resizable(false)
            .exit_on_esc(true)
            .build()
            .unwrap_or_else(|_e| { panic!("Could not create window!")});

    window.set_max_fps(25);

    let mut idx = 0;
    while let Some(e) = window.next() {
        let img = image::open(format!("./data/frames/rawoutput{:0>4}.pgm", idx))
            .unwrap()
            .to_rgba();

        let img_gray = DynamicImage::ImageRgba8(img.clone()).into_luma();

        let features = Detection::detect(&img_gray);

        let tex = piston_window::Texture::from_image(
            &mut window.create_texture_context(),
            &img,
            &piston_window::TextureSettings::new())
            .unwrap();

        window.draw_2d(&e, |c, g, _| {
            piston_window::clear([1.0; 4], g);
            piston_window::image(&tex, c.transform, g);
            for detection in &features {
                ellipse(
                    [0.0, 0.0, 1.0, 1.0],
                    [detection.x as f64, detection.y as f64, 11.0, 11.0],
                    c.transform,
                    g,
                );
            }
        });

        idx = (idx + 1) % 1000;
    }
}
