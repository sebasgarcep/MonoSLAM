extern crate image;
extern crate piston_window;
extern crate mono_slam;
extern crate nalgebra;

use image::{DynamicImage, Pixel};
use mono_slam::detection::Detection;
use nalgebra::DMatrix;
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

        let img_width = img.width() as usize;
        let img_height = img.height() as usize;

        // Transform RGBA image to grayscale image
        let img_gray = DynamicImage::ImageRgba8(img.clone()).into_luma();
        let mat = DMatrix::<f64>::from_iterator(
            img_width,
            img_height,
            img_gray.pixels().map(|x| (x.channels()[0] as f64) / 255.0));

        let features = Detection::detect(&mat);

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
