extern crate image;
extern crate piston_window;

use piston_window::EventLoop;

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

        let tex = piston_window::Texture::from_image(
            &mut window.create_texture_context(),
            &img,
            &piston_window::TextureSettings::new())
            .unwrap();

        window.draw_2d(&e, |c, g, _| {
            piston_window::clear([1.0; 4], g);
            piston_window::image(&tex, c.transform, g);
        });

        idx = (idx + 1) % 1000;
    }
}
