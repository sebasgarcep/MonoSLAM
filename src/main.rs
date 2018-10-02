extern crate image as im;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate ndarray;
extern crate ndarray_linalg;
extern crate openblas_src;
extern crate piston_window;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate serde_json;
extern crate uvc;

mod constants;
mod monoslam;
mod renderer;
mod shared_buffer;
mod typedefs;
mod utils;
mod video_stream;

use monoslam::MonoSLAM;
use renderer::Renderer;
use std::thread;
use video_stream::{Camera, MockStream, VideoStream};

fn main () {
    let streamer = MockStream::new();
    let camera = streamer.get_camera();

    let mut renderer = Renderer::new(camera.width(), camera.height());
    let renderer_image_buffer = renderer.get_image_buffer();

    let monoslam_image_buffer = MonoSLAM::start(camera);

    let rx = streamer.start_stream();
    thread::spawn(move || {
        for received in rx {
            monoslam_image_buffer.update(received.clone());
            renderer_image_buffer.update(received);
        }
    });

    renderer.start();
}
