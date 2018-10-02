extern crate image as im;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate ndarray;
extern crate ndarray_linalg;
extern crate openblas_src;
extern crate piston_window;
extern crate serde;
extern crate serde_json;
// extern crate uvc;

// mod calculus;
// mod constants;
// mod detection;
// mod ellipse_search;
// mod feature;
mod monoslam;
mod renderer;
mod shared_buffer;
// mod state;
mod typedefs;
// mod utils;
mod video_stream;

use monoslam::MonoSLAM;
use renderer::Renderer;
// use state::AppState;
use std::thread;
use video_stream::{Camera, MockStream, VideoStream};

fn main () {
    /*
    let shared_state = AppState::new_shared();
    let mut tracker = Tracker::new(shared_state.clone());
    */

    let streamer = MockStream::new();
    let camera = streamer.get_camera();

    let monoslam = MonoSLAM::new();
    monoslam.start();

    let mut renderer = Renderer::new(camera.width(), camera.height());

    let monoslam_image_buffer = monoslam.get_image_buffer();
    let renderer_image_buffer = renderer.get_image_buffer();

    let rx = streamer.start_stream();
    thread::spawn(move || {
        for received in rx {
            monoslam_image_buffer.update(received.clone());
            renderer_image_buffer.update(received);
        }
    });

    renderer.start();
}
