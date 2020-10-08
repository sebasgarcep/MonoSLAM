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
mod feature;
mod quaternion;
mod monoslam;
mod renderer;
mod shared_buffer;
mod typedefs;
mod utils;
mod video_stream;

use monoslam::MonoSLAM;
use renderer::Renderer;
use video_stream::{Camera, MockStream, VideoStream};

fn main () {
    let streamer = MockStream::new();
    let camera = streamer.get_camera();
    let camera_width = camera.width();
    let camera_height = camera.height();

    let (
        monoslam_image_buffer,
        monoslam_landmark_buffer,
    ) = MonoSLAM::start(camera);

    let mut renderer = Renderer::new(
        camera_width,
        camera_height,
        monoslam_landmark_buffer,
    );

    let renderer_image_buffer = renderer.get_image_buffer();

    streamer.start_stream(vec![
        monoslam_image_buffer,
        renderer_image_buffer
    ]);

    renderer.start();
}
