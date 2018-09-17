extern crate image as im;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate ndarray;
extern crate ndarray_linalg;
extern crate openblas_src;
extern crate piston_window;
extern crate uvc;

mod calculus;
mod camera;
mod constants;
mod detection;
mod device;
mod ellipse_search;
mod feature;
mod renderer;
mod state;
mod tracking;
mod typedefs;
mod utils;

use device::get_stream;
use renderer::Renderer;
use state::AppState;
use std::sync::mpsc::channel;
use std::sync::Mutex;
use std::thread;
use tracking::Tracker;
use utils::frame_to_image;

fn main () {
    let mut streamh = get_stream();

    let shared_state = AppState::new_shared();
    let mut tracker = Tracker::new(shared_state.clone());
    let mut renderer = Renderer::new(shared_state);

    let (tx, rx) = channel();

    let sync_tx = Mutex::new(tx.clone());
    let _stream = streamh
        .start_stream(|frame, received_sync_tx| {
            let image = frame_to_image(frame).unwrap();
            let int_tx = Mutex::lock(received_sync_tx).unwrap();
            int_tx.send(image).unwrap();
        }, sync_tx)
        .unwrap();

    thread::spawn(move || {
        renderer.start();
    });

    for received in rx {
        tracker.process_frame(received);
    }
}
