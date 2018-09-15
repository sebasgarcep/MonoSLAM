extern crate image as im;
#[macro_use]
extern crate lazy_static;
#[macro_use]
extern crate ndarray;
extern crate ndarray_linalg;
extern crate openblas_src;
extern crate piston_window;
extern crate uvc;

// mod analysis;
mod calculus;
mod camera;
mod constants;
mod detection;
mod device;
mod feature;
// mod init;
mod renderer;
mod state;
mod tracking;
mod utils;

// use analysis::Analyzer;
use device::get_stream;
// use init::Initializer;
use state::AppState;
use tracking::Tracker;
use renderer::Renderer;

fn main () {
    let mut streamh = get_stream();

    let shared_state = AppState::new_shared();
    // let analyzer = Analyzer::new(shared_state.clone());
    // let initializer = Initializer::new(shared_state.clone());
    let tracker = Tracker::new(shared_state.clone());
    let mut renderer = Renderer::new(shared_state);

    let _stream = streamh
        /*
        .start_stream(|frame, received_initializer| {
            received_initializer.process_frame(frame);
        }, initializer)
        */
        /*
        .start_stream(|frame, received_analyzer| {
            received_analyzer.process_frame(frame);
        }, analyzer)
        */
        .start_stream(|frame, received_tracker| {
            received_tracker.process_frame(frame);
        }, tracker)
        .unwrap();

    renderer.start();
}
