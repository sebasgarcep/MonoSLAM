extern crate image as im;
#[macro_use]
extern crate ndarray;
extern crate piston_window;
extern crate uvc;
#[macro_use]
extern crate lazy_static;

mod analysis;
mod constants;
mod device;
mod feature;
mod init;
mod renderer;
mod state;

use analysis::Analyzer;
use device::get_stream;
use state::AppState;
use renderer::Renderer;

fn main () {
    let mut streamh = get_stream();

    let shared_state = AppState::new_shared();
    let analyzer = Analyzer::new(shared_state.clone());
    let renderer = Renderer::new(shared_state);

    let _stream = streamh
        .start_stream(move |frame, _| {
            analyzer.process_frame(frame);
        }, ())
        .unwrap();

    renderer.start();
}
