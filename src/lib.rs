#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_assignments)]
#![allow(unused_imports)]
// FIXME: remove this once we are done prototyping

extern crate image;
extern crate nalgebra;
extern crate piston_window;
#[macro_use]
extern crate lazy_static;
extern crate serde;
extern crate serde_json;

pub mod app_state;
pub mod calculus;
pub mod camera_model;
pub mod constants;
pub mod detection;
pub mod utils;
pub mod video_stream;
