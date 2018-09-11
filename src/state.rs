use feature::Feature;
use im::RgbaImage;
use std::sync::{Arc, Mutex};

pub struct AppState {
    pub image: RgbaImage,
    pub features: Vec<Feature>,
}

pub type SharedAppState = Arc<Mutex<Option<AppState>>>;

impl AppState {
    pub fn new_shared () -> SharedAppState {
        Arc::new(Mutex::new(None))
    }
}
