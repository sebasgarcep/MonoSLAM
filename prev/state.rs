use feature::Feature;
use im::RgbaImage;
use std::sync::{Arc, Mutex};

pub enum AppState {
    FeatureSearchState { image: RgbaImage, features: Vec<Feature> },
    InitState { image: RgbaImage, init_feature: Option<Feature> },
    TrackingState { image: RgbaImage, features: Vec<Feature> },
}

pub type SharedAppState = Arc<Mutex<Option<AppState>>>;

impl AppState {
    pub fn new_shared () -> SharedAppState {
        Arc::new(Mutex::new(None))
    }
}