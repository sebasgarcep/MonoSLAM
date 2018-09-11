use constants::{
    FOCAL_LENGTH_X,
    FOCAL_LENGTH_Y,
    PRINCIPAL_POINT_X,
    PRINCIPAL_POINT_Y,
    INIT_MAX_DISTANCE,
    INIT_MIN_DISTANCE,
    NUM_INIT_PARTICLES,
};
use feature::Feature;
use im::{ConvertBuffer, RgbaImage, RgbImage};
use ndarray::{arr2, Array2, ShapeBuilder};
use state::{AppState, SharedAppState};
use std::error::Error;
use std::sync::Mutex;
use uvc::Frame;

lazy_static! {
    static ref K: Array2<f64> = arr2(&[
        [FOCAL_LENGTH_X, 0.0, PRINCIPAL_POINT_X],
        [0.0, FOCAL_LENGTH_Y, PRINCIPAL_POINT_Y],
        [0.0, 0.0, 1.0],
    ]);

    static ref K_INV: Array2<f64> = arr2(&[
        [FOCAL_LENGTH_X.recip(), 0.0, - FOCAL_LENGTH_X.recip() * PRINCIPAL_POINT_X],
        [0.0, FOCAL_LENGTH_Y.recip(), - FOCAL_LENGTH_Y.recip() * PRINCIPAL_POINT_Y],
        [0.0, 0.0, 1.0],
    ]);
}

pub struct Initializer {
    app_state: SharedAppState,
    points: Option<Vec<f64>>,
}

impl Initializer {
    pub fn new (app_state: SharedAppState) -> Initializer {
        /*
        let mut points = Vec::with_capacity(NUM_INIT_PARTICLES);

        for idx in 0..NUM_INIT_PARTICLES {
            let ratio = (idx as f64) / ((NUM_INIT_PARTICLES - 1) as f64);
            points.push(INIT_MIN_DISTANCE + ratio * (INIT_MAX_DISTANCE - INIT_MIN_DISTANCE));
        }
        */

        Initializer {
            app_state,
            points: None,
        }
    }

    pub fn process_image (&mut self, image: RgbaImage) -> AppState {
        AppState::InitState {
            image,
        }
    }

    pub fn frame_to_image (
        frame: &Frame,
    ) -> Result<RgbaImage, Box<dyn Error>> {
        let width = frame.width();
        let height = frame.height();

        let new_frame = frame.to_rgb()?;
        let data = new_frame.to_bytes();
        let image: RgbaImage = RgbImage::from_raw(
            width,
            height,
            data.to_vec(),
        ).ok_or("This shouldn't happen")?.convert();

        Ok(image)
    }

    pub fn process_frame (&mut self, frame: &Frame) {
        let maybe_image = Self::frame_to_image(frame);
        let maybe_app_state = maybe_image.map(|image| self.process_image(image));
        match maybe_app_state {
            Err(x) => println!("{:#?}", x),
            Ok(x) => {
                let mut data = Mutex::lock(&self.app_state).unwrap();
                *data = Some(x);
            }
        };
    }
}
