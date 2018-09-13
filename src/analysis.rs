use constants::PADDING;
use feature::Feature;
use im::{ConvertBuffer, RgbaImage, RgbImage};
use state::{AppState, SharedAppState};
use std::error::Error;
use std::sync::Mutex;
use uvc::Frame;

pub struct Analyzer {
    app_state: SharedAppState,
}

impl Analyzer {
    pub fn new (app_state: SharedAppState) -> Analyzer {
        Analyzer { app_state }
    }

    // Shi-Tomasi insipired by: https://github.com/onkursen/corner-detection
    pub fn process_image (&self, image: RgbaImage) -> AppState {
        let features = Feature::detect(
            &image,
            PADDING,
            PADDING,
            image.width() - PADDING,
            image.height() - PADDING,
        );

        AppState::FeatureSearchState {
            image,
            features,
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

    pub fn process_frame (
        &self,
        frame: &Frame
    ) {
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
