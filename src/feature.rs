use constants::BLOCKSIZE;
use im::{GenericImage, RgbaImage};
use ndarray::arr1;
use std::rc::Rc;
use typedefs::{SharedVector, Vector};
use video_stream::Camera;

#[derive(Debug, Deserialize)]
pub struct DeserializedFeature {
    yi: [f64; 3],
    xp_orig: [f64; 7],
}

pub struct Feature {
    pub yi: SharedVector,
    pub xp_orig: SharedVector,
    pub image: RgbaImage,
}

impl DeserializedFeature {
    pub fn convert <C: Camera> (&self, camera: &C, image: &mut RgbaImage) -> Feature {
        let yi = Rc::new(arr1(&self.yi));
        let h = camera.project(&yi);

        Feature {
            yi,
            xp_orig: Rc::new(arr1(&self.xp_orig)),
            image: image
                .sub_image(
                    (h[0] - (BLOCKSIZE as f64) / 2.0) as u32,
                    (h[1] - (BLOCKSIZE as f64) / 2.0) as u32,
                    BLOCKSIZE,
                    BLOCKSIZE,
                )
                .to_image()
        }
    }
}
