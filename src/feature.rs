use detection::Detection;
use im::RgbaImage;
use ndarray::{arr1, arr2, Array1, Array2};
use std::convert::From;

lazy_static! {
    static ref SOBEL_GX: Array2<f64> = arr2(&[
        [1.0, 0.0, -1.0],
        [2.0, 0.0, -2.0],
        [1.0, 0.0, -1.0],
    ]);

    static ref SOBEL_GY: Array2<f64> = arr2(&[
        [1.0, 2.0, 1.0],
        [0.0, 0.0, 0.0],
        [-1.0, -2.0, -1.0]
    ]);
}

#[derive(Clone, Debug)]
pub struct Feature {
    pub u: u32,
    pub v: u32,
    pub score: f64,
    pub image: RgbaImage,
    // FIXME: instroduce later pub matrix: Array2<f64>,
    // FIXME: instroduce later pub pos: Array1<f64>,
}

impl Feature {
    pub fn get_h (&self) -> Array1<f64> {
        arr1(&[self.u as f64, self.v as f64])
    }
}

impl From<Detection> for Feature {
    fn from(detection: Detection) -> Feature {
        Feature {
            u: detection.u,
            v: detection.v,
            score: detection.score,
            image: detection.image,
        }
    }
}
