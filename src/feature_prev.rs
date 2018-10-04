use detection::Detection;
use im::RgbaImage;
use ndarray::{arr1, arr2};
use std::rc::Rc;
use std::convert::From;
use typedefs::{Matrix, SharedMatrix, SharedVector};

#[derive(Clone, Debug)]
pub struct Feature {
    pub u: u32,
    pub v: u32,
    pub score: f64,
    pub image: RgbaImage,
    // FIXME: instroduce later pub matrix: Array2<f64>,
    // FIXME: instroduce later pub pos: Array1<f64>,
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
