use detection::Detection;
use im::RgbaImage;
use ndarray::{arr1, arr2};
use std::rc::Rc;
use std::convert::From;
use typedefs::{Matrix, SharedMatrix, SharedVector};

lazy_static! {
    static ref SOBEL_GX: Matrix = arr2(&[
        [1.0, 0.0, -1.0],
        [2.0, 0.0, -2.0],
        [1.0, 0.0, -1.0],
    ]);

    static ref SOBEL_GY: Matrix = arr2(&[
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
    pub fn get_h (&self) -> SharedVector {
        Rc::new(arr1(&[self.u as f64, self.v as f64]))
    }

    pub fn get_yi (&self) -> SharedVector {
        unimplemented!()
    }

    pub fn get_pxy (&self) -> SharedMatrix {
        unimplemented!()
    }

    pub fn get_pyy (&self) -> SharedMatrix {
        unimplemented!()
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
