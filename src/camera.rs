use constants::{
    FOCAL_LENGTH_X,
    FOCAL_LENGTH_Y,
    PRINCIPAL_POINT_X,
    PRINCIPAL_POINT_Y,
};
use ndarray::{arr1, arr2, Array, Array1, Array2};
use ndarray_linalg::norm::Norm;

pub struct Camera {
    k: Array2<f64>,
    kinv: Array2<f64>,
    principal_point: Array1<f64>,
    focal_length: Array1<f64>,
}

impl Camera {
    pub fn new () -> Camera {
        let k = arr2(&[
            [FOCAL_LENGTH_X, 0.0, PRINCIPAL_POINT_X],
            [0.0, FOCAL_LENGTH_Y, PRINCIPAL_POINT_Y],
            [0.0, 0.0, 1.0],
        ]);

        let kinv = arr2(&[
            [FOCAL_LENGTH_X.recip(), 0.0, - FOCAL_LENGTH_X.recip() * PRINCIPAL_POINT_X],
            [0.0, FOCAL_LENGTH_Y.recip(), - FOCAL_LENGTH_Y.recip() * PRINCIPAL_POINT_Y],
            [0.0, 0.0, 1.0],
        ]);

        let principal_point = arr1(&[
            PRINCIPAL_POINT_X,
            PRINCIPAL_POINT_Y,
        ]);

        let focal_length = arr1(&[
            FOCAL_LENGTH_X,
            FOCAL_LENGTH_Y,
        ]);

        Camera {
            k,
            kinv,
            principal_point,
            focal_length,
        }
    }

    pub fn project (&self, hrl: &Array1<f64>) -> Array1<f64> {
        arr1(&[
            PRINCIPAL_POINT_X - FOCAL_LENGTH_X * hrl[0] / hrl[2],
            PRINCIPAL_POINT_Y - FOCAL_LENGTH_Y * hrl[1] / hrl[2],
        ])
    }

    /**
     *
     * When projecting from 2D to 3D space we cannot predict depth,
     * so we return a normalized vector, and the actual position vector
     * should be parallel to the direction vector we return.
     *
     */
    pub fn unproject (&self, h: &Array1<f64>) -> Array1<f64> {
        let diff = (&self.principal_point - h) / &self.focal_length;
        let deprojection = arr1(&[diff[0], diff[1], 1.0]);
        let norm = deprojection.norm_l2();
        deprojection.mapv(|e| e / norm)
    }

    pub fn measurement_noise (&self, h: &Array1<f64>) -> Array2<f64> {
        let distance = (&self.principal_point - h).norm_l2();
        let max_distance = self.principal_point.norm_l2();
        let ratio = distance / max_distance;
        let sd_image_filter_to_use = 1.0 /* camera noise */ + (1.0 + ratio);
        let variance = sd_image_filter_to_use.powi(2);
        let mut noise = Array::zeros((2, 2));
        for idx in 0..2 {
            noise[[idx, idx]] = variance
        }
        noise
    }

    pub fn projection_jacobian (&self, hrl: &Array1<f64>) -> Array2<f64> {
        let principal_point_xz = self.principal_point[0] / hrl[2];
        let principal_point_yz = self.principal_point[1] / hrl[2];
        arr2(&[
            [- principal_point_xz, 0.0, principal_point_xz * hrl[0] / hrl[2]],
            [0.0, - principal_point_yz, principal_point_yz * hrl[1] / hrl[2]],
        ])
    }

    pub fn unprojection_jacobian (&self, h: &Array1<f64>) -> Array2<f64> {
        arr2(&[
            [- self.principal_point[0].recip(), 0.0],
            [0.0, - self.principal_point[1].recip()],
            [0.0, 0.0],
        ])
    }
}

