use std::fs::File;
use std::io::BufReader;
use nalgebra::{Matrix2, Matrix2x3, Matrix3x2, Vector2, Vector3};
use serde::Deserialize;

#[derive(Deserialize)]
pub struct CameraParams {
    #[allow(dead_code)] model: String,
    width: usize,
    height: usize,
    #[allow(dead_code)] fps: f64,
    focal_length_x: f64,
    focal_length_y: f64,
    principal_point_x: f64,
    principal_point_y: f64,
    #[allow(dead_code)] skew: f64,
    radial_distortion_x: f64,
    #[allow(dead_code)] radial_distortion_y: f64,
    #[allow(dead_code)] tangential_distortion_x:  f64,
    #[allow(dead_code)] tangential_distortion_y: f64,
    measurement_noise: f64,
}

pub struct WideAngleCameraModel {
    params: CameraParams,
    m_center: Vector2<f64>,
}

impl WideAngleCameraModel {
    pub fn from_json(path: &str) -> Self {
        // Read JSON file
        let file = File::open(path).unwrap();
        let reader = BufReader::new(file);
        let params: CameraParams = serde_json::from_reader(reader).unwrap();

        let principal_point_x = params.principal_point_x;
        let principal_point_y = params.principal_point_y;

        WideAngleCameraModel {
            params,
            m_center: Vector2::new(
                principal_point_x,
                principal_point_y,
            )
        }
    }

    pub fn project(&self, v: &Vector3<f64>) -> Vector2<f64> {
        let imagepos_centred = Vector2::new(
            -self.params.focal_length_x * v[0] / v[2],
            -self.params.focal_length_y * v[1] / v[2],
        );

        let radius2 = imagepos_centred.norm_squared();
        let factor = (1.0 + 2.0 * self.params.radial_distortion_x * radius2).sqrt();

        imagepos_centred / factor + self.m_center
    }

    pub fn unproject(&self, v: &Vector2<f64>) -> Vector3<f64> {
        let v_centered = v - self.m_center;
        let radius2 = v_centered.norm_squared();
        let factor = (1.0 - 2.0 * self.params.radial_distortion_x * radius2).sqrt();
        let undistorted = v_centered / factor;

        Vector3::new(
            -undistorted[0] / self.params.focal_length_x,
            -undistorted[1] / self.params.focal_length_y,
            1.0,
        )
    }

    // Use formulas from SceneLib
    pub fn project_jacobian(&self, v: &Vector3<f64>) -> Matrix2x3<f64> {
        let fkx = self.params.focal_length_x / v[2];
        let fky = self.params.focal_length_y / v[2];

        let du_dv = Matrix2x3::<f64>::new(
            -fkx, 0.0, fkx * v[0] / v[2],
            0.0, -fky, fky * v[1] / v[2],
        );

        let imagepos_centred = Vector2::new(
            -self.params.focal_length_x * v[0] / v[2],
            -self.params.focal_length_y * v[1] / v[2],
        );

        let mut dh_du = imagepos_centred * imagepos_centred.transpose();
        let radius2 = dh_du.trace();
        let distor = 1.0 + 2.0 * self.params.radial_distortion_x * radius2;
        let distor1_2 = distor.sqrt();
        let distor3_2 = distor1_2 * distor;
        dh_du = -2.0 * self.params.radial_distortion_x / distor3_2 * dh_du;
        dh_du[(0, 0)] = dh_du[(0, 0)] + 1.0 / distor1_2;
        dh_du[(1, 1)] = dh_du[(1, 1)] + 1.0 / distor1_2;

        dh_du * du_dv
    }

    // Use formulas from SceneLib
    pub fn unproject_jacobian(&self, v: &Vector2<f64>) -> Matrix3x2<f64> {
        let dy_du = Matrix3x2::<f64>::new(
            -1.0 / self.params.focal_length_x, 0.0,
            0.0, -1.0 / self.params.focal_length_y,
            0.0, 0.0,
        );

        let v_centered = v - self.m_center;
        let mut du_dh = v_centered * v_centered.transpose();
        let radius2 = du_dh.trace();

        let distor = 1.0 - 2.0 * self.params.radial_distortion_x * radius2;
        let distor1_2 = distor.sqrt();
        let distor3_2 = distor1_2 * distor;

        du_dh = 2.0 * self.params.radial_distortion_x / distor3_2 * du_dh;
        du_dh[(0, 0)] = du_dh[(0, 0)] + 1.0 / distor1_2;
        du_dh[(1, 1)] = du_dh[(1, 1)] + 1.0 / distor1_2;

        dy_du * du_dh
    }

    pub fn width(&self) -> usize {
        self.params.width
    }

    pub fn height(&self) -> usize {
        self.params.height
    }

    pub fn measurement_noise(&self, v: &Vector2<f64>) -> Matrix2<f64> {
        let distance = (v - &self.m_center).norm();
        let max_distance = &self.m_center.norm();
        let ratio = distance / max_distance;

        let sd_image_filter = self.params.measurement_noise * (1.0 + ratio);

        sd_image_filter.powi(2) * Matrix2::identity()
    }
}
