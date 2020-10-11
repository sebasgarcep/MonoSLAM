use nalgebra::{Vector2, Vector3};

pub struct WideAngleCamera {
    width: usize,
    height: usize,
    fps: f64,
    focal_length_x: f64,
    focal_length_y: f64,
    principal_point_x: f64,
    principal_point_y: f64,
    skew: f64,
    radial_distortion_x: f64,
    radial_distortion_y: f64,
    tangential_distortion_x:  f64,
    tangential_distortion_y: f64,
    measurement_noise: f64,
    m_center: Vector2<f64>,
}

impl WideAngleCamera {
    pub fn new(
        width: usize,
        height: usize,
        fps: f64,
        focal_length_x: f64,
        focal_length_y: f64,
        principal_point_x: f64,
        principal_point_y: f64,
        skew: f64,
        radial_distortion_x: f64,
        radial_distortion_y: f64,
        tangential_distortion_x:  f64,
        tangential_distortion_y: f64,
        measurement_noise: f64,
    ) -> Self {
        WideAngleCamera {
            width,
            height,
            fps,
            focal_length_x,
            focal_length_y,
            principal_point_x,
            principal_point_y,
            skew,
            radial_distortion_x,
            radial_distortion_y,
            tangential_distortion_x,
            tangential_distortion_y,
            measurement_noise,
            m_center: Vector2::new(
                principal_point_x,
                principal_point_y,
            ),
        }
    }

    pub fn project(&self, v: Vector3<f64>) -> Vector2<f64> {
        let imagepos_centred = Vector2::new(
            -self.focal_length_x * v[0] / v[2],
            -self.focal_length_y * v[1] / v[2],
        );

        let radius2 = imagepos_centred.norm_squared();
        let factor = (1.0 + 2.0 * self.radial_distortion_x * radius2).sqrt();

        imagepos_centred / factor + self.m_center
    }

    pub fn unproject(&self, v: Vector2<f64>) -> Vector3<f64> {
        let v_centered = v - self.m_center;
        let radius2 = v_centered.norm_squared();
        let factor = (1.0 - 2.0 * self.radial_distortion_x * radius2).sqrt();
        let undistorted = v_centered / factor;

        Vector3::new(
            -undistorted[0] / self.focal_length_x,
            -undistorted[1] / self.focal_length_y,
            1.0,
        )
    }
}
