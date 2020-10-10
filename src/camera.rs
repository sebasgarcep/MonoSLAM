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
        }
    }

    pub fn project(&self, v: Vector3<f64>) -> Vector2<f64> {
        let imagepos_centred = Vector2::new(
            -self.focal_length_x * v[0] / v[2],
            -self.focal_length_y * v[1] / v[2],
        );

        let radius = imagepos_centred.norm();
        let factor = (1.0 + 2.0 * self.radial_distortion_x * radius.powi(2)).sqrt();

        let m_center = Vector2::new(
            self.principal_point_x,
            self.principal_point_y
        );

        imagepos_centred / factor + m_center
    }
}
