use nalgebra::{Matrix2x3, Vector2, Vector3};

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

    // Use formulas from SceneLib
    pub fn project_jacobian(&self, v: &Vector3<f64>) -> Matrix2x3<f64> {
        let fkx = self.focal_length_x / v[2];
        let fky = self.focal_length_y / v[2];

        let du_dv = Matrix2x3::<f64>::new(
            -fkx, 0.0, fkx * v[0] / v[2],
            0.0, -fky, fky * v[1] / v[2],
        );

        let imagepos_centred = Vector2::new(
            -self.focal_length_x * v[0] / v[2],
            -self.focal_length_y * v[1] / v[2],
        );

        let mut dh_du = imagepos_centred * imagepos_centred.transpose();
        let radius2 = dh_du.trace();
        let distor = 1.0 + 2.0 * self.radial_distortion_x * radius2;
        let distor1_2 = distor.sqrt();
        let distor3_2 = distor1_2 * distor;
        dh_du = -2.0 * self.radial_distortion_x / distor3_2 * dh_du;
        dh_du[(0, 0)] = dh_du[(0, 0)] + 1.0 / distor1_2;
        dh_du[(1, 1)] = dh_du[(1, 1)] + 1.0 / distor1_2;

        dh_du * du_dv
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn measurement_noise(&self) -> f64 {
        self.measurement_noise
    }
}
