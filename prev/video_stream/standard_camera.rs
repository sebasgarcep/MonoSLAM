use video_stream::base::Camera;

pub struct StandardCamera {}

impl StandardCamera {
    pub fn new () -> StandardCamera {
        StandardCamera {}
    }
}

impl Camera for StandardCamera {
    fn width (&self) -> u32 {
        self.params.width
    }

    fn height (&self) -> u32 {
        self.params.height
    }

    fn project (&self, hrl: &Vector) -> Vector {
        let mut imagepos_centred = arr1(&[
            - self.params.focal_length_x * hrl[0] / hrl[2],
            - self.params.focal_length_y * hrl[1] / hrl[2],
        ]);

        let m_center = arr1(&[
            self.params.principal_point_x,
            self.params.principal_point_y
        ]);

        imagepos_centred + m_center
    }

    // FIXME: this does not work -----------------------------------------------
    fn unproject (&self, h: &Vector) -> Vector {
        let diff = (&self.principal_point - h) / &self.focal_length;
        let deprojection = arr1(&[diff[0], diff[1], 1.0]);
        let norm = deprojection.norm_l2();
        deprojection.mapv(|e| e / norm)
    }

    fn measurement_noise (&self, h: &Array1<f64>) -> Array2<f64> {
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
    // -------------------------------------------------------------------------
}
