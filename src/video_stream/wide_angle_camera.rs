use ndarray::arr1;
use ndarray_linalg::norm::Norm;
use typedefs::Vector;
use video_stream::base::Camera;
use video_stream::camera_params::CameraParams;

pub struct WideAngleCamera {
    params: CameraParams,
}

impl WideAngleCamera {
    pub fn new (params: CameraParams) -> WideAngleCamera {
        WideAngleCamera { params }
    }
}

impl Camera for WideAngleCamera {
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

        let radius = imagepos_centred.norm_l2();
        let factor = (1.0 + 2.0 * self.params.radial_distortion_x * radius.powi(2)).sqrt();
        imagepos_centred.mapv_inplace(|e| e / factor);

        let m_center = arr1(&[
            self.params.principal_point_x,
            self.params.principal_point_y
        ]);

        imagepos_centred + m_center
    }

    fn unproject (&self, h: &Vector) -> Vector {
        let m_center = arr1(&[
            self.params.principal_point_x,
            self.params.principal_point_y
        ]);

        let mut centered = h - &m_center;
        let radius = centered.norm_l2();
        let factor = (1.0 - 2.0 * self.params.radial_distortion_x * radius.powi(2)).sqrt();

        centered.mapv_inplace(|e| e / factor);

        let mut camera = arr1(&[
            centered[0] / (- self.params.focal_length_x),
            centered[1] / (- self.params.focal_length_y),
            1.0,
        ]);

        let norm = camera.norm_l2();
        camera.mapv_inplace(|e| e / norm);

        camera
    }
}
