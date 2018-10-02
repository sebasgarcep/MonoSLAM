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
}
