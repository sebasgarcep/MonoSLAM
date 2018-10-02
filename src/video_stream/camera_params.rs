#[derive(Debug, Deserialize)]
pub struct CameraParams {
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    pub focal_length_x: f64,
    pub focal_length_y: f64,
    pub principal_point_x: f64,
    pub principal_point_y: f64,
    pub skew: f64,
    pub radial_distortion_x: f64,
    pub radial_distortion_y: f64,
    pub tangential_distortion_x: f64,
    pub tangential_distortion_y: f64,
    pub measurement_noise: f64,
}
