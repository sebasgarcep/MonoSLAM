use serde_json::{from_str, Value};
use std::collections::HashMap;
use std::fs::File;
use std::io::Read;
use std::path::Path;

#[derive(Debug)]
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

impl CameraParams {
    fn get_u32_from_json (obj: &HashMap<String, Value>, key: &str) -> u32 {
        obj.get(key).unwrap().as_u64().unwrap() as u32
    }

    fn get_f64_from_json (obj: &HashMap<String, Value>, key: &str) -> f64 {
        obj.get(key).unwrap().as_f64().unwrap()
    }

    pub fn load_from_json <P: AsRef<Path>> (path: P) -> CameraParams {
        let mut file = File::open(path).unwrap();
        let mut contents = String::new();
        let _ = file.read_to_string(&mut contents);
        let obj: HashMap<String, Value> = from_str(&contents).unwrap();

        CameraParams {
            width: Self::get_u32_from_json(&obj, "width"),
            height: Self::get_u32_from_json(&obj, "height"),
            fps: Self::get_f64_from_json(&obj, "fps"),
            focal_length_x: Self::get_f64_from_json(&obj, "focal_length_x"),
            focal_length_y: Self::get_f64_from_json(&obj, "focal_length_y"),
            principal_point_x: Self::get_f64_from_json(&obj, "principal_point_x"),
            principal_point_y: Self::get_f64_from_json(&obj, "principal_point_y"),
            skew: Self::get_f64_from_json(&obj, "skew"),
            radial_distortion_x: Self::get_f64_from_json(&obj, "radial_distortion_x"),
            radial_distortion_y: Self::get_f64_from_json(&obj, "radial_distortion_y"),
            tangential_distortion_x: Self::get_f64_from_json(&obj, "tangential_distortion_x"),
            tangential_distortion_y: Self::get_f64_from_json(&obj, "tangential_distortion_y"),
            measurement_noise: Self::get_f64_from_json(&obj, "measurement_noise"),
        }
    }
}
