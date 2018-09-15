use camera::Camera;
use constants::{
    NUM_FEATURES,
    PRINCIPAL_POINT_X,
    PRINCIPAL_POINT_Y,
};
use detection::Detection;
use feature::Feature;
use im::{ConvertBuffer, RgbaImage, RgbImage};
use ndarray::{arr1, Array, Array1, Array2};
use state::{AppState, SharedAppState};
use std::error::Error;
use std::sync::Mutex;
use std::time::{SystemTime};
use utils::{
    convert_matrix_to_quaternion,
    get_rotation_matrix_from_angular_displacement,
    quaternion_product
};
use uvc::Frame;

pub struct Tracker {
    app_state: SharedAppState,
    camera: Camera,
    features: Vec<Feature>,
    last_time: SystemTime,
    rw: Array1<f64>,
    rotwr: Array2<f64>,
    vw: Array1<f64>,
    ww: Array1<f64>,
    pxx: Array2<f64>,
}

impl Tracker {
    pub fn new (app_state: SharedAppState) -> Tracker {
        let mut pxx = Array::zeros((13, 13));
        for idx in 0..3 {
            pxx[[idx, idx]] = 4e-4;
        }

        Tracker {
            app_state,
            camera: Camera::new(),
            features: vec![],
            last_time: SystemTime::now(),
            rw: Array::zeros(3),
            rotwr: Array::eye(3),
            vw: Array::zeros(3),
            ww: Array::zeros(3),
            pxx,
        }
    }

    pub fn get_xv (&self) -> Array1<f64> {
        let quat = convert_matrix_to_quaternion(&self.rotwr);

        arr1(&[
            self.rw[0],
            self.rw[1],
            self.rw[2],
            quat[0],
            quat[1],
            quat[2],
            quat[3],
            self.vw[0],
            self.vw[1],
            self.vw[2],
            self.ww[0],
            self.ww[1],
            self.ww[2],
        ])
    }

    // V = 0, OMEGA = 0
    pub fn get_model_prediction (&self) -> Array1<f64> {
        let quat = convert_matrix_to_quaternion(&self.rotwr);

        let duration = self.last_time.elapsed().unwrap();
        let delta_t = (duration.as_secs() as f64) + (duration.subsec_micros() as f64) / 1e6;

        let angular_displacement = self.ww.mapv(|e| e * delta_t);
        let rot_mat = get_rotation_matrix_from_angular_displacement(&angular_displacement);
        let qupd = convert_matrix_to_quaternion(&rot_mat);
        let qnew = quaternion_product(&quat, &qupd);

        arr1(&[
            self.rw[0] + self.vw[0] * delta_t,
            self.rw[1] + self.vw[1] * delta_t,
            self.rw[2] + self.vw[2] * delta_t,
            qnew[0],
            qnew[1],
            qnew[2],
            qnew[3],
            self.vw[0],
            self.vw[1],
            self.vw[2],
            self.ww[0],
            self.ww[1],
            self.ww[2],
        ])
    }

    /**
     *
     * Detect first few features and set them at a distance of 0.75m.
     *
     */
    pub fn process_image (&mut self, mut image: RgbaImage) -> AppState {
        // Match old features in new image using
        // Normalized sum of square difference correlation
        let mut ellipses = Vec::with_capacity(self.features.len());

        for feature in &mut self.features {
            let h = feature.get_h();

            // HOW TO DO EKF HERE???
            ellipses.push((h));
        }

        // Detect new features to replace old ones that have become stale
        if self.features.len() < NUM_FEATURES {
            let size = 100.0;

            let detections = Detection::detect(
                &mut image,
                (PRINCIPAL_POINT_X - size / 2.0) as u32,
                (PRINCIPAL_POINT_Y - size / 2.0) as u32,
                size as u32,
                size as u32,
            );

            let maybe_best_detection = detections
                .into_iter()
                .fold(None, |acc: Option<Detection>, detection| {
                    // DO SOME CHECKS TO PREVENT EDGE CASES
                    // LIKE DETECTIONS TOO CLOSE
                    if let Some(curr_detection) = acc {
                        if curr_detection.score < detection.score {
                            Some(detection)
                        } else {
                            Some(curr_detection)
                        }
                    } else {
                        Some(detection)
                    }
                });

            if let Some(mut best_detection) = maybe_best_detection {
                if self.features.len() == 0 {
                    /*
                    let default_distance = 0.75;
                    let h = best_feature.get_h();
                    let hrl_hat = self.camera.unproject(&h);
                    let hrl = hrl_hat.mapv(|e| e * default_distance);
                    let yw = &self.rw + &self.rotwr.dot(&hrl);

                    best_feature.pos = Some(
                        arr1(&[yw[0], yw[1], yw[2], - hrl_hat[0], - hrl_hat[1], - hrl_hat[2]])
                    );
                    */
                }

                // self.features.push(best_feature);
            }
        }

        self.last_time = SystemTime::now();

        AppState::TrackingState {
            image,
            features: self.features.clone(),
        }
    }

    pub fn frame_to_image (
        frame: &Frame,
    ) -> Result<RgbaImage, Box<dyn Error>> {
        let width = frame.width();
        let height = frame.height();

        let new_frame = frame.to_rgb()?;
        let data = new_frame.to_bytes();
        let image: RgbaImage = RgbImage::from_raw(
            width,
            height,
            data.to_vec(),
        ).ok_or("This shouldn't happen")?.convert();

        Ok(image)
    }

    pub fn process_frame (
        &mut self,
        frame: &Frame
    ) {
        let maybe_image = Self::frame_to_image(frame);
        let maybe_app_state = maybe_image.map(|image| self.process_image(image));
        match maybe_app_state {
            Err(x) => println!("{:#?}", x),
            Ok(x) => {
                let mut data = Mutex::lock(&self.app_state).unwrap();
                *data = Some(x);
            }
        };
    }
}
