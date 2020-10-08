use calculus::CalculusContext;
use constants::NUM_FEATURES;
use detection::Detection;
use ellipse_search::ellipse_search;
use feature::Feature;
use im::RgbaImage;
use ndarray::{arr1, Array};
use ndarray_linalg::{Determinant, Inverse};
use state::{AppState, SharedAppState};
use std::rc::Rc;
use std::sync::Mutex;
use std::time::{SystemTime};
use typedefs::{SharedMatrix, SharedVector};

pub struct Tracker {
    app_state: SharedAppState,
    context: CalculusContext,
    features: Vec<Feature>,
    last_time: SystemTime,
    rw: SharedVector,
    qwr: SharedVector,
    vw: SharedVector,
    ww: SharedVector,
    pxx: SharedMatrix,
}

impl Tracker {
    pub fn new (app_state: SharedAppState) -> Tracker {
        let mut pxx = Array::zeros((13, 13));
        for idx in 0..3 {
            pxx[[idx, idx]] = 4e-4;
        }

        Tracker {
            app_state,
            context: CalculusContext::new(),
            features: vec![],
            last_time: SystemTime::now(),
            rw: Rc::new(Array::zeros(3)),
            qwr: Rc::new(arr1(&[1.0, 0.0, 0.0, 0.0])),
            vw: Rc::new(Array::zeros(3)),
            ww: Rc::new(Array::zeros(3)),
            pxx: Rc::new(pxx),
        }
    }

    // V = 0, OMEGA = 0
    /*
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
    */

    fn add_features (&mut self, image: &mut RgbaImage) {
        let size = 100.0;

        let detections = Detection::detect(
            image,
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

    /**
     *
     * Detect first few features and set them at a distance of 0.75m.
     *
     */
    pub fn process_image (&mut self, mut image: RgbaImage) -> AppState {
        // DO PREDICTION HERE

        self.context.set_robot_state(
            self.rw.clone(),
            self.qwr.clone(),
            self.vw.clone(),
            self.ww.clone(),
            self.pxx.clone(),
        );

        // Match old features in new image using
        // Normalized sum of square difference correlation
        let num_features = self.features.len();
        let mut ellipses = Vec::with_capacity(num_features);

        for feature in &mut self.features {
            let h = feature.get_h();
            let yi = feature.get_yi();
            let pxy = feature.get_pxy();
            let pyy = feature.get_pyy();

            self.context.set_feature_state(
                h.clone(),
                yi.clone(),
                pxy.clone(),
                pyy.clone(),
            );

            let si = self.context.get_si();
            let si_inv = si.inv().unwrap();
            let det_si = si.det().unwrap();

            ellipses.push((h.as_ref().clone(), si, si_inv, det_si));
        }

        let prev_image = unimplemented!();
        let curr_image = unimplemented!();
        let results = ellipse_search(&prev_image, &curr_image, &ellipses);

        for idx in 0..num_features {
            let feature = &self.features[idx];
            let ellipse = &ellipses[idx];
            let result = &results[idx];

            // DO EKF HERE
        }

        // Detect new features to replace old ones that have become stale
        if self.features.len() < NUM_FEATURES {
            self.add_features(&mut image);
        }

        self.last_time = SystemTime::now();

        AppState::TrackingState {
            image,
            features: self.features.clone(),
        }
    }

    pub fn process_frame (&mut self, image: RgbaImage) {
        let app_state = self.process_image(image);
        let mut data = Mutex::lock(&self.app_state).unwrap();
        *data = Some(app_state);
    }
}
