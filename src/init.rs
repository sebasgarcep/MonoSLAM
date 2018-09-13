use camera::Camera;
use constants::{
    BLOCKSIZE,
    CENTER_SIZE,
    CORRELATION_SIGMA_THRESHOLD,
    CORRELATION_THRESHOLD,
    FOCAL_LENGTH_X,
    FOCAL_LENGTH_Y,
    INIT_MAX_DISTANCE,
    INIT_MIN_DISTANCE,
    LOW_SIGMA_PENALTY,
    NUM_INIT_PARTICLES,
    NUM_SIGMA,
    PADDING,
    PRINCIPAL_POINT_X,
    PRINCIPAL_POINT_Y,
    PROBABILITY_RATIO_THRESHOLD,
    PROBABILITY_THRESHOLD,
};
use feature::Feature;
use im::{ConvertBuffer, RgbaImage, RgbImage};
use ndarray::{arr1, arr2, Array, Array1, Array2, Axis, ShapeBuilder};
use ndarray_linalg::solve::{Determinant, Inverse};
use ndarray_linalg::norm::Norm;
use state::{AppState, SharedAppState};
use std;
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::sync::Mutex;
use utils::get_grayscale_matrix_from_image;
use uvc::Frame;

struct Particle {
    lambda: f64,
    probability: f64,
    u: u32,
    v: u32,
}

impl Particle {
    pub fn new (lambda: f64, probability: f64, u: u32, v: u32) -> Particle {
        Particle {
            lambda,
            probability,
            u,
            v,
        }
    }

    pub fn get_lambda (&self) -> f64 {
        self.lambda
    }

    pub fn get_yw (
        &self,
        rw: &Array1<f64>,
        rotrw: &Array2<f64>,
        hr_hat: &Array1<f64>
    ) -> Array1<f64> {
        let hr = hr_hat.mapv(|e| e * self.lambda);
        rw + &rotrw.dot(&hr)
    }

    pub fn get_hrl (
        &self,
        rw: &Array1<f64>,
        rotrw: &Array2<f64>,
        yw: &Array1<f64>,
    ) -> Array1<f64> {
        let deviation = yw - rw;
        rotrw.dot(&deviation)
    }

    pub fn get_h (&self) -> Array1<f64> {
        arr1(&[self.u as f64, self.v as f64])
    }
}

pub struct Initializer {
    app_state: SharedAppState,
    camera: Camera,
    particles: Option<Vec<Particle>>,
    prev_grayscale_image: Option<Array2<f64>>,
    pxvxv: Array2<f64>,
    skipped_frames: u32,
}

impl Initializer {
    pub fn new (app_state: SharedAppState) -> Initializer {
        let mut pxvxv = Array::zeros((13, 13));
        for idx in 0..3 {
            pxvxv[[idx, idx]] = 0.0004;
        }

        Initializer {
            app_state,
            camera: Camera::new(),
            particles: None,
            prev_grayscale_image: None,
            pxvxv,
            skipped_frames: 0,
        }
    }

    pub fn calculate_drqadq (q: &Array1<f64>, a: &Array1<f64>) -> Array2<f64> {
        let q0 = q[0];
        let qx = q[1];
        let qy = q[2];
        let qz = q[3];

        let drdq0 = arr2(&[
            [2.0 * q0, - 2.0 * qz, 2.0 * qy],
            [2.0 * qz, 2.0 * q0, - 2.0 * qx],
            [- 2.0 * qy, 2.0 * qx, 2.0 * q0],
        ]);
        let drdqx = arr2(&[
            [2.0 * qx, 2.0 * qy, 2.0 * qz],
            [2.0 * qy, - 2.0 * qx, - 2.0 * q0],
            [2.0 * qz, 2.0 * q0, - 2.0 * qx],
        ]);
        let drdqy = arr2(&[
            [- 2.0 * qy, 2.0 * qx, 2.0 * q0],
            [2.0 * qx, 2.0 * qy, 2.0 * qz],
            [- 2.0 * q0, 2.0 * qz, - 2.0 * qy],
        ]);
        let drdqz = arr2(&[
            [- 2.0 * qz, - 2.0 * q0, 2.0 * qx],
            [2.0 * q0, - 2.0 * qz, 2.0 * qy],
            [2.0 * qx, 2.0 * qy, 2.0 * qz],
        ]);

        let mut drqadq = Array::zeros((3, 4));
        drqadq.column_mut(0).assign(&drdq0.dot(a));
        drqadq.column_mut(1).assign(&drdqx.dot(a));
        drqadq.column_mut(2).assign(&drdqy.dot(a));
        drqadq.column_mut(3).assign(&drdqz.dot(a));

        drqadq
    }

    pub fn dqidqi (qi: f64, qq: f64) -> f64 {
        (1.0 - qi.powi(2) / qq.powi(2)) / qq
    }

    pub fn dqidqj (qi: f64, qj: f64, qq: f64) -> f64 {
        - qi * qj / qq.powi(3)
    }

    pub fn calculate_dvnormdv (v: &Array1<f64>) -> Array2<f64> {
        let vx = v[0];
        let vy = v[1];
        let vz = v[2];
        let vv = vx.powi(2) + vy.powi(2) + vz.powi(2);
        arr2(&[
            [Self::dqidqi(vx, vv), Self::dqidqj(vx, vy, vv), Self::dqidqj(vx, vz, vv)],
            [Self::dqidqj(vy, vx, vv), Self::dqidqi(vy, vv), Self::dqidqj(vy, vz, vv)],
            [Self::dqidqj(vz, vx, vv), Self::dqidqj(vz, vy, vv), Self::dqidqi(vz, vv)],
        ])
    }

    pub fn inside_relative (si_inv: &Array2<f64>, uu: u32, uv: u32) -> bool {
        let u = uu as f64;
        let v = uv as f64;
        let pos = si_inv[[0, 0]] * u.powi(2) +
            2.0 * si_inv[[0, 1]] * u * v +
            si_inv[[1, 1]] * v.powi(2);
        pos < NUM_SIGMA.powi(2)
    }

    pub fn get_image_correlation (
        prev_image_normalized: &Array2<f64>,
        prev_ui: usize,
        prev_vi: usize,
        prev_uf: usize,
        prev_vf: usize,
        curr_image_normalized: &Array2<f64>,
        curr_ui: usize,
        curr_vi: usize,
        curr_uf: usize,
        curr_vf: usize,
    ) -> f64 {
        let prev_image_win = prev_image_normalized.slice(
            s![prev_ui..(prev_uf + 1), prev_vi..(prev_vf + 1)]
        );
        let curr_image_win = curr_image_normalized.slice(
            s![curr_ui..(curr_uf + 1), curr_vi..(curr_vf + 1)]
        );
        let mut diff = &prev_image_win - &curr_image_win;
        diff.mapv_inplace(|e| e.powi(2));
        diff.scalar_sum() // FIXME: return sdimage
    }

    pub fn normalize_image (
        image: &Array2<f64>,
    ) -> Array2<f64> {
        let mean = image.scalar_sum() / (image.len() as f64);
        let mut result = image.clone();
        result.mapv_inplace(|e| e - mean);
        let norm = result.mapv(|e| e.powi(2)).scalar_sum().sqrt();
        result.mapv_inplace(|e| e / norm);
        result
    }

    pub fn search_ellipses (
        prev_image: &Array2<f64>,
        curr_image: &Array2<f64>,
        ellipses: &Vec<(Array1<f64>, Array2<f64>, Array2<f64>, f64)>
    ) -> Vec<(u32, u32, f64)> {
        let shape = prev_image.shape();
        let width = shape[0] as i32;
        let height = shape[1] as i32;
        let halfblocksize = (BLOCKSIZE / 2) as i32;
        let totalblocksize = BLOCKSIZE as i32;
        let mut correlation_map = HashMap::new();
        let prev_image_normalized = Self::normalize_image(prev_image);
        let curr_image_normalized = Self::normalize_image(curr_image);
        let mut result = vec![];

        for ellipse in ellipses {
            let (hi, _, si_inv, _) = ellipse;
            let si_diag_prod = si_inv[[0, 1]].powi(2);
            let halfwidth = (NUM_SIGMA / (si_inv[[0, 0]] - si_diag_prod / si_inv[[1, 1]]).sqrt()) as i32;
            let halfheight = (NUM_SIGMA / (si_inv[[1, 1]] - si_diag_prod / si_inv[[0, 0]]).sqrt()) as i32;
            let mut urelstart = - halfwidth;
            let mut urelfinish = halfwidth;
            let mut vrelstart = - halfheight;
            let mut vrelfinish = halfheight;
            let ucentre = hi[0] as i32;
            let vcentre = hi[1] as i32;

            if ucentre + urelstart - halfblocksize < 0 {
                urelstart = halfblocksize - ucentre;
            }

            if ucentre + urelfinish - halfblocksize > width - totalblocksize {
                urelfinish = width - totalblocksize - ucentre + halfblocksize;
            }

            if vcentre + vrelstart - halfblocksize < 0 {
                vrelstart = halfblocksize - vcentre;
            }

            if vcentre + vrelfinish - halfblocksize > height - totalblocksize {
                vrelfinish = height - totalblocksize - vcentre + halfblocksize;
            }

            let mut corrmax = std::f64::INFINITY;
            let mut res_u = 0;
            let mut res_v = 0;

            /* Do the search */
            for urel in urelstart..(urelfinish + 1) {
                for vrel in vrelstart..(vrelfinish + 1) {
                    if Self::inside_relative(si_inv, urel as u32, vrel as u32) {
                        // We are inside ellipse
                        // Has this place been searched before?
                        let pos = (ucentre + urel, vcentre + vrel);
                        let computation_performed = correlation_map.contains_key(&pos);

                        let corr: f64 = if computation_performed {
                            *correlation_map.get(&pos).unwrap()
                        } else {
                            let corr_result = Self::get_image_correlation(
                                &prev_image_normalized,
                                (ucentre - halfblocksize) as usize,
                                (vcentre - halfblocksize) as usize,
                                (ucentre + halfblocksize) as usize,
                                (vcentre + halfblocksize) as usize,
                                &curr_image_normalized,
                                (ucentre + urel - halfblocksize) as usize,
                                (vcentre + vrel - halfblocksize) as usize,
                                (ucentre + urel + halfblocksize) as usize,
                                (vcentre + vrel + halfblocksize) as usize,
                            );

                            correlation_map.insert(pos, corr_result);

                            /* FIXME: ADD THIS
                            if sdimage < CORRELATION_SIGMA_THRESHOLD {
                                corr_result += LOW_SIGMA_PENALTY;
                            }
                            */

                            corr_result
                        };

                        if corr <= corrmax {
                            corrmax = corr;
                            res_u = urel + ucentre;
                            res_v = vrel + vcentre;
                        }
                    }
                }
            }

            /* FIXME: THIS HAS NOT BEEN PORTED
            if corrmax > CORRELATION_THRESHOLD {
                Correlation not good enough.
            } else {
                Correlation good enough.
            }
            */

            result.push((res_u as u32, res_v as u32, corrmax));
        }

        result
    }

    /**
     *
     * If points have not been initialized:
     * 1. Find best feature in image.
     * 2. Construct a ray from the camera (R = I, T = 0) to that feature and
     *    create NUM_INIT_PARTICLES along that ray that are evenly spaced and
     *    have uniform probability.
     *
     * If points have been initialized:
     * 1. predict_partially_initialised_feature_measurements
     * 2. measure_feature_with_multiple_priors
     * 3. update_partially_initialised_feature_probabilities
     * 4. Check out how the particle collapse is going.
     *
     */
    pub fn process_image (&mut self, image: RgbaImage) -> AppState {
        let width = image.width();
        let height = image.height();

        let is_initialized = self.particles.is_some();
        if is_initialized {
            let mut particles = self.particles.as_mut().unwrap();
            let rw = Array::zeros(3);
            let rotrw = Array::eye(3);
            let rotwr = Array::eye(3);

            let pxvxv = &self.pxvxv;

            let mut dxpdxv = Array::zeros((13, 13));
            for idx in 0..7 {
                dxpdxv[[idx, idx]] = 1.0;
            }

            let qwr = arr1(&[1.0, 0.0, 0.0, 0.0]);
            let qrw = arr1(&[1.0, 0.0, 0.0, 0.0]);

            let dqrwdq = arr2(&[
                [1.0, 0.0, 0.0, 0.0],
                [0.0, - 1.0, 0.0, 0.0],
                [0.0, 0.0, - 1.0, 0.0],
                [0.0, 0.0, 0.0, - 1.0],
            ]);

            let dzeroedyidyi = &rotrw;
            let dzeroedyidr = rotrw.mapv(|e| - 1.0 * e);
            let mut ellipses = vec![];

            for particle in particles.iter() {
                let hi = particle.get_h();
                let lambda = particle.get_lambda();
                let hrli_hat = self.camera.unproject(&hi);
                let hrli = hrli_hat.mapv(|e| e * lambda);

                let drqhdq = Self::calculate_drqadq(&qwr, &hrli_hat);

                let mut dyidxp = Array::zeros((6, 7));
                for idx in 0..3 {
                    dyidxp[[idx, idx]] = 1.0;
                }
                dyidxp.slice_mut(s![3.., 3..]).assign(&drqhdq);

                let dyidxv = dyidxp.dot(&dxpdxv);
                let pxvyi = pxvxv.dot(&dyidxv.t());
                let ri = self.camera.measurement_noise(&hi);
                let dhrlidzeroedyi = arr2(&[
                    [1.0, 0.0, 0.0, lambda, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, lambda, 0.0],
                    [0.0, 0.0, 1.0, 0.0, 0.0, lambda],
                ]);

                let yi = particle.get_yw(&rw, &rotrw, &hrli_hat);
                let diff_yirw = &yi - &rw;
                let dzeroedyidqrw = Self::calculate_drqadq(&qrw, &diff_yirw);

                let dzeroedyidq = dzeroedyidqrw.dot(&dqrwdq);

                let mut dzeroedyidxp = Array::zeros((3, 7));
                dzeroedyidxp.slice_mut(s![.., ..3]).assign(&dzeroedyidr);
                dzeroedyidxp.slice_mut(s![.., 3..]).assign(&dzeroedyidq);

                let dhidhrli = self.camera.projection_jacobian(&hrli);
                let dhidxp = dhidhrli.dot(&dhrlidzeroedyi).dot(&dzeroedyidxp);
                let dhidxv = dhidxp.dot(&dxpdxv);
                let dhidyi = dhidhrli.dot(&dhrlidzeroedyi).dot(dzeroedyidyi);

                let dhrli_hatdhrli = Self::calculate_dvnormdv(&hrli);
                let dhwli_hatdhi = rotwr.dot(&dhrli_hatdhrli).dot(&self.camera.unprojection_jacobian(&hi));

                let mut dyidhi = Array::zeros((6, 2));
                dyidhi.slice_mut(s![3.., ..]).assign(&dhwli_hatdhi);

                let pyiyi = dyidxv.dot(pxvxv).dot(&dyidxv.t()) +
                    dyidhi.dot(&ri).dot(&dyidhi.t());

                let temp = dhidxv.dot(&pxvyi).dot(&dhidyi);
                let si =
                    &dhidxv.dot(pxvxv).dot(&dhidxv.t()) +
                    &temp +
                    &temp.t() +
                    &dhidyi.dot(&pyiyi).dot(&dhidyi.t()) +
                    &ri;

                let si_inv = si.inv().unwrap();
                let det_si = si.det().unwrap();

                ellipses.push((hi, si, si_inv, det_si));
            }

            let curr_grayscale_image = get_grayscale_matrix_from_image(
                &image,
                0,
                0,
                image.width(),
                image.height(),
            );

            let search_results = Self::search_ellipses(
                self.prev_grayscale_image.as_ref().unwrap(),
                &curr_grayscale_image,
                &ellipses
            );

            for idx in 0..ellipses.len() {
                let (hi, _, si_inv, det_si) = &ellipses[idx];
                let (u, v, _) = search_results[idx];
                let z = arr1(&[u as f64, v as f64]);
                let deviation = &z - hi;
                let conjugate = &deviation.t().dot(si_inv).dot(&deviation);
                let probability = (- 0.5 * conjugate).exp() / (2.0 * std::f64::consts::PI * det_si);
                let particle = &mut particles[idx];
                particle.probability = probability;
            }

            let mut prob_mean: f64;

            // Normalize all probabilities
            prob_mean = particles.iter().fold(0.0, |acc, particle| acc + particle.probability);
            for particle in particles.iter_mut() {
                particle.probability = particle.probability / prob_mean;
            }
            particles.retain(|particle| particle.probability > PROBABILITY_THRESHOLD);

            // Renormalize
            prob_mean = particles.iter().fold(0.0, |acc, particle| acc + particle.probability);
            for particle in particles.iter_mut() {
                particle.probability = particle.probability / prob_mean;
            }

            let mut mean = 0.0;
            let mut ex2 = 0.0;
            for particle in particles.iter() {
                mean += particle.probability * particle.lambda;
                ex2 += particle.probability * particle.lambda.powi(2);
            }
            let covariance = ex2 - mean.powi(2);
            let ratio = covariance.sqrt() / mean;

            if ratio < PROBABILITY_RATIO_THRESHOLD /* FIXME: && self.particles.len() > min number of particles */ {
                // Initialize feature
                println!("Feature Initialized: {}", mean);
                panic!();
            }

            self.prev_grayscale_image = Some(curr_grayscale_image);
        } else if self.skipped_frames >= 90 {
            let mut points = Vec::with_capacity(NUM_INIT_PARTICLES);

            let probability = 1.0 / (NUM_INIT_PARTICLES as f64);
            for idx in 0..NUM_INIT_PARTICLES {
                let ratio = (idx as f64) / ((NUM_INIT_PARTICLES - 1) as f64);
                let lambda = INIT_MIN_DISTANCE + ratio * (INIT_MAX_DISTANCE - INIT_MIN_DISTANCE);

                let features = Feature::detect(
                    &image,
                    (PRINCIPAL_POINT_X as u32) - CENTER_SIZE / 2,
                    (PRINCIPAL_POINT_Y as u32) - CENTER_SIZE / 2,
                    CENTER_SIZE,
                    CENTER_SIZE,
                );

                let init_feature = features
                    .into_iter()
                    .fold(None, |acc: Option<Feature>, feat| {
                        if let Some(best_feat) = acc {
                            if best_feat.score < feat.score {
                                Some(feat)
                            } else {
                                Some(best_feat)
                            }
                        } else {
                            Some(feat)
                        }
                    })
                    .unwrap();

                let particle = Particle::new(
                    lambda,
                    probability,
                    init_feature.u,
                    init_feature.v,
                );

                points.push(particle);
            }

            self.particles = Some(points);
            self.prev_grayscale_image = Some(
                get_grayscale_matrix_from_image(
                    &image,
                    0,
                    0,
                    image.width(),
                    image.height(),
                )
            );
        } else {
            self.skipped_frames += 1;
        }

        AppState::InitState {
            image,
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

    pub fn process_frame (&mut self, frame: &Frame) {
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
