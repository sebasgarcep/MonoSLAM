use calculus;
use camera_model::WideAngleCameraModel;
use constants::{ANGULAR_VELOCITY_NOISE, LINEAR_VELOCITY_NOISE};
use detection::Detection;
use feature::{FullFeature, PartialFeature};
use nalgebra::{
    DMatrix, DVector, Quaternion, Matrix3, Matrix6, MatrixMN,
    SliceStorage, U1, U2, U3, U6, U13, UnitQuaternion, Vector, VectorN,
};
use rand::Rng;
use serde::Deserialize;
use std::fs::File;
use std::io::BufReader;
use utils::{ellipse_search, image_to_matrix, matrix_set_block, unit_quaternion_from_angular_displacement};

#[derive(Deserialize)]
struct FeatureInit {
    yi: Vec<f64>,
    #[allow(dead_code)] xp_orig: Vec<f64>,
}

#[derive(Deserialize)]
struct AppStateInit {
    xv: Vec<f64>,
    pxx: Vec<Vec<f64>>,
    features: Vec<FeatureInit>,
}

pub struct AppState {
    pub xv: VectorN<f64, U13>,
    pub pxx: MatrixMN<f64, U13, U13>,
    pub camera_model: WideAngleCameraModel,
    pub full_features: Vec<FullFeature>,
    pub partial_features: Vec<PartialFeature>,
}

impl AppState {
    pub fn from_json(
        json_path: &str,
        patch_path_vec: Vec<&str>,
        camera_model: WideAngleCameraModel,
    ) -> Self {
        // Read JSON file
        let file = File::open(json_path).unwrap();
        let reader = BufReader::new(file);
        let app_state_init: AppStateInit = serde_json::from_reader(reader).unwrap();

        // Initialize state
        let xv = VectorN::<f64, U13>::from_iterator(app_state_init.xv);

        let mut pxx = MatrixMN::<f64, U13, U13>::zeros();
        for i in 0..13 {
            for j in 0..13 {
                pxx[(i, j)] = app_state_init.pxx[i][j];
            }
        }

        let num_active_features = app_state_init.features.len();
        let mut full_features = vec![];
        for idx in 0..num_active_features {
            let y = VectorN::<f64, U3>::from_iterator(app_state_init.features[idx].yi.clone());
            let img = image::open(patch_path_vec[idx]).unwrap().to_rgba();
            let patch = image_to_matrix(&img);
            let feature = FullFeature::new_from_position(&full_features, y, patch);
            full_features.push(feature);
        }

        let partial_features = vec![];

        AppState { xv, pxx, camera_model, full_features, partial_features }
    }

    pub fn position(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, U13>> {
        self.xv.fixed_rows::<U3>(0)
    }

    pub fn orientation(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_quaternion(Quaternion::new(self.xv[3], self.xv[4], self.xv[5], self.xv[6]))
    }

    pub fn linear_velocity(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, U13>> {
        self.xv.fixed_rows::<U3>(7)
    }

    pub fn angular_velocity(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, U13>> {
        self.xv.fixed_rows::<U3>(10)
    }

    pub fn full_state_size(&self) -> usize {
        13 + 3 * self.full_features.len()
    }

    pub fn num_active_features(&self) -> usize {
        self.full_features.len()
    }

    pub fn full_state(&self) -> DVector<f64> {
        let state_size = self.full_state_size();
        let num_active_features = self.num_active_features();

        let mut x_full = DVector::<f64>::zeros(state_size);
        matrix_set_block(&mut x_full, 0, 0, &self.xv);
        for idx in 0..num_active_features {
            let ref full_feature = self.full_features[idx];
            matrix_set_block(&mut x_full, 13 + 3 * idx, 0, &full_feature.feature.y);
        }

        x_full
    }

    pub fn full_covariance(&self) -> DMatrix<f64> {
        let state_size = self.full_state_size();
        let num_active_features = self.num_active_features();

        let mut p_full = DMatrix::<f64>::zeros(state_size, state_size);
        matrix_set_block(&mut p_full, 0, 0, &self.pxx);
        for idx in 0..num_active_features {
            let ref full_feature = self.full_features[idx];
            matrix_set_block(&mut p_full, 0, 13 + 3 * idx, &full_feature.feature.pxy);
            matrix_set_block(&mut p_full, 13 + 3 * idx, 0, &full_feature.feature.pxy.transpose());
            for jdx in 0..full_feature.pyiyj.len() {
                let ref pyiyj = full_feature.pyiyj[jdx];
                matrix_set_block(&mut p_full, 13 + 3 * idx, 13 + 3 * jdx, &pyiyj);
                matrix_set_block(&mut p_full, 13 + 3 * jdx, 13 + 3 * idx, &pyiyj.transpose());
            }
        }

        p_full
    }

    pub fn update_particles(&mut self, mat: &DMatrix<f64>) {
        let rw = self.position().clone_owned();
        let qwr = self.orientation();
        let qrw = qwr.inverse();
        let rrw = qrw.to_rotation_matrix();
        for partial_feature in &mut self.partial_features {
            partial_feature.update_particles(
                &rw,
                &qrw,
                &rrw,
                &self.pxx,
                &self.camera_model,
                &mat,
            );
        }
    }

    pub fn consume_detection(&mut self, mat: &DMatrix<f64>, detection: &Detection) {
        // FIXME: Remove this condition
        if self.partial_features.len() == 0 {
            let partial_feature = PartialFeature::new(&self, mat, detection);
            self.partial_features.push(partial_feature);
        }
    }

    pub fn predict(&mut self, delta_t: f64) {
        // Get current state
        let rw = self.position();
        let qwr = self.orientation();
        let vw = self.linear_velocity();
        let wr = self.angular_velocity();

        // Project to next state
        // Assume linear and angular acceleration are 0
        let rw_new = rw + vw * delta_t;
        let qwr_new = qwr * unit_quaternion_from_angular_displacement(&(wr * delta_t));
        let mut xv_new = self.xv.clone();
        matrix_set_block(&mut xv_new, 0, 0, &rw_new);
        xv_new[3] = qwr_new.w;
        xv_new[4] = qwr_new.i;
        xv_new[5] = qwr_new.j;
        xv_new[5] = qwr_new.k;

        // Calculate the covariance of the impulse vector
        let mut pn = Matrix6::<f64>::zeros();
        matrix_set_block(&mut pn, 0, 0, &(Matrix3::identity() * LINEAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2)));
        matrix_set_block(&mut pn, 3, 3, &(Matrix3::identity() * ANGULAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2)));

        // Calculate the Jacobian of the process vector with respect to the impulse vector
        // Let qt = q((wr + omegar) * delta_t).
        let dqwr_new_dqt = calculus::dqwr_new_dqt(&qwr);
        let dqt_domegar = calculus::dqt_dwr(&wr, delta_t);
        let dqwrnew_domegar = dqwr_new_dqt * dqt_domegar;
        let mut dfv_dn = MatrixMN::<f64, U13, U6>::zeros();
        matrix_set_block(&mut dfv_dn, 0, 0, &(Matrix3::identity() * delta_t));
        matrix_set_block(&mut dfv_dn, 3, 3, &dqwrnew_domegar);
        matrix_set_block(&mut dfv_dn, 7, 0, &Matrix6::identity());

        // Calculate the process noise covariance
        let qv = dfv_dn * pn * dfv_dn.transpose();

        // Calculate the jacobian of the motion model
        let dqwrnew_dqwr = calculus::dqwr_new_dqwr(&qwr);
        let dqwrnew_dwr = dqwrnew_domegar;
        let mut dfv_dxv = MatrixMN::<f64, U13, U13>::zeros();
        matrix_set_block(&mut dfv_dxv, 0, 0, &Matrix3::identity());
        matrix_set_block(&mut dfv_dxv, 0, 7, &(Matrix3::identity() * delta_t));
        matrix_set_block(&mut dfv_dxv, 3, 3, &dqwrnew_dqwr);
        matrix_set_block(&mut dfv_dxv, 3, 10, &dqwrnew_dwr);
        matrix_set_block(&mut dfv_dxv, 7, 7, &Matrix6::identity());

        // Update state and covariance matrix using model predictions
        // xv
        self.xv = xv_new;

        // Pxx
        let pxx_new = &dfv_dxv * &self.pxx * &dfv_dxv.transpose() + &qv;
        self.pxx = pxx_new;

        // Pxy
        for full_feature in &mut self.full_features {
            full_feature.feature.pxy = &dfv_dxv * &full_feature.feature.pxy;
        }
        for partial_feature in &mut self.partial_features {
            partial_feature.feature.pxy = &dfv_dxv * &partial_feature.feature.pxy;
        }
    }

    // FIXME: Normalization should be done on the quaternion after data assimilation
    // This normalization should propagate also to the state uncertainty.
    pub fn measure(&mut self, mat: DMatrix<f64>) {
        let state_size = self.full_state_size();
        let num_active_features = self.num_active_features();

        // Construct the full state vector
        let x_full = self.full_state();

        // Construct the full covariance matrix
        let p_full = self.full_covariance();

        // The matrix Rk is simply sigma_R^2 * I, where sigma_R = 1 is the camera error due to discretization errors.
        let mut r = DMatrix::<f64>::zeros(2 * num_active_features, 2 * num_active_features);

        // Calculate Jacobian of observation operator.
        let mut h = DMatrix::<f64>::zeros(2 * num_active_features, state_size);
        let rw = self.position();
        let qwr = self.orientation();
        let qrw = qwr.inverse();
        let rot_rw = qrw.to_rotation_matrix();
        let dqrw_dqwr = calculus::dqinv_dq();
        for idx in 0..num_active_features {
            let ref full_feature = self.full_features[idx];
            let ref yi_w = full_feature.feature.y;
            let yi_w_minus_rw = yi_w - rw;
            let yi_r = rot_rw * &yi_w_minus_rw;
            let h_i = self.camera_model.project(&(yi_w - rw));
            // Calculate R_i
            let r_i = self.camera_model.measurement_noise(&h_i);
            matrix_set_block(&mut r, 2 * idx, 2 * idx, &r_i);

            let dzi_dyi_r = self.camera_model.project_jacobian(&yi_r);
            // Calculate dzi_dyi_r
            let dzi_dyi_w = dzi_dyi_r * rot_rw;
            matrix_set_block(&mut h, 2 * idx, 13 + 3 * idx, &dzi_dyi_w);
            // Calculate dzi_drw
            let dzi_drw = -dzi_dyi_w;
            matrix_set_block(&mut h, 2 * idx, 0, &dzi_drw);
            // Calculate dzi_dqwr
            let dyi_r_dqrw = calculus::dqv_dq(&qrw, &yi_w_minus_rw);
            let dzi_dqwr = dzi_dyi_r * dyi_r_dqrw * dqrw_dqwr;
            matrix_set_block(&mut h, 2 * idx, 3, &dzi_dqwr);
        }

        // Calculate the innovation covariance matrix
        let s = &h * &p_full * &h.transpose() + &r;

        // Calculate innovation vector
        let mut y_innov = DVector::<f64>::zeros(2 * num_active_features);
        for idx in 0..num_active_features {
            let ref full_feature = self.full_features[idx];
            let ref yi_w = full_feature.feature.y;
            let h_i = self.camera_model.project(&(yi_w - rw));
            let (h_i_x, h_i_y) = (h_i[0].round() as usize, h_i[1].round() as usize);

            // Calculate best x, y
            let s_i = s.fixed_slice::<U2, U2>(2 * idx, 2 * idx);
            let (best_x, best_y, _) = ellipse_search(
                h_i_x,
                h_i_y,
                &s_i,
                &mat,
                &full_feature.feature.patch,
            );

            // Set the innovation vector = measurements - camera projection of current state of features
            y_innov[2 * idx] = (best_x as f64) - (h_i_x as f64);
            y_innov[2 * idx + 1] = (best_y as f64) - (h_i_y as f64);
        }

        let s_inv = &s.pseudo_inverse(1e-6).unwrap();
        let k = &p_full * &h.transpose() * s_inv;
        let x_next = &x_full + &k * &y_innov;
        let mut p_next = (DMatrix::<f64>::identity(state_size, state_size) - &k * &h) * &p_full;
        // Let's enforce symmetry of covariance matrix
        p_next = 0.5 * &p_next + 0.5 * &p_next.transpose();

        // Update state and covariance matrix using Kalman Filter corrections
        self.xv = x_next.fixed_rows::<U13>(0).clone_owned();
        for idx in 0..num_active_features {
            let ref mut full_feature = self.full_features[idx];
            full_feature.feature.y = x_next.fixed_rows::<U3>(13 + 3 * idx).clone_owned();
        }

        self.pxx = p_next.fixed_slice::<U13, U13>(0, 0).clone_owned();
        for idx in 0..num_active_features {
            let ref mut full_feature = self.full_features[idx];
            full_feature.feature.pxy = p_next.fixed_slice::<U13, U3>(0, 13 + 3 * idx).clone_owned();
            full_feature.feature.pyy = p_next.fixed_slice::<U3, U3>(13 + 3 * idx, 13 + 3 * idx).clone_owned();
            for jdx in 0..idx {
                full_feature.pyiyj[jdx] = p_next.fixed_slice::<U3, U3>(13 + 3 * idx, 13 + 3 * jdx).clone_owned();
            }
        }

        // FIXME: This should only run if the number of
        // Detect new features
        let window_width = self.camera_model.width() / 4;
        let window_height = self.camera_model.height() / 4;
        // FIXME: Right now we are generating windows randomly.
        // Section 3.7 defines a better way to generate this windows.
        let mut rng = rand::thread_rng();
        let pos_x = rng.gen_range(0, self.camera_model.width() - window_width);
        let pos_y = rng.gen_range(0, self.camera_model.height() - window_height);
        let detection_vec = Detection::detect(&mat.slice((pos_x, pos_y), (window_width, window_height)));
        // FIXME: Should pick N non-overlapping detections
        let detection = detection_vec.first().unwrap();

        self.update_particles(&mat);
        self.consume_detection(&mat, detection);
    }
}
