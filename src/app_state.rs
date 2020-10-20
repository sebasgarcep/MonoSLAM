use calculus;
use camera_model::WideAngleCameraModel;
use constants::{
    ANGULAR_VELOCITY_NOISE, BLOCKSIZE, LINEAR_VELOCITY_NOISE,
    MIN_DISTANCE_HYPOTHESIS, MAX_DISTANCE_HYPOTHESIS, NUM_PARTICLES,
};
use detection::Detection;
use nalgebra::{
    DMatrix, DVector, Dynamic, Quaternion, Matrix, Matrix2, Matrix3, Matrix6, MatrixMN,
    SliceStorage, U1, U2, U3, U6, U13, UnitQuaternion, Vector, Vector3,
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

struct Particle {
    depth: f64,
    probability: f64,
}

struct PartialFeature {
    rwg: Vector3<f64>,
    hwg: Vector3<f64>,
    patch: DMatrix<f64>,
    pxyg: MatrixMN<f64, U13, U6>,
    pyyg: Matrix6<f64>,
    particles: Vec<Particle>,
}

pub struct AppState {
    x: DVector<f64>,
    p: DMatrix<f64>,
    patches: Vec<DMatrix<f64>>,
    camera_model: WideAngleCameraModel,
    partial_features: Vec<PartialFeature>,
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
        let num_features = app_state_init.features.len();

        let mut x_vec = app_state_init.xv.clone();
        for feature in &app_state_init.features {
            x_vec.extend(feature.yi.clone());
        }

        let x_size = 13 + 3 * num_features;
        let x = DVector::<f64>::from_iterator(x_size, x_vec);

        let mut p = DMatrix::<f64>::zeros(x_size, x_size);
        for i in 0..13 {
            for j in 0..13 {
                p[(i, j)] = app_state_init.pxx[i][j];
            }
        }

        let patches = patch_path_vec.iter()
            .map(|patch_path| {
                let img = image::open(patch_path).unwrap().to_rgba();
                image_to_matrix(&img)
            })
            .collect();

        let partial_features = vec![];

        AppState { x, p, patches, camera_model, partial_features }
    }

    pub fn state(&self) -> Matrix<f64, Dynamic, Dynamic, SliceStorage<f64, Dynamic, Dynamic, U1, Dynamic>> {
        self.x.slice((0, 0), (self.x.len(), 1))
    }

    pub fn position(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, Dynamic>> {
        self.x.fixed_rows::<U3>(0)
    }

    pub fn orientation(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_quaternion(Quaternion::new(self.x[3], self.x[4], self.x[5], self.x[6]))
    }

    pub fn linear_velocity(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, Dynamic>> {
        self.x.fixed_rows::<U3>(7)
    }

    pub fn angular_velocity(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, Dynamic>> {
        self.x.fixed_rows::<U3>(10)
    }

    pub fn state_size(&self) -> usize {
        self.x.len()
    }

    pub fn robot_covariance(&self) -> Matrix<f64, U13, U13, SliceStorage<f64, U13, U13, U1, Dynamic>> {
        self.p.fixed_slice::<U13, U13>(0, 0)
    }

    pub fn num_active_features(&self) -> usize {
        let state_size = self.state_size();
        (state_size - 13) / 3
    }

    pub fn active_feature_yi(&self, idx: usize) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, Dynamic>> {
        self.x.fixed_rows::<U3>(13 + 3 * idx)
    }

    pub fn consume_detection(&mut self, mat: &DMatrix<f64>, detection: &Detection) {
        let patch = mat.slice(
            ((detection.pos[0] as usize) - BLOCKSIZE / 2, (detection.pos[1] as usize) - BLOCKSIZE / 2),
            (BLOCKSIZE, BLOCKSIZE)
        ).clone_owned();

        let pxx = self.robot_covariance();
        let rwg = self.position().clone_owned();
        let hrg = self.camera_model.unproject(&detection.pos);
        let qwr = self.orientation();
        let hwg = qwr * hrg / hrg.norm();

        let particles = (0..NUM_PARTICLES)
            .map(|i| i as f64)
            .map(|i| MIN_DISTANCE_HYPOTHESIS +  (MAX_DISTANCE_HYPOTHESIS - MIN_DISTANCE_HYPOTHESIS) * i / ((NUM_PARTICLES  - 1) as f64))
            .map(|depth| Particle{ depth, probability: 1.0 / (NUM_PARTICLES as f64) })
            .collect();

        let dhwg_hat_dhwg = calculus::dv_hat_dv(&hwg);
        let dhwg_dqwr = calculus::dqv_dq(&qwr, &hrg);
        let dhwg_hat_dqwr = &dhwg_hat_dhwg * &dhwg_dqwr;
        let mut dyg_dxv = MatrixMN::<f64, U6, U13>::zeros();
        matrix_set_block(&mut dyg_dxv, 0, 0, &Matrix3::identity());
        matrix_set_block(&mut dyg_dxv, 3, 3, &dhwg_hat_dqwr);

        let dhrg_dg = self.camera_model.unproject_jacobian(&detection.pos);
        let dhwg_dhrg = qwr.to_rotation_matrix();
        let dhwg_hat_dg = dhwg_hat_dhwg * &dhwg_dhrg * &dhrg_dg;
        let mut dyg_dg = MatrixMN::<f64, U6, U2>::zeros();
        matrix_set_block(&mut dyg_dg, 3, 0, &dhwg_hat_dg);

        let rg = self.camera_model.measurement_noise().powi(2) * Matrix2::<f64>::identity();
        let pxyg = &pxx * &dyg_dxv.transpose();
        let pyyg = &dyg_dxv * &pxx * &dyg_dxv.transpose() + &dyg_dg * &rg * &dyg_dg.transpose();

        let partial_feature = PartialFeature { rwg, hwg, patch, pxyg, pyyg, particles };

        // FIXME: Remove this condition
        if self.partial_features.len() == 0 {
            self.partial_features.push(partial_feature);
        }

        // Use this to calculate Si and do ellipse search

        // Calculate likelihood of ellipse search and apply Bayes' Theorem
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
        let mut x_new = self.x.clone();
        matrix_set_block(&mut x_new, 0, 0, &rw_new);
        x_new[3] = qwr_new.coords[3]; // w
        x_new[4] = qwr_new.coords[0]; // x
        x_new[5] = qwr_new.coords[1]; // y
        x_new[5] = qwr_new.coords[2]; // z

        // Calculate the covariance of the impulse vector.
        let mut pn = Matrix6::<f64>::zeros();
        matrix_set_block(&mut pn, 0, 0, &(Matrix3::identity() * LINEAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2)));
        matrix_set_block(&mut pn, 3, 3, &(Matrix3::identity() * ANGULAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2)));

        /*
            Calculate the Jacobian of the process vector with respect to the impulse vector:

            dfv_dn = | I * delta_t                0 |
                     | 0            dqwrnew_domegar |
                     | I                          0 |
                     | 0                          I |

            Let qt = q((wr + omegar) * delta_t). Then:

            dqwrnew_domegar = dqwrnew_dqt * dqt_domegar
        */
        let dqwr_new_dqt = calculus::dqwr_new_dqt(&qwr);
        let dqt_domegar = calculus::dqt_dwr(&wr, delta_t);
        let dqwrnew_domegar = dqwr_new_dqt * dqt_domegar;
        let mut dfv_dn = MatrixMN::<f64, U13, U6>::zeros();
        matrix_set_block(&mut dfv_dn, 0, 0, &(Matrix3::identity() * delta_t));
        matrix_set_block(&mut dfv_dn, 3, 3, &dqwrnew_domegar);
        matrix_set_block(&mut dfv_dn, 7, 0, &Matrix6::identity());

        // Calculate the process noise covariance.
        let qv = dfv_dn * pn * dfv_dn.transpose();

        /*
            The motion model maps it xv -> fv(xv) and yi -> yi. Therefore the jacobian of this model is:

            df_dx = | dfv_dxv    0 |
                    |       0    I |

            The Jacobian of fv with respect to xv is:

            dfv_dxv = | I    0               I * delta_t    0           |
                      | 0    dqwrnew_dqwr    0              dqwrnew_dwr |
                      | 0    0               I              0           |
                      | 0    0               0              I           |

            Let qt = q((wr + omegar) * delta_t). Then:

            dqwrnew_dwr = dqwrnew_dqt * dqt_dwr

            Same calculation as before.
        */
        let dqwrnew_dqwr = calculus::dqwr_new_dqwr(&qwr);
        let dqwrnew_dwr = dqwrnew_domegar;
        let mut dfv_dxv = MatrixMN::<f64, U13, U13>::zeros();
        matrix_set_block(&mut dfv_dxv, 0, 0, &Matrix3::identity());
        matrix_set_block(&mut dfv_dxv, 0, 7, &(Matrix3::identity() * delta_t));
        matrix_set_block(&mut dfv_dxv, 3, 3, &dqwrnew_dqwr);
        matrix_set_block(&mut dfv_dxv, 3, 10, &dqwrnew_dwr);
        matrix_set_block(&mut dfv_dxv, 7, 7, &Matrix6::identity());

        /*
            The propagation of the covariance matrix is:

            = df_dx * P * df_dx^T + Q
            = | dfv_dxv    0 |   | Pxx Pxy |   | dfv_dxv^T    0 |
              |       0    I | * | Pyx Pyy | * |       0      I | + Q
            = | dfv_dxv * Pxx * dfv_dxv^T    dfv_dxv * Pxy |
              | Pyx * dfv_dxv^T                        Pyy | + Q

            where Q is:

            Q = | Qv    0 |
                |  0    0 |
        */
        let state_size = self.state_size();
        let mut p_new = DMatrix::<f64>::zeros(state_size, state_size);
        // dfv_dxv * Pxx * dfv_dxv^T
        let pxx = self.p.fixed_slice::<U13, U13>(0, 0);
        let pxx_new = dfv_dxv * pxx * dfv_dxv.transpose() + qv;
        matrix_set_block(&mut p_new, 0, 0, &pxx_new);
        // dfv_dxv * p_new
        let pxy_new = dfv_dxv * self.p.slice((0, 13), (13, state_size - 13));
        matrix_set_block(&mut p_new, 0, 13, &pxy_new);
        matrix_set_block(&mut p_new, 13, 0, &pxy_new.transpose());
        // Pyy
        let pyy = self.p.slice((13, 13), (state_size - 13, state_size - 13));
        matrix_set_block(&mut p_new, 13, 13, &pyy);

        // Update state and covariance matrix using model predictions
        self.x = x_new;
        self.p = p_new;

        // FIXME: project partial features forward
    }

    pub fn measure(&mut self, mat: DMatrix<f64>) {
        let state_size = self.state_size();
        let num_active_features = self.num_active_features();

        // The matrix Rk is simply sigma_R^2 * I, where sigma_R = 1 is the camera error due to discretization errors.
        let r = self.camera_model.measurement_noise().powi(2) * DMatrix::<f64>::identity(2 * num_active_features, 2 * num_active_features);

        // Calculate Jacobian of observation operator.
        let mut h = DMatrix::<f64>::zeros(2 * num_active_features, state_size);
        let rw = self.position();
        let qwr = self.orientation();
        let qrw = qwr.inverse();
        let rot_rw = qrw.to_rotation_matrix();
        let dqrw_dqwr = calculus::dqinv_dq();
        for idx in 0..num_active_features {
            let yi_w = self.active_feature_yi(idx);
            let yi_w_minus_rw = yi_w - rw;
            let yi_r = rot_rw * &yi_w_minus_rw;
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
        let s = &h * &self.p * &h.transpose() + &r;

        // Calculate innovation vector
        let mut y_innov = DVector::<f64>::zeros(2 * num_active_features);
        for idx in 0..num_active_features {
            let yi_w = self.active_feature_yi(idx);
            let h_i = self.camera_model.project(&(yi_w - rw));
            let (h_i_x, h_i_y) = (h_i[0].round() as usize, h_i[1].round() as usize);

            // Calculate best x, y
            let s_i = s.fixed_slice::<U2, U2>(2 * idx, 2 * idx);
            let (best_x, best_y) = ellipse_search(
                self.camera_model.width(),
                self.camera_model.height(),
                h_i_x,
                h_i_y,
                &s_i,
                &mat,
                &self.patches[idx]
            );

            // Set the innovation vector = measurements - camera projection of current state of features
            y_innov[2 * idx] = (best_x as f64) - (h_i_x as f64);
            y_innov[2 * idx + 1] = (best_y as f64) - (h_i_y as f64);
        }

        let s_inv = &s.pseudo_inverse(1e-6).unwrap();
        let k = &self.p * &h.transpose() * s_inv;
        let x_next = &self.x + &k * &y_innov;
        let p_next = (DMatrix::<f64>::identity(state_size, state_size) - &k * &h) * &self.p;

        // Update state and covariance matrix using Kalman Filter corrections
        self.x = x_next;
        // Let's enforce symmetry of covariance matrix
        self.p = 0.5 * &p_next + 0.5 * &p_next.transpose();

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
        // FIXME: Should pick the first non-overlapping detection
        let detection = detection_vec.first().unwrap();
        self.consume_detection(&mat, detection);
    }
}
