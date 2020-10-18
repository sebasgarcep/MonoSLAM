use calculus;
use camera_model::WideAngleCameraModel;
use constants::{
    ANGULAR_VELOCITY_NOISE, BLOCKSIZE, LINEAR_VELOCITY_NOISE,
    MIN_DISTANCE_HYPOTHESIS, MAX_DISTANCE_HYPOTHESIS, NUM_PARTICLES,
};
use detection::Detection;
use nalgebra::{
    DMatrix, DVector, Dynamic, Quaternion, Matrix, Matrix3, Matrix6, MatrixMN,
    SliceStorage, U1, U2, U3, U6, U13, UnitQuaternion, Vector,
};
use rand::Rng;
use serde::Deserialize;
use std::fs::File;
use std::io::BufReader;
use utils::{ellipse_search, image_to_matrix, matrix_set_block, unit_quaternion_from_angular_displacement};

#[derive(Deserialize)]
struct FeatureInit {
    yi: Vec<f64>,
    xp_orig: Vec<f64>,
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

impl Particle {
    fn new(depth: f64, probability: f64) -> Self {
        Particle { depth, probability }
    }
}

struct PartialFeature {
    particles: Vec<Particle>,
}

pub struct AppState {
    x: DVector<f64>,
    p: DMatrix<f64>,
    #[allow(dead_code)] xp_orig: DVector<f64>,
    patches: Vec<DMatrix<f64>>,
    camera_model: WideAngleCameraModel,
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

        let mut xp_orig_vec = vec![];
        for feature in &app_state_init.features {
            xp_orig_vec.extend(feature.xp_orig.clone());
        }

        let xp_orig_size = 7 * num_features;
        let xp_orig = DVector::<f64>::from_iterator(xp_orig_size, xp_orig_vec);

        let patches = patch_path_vec.iter()
            .map(|patch_path| {
                let img = image::open(patch_path).unwrap().to_rgba();
                image_to_matrix(&img)
            })
            .collect();

        AppState { x, p, xp_orig, patches, camera_model }
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

    pub fn num_active_features(&self) -> usize {
        let state_size = self.state_size();
        (state_size - 13) / 3
    }

    pub fn feature_yi(&self, idx: usize) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, Dynamic>> {
        self.x.fixed_rows::<U3>(13 + 3 * idx)
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
    }

    pub fn measure(&mut self, mat: DMatrix<f64>) {
        let state_size = self.state_size();
        let num_active_features = self.num_active_features();

        // The matrix Rk is simply sigma_R^2 * I, where sigma_R = 1 is the camera error due to discretization errors.
        let r = self.camera_model.measurement_noise().powi(2) * DMatrix::<f64>::identity(2 * num_active_features, 2 * num_active_features);

        // Calculate Jacobian of observation operator.
        // FIXME: Look at the document as this calculation is wrong!
        let mut h = DMatrix::<f64>::zeros(2 * num_active_features, state_size);
        let rw = self.position();
        let qwr = self.orientation();
        for idx in 0..num_active_features {
            let yi_w = self.feature_yi(idx);
            let yi_r = qwr.inverse_transform_vector(&(yi_w - rw));
            let zi = self.camera_model.project_jacobian(&yi_r);
            matrix_set_block(&mut h, 2 * idx, 13 + 3 * idx, &zi);
        }

        // Calculate the innovation covariance matrix
        let s = &h * &self.p * &h.transpose() + &r;

        // Calculate innovation vector
        let mut y_innov = DVector::<f64>::zeros(2 * num_active_features);
        for idx in 0..num_active_features {
            let yi_w = self.feature_yi(idx);
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
        self.p = p_next; // FIXME: do the transpose trick

        // Detect new features
        let window_width = self.camera_model.width() / 4;
        let window_height = self.camera_model.height() / 4;
        // FIXME: Right now we are generating windows randomly.
        // Section 3.7 defines a better way to generate this windows.
        let mut rng = rand::thread_rng();
        let pos_x = rng.gen_range(0, self.camera_model.width() - window_width);
        let pos_y = rng.gen_range(0, self.camera_model.height() - window_height);
        let detection_vec = Detection::detect(&mat.slice((pos_x, pos_y), (window_width, window_height)));

        let detection = detection_vec.first().unwrap();
        let patch = mat.slice(
            ((detection.pos[0] as usize) - BLOCKSIZE / 2, (detection.pos[1] as usize) - BLOCKSIZE / 2),
            (BLOCKSIZE, BLOCKSIZE)
        ).clone_owned();

        let particles: Vec<_> = (0..NUM_PARTICLES)
            .map(|i| i as f64)
            .map(|i| MIN_DISTANCE_HYPOTHESIS +  (MAX_DISTANCE_HYPOTHESIS - MIN_DISTANCE_HYPOTHESIS) * i / ((NUM_PARTICLES  - 1) as f64))
            .map(|depth| Particle::new(depth, 1.0 / (NUM_PARTICLES as f64)))
            .collect();

        let hr = self.camera_model.unproject(&detection.pos);
        let qwr = self.orientation();
        let hw = qwr * hr / hr.norm();

        // Calculate Pxy = Pxx * dypi_dxv^T (Why ???)

        // Calculate Pyy = dypi_dxv * Pxx * dypi_dxv^T + dypi_dhi * R * dypi_dhi^T (Why ???)

        // Use this to calculate Si and do ellipse search

        // Calculate likelihood of ellipse search and apply Bayes' Theorem
    }
}
