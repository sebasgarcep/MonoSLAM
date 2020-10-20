use super::abstract_feature::AbstractFeature;
use app_state::AppState;
use calculus;
use camera_model::WideAngleCameraModel;
use constants::{BLOCKSIZE, MIN_DISTANCE_HYPOTHESIS, MAX_DISTANCE_HYPOTHESIS, NUM_PARTICLES};
use detection::Detection;
use nalgebra::{
    DMatrix, Matrix2, Matrix3, MatrixMN, Rotation3, SliceStorage, U1, U2, U3, U6, U13,
    UnitQuaternion, Vector, Vector2, Vector3, VectorN
};
// use stats::{mean, stddev};
use utils::{ellipse_search, matrix_set_block, probability_normal_dist};

pub struct Particle {
    pub depth: f64,
    pub probability: f64,
}

pub struct PartialFeature {
    pub feature: AbstractFeature<U6>,
    pub full_pyiyj: Vec<MatrixMN<f64, U6, U3>>,
    pub partial_pyiyj: Vec<MatrixMN<f64, U6, U6>>,
    pub particles: Vec<Particle>,
}

impl PartialFeature {
    pub fn new(
        app_state: &AppState,
        mat: &DMatrix<f64>,
        detection: &Detection,
    ) -> Self {
        let patch = mat.slice(
            ((detection.pos[0] as usize) - BLOCKSIZE / 2, (detection.pos[1] as usize) - BLOCKSIZE / 2),
            (BLOCKSIZE, BLOCKSIZE)
        ).clone_owned();

        let pxx = app_state.pxx;
        let rwg = app_state.position();
        let hrg = app_state.camera_model.unproject(&detection.pos);
        let qwr = app_state.orientation();
        let hwg = qwr * hrg / hrg.norm();
        let mut yg = VectorN::<f64, U6>::zeros();
        matrix_set_block(&mut yg, 0, 0, &rwg);
        matrix_set_block(&mut yg, 0, 0, &hwg);

        let dhwg_hat_dhwg = calculus::dv_hat_dv(&hwg);
        let dhwg_dqwr = calculus::dqv_dq(&qwr, &hrg);
        let dhwg_hat_dqwr = &dhwg_hat_dhwg * &dhwg_dqwr;
        let mut dyg_dxv = MatrixMN::<f64, U6, U13>::zeros();
        matrix_set_block(&mut dyg_dxv, 0, 0, &Matrix3::identity());
        matrix_set_block(&mut dyg_dxv, 3, 3, &dhwg_hat_dqwr);

        let dhrg_dg = app_state.camera_model.unproject_jacobian(&detection.pos);
        let dhwg_dhrg = qwr.to_rotation_matrix();
        let dhwg_hat_dg = dhwg_hat_dhwg * &dhwg_dhrg * &dhrg_dg;
        let mut dyg_dg = MatrixMN::<f64, U6, U2>::zeros();
        matrix_set_block(&mut dyg_dg, 3, 0, &dhwg_hat_dg);

        let rg = app_state.camera_model.measurement_noise().powi(2) * Matrix2::<f64>::identity();
        let pxyg = &pxx * &dyg_dxv.transpose();
        let pyyg = &dyg_dxv * &pxx * &dyg_dxv.transpose() + &dyg_dg * &rg * &dyg_dg.transpose();
        let feature = AbstractFeature::new(yg, pxyg, pyyg, patch);

        let full_pyiyj = app_state.full_features.iter()
            .map(|full_feature| &dyg_dxv * &full_feature.feature.pxy)
            .collect();

        let partial_pyiyj = app_state.partial_features.iter()
            .map(|partial_feature| &dyg_dxv * &partial_feature.feature.pxy)
            .collect();

        let particles = (0..NUM_PARTICLES)
            .map(|i| i as f64)
            .map(|i| MIN_DISTANCE_HYPOTHESIS +  (MAX_DISTANCE_HYPOTHESIS - MIN_DISTANCE_HYPOTHESIS) * i / ((NUM_PARTICLES  - 1) as f64))
            .map(|depth| Particle{ depth, probability: 1.0 / (NUM_PARTICLES as f64) })
            .collect();

        PartialFeature { feature, full_pyiyj, partial_pyiyj, particles }
    }

    pub fn position(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, U6>> {
        self.feature.y.fixed_rows::<U3>(0)
    }

    pub fn direction(&self) -> Vector<f64, U3, SliceStorage<f64, U3, U1, U1, U6>> {
        self.feature.y.fixed_rows::<U3>(3)
    }

    pub fn update_particles(
        &mut self,
        rw: &Vector3<f64>,
        qrw: &UnitQuaternion<f64>,
        rrw: &Rotation3<f64>,
        pxx: &MatrixMN<f64, U13, U13>,
        camera_model: &WideAngleCameraModel,
        mat: &DMatrix<f64>,
    ) {
        let rwg = self.position().clone_owned();
        let hwg = self.direction().clone_owned();
        for particle in &mut self.particles {
            // Calculate search center
            let ywl = rwg + particle.depth * hwg;
            let ywl_minus_rw = &ywl - rw;
            let yrl = qrw * ywl_minus_rw;
            let mu = camera_model.project(&yrl);

            // Calculate search covariance
            let dzl_dyrl = camera_model.project_jacobian(&yrl);
            let dzl_drw = -1.0 * &dzl_dyrl * rrw;
            let dyrl_dqrw = calculus::dqv_dq(&qrw, &ywl_minus_rw);
            let dqrw_dqwr = calculus::dqinv_dq();
            let dzl_dqwr = &dzl_dyrl * &dyrl_dqrw * &dqrw_dqwr;
            let mut dzl_dxv = MatrixMN::<f64, U2, U13>::zeros();
            matrix_set_block(&mut dzl_dxv, 0, 0, &dzl_drw);
            matrix_set_block(&mut dzl_dxv, 0, 3, &dzl_dqwr);

            let dzl_drwg = &dzl_dyrl * rrw;
            let dzl_dhwg = particle.depth * &dzl_drwg;
            let mut dzl_yg = MatrixMN::<f64, U2, U6>::zeros();
            matrix_set_block(&mut dzl_yg, 0, 0, &dzl_drwg);
            matrix_set_block(&mut dzl_yg, 0, 3, &dzl_dhwg);

            let sl_xy = &dzl_dxv * &self.feature.pxy * &dzl_yg.transpose();
            let sl = &dzl_dxv * pxx * &dzl_dxv.transpose()
                + sl_xy + sl_xy.transpose()
                + &dzl_yg * &self.feature.pyy * &dzl_yg.transpose();

            let (best_x, best_y, sl_inv) = ellipse_search(
                mu[0].round() as usize,
                mu[1].round() as usize,
                &sl,
                &mat,
                &self.feature.patch,
            );

            let best_mu = Vector2::<f64>::new(best_x as f64, best_y as f64);

            // FIXME: sl is has negative determinant
            particle.probability *= probability_normal_dist(&mu, &sl, &sl_inv, &best_mu);
            if particle.probability.is_nan() {
                let x_diff = &best_mu - &mu;
                let z_2_mat = &x_diff.transpose() * &sl_inv * &x_diff;
                let z_2_val = z_2_mat[(0, 0)];
                let sigma_det_sqrt = &sl.determinant().sqrt();
                let div = &sl.determinant();
                println!("mu = {:?}, S = {:?}, S^-1 = {:?}, x = {:?}, Z^2 = {:?}, Div = {:?}", mu, sl, sl_inv, best_mu, z_2_val, div);
                panic!();
            }
        }

        // Normalize particles
        let particle_sum: f64 = self.particles.iter().map(|p| p.probability).sum();
        for particle in &mut self.particles {
            particle.probability /= particle_sum;
        }

        println!("b = {:?}", self.particles.iter().map(|p| p.probability).collect::<Vec<_>>());
    }
}
