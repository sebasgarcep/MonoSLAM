use super::abstract_feature::AbstractFeature;
use app_state::AppState;
use calculus;
use constants::{BLOCKSIZE, MIN_DISTANCE_HYPOTHESIS, MAX_DISTANCE_HYPOTHESIS, NUM_PARTICLES};
use detection::Detection;
use nalgebra::{DMatrix, Matrix2, Matrix3, MatrixMN, U2, U3, U6, U13, VectorN};
use utils::matrix_set_block;

pub struct Particle {
    depth: f64,
    probability: f64,
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
}
