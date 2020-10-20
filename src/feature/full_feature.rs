use super::abstract_feature::AbstractFeature;
use nalgebra::{DMatrix, MatrixMN, U3, U13, Vector3};

pub struct FullFeature {
    pub feature: AbstractFeature<U3>,
    pub pyiyj: Vec<MatrixMN<f64, U3, U3>>,
}

impl FullFeature {
    pub fn new_from_position(features: &Vec<Self>, y: Vector3<f64>, patch: DMatrix<f64>) -> Self {
        let pxy = MatrixMN::<f64, U13, U3>::zeros();
        let pyy = MatrixMN::<f64, U3, U3>::zeros();
        let feature = AbstractFeature::new(y, pxy, pyy, patch);
        let pyiyj = features.iter().map(|_| MatrixMN::<f64, U3, U3>::zeros()).collect();
        FullFeature { feature, pyiyj }
    }
}
