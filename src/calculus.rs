use nalgebra::{Matrix4, Matrix4x3, UnitQuaternion, U1, U3, Vector};
use nalgebra::storage::Storage;

pub fn dqwr_new_dqwr(qwr: &UnitQuaternion<f64>) -> Matrix4<f64> {
    Matrix4::new(
        qwr[0], -qwr[1], -qwr[2], -qwr[3],
        qwr[1],  qwr[0],  qwr[3], -qwr[2],
        qwr[2], -qwr[3],  qwr[0],  qwr[1],
        qwr[3],  qwr[2], -qwr[1],  qwr[0],
    )
}

pub fn dqwr_new_dqt(qwr: &UnitQuaternion<f64>) -> Matrix4<f64> {
    Matrix4::new(
        qwr[0], -qwr[1], -qwr[2], -qwr[3],
        qwr[1],  qwr[0], -qwr[3],  qwr[2],
        qwr[2],  qwr[3],  qwr[0], -qwr[1],
        qwr[3], -qwr[2],  qwr[1],  qwr[0],
    )
}

pub fn dqt_dwr<S: Storage<f64, U3>>(wr: &Vector<f64, U3, S>, delta_t: f64) -> Matrix4x3<f64> {
    let modulus = wr.norm();
    Matrix4x3::new(
        dq0_domega_a(wr[0], modulus, delta_t), dq0_domega_a(wr[1], modulus, delta_t), dq0_domega_a(wr[2], modulus, delta_t),
        dqa_domega_a(wr[0], modulus, delta_t), dqa_domega_b(wr[0], wr[1], modulus, delta_t), dqa_domega_b(wr[0], wr[2], modulus, delta_t),
        dqa_domega_b(wr[1], wr[0], modulus, delta_t), dqa_domega_a(wr[1], modulus, delta_t), dqa_domega_b(wr[1], wr[2], modulus, delta_t),
        dqa_domega_b(wr[2], wr[0], modulus, delta_t), dqa_domega_b(wr[2], wr[1], modulus, delta_t), dqa_domega_a(wr[2], modulus, delta_t),
    )
}

fn dq0_domega_a(omega_a: f64, modulus: f64, delta_t: f64) -> f64 {
    (-delta_t / 2.0) * (omega_a / modulus) * (modulus * delta_t / 2.0).sin()
}

fn dqa_domega_a(omega_a: f64, modulus: f64, delta_t: f64) -> f64 {
    (delta_t / 2.0) * omega_a.powi(2) / modulus.powi(2) * (modulus * delta_t / 2.0).cos()
    + (1.0 / modulus) * (1.0 - omega_a.powi(2) / modulus.powi(2)) * (modulus * delta_t / 2.0).sin()
}

fn dqa_domega_b(omega_a: f64, omega_b: f64, modulus: f64, delta_t: f64) -> f64 {
    (omega_a * omega_b / modulus.powi(2)) *
    ((delta_t / 2.0) * (modulus * delta_t / 2.0).cos() - (1.0 / modulus) * (modulus * delta_t / 2.0).sin())
}
