use nalgebra::{Matrix3, Matrix3x4, Matrix4, Matrix4x3, UnitQuaternion, U3, Vector};
use nalgebra::storage::Storage;
use utils::matrix_set_block;

pub fn dqinv_dq() -> Matrix4<f64> {
    Matrix4::new(
        1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 0.0, -1.0,
    )
}

pub fn dqv_dq<S: Storage<f64, U3>>(q: &UnitQuaternion<f64>, v: &Vector<f64, U3, S>) -> Matrix3x4<f64> {
    let mut res = Matrix3x4::zeros();

    let col0 = drq_dqw(q) * v;
    matrix_set_block(&mut res, 0, 0, &col0);

    let col1 = drq_dqx(q) * v;
    matrix_set_block(&mut res, 0, 1, &col1);

    let col2 = drq_dqy(q) * v;
    matrix_set_block(&mut res, 0, 2, &col2);

    let col3 = drq_dqz(q) * v;
    matrix_set_block(&mut res, 0, 3, &col3);

    res
}

fn drq_dqw(q: &UnitQuaternion<f64>) -> Matrix3<f64> {
    Matrix3::new(
         2.0 * q.w, -2.0 * q.k,  2.0 * q.j,
         2.0 * q.k,  2.0 * q.w, -2.0 * q.i,
        -2.0 * q.j,  2.0 * q.i,  2.0 * q.w,
    )
}

fn drq_dqx(q: &UnitQuaternion<f64>) -> Matrix3<f64> {
    Matrix3::new(
        2.0 * q.i,  2.0 * q.j,  2.0 * q.k,
        2.0 * q.j, -2.0 * q.i, -2.0 * q.w,
        2.0 * q.k,  2.0 * q.w, -2.0 * q.i,
    )
}

fn drq_dqy(q: &UnitQuaternion<f64>) -> Matrix3<f64> {
    Matrix3::new(
        -2.0 * q.j, 2.0 * q.i,  2.0 * q.w,
         2.0 * q.i, 2.0 * q.j,  2.0 * q.k,
        -2.0 * q.w, 2.0 * q.k, -2.0 * q.j,
    )
}

fn drq_dqz(q: &UnitQuaternion<f64>) -> Matrix3<f64> {
    Matrix3::new(
        -2.0 * q.k, -2.0 * q.w, 2.0  * q.i,
         2.0 * q.w, -2.0 * q.k, 2.0  * q.j,
         2.0 * q.i,  2.0 * q.j, 2.0  * q.k,
    )
}

pub fn dqwr_new_dqwr(qwr: &UnitQuaternion<f64>) -> Matrix4<f64> {
    Matrix4::new(
        qwr.w, -qwr.i, -qwr.j, -qwr.k,
        qwr.i,  qwr.w,  qwr.k, -qwr.j,
        qwr.j, -qwr.k,  qwr.w,  qwr.i,
        qwr.k,  qwr.j, -qwr.i,  qwr.w,
    )
}

pub fn dqwr_new_dqt(qwr: &UnitQuaternion<f64>) -> Matrix4<f64> {
    Matrix4::new(
        qwr.w, -qwr.i, -qwr.j, -qwr.k,
        qwr.i,  qwr.w, -qwr.k,  qwr.j,
        qwr.j,  qwr.k,  qwr.w, -qwr.i,
        qwr.k, -qwr.j,  qwr.i,  qwr.w,
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
