use ndarray::{arr1, arr2};
use typedefs::{Matrix, Vector};

pub fn to_rotation_matrix (q: &Vector) -> Matrix {
    let qw = q[0];
    let qx = q[1];
    let qy = q[2];
    let qz = q[3];

    let sqw = qw.powi(2);
    let sqx = qx.powi(2);
    let sqy = qy.powi(2);
    let sqz = qz.powi(2);

    let temp_wx = qw * qx;
    let temp_wy = qw * qy;
    let temp_wz = qw * qz;
    let temp_xy = qx * qy;
    let temp_xz = qx * qz;
    let temp_yz = qy * qz;

    arr2(&[
        [sqx - sqy - sqz + sqw, 2.0 * (temp_xy - temp_wz), 2.0 * (temp_xz - temp_wy)],
        [2.0 * (temp_xy + temp_wz), - sqx + sqy - sqz + sqw, 2.0 * (temp_yz - temp_wx)],
        [2.0 * (temp_xz + temp_wy), 2.0 * (temp_yz + temp_wx), - sqx - sqy + sqz + sqw],
    ])
}

pub fn invert (q: &Vector) -> Vector {
    arr1(&[q[0], - q[1], - q[2], - q[3]])
}

pub fn multiply (q: &Vector, r: &Vector) -> Vector {
    arr1(&[
        r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
        r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2],
        r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1],
        r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0],
    ])
}
