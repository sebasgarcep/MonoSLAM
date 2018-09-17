use im::{ConvertBuffer, RgbImage, RgbaImage};
use ndarray::{arr1, arr2, Array, ShapeBuilder};
use ndarray_linalg::trace::Trace;
use std::error::Error;
use typedefs::{Matrix, Vector};
use uvc::Frame;

pub fn frame_to_image (frame: &Frame) -> Result<RgbaImage, Box<dyn Error>> {
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

pub fn get_grayscale_matrix_from_image (
    image: &RgbaImage,
    x: u32,
    y: u32,
    w: u32,
    h: u32,
) -> Matrix {
    let shape = (w as usize, h as usize).f();
    let mut image_mat = Array::zeros(shape);

    for u in x..(x + w) {
        for v in y..(y + h) {
            let pixel = image.get_pixel(u, v);
            let r = pixel.data[0] as f64;
            let g = pixel.data[1] as f64;
            let b = pixel.data[2] as f64;
            let xpos = u - x;
            let ypos = v - y;
            let idx = [xpos as usize, ypos as usize];
            image_mat[idx] = (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0;
        }
    }

    image_mat
}

pub fn convert_matrix_to_quaternion (
    matrix: &Matrix
) -> Vector {
    let s;

    let qw;
    let qx;
    let qy;
    let qz;

    let tr = matrix.trace().unwrap();

    if tr > 0.0 {
        s = (tr + 1.0).sqrt() * 2.0;
        qw = 0.25 * s;
        qx = (matrix[[2, 1]] - matrix[[1, 2]]) / s;
        qy = (matrix[[0, 2]] - matrix[[2, 0]]) / s;
        qz = (matrix[[1, 0]] - matrix[[0, 1]]) / s;
    } else if matrix[[0, 0]] > matrix[[1, 1]] && matrix[[0, 0]] > matrix[[2, 2]] {
        s = (1.0 + matrix[[0, 0]] - matrix[[1, 1]] - matrix[[2, 2]]).sqrt() * 2.0;
        qw = (matrix[[2, 1]] - matrix[[1, 2]]) / s;
        qx = 0.25 * s;
        qy = (matrix[[0, 1]] + matrix[[1, 0]]) / s;
        qz = (matrix[[0, 2]] + matrix[[2, 0]]) / s;
    } else if matrix[[1, 1]] > matrix[[2, 2]] {
        s = (1.0 + matrix[[1, 1]] - matrix[[0, 0]] - matrix[[2, 2]]).sqrt() * 2.0;
        qw = (matrix[[0, 2]] - matrix[[2, 0]]) / s;
        qx = (matrix[[0, 1]] + matrix[[1, 0]]) / s;
        qy = 0.25 * s;
        qz = (matrix[[1, 2]] + matrix[[2, 1]]) / s;
    } else {
        s = (1.0 + matrix[[2, 2]] - matrix[[0, 0]] - matrix[[1, 1]]).sqrt() * 2.0;
        qw = (matrix[[1, 0]] - matrix[[0, 1]]) / s;
        qx = (matrix[[0, 2]] + matrix[[2, 0]]) / s;
        qy = (matrix[[1, 2]] + matrix[[2, 1]]) / s;
        qz = 0.25 * s;
    }

    arr1(&[qw, qx, qy, qz])
}

pub fn quaternion_product (q: &Vector, r: &Vector) -> Vector {
    arr1(&[
        r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
        r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2],
        r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1],
        r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0],
    ])
}

pub fn get_rotation_matrix_from_angular_displacement (
    w: &Vector
) -> Matrix {
    let wx = w[0];
    let wy = w[1];
    let wz = w[2];

    arr2(&[
        [0.0, - wz, wy],
        [wz, 0.0, - wx],
        [- wy, wx, 0.0],
    ])
}

pub fn convert_quaternion_to_matrix (
    q: &Vector
) -> Matrix {
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

pub fn quaternion_inverse (q: &Vector) -> Vector {
    arr1(&[q[0], - q[1], - q[2], - q[3]])
}
