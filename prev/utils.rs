use im::RgbaImage;
use ndarray::{arr1, arr2, Array, ShapeBuilder};
use ndarray_linalg::trace::Trace;
use serde::de::DeserializeOwned;
use serde_json::from_str;
use std::fs::File;
use std::io::Read;
use std::path::Path;
use typedefs::{Matrix, Vector};

pub fn load_from_json <'a, P: AsRef<Path>, T: DeserializeOwned> (path:  P) -> T {
    let mut file = File::open(path).unwrap();
    let mut contents = String::new();
    let _ = file.read_to_string(&mut contents);
    from_str(&contents).unwrap()
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

