use constants::{BLOCKSIZE, NUM_SIGMA};
use image::{DynamicImage, Pixel, RgbaImage};
use nalgebra::storage::{Storage, StorageMut};
use nalgebra::{
    Dim, Dynamic, DMatrix, Quaternion, Matrix, Matrix2, Scalar, U2, UnitQuaternion,
    Vector2, Vector3,
};
use std::cmp::{max, min};

pub fn image_to_matrix(img: &RgbaImage) -> DMatrix<f64> {
    let img_gray = DynamicImage::ImageRgba8(img.clone()).into_luma();
    let img_width = img_gray.width() as usize;
    let img_height = img_gray.height() as usize;
    DMatrix::<f64>::from_iterator(
        img_width,
        img_height,
        img_gray.pixels().map(|x| (x.channels()[0] as f64) / 255.0))
}

pub fn normalized_cross_correlation<S1: Storage<f64, Dynamic, Dynamic>, S2: Storage<f64, Dynamic, Dynamic>>(
    template: &Matrix<f64, Dynamic, Dynamic, S1>,
    image: &Matrix<f64, Dynamic, Dynamic, S2>,
) -> f64 {
    let template_mean = template.mean();
    let template_sub = template.map(|x| x - template_mean);
    let image_mean = image.mean();
    let image_sub = image.map(|x| x - image_mean);
    let numerator = image_sub.component_mul(&template_sub).sum();
    let template_div = template_sub.map(|x| x * x).sum();
    let image_div = image_sub.map(|x| x * x).sum();
    let denominator = (image_div * template_div).sqrt();
    numerator / denominator
}

pub fn unit_quaternion_from_angular_displacement(ang_delta: &Vector3<f64>) -> UnitQuaternion<f64> {
    let angle = ang_delta.norm();

    let (mut w, mut x, mut y, mut z) = (1.0, 0.0, 0.0, 0.0);
    if angle > 0.0 {
        let s = (angle / 2.0).sin() / angle;
        let c = (angle / 2.0).cos();
        w = c;
        x = s * ang_delta[0];
        y = s * ang_delta[1];
        z = s * ang_delta[2];
    }

    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
}

pub fn matrix_set_block<N: Scalar + Copy, R1: Dim, C1: Dim, S1: StorageMut<N, R1, C1>, R2: Dim, C2: Dim, S2: Storage<N, R2, C2>>(
    mat: &mut Matrix<N, R1, C1, S1>,
    start_x: usize,
    start_y: usize,
    block: &Matrix<N, R2, C2, S2>,
) {
    let (width, height) = block.shape();
    for i in 0..width {
        for j in 0..height {
            mat[(start_x + i, start_y + j)] = block[(i, j)];
        }
    }
}

fn inside_relative (si_inv: &Matrix2<f64>, uu: isize, uv: isize) -> bool {
    let u = uu as f64;
    let v = uv as f64;
    let pos =
        si_inv[(0, 0)] * u.powi(2) +
        2.0 * si_inv[(0, 1)] * u * v +
        si_inv[(1, 1)] * v.powi(2);
    pos < NUM_SIGMA.powi(2)
}

pub fn ellipse_search<S1: Storage<f64, U2, U2>>(
    x_center: usize,
    y_center: usize,
    si: &Matrix<f64, U2, U2, S1>,
    mat: &DMatrix<f64>,
    template: &DMatrix<f64>,
) -> (usize, usize, Matrix2<f64>) {
    // Calculate S_i auxiliary values
    let si_det = si.determinant();
    let si_inv = Matrix2::<f64>::new(
         si[(1, 1)], -si[(0, 1)],
        -si[(1, 0)],  si[(0, 0)],
    ) / si_det;
    let si_diag_prod = si_inv[(0, 1)].powi(2);
    let halfwidth = (NUM_SIGMA / (si_inv[(0, 0)] - si_diag_prod / si_inv[(1, 1)]).sqrt()).round() as usize;
    let halfheight = (NUM_SIGMA / (si_inv[(1, 1)] - si_diag_prod / si_inv[(0, 0)]).sqrt()).round() as usize;

    // Find search limits
    let (width, height) = mat.shape();
    let x_min = max(BLOCKSIZE / 2, x_center - halfwidth);
    let x_max = min(width - BLOCKSIZE / 2, x_center + halfwidth);
    let y_min = max(BLOCKSIZE / 2, y_center - halfheight);
    let y_max = min(height - BLOCKSIZE / 2, y_center + halfheight);

    // Find best x, y
    let (mut best_x, mut best_y) = (x_center, y_center);
    let mut best_corr = f64::NEG_INFINITY;
    for x in x_min..x_max {
        for y in y_min..y_max {
            // Test if inside ellipse
            let x_diff = (x as isize) - (x_center as isize);
            let y_diff = (y as isize) - (y_center as isize);
            if inside_relative(&si_inv, x_diff, y_diff) {
                let patch = mat.slice((x - BLOCKSIZE / 2, y - BLOCKSIZE / 2), (BLOCKSIZE, BLOCKSIZE));
                // Calculate NCC
                let corr = normalized_cross_correlation(&template, &patch);
                if best_x == 0 || corr > best_corr {
                    best_x = x;
                    best_y = y;
                    best_corr = corr;
                }
            }
        }
    }

    (best_x, best_y, si_inv)
}

pub fn probability_normal_dist(
    mu: &Vector2<f64>,
    sigma: &Matrix2<f64>,
    sigma_inv: &Matrix2<f64>,
    x: &Vector2<f64>,
) -> f64 {
    let x_diff = x - mu;
    let z_2_mat = &x_diff.transpose() * sigma_inv * &x_diff;
    let z_2_val = z_2_mat[(0, 0)];
    let sigma_det_sqrt = sigma.determinant().sqrt();

    (-0.5 * z_2_val).exp() / (2.0 * std::f64::consts::PI * sigma_det_sqrt)
}
