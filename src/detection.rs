use constants::{BLOCKSIZE, QUALITY_LEVEL};
use nalgebra::{Vector2, DMatrix, Matrix3};

lazy_static! {
    static ref SOBEL_X: Matrix3<f64> = Matrix3::<f64>::from_vec(vec![
        1.0, 0.0, -1.0,
        2.0, 0.0, -2.0,
        1.0, 0.0, -1.0]);

    static ref SOBEL_Y: Matrix3<f64> = Matrix3::<f64>::from_vec(vec![
        1.0, 2.0, 1.0,
        0.0, 0.0, 0.0,
        -1.0, -2.0, -1.0]);
}

#[derive(Clone, Debug)]
pub struct Detection {
    pub pos: Vector2<f64>,
    pub score: f64,
}

impl Detection {
    // Implementation of the Shi-Tomasi operator to find corner features
    pub fn detect(mat: &DMatrix<f64>) -> Vec<Detection> {
        let (width, height) = mat.shape();

        // Calculate the convolution with the Sobel operator to obtain the image derivative
        let mut mat_dx = DMatrix::<f64>::zeros(width, height);
        let mut mat_dy = DMatrix::<f64>::zeros(width, height);
        for x in 1..(width - 1) {
            for y in 1..(height - 1) {
                let mat_s = mat.slice((x - 1, y - 1), (3, 3));
                mat_dx[(x, y)] = SOBEL_X.component_mul(&mat_s).sum();
                mat_dy[(x, y)] = SOBEL_Y.component_mul(&mat_s).sum();
            }
        }

        // Obtain the Hadamard product between the derivatives
        let mat_dxx = mat_dx.component_mul(&mat_dx);
        let mat_dyy = mat_dy.component_mul(&mat_dy);
        let mat_dxy = mat_dx.component_mul(&mat_dy);

        // Calculate the score for each window in the image
        let mut detection_list = Vec::with_capacity(width * height);
        for x in 0..(width - BLOCKSIZE + 1) {
            for y in 0..(height - BLOCKSIZE + 1) {
                // The Z matrix is calculated by adding over all elements of the window
                let g_xx = mat_dxx.slice((x, y), (BLOCKSIZE, BLOCKSIZE)).sum();
                let g_yy = mat_dyy.slice((x, y), (BLOCKSIZE, BLOCKSIZE)).sum();
                let g_xy = mat_dxy.slice((x, y), (BLOCKSIZE, BLOCKSIZE)).sum();

                // Find the smallest eigenvalue of the Z matrix
                // Because the Z matrix is a 2x2 matrix, there is a simple explicit formula
                // for the eigenvalues.
                let bb = ((g_xx + g_yy).powi(2) - 4.0 * (g_xx * g_yy - g_xy * g_xy)).sqrt();
                let score = (g_xx + g_yy - bb) / 2.0;

                // The eigenvalue has to be larger than the given threshold
                if score > QUALITY_LEVEL {
                    let pos = Vector2::<f64>::new(
                        (x + BLOCKSIZE / 2) as f64,
                        (y + BLOCKSIZE / 2) as f64,
                    );
                    let detection = Detection { pos, score };
                    detection_list.push(detection);
                }
            }
        }

        // Pick the best features possible but make sure they are well spaced out
        detection_list.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        detection_list
    }
}
