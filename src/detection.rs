use image::{GrayImage, Pixel};
use nalgebra::{DMatrix, Matrix3};

const BLOCKSIZE: usize = 11;
const QUALITY_LEVEL: f64 = 1.0;
const NUM_FEATURES: usize = 100;

lazy_static! {
    static ref SOBEL_X: Matrix3<f64> = Matrix3::<f64>::from_vec(vec![
        1.0, 0.0, -1.0,
        2.0, 0.0, -2.0,
        1.0, 0.0, -1.0]);

    static ref SOBEL_Y: Matrix3<f64> = Matrix3::<f64>::from_vec(vec![
        1.0, 2.0, 1.0,
        0.0, 0.0, 0.0,
        -1.0, -2.0, -1.0]);

    static ref MIN_DISTANCE_SQ: f64 = (BLOCKSIZE as f64).powi(2) * 2.0;
}

#[derive(Clone, Debug)]
pub struct Detection {
    pub x: usize,
    pub y: usize,
    pub score: f64,
}

impl Detection {
    // Implementation of the Shi-Tomasi operator to find corner features
    pub fn detect(img: &GrayImage) -> Vec<Detection> {
        let img_width = img.width() as usize;
        let img_height = img.height() as usize;

        // Transform RGBA image to grayscale image
        let mat = DMatrix::<f64>::from_iterator(
            img_width,
            img_height,
            img.pixels().map(|x| x.channels()[0] as f64));

        // Calculate the convolution with the Sobel operator to obtain the image derivative
        let mut mat_dx = DMatrix::<f64>::zeros(img_width, img_height);
        let mut mat_dy = DMatrix::<f64>::zeros(img_width, img_height);
        for x in 1..(img_width - 1) {
            for y in 1..(img_height - 1) {
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
        let mut detection_list = Vec::with_capacity(img_width * img_height);
        for x in 0..(img_width - BLOCKSIZE + 1) {
            for y in 0..(img_height - BLOCKSIZE + 1) {
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
                    let detection = Detection { x, y, score };
                    detection_list.push(detection);
                }
            }
        }

        // Pick the best features possible but make sure they are well spaced out
        detection_list.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        let mut feature_list: Vec<Detection> = Vec::with_capacity(NUM_FEATURES);
        for detection in detection_list {
            // If we have found enough features, stop
            if feature_list.len() >= NUM_FEATURES { break; }
            // Determine if this feature is OK by comparing against all previously picked features
            let should_pick = feature_list.iter().all(|feature| {
                // The distance between the top-left corners has to be larger than a certain value
                // so that the regions do not overlap
                let diff_x = (detection.x as f64) - (feature.x as f64);
                let diff_y = (detection.y as f64) - (feature.y as f64);
                let distance_sq = diff_x.powi(2) + diff_y.powi(2);
                // If they overlap ignore this feature
                distance_sq >= *MIN_DISTANCE_SQ
            });
            // If it passes the test add to the list of detected features
            if should_pick { feature_list.push(detection); }
        }

        feature_list
    }
}
