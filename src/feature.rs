use constants::{
    BLOCKSIZE,
    QUALITY_LEVEL,
    MIN_DISTANCE_SQ,
    NUM_FEATURES,
    PADDING,
};
use im::RgbaImage;
use ndarray::{arr2, Array2, ShapeBuilder};
use std::cmp::{max, min};
use utils::get_grayscale_matrix_from_image;

lazy_static! {
    static ref SOBEL_GX: Array2<f64> = arr2(&[
        [1.0, 0.0, -1.0],
        [2.0, 0.0, -2.0],
        [1.0, 0.0, -1.0],
    ]);

    static ref SOBEL_GY: Array2<f64> = arr2(&[
        [1.0, 2.0, 1.0],
        [0.0, 0.0, 0.0],
        [-1.0, -2.0, -1.0]
    ]);
}

#[derive(Clone, Copy, Debug)]
pub struct Feature {
    pub u: u32,
    pub v: u32,
    pub score: f64,
}

impl Feature {
    pub fn distance_sq(&self, feat: &Feature) -> u32 {
        (self.u - feat.u).pow(2) + (self.v - feat.v).pow(2)
    }

    pub fn find_eigenvalues(m_xx: f64, m_xy: f64, m_yy: f64) -> (f64, f64) {
        let bb = ((m_xx + m_yy).powi(2) - 4.0 * (m_xx * m_yy - m_xy * m_xy)).sqrt();
        ((m_xx + m_yy + bb) / 2.0, (m_xx + m_yy - bb) / 2.0)
    }

    // FIXME: no longer detecting features
    pub fn detect(
        image: &RgbaImage,
        uxi: u32,
        uyi: u32,
        uw: u32,
        uh: u32,
    ) -> Vec<Feature> {
        let half_len = (BLOCKSIZE / 2) as i32;
        let img_width = image.width() as i32;
        let img_height = image.height() as i32;

        let xi = uxi as i32;
        let yi = uyi as i32;
        let pre_w = uw as i32;
        let pre_h = uh as i32;
        let w = min(img_width - xi, pre_w);
        let h = min(img_height - yi, pre_h);

        let shape = (w as usize, h as usize).f();
        let mut image_dx = Array2::<f64>::zeros(shape);
        let mut image_dy = Array2::<f64>::zeros(shape);

        let image_mat = get_grayscale_matrix_from_image(
            &image,
            xi as u32,
            yi as u32,
            w as u32,
            h as u32,
        );

        for x in 0..w {
            for y in 0..h {
                let image_win = image_mat.slice(s![
                    max(0, x - 1)..(min(w - 1, x + 1) + 1),
                    max(0, y - 1)..(min(h - 1, y + 1) + 1),
                ]);

                let gx_win = SOBEL_GX.slice(s![
                    (if x == 0 { 1 } else { 0 })..((if x == w - 1 { 1 } else { 2 }) + 1),
                    (if y == 0 { 1 } else { 0 })..((if y == h - 1 { 1 } else { 2 }) + 1),
                ]);

                let gy_win = SOBEL_GY.slice(s![
                    (if x == 0 { 1 } else { 0 })..((if x == w - 1 { 1 } else { 2 }) + 1),
                    (if y == 0 { 1 } else { 0 })..((if y == h - 1 { 1 } else { 2 }) + 1),
                ]);

                image_dx[[x as usize, y as usize]] = (&gx_win * &image_win).scalar_sum();
                image_dy[[x as usize, y as usize]] = (&gy_win * &image_win).scalar_sum();
            }
        }

        let image_dxx = &image_dx * &image_dx;
        let image_dxy = &image_dx * &image_dy;
        let image_dyy = &image_dy * &image_dy;
        let mut features: Vec<Feature> = vec![];

        // Corner and border pixels are prone to be recognized as features
        let total_len = BLOCKSIZE as i32;
        for x in total_len..(w - total_len) {
            for y in total_len..(h - total_len) {
                let mut m_xx = 0.0;
                let mut m_xy = 0.0;
                let mut m_yy = 0.0;

                let x_min = max(0, x - half_len);
                let y_min = max(0, y - half_len);
                let x_max = min(w - 1, x + half_len);
                let y_max = min(h - 1, y + half_len);

                for u in x_min..(x_max + 1) {
                    for v in y_min..(y_max + 1) {
                        m_xx += image_dxx[[u as usize, v as usize]];
                        m_xy += image_dxy[[u as usize, v as usize]];
                        m_yy += image_dyy[[u as usize, v as usize]];
                    }
                }

                let (ev1, ev2) = Self::find_eigenvalues(m_xx, m_xy, m_yy);

                let grad = if ev1 < ev2 { ev1 } else { ev2 };

                if grad < QUALITY_LEVEL {
                    continue;
                }

                let corner = Feature { u: (x + xi) as u32, v: (y + yi) as u32, score: grad };
                let mut should_push = features.len() < NUM_FEATURES;

                for idx in 0..features.len() {
                    let test_corner = features[idx];
                    let distance = corner.distance_sq(&test_corner);

                    if distance < MIN_DISTANCE_SQ {
                        if test_corner.score < corner.score {
                            features[idx] = corner;
                        }
                        should_push = false;
                        break;
                    } else if !should_push {
                        if test_corner.score < corner.score {
                            features[idx] = corner;
                            break;
                        }
                    }
                }

                if should_push {
                    features.push(corner);
                }
            }
        }

        features
    }
}
