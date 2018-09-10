use constants::{
    BLOCKSIZE,
    QUALITY_LEVEL,
    MIN_DISTANCE_SQ,
    NUM_FEATURES,
    PADDING,
};
use im::{RgbaImage};
use ndarray::{arr2, Array2, ShapeBuilder};

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

#[derive(Clone, Copy)]
pub struct Feature {
    pub x: u32,
    pub y: u32,
    pub score: f64,
}

impl Feature {
    pub fn distance(&self, feat: &Feature) -> u32 {
        (self.x - feat.x).pow(2) + (self.y - feat.y).pow(2)
    }
}

pub struct AnalysisResult {
    pub image: RgbaImage,
    pub features: Vec<Feature>,
}

pub struct Analyzer {}

impl Analyzer {
    pub fn new () -> Analyzer {
        Analyzer {}
    }

    pub fn find_eigenvalues(m_xx: f64, m_xy: f64, m_yy: f64) -> (f64, f64) {
        let bb = ((m_xx + m_yy).powi(2) - 4.0 * (m_xx * m_yy - m_xy * m_xy)).sqrt();
        ((m_xx + m_yy + bb) / 2.0, (m_xx + m_yy - bb) / 2.0)
    }

    pub fn process (&self, image: RgbaImage) -> AnalysisResult {
        let height = image.height();
        let width = image.width();

        let shape = (width as usize, height as usize).f();
        let mut image_mat = Array2::<f64>::zeros(shape);
        let mut image_dx = Array2::<f64>::zeros(shape);
        let mut image_dy = Array2::<f64>::zeros(shape);

        for x in 0..width {
            for y in 0..height {
                let pixel = image.get_pixel(x, y);
                let r = pixel.data[0] as f64;
                let g = pixel.data[1] as f64;
                let b = pixel.data[2] as f64;
                image_mat[[x as usize, y as usize]] = (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0;
            }
        }

        for x in 0..width {
            for y in 0..height {
                let image_win = image_mat.slice(s![
                    ((if x == 0 { x } else { x - 1 }) as usize)..(((if x == width - 1 { x } else { x + 1 }) + 1) as usize),
                    ((if y == 0 { y } else { y - 1 }) as usize)..(((if y == height - 1 { y } else { y + 1 }) + 1) as usize),
                ]);

                let gx_win = SOBEL_GX.slice(s![
                    (if x == 0 { 1 } else { 0 })..((if x == width - 1 { 1 } else { 2 }) + 1),
                    (if y == 0 { 1 } else { 0 })..((if y == height - 1 { 1 } else { 2 }) + 1),
                ]);

                let gy_win = SOBEL_GY.slice(s![
                    (if x == 0 { 1 } else { 0 })..((if x == width - 1 { 1 } else { 2 }) + 1),
                    (if y == 0 { 1 } else { 0 })..((if y == height - 1 { 1 } else { 2 }) + 1),
                ]);

                image_dx[[x as usize, y as usize]] = (&gx_win * &image_win).scalar_sum();
                image_dy[[x as usize, y as usize]] = (&gy_win * &image_win).scalar_sum();
            }
        }

        let image_dxx = &image_dx * &image_dx;
        let image_dxy = &image_dx * &image_dy;
        let image_dyy = &image_dy * &image_dy;
        let mut good_corners: Vec<Feature> = vec![];

        for x in PADDING..(width - PADDING) {
            for y in PADDING..(height - PADDING) {
                let mut m_xx = 0.0;
                let mut m_xy = 0.0;
                let mut m_yy = 0.0;

                let half_len = BLOCKSIZE / 2;
                let x_min = if x < half_len { 0 } else { x - half_len };
                let y_min = if y < half_len { 0 } else { y - half_len };
                let x_max = if x + half_len > width - 1 { width - 1 } else { x + half_len };
                let y_max = if y + half_len > height - 1 { height - 1 } else { y + half_len };

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

                let corner = Feature { x, y, score: grad };
                let mut should_push = good_corners.len() < NUM_FEATURES;

                for idx in 0..good_corners.len() {
                    let test_corner = good_corners[idx];
                    let distance = corner.distance(&test_corner);

                    if distance < MIN_DISTANCE_SQ {
                        if test_corner.score < corner.score {
                            good_corners[idx] = corner;
                        }
                        should_push = false;
                        break;
                    } else if !should_push {
                        if test_corner.score < corner.score {
                            good_corners[idx] = corner;
                            break;
                        }
                    }
                }

                if should_push {
                    good_corners.push(corner);
                }
            }
        }

        AnalysisResult {
            image,
            features: good_corners,
        }
    }
}
