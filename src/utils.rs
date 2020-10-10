use image::{DynamicImage, Pixel, RgbaImage};
use nalgebra::DMatrix;

pub fn image_to_matrix(img: &RgbaImage) -> DMatrix<f64> {
    let img_gray = DynamicImage::ImageRgba8(img.clone()).into_luma();
    let img_width = img_gray.width() as usize;
    let img_height = img_gray.height() as usize;
    DMatrix::<f64>::from_iterator(
        img_width,
        img_height,
        img_gray.pixels().map(|x| (x.channels()[0] as f64) / 255.0))
}
