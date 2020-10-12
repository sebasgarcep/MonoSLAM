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

pub fn normalized_cross_correlation(template: &DMatrix<f64>, image: &DMatrix<f64>) -> f64 {
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
