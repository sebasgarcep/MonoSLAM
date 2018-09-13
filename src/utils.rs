use im::RgbaImage;
use ndarray::{Array2, ShapeBuilder};

pub fn get_grayscale_matrix_from_image (
    image: &RgbaImage,
    x: u32,
    y: u32,
    w: u32,
    h: u32,
) -> Array2<f64> {
    let shape = (w as usize, h as usize).f();
    let mut image_mat = Array2::<f64>::zeros(shape);

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
