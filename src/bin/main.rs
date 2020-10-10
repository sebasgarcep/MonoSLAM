extern crate image;
extern crate nalgebra;
extern crate piston_window;
extern crate serde_json;
extern crate itertools;
extern crate mono_slam;

use image::{DynamicImage, Pixel};
use mono_slam::detection::Detection;
use nalgebra::{DMatrix, MatrixN, U3, U7, U13, VectorN};
use serde_json::Value;
use std::fs::File;
use std::io::BufReader;

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;

fn obj_to_vec(obj: &Value) -> Vec<f64> {
    obj.as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect()
}

fn prop_to_vec(obj: &Value, prop: &str) -> Vec<f64> {
    obj_to_vec(obj.get(prop).unwrap())
}

fn main() {
    let file = File::open("./data/init/mock.json").unwrap();
    let reader = BufReader::new(file);
    let init_json: Value = serde_json::from_reader(reader).unwrap();

    let xv = VectorN::<f64, U13>::from_iterator(prop_to_vec(&init_json, "xv"));
    println!("{:?}", xv);

    for (idx, feature_object) in init_json.get("features").unwrap().as_array().unwrap().iter().enumerate() {
        println!("{:?}", idx);
        let yi = VectorN::<f64, U3>::from_iterator(prop_to_vec(feature_object, "yi"));
        println!("{:?}", yi);
        let xp = VectorN::<f64, U7>::from_iterator(prop_to_vec(feature_object, "xp_orig"));
        println!("{:?}", xp);
        let img = image::open(format!("./data/init/known_patch{}.pgm", idx))
            .unwrap()
            .to_rgba();
        println!("{:?}", img.dimensions());
    }

    let pxx = MatrixN::<f64, U13>::from_iterator(
        itertools::concat(
            init_json.get("pxx")
                .unwrap()
                .as_array()
                .unwrap()
                .iter()
                .map(|a| obj_to_vec(a))));
    println!("{:?}", pxx);

    // Get first frame and identify all features
    let img = image::open("./data/frames/rawoutput0000.pgm")
        .unwrap()
        .to_rgba();

    let img_width = img.width() as usize;
    let img_height = img.height() as usize;

    // Transform RGBA image to grayscale image
    let img_gray = DynamicImage::ImageRgba8(img.clone()).into_luma();
    let mat = DMatrix::<f64>::from_iterator(
        img_width,
        img_height,
        img_gray.pixels().map(|x| (x.channels()[0] as f64) / 255.0));
}
