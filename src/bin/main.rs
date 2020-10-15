#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_assignments)]
#![allow(unused_imports)]
// FIXME: remove this once we are done prototyping
extern crate image;
extern crate nalgebra;
extern crate piston_window;
extern crate serde_json;
extern crate mono_slam;
extern crate stats;

use mono_slam::app_state::AppState;
use mono_slam::camera_model::WideAngleCameraModel;
use mono_slam::constants::{BLOCKSIZE, NUM_FEATURES, NUM_SIGMA, MIN_DISTANCE_SQ, LINEAR_VELOCITY_NOISE, ANGULAR_VELOCITY_NOISE};
use mono_slam::detection::Detection;
use mono_slam::utils::{image_to_matrix, normalized_cross_correlation, unit_quaternion_from_angular_velocity, matrix_set_block};
use mono_slam::video_stream::MockStream;
use nalgebra::{DMatrix, DVector, Matrix2, Matrix4, Matrix4x3, Matrix6, MatrixN, MatrixMN, U2, U3, U6, U7, U13, Vector2, Vector3, VectorN, Quaternion, UnitQuaternion};
use serde_json::Value;
use std::fs::File;
use std::io::BufReader;
use piston_window::{EventLoop, rectangle};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;

fn main() {
    let mut app_state = AppState::from_json(
        "./data/init/mock.json",
        vec![
            "./data/init/known_patch0.pgm",
            "./data/init/known_patch1.pgm",
            "./data/init/known_patch2.pgm",
            "./data/init/known_patch3.pgm",
        ],
        WideAngleCameraModel::from_json("./data/cameras/mock.json"),
    );

    let mut video_stream = MockStream::new();

    // Skip Frame 0
    video_stream.next();

    for i in 1..2 {
        let (delta_t, mat) = video_stream.next().unwrap();
        app_state.predict(delta_t);
        app_state.measure(delta_t, mat);
    }

    /*
    // Get first frame and identify all features

    let rw = x_state.fixed_rows::<U3>(0);
    let qwr = UnitQuaternion::from_quaternion(Quaternion::new(x_state[3], x_state[4], x_state[5], x_state[6]));
    let full_feature_rel_pos: Vec<_> = full_feature_vec
        .iter()
        .map(|feature| {
            let zp = feature.yi - rw;
            let zi = qwr.inverse_transform_vector(&zp);
            camera.project(zi)
        })
        .collect();

    let mat = image_to_matrix(&img);
    let mut detection_vec: Vec<Detection> = Vec::with_capacity(NUM_FEATURES);
    for detection in Detection::detect(&mat) {
        // If we have found enough features, stop
        if detection_vec.len() + full_feature_vec.len() >= NUM_FEATURES { break; }
        // Determine if this feature is OK by comparing against all previously picked features
        let should_pick = detection_vec
            .iter()
            .map(|feature| feature.pos)
            .chain(full_feature_rel_pos.clone())
            .all(|feature_pos| {
                // The distance between the top-left corners has to be larger than a certain value
                // so that the regions do not overlap
                let distance_sq = (detection.pos - feature_pos).norm_squared();
                // If they overlap ignore this feature
                distance_sq >= *MIN_DISTANCE_SQ
            });
        // If it passes the test add to the list of detected features
        if should_pick { detection_vec.push(detection); }
    }

    let partial_feature_vec: Vec<PartialFeature> = detection_vec
        .iter()
        .map(|detection| {
            let mut hri = camera.unproject(detection.pos);
            hri = hri / hri.norm();
            let hwi = qwr.transform_vector(&hri);
            let xp_orig = x_state.fixed_rows::<U7>(0).clone_owned();
            let patch_top_corner = ((detection.pos[0] as usize) - BLOCKSIZE / 2, (detection.pos[1] as usize) - BLOCKSIZE / 2);
            let patch = mat.slice(patch_top_corner, (BLOCKSIZE, BLOCKSIZE)).clone_owned();
            let particles = (0..100).map(|n| 0.5 + ((n as f64) / 100.0) * (5.0 - 0.5)).collect();
            PartialFeature {
                hwi,
                xp_orig,
                patch,
                particles,
            }
        })
        .collect();

    for idx in 1..2 {
        println!("{:?}", x_state_next);

        /*
        for partial_feature in &partial_feature_vec {
            let particle_mean = stats::mean(partial_feature.particles.clone().into_iter());
            let particle_std = stats::stddev(partial_feature.particles.clone().into_iter());
            println!("{:?} {:?}", particle_mean, particle_std);
        }
        */
    }

    let mut window: piston_window::PistonWindow =
        piston_window::WindowSettings::new("Raytracer", [WIDTH, HEIGHT])
            .resizable(false)
            .exit_on_esc(true)
            .build()
            .unwrap_or_else(|_e| { panic!("Could not create window!")});

    window.set_max_fps(25);

    let mut idx = 0;
    while let Some(e) = window.next() {
        let tex = piston_window::Texture::from_image(
            &mut window.create_texture_context(),
            &img,
            &piston_window::TextureSettings::new())
            .unwrap();

        window.draw_2d(&e, |c, g, _| {
            piston_window::clear([1.0; 4], g);
            piston_window::image(&tex, c.transform, g);
            for feature in &feature_rel_pos {
                rectangle(
                    [1.0, 0.0, 0.0, 0.5],
                    [feature[0] - 5.0, feature[1] - 5.0, 11.0, 11.0],
                    c.transform,
                    g,
                );
            }
            for detection in &detection_vec {
                rectangle(
                    [0.0, 0.0, 1.0, 0.5],
                    [detection.pos[0] - 5.0, detection.pos[1] - 5.0, 11.0, 11.0],
                    c.transform,
                    g,
                );
            }
        });

        idx = (idx + 1) % 1000;
    }
    */
}
