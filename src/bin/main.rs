#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_assignments)]
#![allow(unused_imports)]
// FIXME: remove this once we are done prototyping
extern crate image;
extern crate nalgebra;
extern crate piston_window;
extern crate serde_json;
extern crate itertools;
extern crate mono_slam;
extern crate stats;

use mono_slam::camera::WideAngleCamera;
use mono_slam::constants::{BLOCKSIZE, NUM_FEATURES, MIN_DISTANCE_SQ, LINEAR_VELOCITY_NOISE, ANGULAR_VELOCITY_NOISE};
use mono_slam::detection::Detection;
use mono_slam::utils::{image_to_matrix, normalized_cross_correlation, unit_quaternion_from_angular_velocity};
use nalgebra::{DMatrix, Matrix2, Matrix4, Matrix4x3, Matrix6, MatrixN, MatrixMN, U3, U6, U7, U13, Vector2, Vector3, VectorN, Quaternion, UnitQuaternion};
use serde_json::Value;
use std::fs::File;
use std::io::BufReader;
use piston_window::{EventLoop, rectangle};
use std::cmp::{max, min};

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

fn get_dqwrnew_dqt(qwr: UnitQuaternion<f64>) -> Matrix4<f64> {
    Matrix4::new(
        qwr[0], -qwr[1], -qwr[2], -qwr[3],
        qwr[1],  qwr[0],  qwr[3], -qwr[2],
        qwr[2], -qwr[3],  qwr[0],  qwr[1],
        qwr[3],  qwr[2], -qwr[1],  qwr[0],
    )
}

fn get_dqt_domegar(wr: Vector3<f64>, delta_t: f64) -> Matrix4x3<f64> {
    let modulus = wr.norm();
    Matrix4x3::new(
        get_dq0_domega_a(wr[0], modulus, delta_t), get_dq0_domega_a(wr[1], modulus, delta_t), get_dq0_domega_a(wr[2], modulus, delta_t),
        get_dqa_domega_a(wr[0], modulus, delta_t), get_dqa_domega_b(wr[0], wr[1], modulus, delta_t), get_dqa_domega_b(wr[0], wr[2], modulus, delta_t),
        get_dqa_domega_b(wr[1], wr[0], modulus, delta_t), get_dqa_domega_a(wr[1], modulus, delta_t), get_dqa_domega_b(wr[1], wr[2], modulus, delta_t),
        get_dqa_domega_b(wr[2], wr[0], modulus, delta_t), get_dqa_domega_b(wr[2], wr[1], modulus, delta_t), get_dqa_domega_a(wr[2], modulus, delta_t),
    )
}

fn get_dq0_domega_a(omega_a: f64, modulus: f64, delta_t: f64) -> f64 {
    (-delta_t / 2.0) * (omega_a / modulus) * (modulus * delta_t / 2.0).sin()
}

fn get_dqa_domega_a(omega_a: f64, modulus: f64, delta_t: f64) -> f64 {
    (delta_t / 2.0) * omega_a.powi(2) / modulus.powi(2) * (modulus * delta_t / 2.0).cos()
    + (1.0 / modulus) * (1.0 - omega_a.powi(2) / modulus.powi(2)) * (modulus * delta_t / 2.0).sin()
}

fn get_dqa_domega_b(omega_a: f64, omega_b: f64, modulus: f64, delta_t: f64) -> f64 {
    (omega_a * omega_b / modulus.powi(2)) *
    ((delta_t / 2.0) * (modulus * delta_t / 2.0).cos() - (1.0 / modulus) * (modulus * delta_t / 2.0).sin())
}

struct FullFeature {
    yi: Vector3<f64>,
    xp_orig: VectorN<f64, U7>,
    patch: DMatrix<f64>,
}

struct PartialFeature {
    hwi: Vector3<f64>,
    xp_orig: VectorN<f64, U7>,
    patch: DMatrix<f64>,
    particles: Vec<f64>,
}

fn main() {
    let file = File::open("./data/init/mock.json").unwrap();
    let reader = BufReader::new(file);
    let init_json: Value = serde_json::from_reader(reader).unwrap();

    let xv = VectorN::<f64, U13>::from_iterator(prop_to_vec(&init_json, "xv"));

    let full_feature_vec: Vec<FullFeature> = init_json.get("features")
        .unwrap()
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
        .map(|(idx, feature_object)| {
            let img = image::open(format!("./data/init/known_patch{}.pgm", idx))
                .unwrap()
                .to_rgba();
            FullFeature {
                yi: VectorN::<f64, U3>::from_iterator(prop_to_vec(feature_object, "yi")),
                xp_orig: VectorN::<f64, U7>::from_iterator(prop_to_vec(feature_object, "xp_orig")),
                patch: image_to_matrix(&img),
            }
        })
        .collect();

    let pxx = MatrixN::<f64, U13>::from_iterator(
        itertools::concat(
            init_json.get("pxx")
                .unwrap()
                .as_array()
                .unwrap()
                .iter()
                .map(|a| obj_to_vec(a))
        )
    );

    // Get first frame and identify all features
    let camera = WideAngleCamera::new(
        320,
        240,
        25.0,
        195.0,
        195.0,
        162.0,
        125.0,
        0.0,
        9e-6,
        9e-6,
        0.0,
        0.0,
        1.0,
    );

    let img = image::open("./data/frames/rawoutput0000.pgm")
        .unwrap()
        .to_rgba();

    let rw = xv.fixed_rows::<U3>(0);
    let qwr = UnitQuaternion::from_quaternion(Quaternion::new(xv[3], xv[4], xv[5], xv[6]));
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
            let xp_orig = xv.fixed_rows::<U7>(0).clone_owned();
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

    let delta_t = 1.0 / 30.0;
    for idx in 1..2 {
        println!("{:?}", idx);

        let img = image::open(format!("./data/frames/rawoutput{:0>4}.pgm", idx))
            .unwrap()
            .to_rgba();

        let mat = image_to_matrix(&img);

        // Prediction step
        let rw = xv.fixed_rows::<U3>(0);
        let qwr = UnitQuaternion::from_quaternion(Quaternion::new(xv[3], xv[4], xv[5], xv[6]));
        let vw = xv.fixed_rows::<U3>(7);
        let wr = xv.fixed_rows::<U3>(10).clone_owned();

        // Assume linear and angular acceleration are 0. Predict next position and orientation.
        let rw_new = rw + vw * delta_t;
        let qwr_new = qwr * unit_quaternion_from_angular_velocity(wr * delta_t);

        // Calculate the covariance of the impulse vector.
        let pn = Matrix6::<f64>::new(
            LINEAR_VELOCITY_NOISE, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, LINEAR_VELOCITY_NOISE, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, LINEAR_VELOCITY_NOISE, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, ANGULAR_VELOCITY_NOISE, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, ANGULAR_VELOCITY_NOISE, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ANGULAR_VELOCITY_NOISE,
        );

        /*
            Calculate the Jacobian of the process vector with respect to the impulse vector:

            dfv_dn = | I * delta_t                0 |
                     | 0            dqwrnew_domegar |
                     | I                          0 |
                     | 0                          I |

            Let qt = q((wr + omegar) * delta_t). Then:

            dqwrnew_domegar = dqwrnew_dqt * dqt_domegar
        */
        let dqwrnew_dqt = get_dqwrnew_dqt(qwr);
        let dqt_domegar = get_dqt_domegar(wr, delta_t);
        let dqwrnew_domegar = dqwrnew_dqt * dqt_domegar;
        let dfv_dn = MatrixMN::<f64, U13, U6>::from_iterator(vec![
            delta_t, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, delta_t, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, delta_t, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, dqwrnew_domegar[(0, 0)], dqwrnew_domegar[(0, 1)], dqwrnew_domegar[(0, 2)],
            0.0, 0.0, 0.0, dqwrnew_domegar[(1, 0)], dqwrnew_domegar[(1, 1)], dqwrnew_domegar[(1, 2)],
            0.0, 0.0, 0.0, dqwrnew_domegar[(2, 0)], dqwrnew_domegar[(2, 1)], dqwrnew_domegar[(2, 2)],
            0.0, 0.0, 0.0, dqwrnew_domegar[(3, 0)], dqwrnew_domegar[(3, 1)], dqwrnew_domegar[(3, 2)],
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]);

        // Calculate the process noise covariance.
        let qn = dfv_dn * pn * dfv_dn.transpose();

        // Measurement step
        for full_feature in &full_feature_vec {
            let si = Matrix2::<f64>::identity() * camera.measurement_noise();
            let size = 10; // FIXME: There should be a way to define how large the search area is.
            let zi = camera.project(full_feature.yi - rw);
            let x_min = max(BLOCKSIZE / 2, (zi[0].round() as usize) - size);
            let x_max = min(camera.width() - BLOCKSIZE / 2, (zi[0].round() as usize) + size);
            let y_min = max(BLOCKSIZE / 2, (zi[1].round() as usize) - size);
            let y_max = min(camera.height() - BLOCKSIZE / 2, (zi[1].round() as usize) + size);
            let (mut best_x, mut best_y) = (0, 0);
            let mut best_corr = f64::NEG_INFINITY;
            for x in x_min..x_max {
                for y in y_min..y_max {
                    let patch = mat.slice((x - BLOCKSIZE / 2, y - BLOCKSIZE / 2), (BLOCKSIZE, BLOCKSIZE)).clone_owned();
                    let corr = normalized_cross_correlation(&full_feature.patch, &patch);
                    if best_x == 0 || corr > best_corr {
                        best_x = x;
                        best_y = y;
                        best_corr = corr;
                    }
                }
            }
        }

        for partial_feature in &partial_feature_vec {
            /*
            let particle_mean = stats::mean(partial_feature.particles.clone().into_iter());
            let particle_std = stats::stddev(partial_feature.particles.clone().into_iter());
            println!("{:?} {:?}", particle_mean, particle_std);
            */
        }
    }

    /*
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
