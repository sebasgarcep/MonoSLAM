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
use nalgebra::{DMatrix, DVector, Matrix2, Matrix4, Matrix4x3, Matrix6, MatrixN, MatrixMN, U3, U6, U7, U13, Vector2, Vector3, VectorN, Quaternion, UnitQuaternion};
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

fn get_dqwrnew_dqwr(qwr: UnitQuaternion<f64>) -> Matrix4<f64> {
    Matrix4::new(
        qwr[0], -qwr[1], -qwr[2], -qwr[3],
        qwr[1],  qwr[0],  qwr[3], -qwr[2],
        qwr[2], -qwr[3],  qwr[0],  qwr[1],
        qwr[3],  qwr[2], -qwr[1],  qwr[0],
    )
}

fn get_dqwrnew_dqt(qwr: UnitQuaternion<f64>) -> Matrix4<f64> {
    Matrix4::new(
        qwr[0], -qwr[1], -qwr[2], -qwr[3],
        qwr[1],  qwr[0], -qwr[3],  qwr[2],
        qwr[2],  qwr[3],  qwr[0], -qwr[1],
        qwr[3], -qwr[2],  qwr[1],  qwr[0],
    )
}

fn get_dqt_dwr(wr: Vector3<f64>, delta_t: f64) -> Matrix4x3<f64> {
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

    // Initialize state
    let xv = prop_to_vec(&init_json, "xv");

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

    let pxx: Vec<_> =
        init_json.get("pxx")
            .unwrap()
            .as_array()
            .unwrap()
            .iter()
            .map(|a| obj_to_vec(a))
            .collect();

    let state_size = 13 + full_feature_vec.len() * 3;
    let mut x_state = DVector::<f64>::zeros(state_size);
    for i in 0..13 {
        x_state[i] = xv[i];
    }
    for j in 0..full_feature_vec.len() {
        for i in 0..3 {
            x_state[13 + 3 * j + i] = full_feature_vec[j].yi[i];
        }
    }
    let mut p_state = DMatrix::<f64>::zeros(state_size, state_size);
    for i in 0..13 {
        for j in 0..13 {
            p_state[(i, j)] = pxx[i][j];
        }
    }

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

    let delta_t = 1.0 / 30.0;
    for idx in 1..2 {
        println!("{:?}", idx);

        let img = image::open(format!("./data/frames/rawoutput{:0>4}.pgm", idx))
            .unwrap()
            .to_rgba();

        let mat = image_to_matrix(&img);

        // Prediction step
        let rw = x_state.fixed_rows::<U3>(0);
        let qwr = UnitQuaternion::from_quaternion(Quaternion::new(x_state[3], x_state[4], x_state[5], x_state[6]));
        let vw = x_state.fixed_rows::<U3>(7);
        let wr = x_state.fixed_rows::<U3>(10).clone_owned();

        // Assume linear and angular acceleration are 0. Predict next position and orientation.
        let rw_new = rw + vw * delta_t;
        let qwr_new = qwr * unit_quaternion_from_angular_velocity(wr * delta_t);
        let mut fv = x_state.clone();
        for i in 0..3 {
            fv[i] = rw_new[i];
        }
        for i in 0..4 {
            fv[3 + i] = qwr_new[i];
        }

        // Calculate the covariance of the impulse vector.
        let pn = Matrix6::<f64>::new(
            LINEAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, LINEAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, LINEAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2), 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, ANGULAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2), 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, ANGULAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2), 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, ANGULAR_VELOCITY_NOISE.powi(2) * delta_t.powi(2),
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
        let dqwrnew_dqt = get_dqwrnew_dqt(qwr); // FIXME: double check this!
        let dqt_domegar = get_dqt_dwr(wr, delta_t); // FIXME: double check this!
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
        let qv = dfv_dn * pn * dfv_dn.transpose();

        /*
            The motion model maps it xv -> fv(xv) and yi -> yi. Therefore the jacobian of this model is:

            df_dx = | dfv_dxv    0 |
                    |       0    I |

            The Jacobian of fv with respect to xv is:

            dfv_dxv = | I    0               I * delta_t    0           |
                      | 0    dqwrnew_dqwr    0              dqwrnew_dwr |
                      | 0    0               I              0           |
                      | 0    0               0              I           |

            Let qt = q((wr + omegar) * delta_t). Then:

            dqwrnew_dwr = dqwrnew_dqt * dqt_dwr

            Same calculation as before.
        */
        let dqwrnew_dqwr = get_dqwrnew_dqwr(qwr);
        let dqwrnew_dwr = dqwrnew_domegar;
        let dfv_dxv = MatrixMN::<f64, U13, U13>::from_iterator(vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, delta_t, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, delta_t, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, delta_t, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, dqwrnew_dqwr[(0, 0)], dqwrnew_dqwr[(0, 1)], dqwrnew_dqwr[(0, 2)], dqwrnew_dqwr[(0, 3)], 0.0, 0.0, 0.0, dqwrnew_dwr[(0, 0)], dqwrnew_dwr[(0, 1)], dqwrnew_dwr[(0, 2)],
            0.0, 0.0, 0.0, dqwrnew_dqwr[(1, 0)], dqwrnew_dqwr[(1, 1)], dqwrnew_dqwr[(1, 2)], dqwrnew_dqwr[(1, 3)], 0.0, 0.0, 0.0, dqwrnew_dwr[(1, 0)], dqwrnew_dwr[(1, 1)], dqwrnew_dwr[(1, 2)],
            0.0, 0.0, 0.0, dqwrnew_dqwr[(2, 0)], dqwrnew_dqwr[(2, 1)], dqwrnew_dqwr[(2, 2)], dqwrnew_dqwr[(2, 3)], 0.0, 0.0, 0.0, dqwrnew_dwr[(2, 0)], dqwrnew_dwr[(2, 1)], dqwrnew_dwr[(2, 2)],
            0.0, 0.0, 0.0, dqwrnew_dqwr[(3, 0)], dqwrnew_dqwr[(3, 1)], dqwrnew_dqwr[(3, 2)], dqwrnew_dqwr[(3, 3)], 0.0, 0.0, 0.0, dqwrnew_dwr[(3, 0)], dqwrnew_dwr[(3, 1)], dqwrnew_dwr[(3, 2)],
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ]);

        /*
            The propagation of the covariance matrix is:

            = df_dx * P * df_dx^T + Q
            = | dfv_dxv    0 |   | Pxx Pxy |   | dfv_dxv^T    0 |
              |       0    I | * | Pyx Pyy | * |       0      I | + Q
            = | dfv_dxv * Pxx * dfv_dxv^T    dfv_dxv * Pxy |
              | Pyx * dfv_dxv^T                        Pyy | + Q

            where Q is:

            Q = | Qv    0 |
                |  0    0 |
        */
        let mut p_state_new = DMatrix::<f64>::zeros(state_size, state_size);
        // dfv_dxv * Pxx * dfv_dxv^T
        let pxx_new = dfv_dxv * p_state.fixed_slice::<U13, U13>(0, 0) * dfv_dxv.transpose() + qv;
        for i in 0..13 {
            for j in 0..13 {
                p_state_new[(i, j)] = pxx_new[(i, j)];
            }
        }
        // dfv_dxv * Pxy
        let pxy_new = dfv_dxv * p_state.slice((0, 13), (13, state_size - 13));
        for i in 0..13 {
            for j in 13..state_size {
                p_state_new[(i, j)] = pxy_new[(i, j - 13)];
                p_state_new[(j, i)] = pxy_new[(i, j - 13)];
            }
        }
        // Pyy
        for i in 13..state_size {
            for j in 13..state_size {
                p_state_new[(i, j)] = p_state[(i, j)];
            }
        }

        // Measurement step
        // Calculate innovation vector
        let num_active_features = full_feature_vec.len();
        let mut y_innov = DVector::<f64>::zeros(num_active_features * 2);
        for i in 0..num_active_features {
            let ref full_feature = full_feature_vec[i];
            let size = 10;
            let h_xi = camera.project(full_feature.yi - rw);
            let h_xi_x = h_xi[0].round() as usize;
            let h_xi_y = h_xi[1].round() as usize;
            // Find best x, y
            let x_min = max(BLOCKSIZE / 2, h_xi_x - size);
            let x_max = min(camera.width() - BLOCKSIZE / 2, h_xi_x + size);
            let y_min = max(BLOCKSIZE / 2, h_xi_y - size);
            let y_max = min(camera.height() - BLOCKSIZE / 2, h_xi_y + size);
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
            // Set the innovation vector = measurements - camera projection of current state of features
            y_innov[2 * i] = (best_x - h_xi_x) as f64;
            y_innov[2 * i + 1] = (best_y - h_xi_y) as f64;
        }

        // The matrix Rk is simply sigma_R^2 * I, where sigma_R = 1 is the camera error due to discretization errors.

        // FIXME: calculate Jacobian of observation operator.

        /*
        for partial_feature in &partial_feature_vec {
            let particle_mean = stats::mean(partial_feature.particles.clone().into_iter());
            let particle_std = stats::stddev(partial_feature.particles.clone().into_iter());
            println!("{:?} {:?}", particle_mean, particle_std);
        }
        */
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
