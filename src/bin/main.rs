extern crate image;
extern crate nalgebra;
extern crate piston_window;
extern crate kiss3d;
extern crate serde_json;
extern crate mono_slam;
extern crate stats;

use kiss3d::light::Light;
use kiss3d::window::Window;
use mono_slam::app_state::AppState;
use mono_slam::camera_model::WideAngleCameraModel;
use mono_slam::video_stream::MockStream;
use nalgebra::{DVector, Point3, Translation3, U3};

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

    let num_frames = 30;
    let app_state_vec: Vec<DVector<f32>> = video_stream
        .map(|(delta_t, mat)| {
            app_state.predict(delta_t);
            app_state.measure(mat);
            DVector::from_iterator(
                app_state.state_size(),
                app_state.state().iter().map(|v| *v as f32)
            )
        })
        .take(num_frames)
        .collect();

    let mut frame_idx = 0;
    let mut window = Window::new("MonoSLAM");

    // 3D Objects
    let mut robot = window.add_sphere(0.1);
    robot.set_color(1.0, 0.0, 0.0);

    // FIXME: Add orientation vector

    let mut features_vec = vec![];

    // Scenery
    window.set_light(Light::StickToCamera);


    while window.render() {
        // Draw axes
        // X axis
        window.draw_line(
            &Point3::new(-10.0, 0.0, 0.0),
            &Point3::new( 10.0, 0.0, 0.0),
            &Point3::new(1.0, 1.0, 1.0),
        );
        // Y axis
        window.draw_line(
            &Point3::new(0.0, -10.0, 0.0),
            &Point3::new(0.0,  10.0, 0.0),
            &Point3::new(1.0, 1.0, 1.0),
        );
        // Z axis
        window.draw_line(
            &Point3::new(0.0, 0.0, -10.0),
            &Point3::new(0.0, 0.0,  10.0),
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Modify 3D Objects
        frame_idx = (frame_idx + 1) % num_frames;
        let ref app_state = app_state_vec[frame_idx];
        let rw = app_state.fixed_rows::<U3>(0).clone_owned();
        let rw_trans = Translation3::from(rw);
        robot.set_local_translation(rw_trans);

        let num_features = (app_state.len() - 13) / 3;
        for feature_idx in 0..num_features {
            let yi = app_state.fixed_rows::<U3>(13 + 3 * feature_idx).clone_owned();
            let yi_trans = Translation3::from(yi);
            if feature_idx >= features_vec.len() {
                // FIXME: Change sphere for ellipsoid using covariances
                let mut feature_obj = window.add_sphere(0.03);
                feature_obj.set_color(1.0, 1.0, 0.0);
                features_vec.push(feature_obj);
            }
            let ref mut feature_obj = features_vec[feature_idx];
            feature_obj.set_local_translation(yi_trans);
        }
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
