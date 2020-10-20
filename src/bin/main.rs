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
                app_state.full_state_size(),
                app_state.full_state().iter().map(|v| *v as f32),
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
    */
}
