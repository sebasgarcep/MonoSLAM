use feature::{DeserializedFeature, Feature};
use im::{open, GenericImage, RgbaImage};
use quaternion;
use ndarray::{arr1, arr2};
use shared_buffer::SharedBuffer;
use std;
use std::rc::Rc;
use std::thread;
use typedefs::{SharedMatrix, SharedVector};
use utils::load_from_json;
use video_stream::Camera;

#[derive(Debug, Deserialize)]
pub struct DeserializedInitAppState {
    xv: [f64; 13],
    pxx: [[f64; 13]; 13],
    features: Vec<DeserializedFeature>,
}

struct AppState {
    xv: SharedVector,
    pxx: SharedMatrix,
    features: Vec<Feature>,
}

pub struct MonoSLAM<C: Camera> {
    camera: C,
    image_buffer: SharedBuffer<RgbaImage>,
    landmark_buffer: SharedBuffer<Vec<(u32, u32)>>,
    state: Option<AppState>,
}

impl <C: Camera> MonoSLAM<C> {
    pub fn start (camera: C) -> (SharedBuffer<RgbaImage>, SharedBuffer<Vec<(u32, u32)>>) {
        let image_buffer = SharedBuffer::new();
        let image_buffer_clone = image_buffer.clone();

        let landmark_buffer = SharedBuffer::new();
        let landmark_buffer_clone = landmark_buffer.clone();

        thread::spawn(move || {
            let worker = MonoSLAM {
                camera,
                image_buffer: image_buffer_clone,
                landmark_buffer: landmark_buffer_clone,
                state: None
            };

            worker.process();
        });

        (image_buffer, landmark_buffer)
    }

    // FIXME: this initialization will only work when using MockStream. Use
    // dependency injection to fix this issue.
    fn initialize_state (&mut self, mut image: RgbaImage) {
        let raw_init_app_state: DeserializedInitAppState;
        raw_init_app_state = load_from_json("./data/init/mock.json");

        let mut app_state = AppState {
            xv: Rc::new(arr1(&raw_init_app_state.xv)),
            pxx: Rc::new(arr2(&raw_init_app_state.pxx)),
            features: vec![],
        };

        let xp_orig = Rc::new(app_state.xv.slice(s![0..3]).to_owned());
        for idx in 0..4 {
            let result_dynamic_image = open(format!("./data/init/known_patch{}.pgm", idx));
            let result_rgba_image = result_dynamic_image.map(|image| image.to_rgba());
            let window = result_rgba_image.unwrap();

            let mut best_diff = std::i64::MAX;
            let mut best_i = 0;
            let mut best_j = 0;
            for i in 100..200 {
                for j in 80..160 {
                    let subimage = image.sub_image(i, j, window.width(), window.height());

                    let mut diff = 0;
                    for x in 0..window.width() {
                        for y in 0..window.width() {
                            let subimage_pixel = subimage.get_pixel(x, y);
                            let window_pixel = window.get_pixel(x, y);

                            for px in 0..3 {
                                let spv = subimage_pixel.data[px] as i64;
                                let wpv = window_pixel.data[px] as i64;
                                diff += (spv - wpv).pow(2)
                            }
                        }
                    }

                    if diff < best_diff {
                        best_diff = diff;
                        best_i = i;
                        best_j = j;
                    }
                }
            }

            let h = arr1(&[
                (best_i as f64) + (window.width() as f64) / 2.0,
                (best_j as f64) + (window.height() as f64) / 2.0,
            ]);

            let px: f64 = 0.105;
            let py: f64 = 0.07425;
            let pz: f64 = 0.6;
            let distance = (px.powi(2) + py.powi(2) + pz.powi(2)).sqrt();
            let mut hrl_normalized = self.camera.unproject(&h);
            hrl_normalized.mapv_inplace(|e| e * distance);
            let xp_orig_clone = xp_orig.clone();
            let yi = Rc::new(&hrl_normalized + &*xp_orig_clone);

            let feature = Feature {
                yi,
                xp_orig: xp_orig_clone,
                image: window,
            };

            app_state.features.push(feature);
        }

        self.state = Some(app_state);
    }

    fn update_state (&mut self, image: RgbaImage) {
         // We assume that the state is known at this point
        let app_state = self.state.as_mut().unwrap();
    }

    fn process_image (&mut self, image: RgbaImage) {
        let is_uninitialized = self.state.is_none();
        if is_uninitialized {
            self.initialize_state(image);
        } else {
            self.update_state(image);
        }
    }

    fn process (mut self) {
        loop {
            let maybe_image = self.image_buffer.take();

            if let Some(image) = maybe_image {
                self.process_image(image);

                if let Some(app_state) = &self.state {
                    let rw = app_state.xv.slice(s![0..3]);
                    let qwr = app_state.xv.slice(s![3..7]).to_owned();
                    let rot_wr = quaternion::to_rotation_matrix(&qwr);
                    let rot_rw = rot_wr.t();
                    let mut landmarks: Vec<(u32, u32)> = app_state.features
                        .iter()
                        .map(|feature| {
                            let diff = &*feature.yi - &rw;
                            let hrl = rot_rw.dot(&diff);
                            let h = self.camera.project(&hrl);
                            (h[0] as u32, h[1] as u32)
                        })
                        .collect();

                    self.landmark_buffer.update(landmarks);
                }
            }
        }
    }
}
