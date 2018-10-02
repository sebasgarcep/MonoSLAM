use constants::BLOCKSIZE;
use im::{GenericImage, RgbaImage};
use ndarray::{arr1, arr2};
use shared_buffer::SharedBuffer;
use std::rc::Rc;
use std::thread;
use typedefs::{SharedMatrix, SharedVector};
use utils::load_from_json;
use video_stream::Camera;

#[derive(Debug, Deserialize)]
struct DeserializedFeature {
    yi: [f64; 3],
    xp_orig: [f64; 7],
}

#[derive(Debug, Deserialize)]
pub struct DeserializedInitAppState {
    xv: [f64; 13],
    pxx: [[f64; 13]; 13],
    features: Vec<DeserializedFeature>,
}

struct Feature {
    yi: SharedVector,
    xp_orig: SharedVector,
    image: RgbaImage,
}

struct AppState {
    xv: SharedVector,
    pxx: SharedMatrix,
    features: Vec<Feature>,
}

pub struct MonoSLAM<C: Camera> {
    camera: C,
    image_buffer: SharedBuffer<RgbaImage>,
    state: Option<AppState>,
}

impl <C: Camera> MonoSLAM<C> {
    pub fn start (camera: C) -> SharedBuffer<RgbaImage> {
        let image_buffer = SharedBuffer::new();
        let image_buffer_clone = image_buffer.clone();

        thread::spawn(move || {
            let worker = MonoSLAM {
                camera,
                image_buffer: image_buffer_clone,
                state: None
            };

            worker.process();
        });

        image_buffer
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

        for raw_feature in raw_init_app_state.features.into_iter() {
            let yi = Rc::new(arr1(&raw_feature.yi));
            let h = self.camera.project(&yi);

            let feature = Feature {
                yi,
                xp_orig: Rc::new(arr1(&raw_feature.xp_orig)),
                image: image
                    .sub_image(
                        (h[0] - (BLOCKSIZE as f64) / 2.0) as u32,
                        (h[1] - (BLOCKSIZE as f64) / 2.0) as u32,
                        BLOCKSIZE,
                        BLOCKSIZE,
                    )
                    .to_image()
            };

            app_state.features.push(feature);
        }

        self.state = Some(app_state);
    }

    fn process_image (&mut self, image: RgbaImage) {
        let is_uninitialized = self.state.is_none();
        if is_uninitialized {
            self.initialize_state(image);
        } else {

        }
    }

    fn process (mut self) {
        loop {
            let maybe_image = self.image_buffer.take();
            if let Some(image) = maybe_image {
                self.process_image(image);
            }
        }
    }
}
