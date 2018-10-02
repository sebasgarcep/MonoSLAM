use im::{open, RgbaImage};
use std::cmp::max;
use std::sync::Mutex;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};
use std::thread;
use std::thread::sleep;
use video_stream::base::VideoStream;
use video_stream::camera_params::CameraParams;
use video_stream::wide_angle_camera::WideAngleCamera;

pub struct MockStream {}

impl MockStream {
    pub fn new () -> MockStream {
        MockStream {}
    }

    fn send_image (idx: &mut u64, sender: &Mutex<Sender<RgbaImage>>) {
        let result_dynamic_image = open(format!("./data/frames/rawoutput{:0>4}.pgm", idx));
        let result_rgba_image = result_dynamic_image.map(|image| image.to_rgba());
        let image = result_rgba_image.unwrap();
        let internal_sender = Mutex::lock(sender).unwrap();
        internal_sender.send(image).unwrap();
        *idx += 1;
    }

    fn start_worker (params: CameraParams, sender: Mutex<Sender<RgbaImage>>) {
        let fps = params.fps;
        let period = 1.0 / fps;
        let period_ns = (period * 1e9) as i64;
        let mut idx = 0;

        Self::send_image(&mut idx, &sender);

        while idx < 1000 {
            let now = Instant::now();
            Self::send_image(&mut idx, &sender);
            let elapsed = now.elapsed().subsec_nanos() as i64;
            let missing = max(0, period_ns - elapsed);

            if missing > 0 {
                let duration = Duration::from_nanos(missing as u64);
                sleep(duration);
            }
        }
    }
}

impl VideoStream for MockStream {
    type C = WideAngleCamera;

    fn get_camera (&self) -> WideAngleCamera {
        let camera_params = CameraParams::load_from_json("./data/cameras/mock.json");
        WideAngleCamera::new(camera_params)
    }

    fn consume_image_transmitter (&self, sender: Mutex<Sender<RgbaImage>>) {
        thread::spawn(move || {
            let camera_params = CameraParams::load_from_json("./data/cameras/mock.json");
            MockStream::start_worker(camera_params, sender);
        });
    }
}
