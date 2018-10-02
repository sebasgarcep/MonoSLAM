use im::RgbaImage;
use std::sync::Mutex;
use std::sync::mpsc::{channel, Receiver, Sender};
use typedefs::{Matrix, Vector};

pub trait Camera: Send + 'static {
    fn width (&self) -> u32;

    fn height (&self) -> u32;

    fn project (&self, hrl: &Vector) -> Vector;

    /**
     *
     * When unprojecting from 2D to 3D space we cannot predict depth,
     * so we return a normalized vector, and the actual position vector
     * should be parallel to the direction vector we return.
     *
     */
    fn unproject (&self, h: &Vector) -> Vector;

    /*
    fn project_jacobian (&self, hrl: &Vector) -> Matrix;

    fn unproject_jacobian (&self, h: &Vector) -> Matrix;

    fn measurement_noise (&self, h: &Vector) -> Matrix;
    */
}

pub trait VideoStream {
    type C: Camera;

    fn get_camera (&self) -> Self::C;

    fn consume_image_transmitter (&self, sender: Mutex<Sender<RgbaImage>>);

    fn start_stream (&self) -> Receiver<RgbaImage> {
        let (tx, rx) = channel();
        let sync_tx = Mutex::new(tx.clone());
        self.consume_image_transmitter(sync_tx);
        rx
    }
}
