use im::RgbaImage;
use shared_buffer::SharedBuffer;
use std::thread;

struct MonoSLAMWorker {
    image_buffer: SharedBuffer<RgbaImage>,
    state: Option<()>,
}

impl MonoSLAMWorker {
    pub fn new (image_buffer: SharedBuffer<RgbaImage>) -> MonoSLAMWorker {
        MonoSLAMWorker {
            image_buffer,
            state: None,
        }
    }

    // FIXME: this initialization will only work when using MockStream. Use
    // dependency injection to fix this issue.
    fn initialize_state (&mut self, image: RgbaImage) {

    }

    fn process_image (&mut self, image: RgbaImage) {
        if let Some(()) = self.state {
            // FIXME: Update current state
        } else {
            self.initialize_state(image);
        }
    }

    pub fn start (mut self) {
        loop {
            let maybe_image = self.image_buffer.take();
            if let Some(image) = maybe_image {
                self.process_image(image);
            }
        }
    }
}

pub struct MonoSLAM {
    image_buffer: SharedBuffer<RgbaImage>,
}

impl MonoSLAM {
    pub fn new () -> MonoSLAM {
        MonoSLAM {
            image_buffer: SharedBuffer::new(),
        }
    }

    pub fn start (&self) {
        let image_buffer = self.get_image_buffer();
        thread::spawn(move || {
            let worker = MonoSLAMWorker::new(image_buffer);
            worker.start();
        });
    }

    pub fn get_image_buffer (&self) -> SharedBuffer<RgbaImage> {
        self.image_buffer.clone()
    }
}
