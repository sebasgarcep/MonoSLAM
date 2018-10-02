use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub struct SharedBuffer<T> {
    buffer: Arc<Mutex<Option<T>>>,
}

impl <T> SharedBuffer<T> {
    pub fn new () -> SharedBuffer<T> {
        SharedBuffer { buffer: Arc::new(Mutex::new(None)) }
    }

    pub fn update (&self, content: T) {
        let mut data = Mutex::lock(&self.buffer).unwrap();
        *data = Some(content);
    }

    pub fn take (&self) -> Option<T> {
        let mut guard = Mutex::lock(&self.buffer).unwrap();
        guard.take()
    }
}
