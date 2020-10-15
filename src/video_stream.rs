use nalgebra::DMatrix;
use utils::image_to_matrix;

type DeltaTime = f64;

pub struct MockStream {
    idx: usize,
}

impl MockStream {
    pub fn new() -> MockStream {
        MockStream { idx: 0 }
    }
}

impl Iterator for MockStream {
    type Item = (DeltaTime, DMatrix<f64>);

    fn next(&mut self) -> Option<Self::Item> {
        let delta_t = 1.0 / 30.0;
        let item = image::open(format!("./data/frames/rawoutput{:0>4}.pgm", self.idx))
            .ok()
            .map(|i| i.to_rgba())
            .map(|i| image_to_matrix(&i))
            .map(|i| (delta_t, i));

        self.idx += 1;

        item
    }
}
