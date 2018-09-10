use constants::{INIT_MAX_DISTANCE, INIT_MIN_DISTANCE, NUM_INIT_PARTICLES};

pub struct Initializer {
    points: Vec<f64>,
}

impl Initializer {
    pub fn new () -> Initializer {
        let mut points = Vec::with_capacity(NUM_INIT_PARTICLES);

        for idx in 0..NUM_INIT_PARTICLES {
            let ratio = (idx as f64) / ((NUM_INIT_PARTICLES - 1) as f64);
            points.push(INIT_MIN_DISTANCE + ratio * (INIT_MAX_DISTANCE - INIT_MIN_DISTANCE));
        }

        Initializer {
            points,
        }
    }
}
