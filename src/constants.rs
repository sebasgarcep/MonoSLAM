pub const BLOCKSIZE: usize = 11;
pub const QUALITY_LEVEL: f64 = 1.0;
pub const NUM_FEATURES: usize = 100;

lazy_static! {
    pub static ref MIN_DISTANCE_SQ: f64 = (BLOCKSIZE as f64).powi(2) * 2.0;
}
