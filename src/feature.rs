#[derive(Clone, Copy)]
pub struct Feature {
    pub x: u32,
    pub y: u32,
    pub score: f64,
}

impl Feature {
    pub fn distance(&self, feat: &Feature) -> u32 {
        (self.x - feat.x).pow(2) + (self.y - feat.y).pow(2)
    }
}
