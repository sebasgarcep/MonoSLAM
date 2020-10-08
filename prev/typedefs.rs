use ndarray::{Array1, Array2};
use std::rc::Rc;

pub type Vector = Array1<f64>;
pub type Matrix = Array2<f64>;
pub type SharedVector = Rc<Vector>;
pub type SharedMatrix = Rc<Matrix>;
