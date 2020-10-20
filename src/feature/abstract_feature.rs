use nalgebra::{DefaultAllocator, Dim, DimName, DMatrix, MatrixMN, U1, U13, VectorN};
use nalgebra::allocator::Allocator;

pub struct AbstractFeature<D: Dim + DimName>
where DefaultAllocator: Allocator<f64, D, U1>
                      + Allocator<f64, U13, D>
                      + Allocator<f64, D, D>
{
    pub y: VectorN<f64, D>,
    pub pxy: MatrixMN<f64, U13, D>,
    pub pyy: MatrixMN<f64, D, D>,
    pub patch: DMatrix<f64>,
}

impl <D: Dim + DimName> AbstractFeature<D>
where DefaultAllocator: Allocator<f64, D, U1>
                      + Allocator<f64, U13, D>
                      + Allocator<f64, D, D>
{
    pub fn new(
        y: VectorN<f64, D>,
        pxy: MatrixMN<f64, U13, D>,
        pyy: MatrixMN<f64, D, D>,
        patch: DMatrix<f64>,
    ) -> Self {
        AbstractFeature {
            y,
            pxy,
            pyy,
            patch,
        }
    }
}
