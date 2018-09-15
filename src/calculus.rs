use camera::Camera;
use ndarray::{arr2, Array, Array1, Array2};
use std::rc::Rc;
use utils::{convert_quaternion_to_matrix, quaternion_inverse};

type Vector = Array1<f64>;
type Matrix = Array2<f64>;
type SharedVector = Rc<Vector>;
type SharedMatrix = Rc<Matrix>;

/**
 *
 * Allows performing all the whacky necessary computations, while memoizing
 * as much of the results as possible. Also allows resetting the parts of the
 * computation tree that need to be reset for whatever computation comes next.
 *
 */
#[derive(Copy, Clone, Debug)]
pub enum VectorField {
    // Memoized Values
    QWR,
    RW,
    VR,
    WR,
    YI,
}

#[derive(Copy, Clone, Debug)]
pub enum MatrixField {
    // Memoized Values
    PXX,
    PXY,
    PYY,
    // Memoized Derivatives
    DH_DXP,
    DH_DXV,
    DH_DYI,
    DH_DZEROEDY,
    DXP_DXV,
    DZEROEDY_DXP,
    DZEROEDY_DYI,
}

#[derive(Default)]
pub struct CalculusContext {
    camera: Rc<Camera>,
    // Memoized Values
    qwr: Option<SharedVector>,
    rw: Option<SharedVector>,
    vr: Option<SharedVector>,
    wr: Option<SharedVector>,
    yi: Option<SharedVector>,
    pxx: Option<SharedMatrix>,
    pxy: Option<SharedMatrix>,
    pyy: Option<SharedMatrix>,
    // Memoized Derivatives
    dh_dxp: Option<SharedMatrix>,
    dh_dxv: Option<SharedMatrix>,
    dh_dyi: Option<SharedMatrix>,
    dh_dzeroedy: Option<SharedMatrix>,
    dxp_dxv: Option<SharedMatrix>,
    dzeroedy_dxp: Option<SharedMatrix>,
    dzeroedy_dyi: Option<SharedMatrix>,
}

impl CalculusContext {
    pub fn wrap_memoized <T> (val: T) -> Option<Rc<T>> {
        Some(Rc::new(val))
    }

    pub fn new (camera: Rc<Camera>) -> CalculusContext {
        CalculusContext { camera, ..Default::default() }
    }

    pub fn set_qwr (&mut self, qwr: Vector) {
        self.qwr = Self::wrap_memoized(qwr);
    }

    pub fn set_rw (&mut self, rw: Vector) {
        self.rw = Self::wrap_memoized(rw);
    }

    pub fn set_vr (&mut self, vr: Vector) {
        self.vr = Self::wrap_memoized(vr);
    }

    pub fn set_wr (&mut self, wr: Vector) {
        self.wr = Self::wrap_memoized(wr);
    }

    pub fn set_yi (&mut self, yi: Vector) {
        self.yi = Self::wrap_memoized(yi);
    }

    fn get_drqa_dq (q: &Vector, a: &Vector) -> Matrix {
        let q0 = q[0];
        let qx = q[1];
        let qy = q[2];
        let qz = q[3];

        let drdq0 = arr2(&[
            [2.0 * q0, - 2.0 * qz, 2.0 * qy],
            [2.0 * qz, 2.0 * q0, - 2.0 * qx],
            [- 2.0 * qy, 2.0 * qx, 2.0 * q0],
        ]);
        let drdqx = arr2(&[
            [2.0 * qx, 2.0 * qy, 2.0 * qz],
            [2.0 * qy, - 2.0 * qx, - 2.0 * q0],
            [2.0 * qz, 2.0 * q0, - 2.0 * qx],
        ]);
        let drdqy = arr2(&[
            [- 2.0 * qy, 2.0 * qx, 2.0 * q0],
            [2.0 * qx, 2.0 * qy, 2.0 * qz],
            [- 2.0 * q0, 2.0 * qz, - 2.0 * qy],
        ]);
        let drdqz = arr2(&[
            [- 2.0 * qz, - 2.0 * q0, 2.0 * qx],
            [2.0 * q0, - 2.0 * qz, 2.0 * qy],
            [2.0 * qx, 2.0 * qy, 2.0 * qz],
        ]);

        let mut drqadq = Array::zeros((3, 4));
        drqadq.column_mut(0).assign(&drdq0.dot(a));
        drqadq.column_mut(1).assign(&drdqx.dot(a));
        drqadq.column_mut(2).assign(&drdqy.dot(a));
        drqadq.column_mut(3).assign(&drdqz.dot(a));

        drqadq
    }

    fn get_dqbar_dq () -> Matrix {
        let mut dqbar_dq = Array::zeros((4, 4));
        for idx in 0..4 {
            let val = if idx == 0 {
                1.0
            } else {
                - 1.0
            };
            dqbar_dq[[idx, idx]] = val;
        }
        dqbar_dq
    }

    fn get_dzeroedy_dxp (&mut self) {
        let rw = self.get_vector_field(VectorField::RW);
        let yi = self.get_vector_field(VectorField::YI);
        let yi_diff_rw = &*yi - &*rw;
        let qwr = self.get_vector_field(VectorField::QWR);
        let qrw = quaternion_inverse(&qwr);
        let rot_rw = convert_quaternion_to_matrix(&qrw);
        let dzeroedy_dr = rot_rw.mapv(|e| - e);
        let dqrw_dq = Self::get_dqbar_dq();
        let dzeroedy_dqrw = Self::get_drqa_dq(&qrw, &yi_diff_rw);
        let dzeroedy_dq = dzeroedy_dqrw.dot(&dqrw_dq);
        let mut dzeroedy_dxp = Array::zeros((6, 7));
        dzeroedy_dxp.slice_mut(s![..3, ..3]).assign(&dzeroedy_dr);
        dzeroedy_dxp.slice_mut(s![..3, 3..]).assign(&dzeroedy_dq);
        self.dzeroedy_dxp = Self::wrap_memoized(dzeroedy_dxp);
    }

    fn get_dh_dzeroedy (&mut self) {
        let rw = self.get_vector_field(VectorField::RW);
        let yi = self.get_vector_field(VectorField::YI);
        let yi_diff_rw = &*yi - &*rw;
        let qwr = self.get_vector_field(VectorField::QWR);
        let qrw = quaternion_inverse(&qwr);
        let rot_rw = convert_quaternion_to_matrix(&qrw);
        let hrl = rot_rw.dot(&yi_diff_rw);
        let dh_dzeroedy = self.camera.projection_jacobian(&hrl);
        self.dh_dzeroedy = Self::wrap_memoized(dh_dzeroedy);
    }

    fn get_dh_dxp (&mut self) {
        let dh_dzeroedy = self.get_matrix_field(MatrixField::DH_DZEROEDY);
        let dzeroedy_dxp = self.get_matrix_field(MatrixField::DZEROEDY_DXP);
        self.dh_dxp = Self::wrap_memoized(dh_dzeroedy.dot(&*dzeroedy_dxp));
    }

    fn get_dh_dxv (&mut self) {
        let dxp_dxv = self.get_matrix_field(MatrixField::DXP_DXV);
        let dh_dxp = self.get_matrix_field(MatrixField::DH_DXP);
        self.dh_dxv = Self::wrap_memoized(dh_dxp.dot(&*dxp_dxv));
    }

    fn get_dxp_dxv (&mut self) {
        let mut dxp_dxv = Array::zeros((7, 13));
        for idx in 0..7 {
            dxp_dxv[[idx, idx]] = 1.0;
        }
        self.dxp_dxv = Self::wrap_memoized(dxp_dxv);
    }

    fn get_dzeroedy_dyi (&mut self) {
        let qwr = self.get_vector_field(VectorField::QWR);
        let qrw = quaternion_inverse(&qwr);
        let rot_rw = convert_quaternion_to_matrix(&qrw);
        let mut dzeroedy_dyi = Array::zeros((6, 3));
        dzeroedy_dyi.slice_mut(s![..3, ..3]).assign(&rot_rw);
        self.dzeroedy_dyi = Self::wrap_memoized(dzeroedy_dyi);
    }

    fn get_dh_dyi (&mut self) {
        let dh_dzeroedy = self.get_matrix_field(MatrixField::DH_DZEROEDY);
        let dzeroedy_dyi = self.get_matrix_field(MatrixField::DZEROEDY_DYI);
        let dh_dyi = dh_dzeroedy.dot(&*dzeroedy_dyi);
        self.dh_dyi = Self::wrap_memoized(dh_dyi);
    }

    pub fn get_si (&mut self, h: &Vector) -> Matrix {
        let dh_dxv = self.get_matrix_field(MatrixField::DH_DXV);
        let dh_dyi = self.get_matrix_field(MatrixField::DH_DYI);
        let pxx = self.get_matrix_field(MatrixField::PXX);
        let pxy = self.get_matrix_field(MatrixField::PXY);
        let pyy = self.get_matrix_field(MatrixField::PYY);
        let ri = self.camera.measurement_noise(h);

        let temp = dh_dxv.dot(&*pxy).dot(&dh_dyi.t());

        dh_dxv.dot(&*pxx).dot(&dh_dxv.t()) +
        &temp +
        &temp.t() +
        dh_dyi.dot(&*pyy).dot(&dh_dyi.t()) +
        ri
    }

    pub fn get_vector_field (&mut self, field: VectorField) -> SharedVector {
        let maybe_ref_counter = match field {
            VectorField::QWR => {
                &self.qwr
            },
            VectorField::RW => {
                &self.rw
            },
            VectorField::VR => {
                &self.vr
            },
            VectorField::WR => {
                &self.wr
            },
            VectorField::YI => {
                &self.yi
            },
        };
        maybe_ref_counter.as_ref().unwrap().clone()
    }

    pub fn get_matrix_field (&mut self, field: MatrixField) -> SharedMatrix {
        let maybe_ref_counter = match field {
            MatrixField::PXX => {
                &self.pxx
            },
            MatrixField::PXY => {
                &self.pxy
            },
            MatrixField::PYY => {
                &self.pyy
            },
            MatrixField::DH_DXP => {
                if self.dh_dxp.is_none() { self.get_dh_dxp(); }
                &self.dh_dxp
            },
            MatrixField::DH_DXV => {
                if self.dh_dxv.is_none() { self.get_dh_dxv(); }
                &self.dh_dxv
            },
            MatrixField::DH_DYI => {
                if self.dh_dyi.is_none() { self.get_dh_dyi(); }
                &self.dh_dyi
            },
            MatrixField::DH_DZEROEDY => {
                if self.dh_dzeroedy.is_none() { self.get_dh_dzeroedy(); }
                &self.dh_dzeroedy
            },
            MatrixField::DXP_DXV => {
                if self.dxp_dxv.is_none() { self.get_dxp_dxv(); }
                &self.dxp_dxv
            },
            MatrixField::DZEROEDY_DXP => {
                if self.dzeroedy_dxp.is_none() { self.get_dzeroedy_dxp(); }
                &self.dzeroedy_dxp
            },
            MatrixField::DZEROEDY_DYI => {
                if self.dzeroedy_dyi.is_none() { self.get_dzeroedy_dyi(); }
                &self.dzeroedy_dyi
            },
        };
        maybe_ref_counter.as_ref().unwrap().clone()
    }
}

