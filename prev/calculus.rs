use ndarray::{arr2, Array};
use std::rc::Rc;
use typedefs::{Matrix, SharedMatrix, SharedVector, Vector};
use utils::{convert_quaternion_to_matrix, quaternion_inverse};

/**
 *
 * Allows performing all the wacky necessary computations, while memoizing
 * as much of the results as possible. Also allows resetting the parts of the
 * computation tree that need to be reset for whatever computation comes next.
 *
 */
#[derive(Copy, Clone, Debug)]
pub enum VectorField {
    // Memoized Values
    H,
    QWR,
    RW,
    VW,
    WW,
    YI,
}

#[derive(Copy, Clone, Debug)]
pub enum MatrixField {
    // Memoized Values
    PXX,
    PXY,
    PYY,
    // Memoized Derived
    DH_DXP,
    DH_DXV,
    DH_DYI,
    DH_DZEROEDY,
    DXP_DXV,
    DZEROEDY_DXP,
    DZEROEDY_DYI,
    RI,
}

#[derive(Default)]
pub struct CalculusContext {
    camera: Camera,
    // Memoized Values
    h: Option<SharedVector>,
    qwr: Option<SharedVector>,
    rw: Option<SharedVector>,
    vw: Option<SharedVector>,
    ww: Option<SharedVector>,
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
    ri: Option<SharedMatrix>,
}

impl CalculusContext {
    pub fn wrap_memoized <T> (val: T) -> Option<Rc<T>> {
        Some(Rc::new(val))
    }

    pub fn new () -> CalculusContext {
        CalculusContext { camera: Camera::new(), ..Default::default() }
    }

    // Utils
    pub fn has_vector_field (&mut self, field: VectorField) -> bool {
        let value = match field {
            VectorField::H => &self.h,
            VectorField::QWR => &self.qwr,
            VectorField::RW => &self.rw,
            VectorField::VW => &self.vw,
            VectorField::WW => &self.ww,
            VectorField::YI => &self.yi,
        };

        value.is_some()
    }

    pub fn has_matrix_field (&mut self, field: MatrixField) -> bool {
        let value = match field {
            MatrixField::DH_DXP => &self.dh_dxp,
            MatrixField::DH_DXV => &self.dh_dxv,
            MatrixField::DH_DYI => &self.dh_dyi,
            MatrixField::DH_DZEROEDY => &self.dh_dzeroedy,
            MatrixField::DXP_DXV => &self.dxp_dxv,
            MatrixField::DZEROEDY_DXP => &self.dzeroedy_dxp,
            MatrixField::DZEROEDY_DYI => &self.dzeroedy_dyi,
            MatrixField::PXX => &self.pxx,
            MatrixField::PXY => &self.pxy,
            MatrixField::PYY => &self.pyy,
            MatrixField::RI => &self.ri,
        };

        value.is_some()
    }

    // Resets
    pub fn clear_vector_field (&mut self, field: VectorField) {
        self.reset_vector_field(field, None);
    }

    pub fn clear_matrix_field (&mut self, field: MatrixField) {
        self.reset_matrix_field(field, None);
    }

    pub fn reset_vector_field (&mut self, field: VectorField, value: Option<SharedVector>) {
        if value.is_none() && !self.has_vector_field(field) {
            return;
        }

        match field {
            VectorField::H => {
                self.h = value;
                self.clear_matrix_field(MatrixField::RI);
            },
            VectorField::QWR => {
                self.qwr = value;
                self.clear_matrix_field(MatrixField::DZEROEDY_DXP);
                self.clear_matrix_field(MatrixField::DH_DZEROEDY);
                self.clear_matrix_field(MatrixField::DZEROEDY_DYI);
            },
            VectorField::RW => {
                self.rw = value;
                self.clear_matrix_field(MatrixField::DZEROEDY_DXP);
                self.clear_matrix_field(MatrixField::DH_DZEROEDY);
            },
            VectorField::VW => {
                self.vw = value;
            },
            VectorField::WW => {
                self.ww = value;
            },
            VectorField::YI => {
                self.yi = value;
                self.clear_matrix_field(MatrixField::DH_DZEROEDY);
                self.clear_matrix_field(MatrixField::DZEROEDY_DXP);
            },
        };
    }

    pub fn reset_matrix_field (&mut self, field: MatrixField, value: Option<SharedMatrix>) {
        if value.is_none() && !self.has_matrix_field(field) {
            return;
        }

        match field {
            MatrixField::DH_DXP => {
                self.dh_dxp = value;
                self.clear_matrix_field(MatrixField::DH_DXV);
            },
            MatrixField::DH_DXV => {
                self.dh_dxv = value;
            },
            MatrixField::DH_DYI => {
                self.dh_dyi = value;
            },
            MatrixField::DH_DZEROEDY => {
                self.dh_dzeroedy = value;
                self.clear_matrix_field(MatrixField::DH_DXP);
                self.clear_matrix_field(MatrixField::DH_DYI);
            },
            MatrixField::DXP_DXV => {
                self.dxp_dxv = value;
                self.clear_matrix_field(MatrixField::DH_DXV);
            },
            MatrixField::DZEROEDY_DXP => {
                self.dzeroedy_dxp = value;
                self.clear_matrix_field(MatrixField::DH_DXP);
            },
            MatrixField::DZEROEDY_DYI => {
                self.dzeroedy_dyi = value;
                self.clear_matrix_field(MatrixField::DH_DYI);
            },
            MatrixField::PXX => {
                self.pxx = value;
            },
            MatrixField::PXY => {
                self.pxy = value;
            },
            MatrixField::PYY => {
                self.pyy = value;
            },
            MatrixField::RI => {
                self.ri = value;
            },
        };
    }

    // Sets
    pub fn set_robot_state (
        &mut self,
        rw: SharedVector,
        qwr: SharedVector,
        vw: SharedVector,
        ww: SharedVector,
        pxx: SharedMatrix,
    ) {
        self.reset_vector_field(VectorField::RW, Some(rw));
        self.reset_vector_field(VectorField::QWR, Some(qwr));
        self.reset_vector_field(VectorField::VW, Some(vw));
        self.reset_vector_field(VectorField::WW, Some(ww));
        self.reset_matrix_field(MatrixField::PXX, Some(pxx));
    }

    pub fn set_feature_state (
        &mut self,
        h: SharedVector,
        yi: SharedVector,
        pxy: SharedMatrix,
        pyy: SharedMatrix,
    ) {
        self.reset_vector_field(VectorField::H, Some(h));
        self.reset_vector_field(VectorField::YI, Some(yi));
        self.reset_matrix_field(MatrixField::PXY, Some(pxy));
        self.reset_matrix_field(MatrixField::PYY, Some(pyy));
    }

    // Gets
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

    fn get_ri (&mut self) {
        let h = self.get_vector_field(VectorField::H);
        let ri = self.camera.measurement_noise(&*h);
        self.ri = Self::wrap_memoized(ri);
    }

    pub fn get_si (&mut self) -> Matrix {
        let dh_dxv = self.get_matrix_field(MatrixField::DH_DXV);
        let dh_dyi = self.get_matrix_field(MatrixField::DH_DYI);
        let pxx = self.get_matrix_field(MatrixField::PXX);
        let pxy = self.get_matrix_field(MatrixField::PXY);
        let pyy = self.get_matrix_field(MatrixField::PYY);
        let ri = self.get_matrix_field(MatrixField::RI);

        let temp = dh_dxv.dot(&*pxy).dot(&dh_dyi.t());

        dh_dxv.dot(&*pxx).dot(&dh_dxv.t()) +
        &temp +
        &temp.t() +
        dh_dyi.dot(&*pyy).dot(&dh_dyi.t()) +
        &*ri
    }

    // General
    pub fn get_vector_field (&mut self, field: VectorField) -> SharedVector {
        let maybe_ref_counter = match field {
            VectorField::H => {
                &self.h
            },
            VectorField::QWR => {
                &self.qwr
            },
            VectorField::RW => {
                &self.rw
            },
            VectorField::VW => {
                &self.vw
            },
            VectorField::WW => {
                &self.ww
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
            MatrixField::RI => {
                if self.ri.is_none() { self.get_ri(); }
                &self.ri
            },
        };
        maybe_ref_counter.as_ref().unwrap().clone()
    }
}

