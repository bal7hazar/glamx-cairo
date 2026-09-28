//! Gas benchmarks of `glamx::eigen3` and of the closed form kept in `benches::alt::eigen3` (the
//! `alt_*` benches).
//!
//! The Jacobi iteration stops as soon as the off-diagonal entries are zero, a plane whose pivot
//! is already zero is skipped, and the polish and refinement only run after a rotation: the
//! cost depends on the input, so the decomposition and the eigenvalues are measured on a generic
//! matrix (10 rotations), a diagonal one (no rotation), a repeated eigenvalue (2 rotations) and a
//! small inertia tensor (1 rotation). Every input goes through `bb` (otherwise the computation is
//! constant-folded away) and every result through `sink`.

use benches::alt::eigen3 as alt;
use benches::harness::{bb, sink};
use fixed::fixed::Fixed;
use glam_core::mat3::Mat3;
use glam_core::vec3::Vec3;
use glamx::eigen3::{Mat3ExtTrait, SymmetricEigen3, SymmetricEigen3Trait};
use glamx::sdp::SdpMatrix3;

/// A rotation of the scaled domain (entries around `2^24`): pivot, diagonal, third row.
const APP: Fixed = Fixed { raw: 0x123456789abcdef };
const AQQ: Fixed = Fixed { raw: 0x3456789abcdef1 };
const APQ: Fixed = Fixed { raw: -0x6789abcdef1234 };
const V: Vec3 = Vec3 {
    x: Fixed { raw: 0x80000000 }, y: Fixed { raw: 0x60000000 }, z: Fixed { raw: -0x70000000 },
};

/// The matrix of the glamx unit test: eigenvalues -7.605, 0.577, 15.028 (10 rotations).
const GENERIC: Mat3 = Mat3 {
    x_axis: Vec3 {
        x: Fixed { raw: 0x200000000 }, y: Fixed { raw: 0x700000000 }, z: Fixed { raw: 0x800000000 },
    },
    y_axis: Vec3 {
        x: Fixed { raw: 0x700000000 }, y: Fixed { raw: 0x600000000 }, z: Fixed { raw: 0x300000000 },
    },
    z_axis: Vec3 {
        x: Fixed { raw: 0x800000000 }, y: Fixed { raw: 0x300000000 }, z: Fixed { raw: 0x0 },
    },
};

/// A diagonal matrix: no rotation at all.
const DIAGONAL: Mat3 = Mat3 {
    x_axis: Vec3 { x: Fixed { raw: 0x200000000 }, y: Fixed { raw: 0x0 }, z: Fixed { raw: 0x0 } },
    y_axis: Vec3 { x: Fixed { raw: 0x0 }, y: Fixed { raw: 0x500000000 }, z: Fixed { raw: 0x0 } },
    z_axis: Vec3 { x: Fixed { raw: 0x0 }, y: Fixed { raw: 0x0 }, z: Fixed { raw: 0x300000000 } },
};

/// Eigenvalues (1, 1, 4): the repeated eigenvalue the closed form cannot resolve.
const TWO_EQUAL: Mat3 = Mat3 {
    x_axis: Vec3 {
        x: Fixed { raw: 0x200000000 }, y: Fixed { raw: 0x100000000 }, z: Fixed { raw: 0x100000000 },
    },
    y_axis: Vec3 {
        x: Fixed { raw: 0x100000000 }, y: Fixed { raw: 0x200000000 }, z: Fixed { raw: 0x100000000 },
    },
    z_axis: Vec3 {
        x: Fixed { raw: 0x100000000 }, y: Fixed { raw: 0x100000000 }, z: Fixed { raw: 0x200000000 },
    },
};

/// The inertia tensor of a thin rod (entries of 1e-2): scaled up by 2^28.
const ROD: Mat3 = Mat3 {
    x_axis: Vec3 {
        x: Fixed { raw: 0x2222222 }, y: Fixed { raw: -0x4444444 }, z: Fixed { raw: 0x0 },
    },
    y_axis: Vec3 {
        x: Fixed { raw: -0x4444444 }, y: Fixed { raw: 0x8888888 }, z: Fixed { raw: 0x0 },
    },
    z_axis: Vec3 { x: Fixed { raw: 0x0 }, y: Fixed { raw: 0x0 }, z: Fixed { raw: 0xaaaaaaa } },
};

/// `GENERIC` as its 6 unique entries.
const GENERIC_SDP: SdpMatrix3 = SdpMatrix3 {
    m11: Fixed { raw: 0x200000000 },
    m12: Fixed { raw: 0x700000000 },
    m13: Fixed { raw: 0x800000000 },
    m22: Fixed { raw: 0x600000000 },
    m23: Fixed { raw: 0x300000000 },
    m33: Fixed { raw: 0x0 },
};

/// A decomposition (the values do not matter to `reverse`).
const EIGEN: SymmetricEigen3 = SymmetricEigen3 {
    eigenvalues: Vec3 {
        x: Fixed { raw: 0x100000000 }, y: Fixed { raw: 0x200000000 }, z: Fixed { raw: 0x300000000 },
    },
    eigenvectors: GENERIC,
};

#[test]
fn new_generic__base() {
    let _a = bb(GENERIC);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn new_generic__op() {
    let a = bb(GENERIC);
    let _r = bb(EIGEN);
    sink(SymmetricEigen3Trait::new(a));
}

#[test]
fn new_diagonal__base() {
    let _a = bb(DIAGONAL);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn new_diagonal__op() {
    let a = bb(DIAGONAL);
    let _r = bb(EIGEN);
    sink(SymmetricEigen3Trait::new(a));
}

#[test]
fn new_two_equal__base() {
    let _a = bb(TWO_EQUAL);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn new_two_equal__op() {
    let a = bb(TWO_EQUAL);
    let _r = bb(EIGEN);
    sink(SymmetricEigen3Trait::new(a));
}

#[test]
fn new_rod__base() {
    let _a = bb(ROD);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn new_rod__op() {
    let a = bb(ROD);
    let _r = bb(EIGEN);
    sink(SymmetricEigen3Trait::new(a));
}

#[test]
fn from_sdp_generic__base() {
    let _a = bb(GENERIC_SDP);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn from_sdp_generic__op() {
    let a = bb(GENERIC_SDP);
    let _r = bb(EIGEN);
    sink(SymmetricEigen3Trait::from_sdp(a));
}

#[test]
fn eigenvalues_generic__base() {
    let _a = bb(GENERIC);
    let r = bb(EIGEN.eigenvalues);
    sink(r);
}

#[test]
fn eigenvalues_generic__op() {
    let a = bb(GENERIC);
    let _r = bb(EIGEN.eigenvalues);
    sink(SymmetricEigen3Trait::eigenvalues(a));
}

#[test]
fn symmetric_eigen_generic__base() {
    let _a = bb(GENERIC);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn symmetric_eigen_generic__op() {
    let a = bb(GENERIC);
    let _r = bb(EIGEN);
    sink(a.symmetric_eigen());
}

#[test]
fn symmetric_eigenvalues_generic__base() {
    let _a = bb(GENERIC);
    let r = bb(EIGEN.eigenvalues);
    sink(r);
}

#[test]
fn symmetric_eigenvalues_generic__op() {
    let a = bb(GENERIC);
    let _r = bb(EIGEN.eigenvalues);
    sink(a.symmetric_eigenvalues());
}

#[test]
fn reverse__base() {
    let _a = bb(EIGEN);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn reverse__op() {
    let a = bb(EIGEN);
    let _r = bb(EIGEN);
    sink(a.reverse());
}

#[test]
fn swap_cols__base() {
    let a = bb(GENERIC);
    let _i = bb(1_usize);
    let _j = bb(2_usize);
    sink(a);
}

#[test]
fn swap_cols__op() {
    let mut a = bb(GENERIC);
    let i = bb(1_usize);
    let j = bb(2_usize);
    a.swap_cols(i, j);
    sink(a);
}

#[test]
fn alt_symmetric_eigen_closed_form_new_generic__base() {
    let _a = bb(GENERIC);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn alt_symmetric_eigen_closed_form_new_generic__op() {
    let a = bb(GENERIC);
    let _r = bb(EIGEN);
    sink(alt::symmetric_eigen_closed_form(a));
}

#[test]
fn alt_symmetric_eigen_closed_form_new_diagonal__base() {
    let _a = bb(DIAGONAL);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn alt_symmetric_eigen_closed_form_new_diagonal__op() {
    let a = bb(DIAGONAL);
    let _r = bb(EIGEN);
    sink(alt::symmetric_eigen_closed_form(a));
}

#[test]
fn alt_symmetric_eigen_closed_form_new_two_equal__base() {
    let _a = bb(TWO_EQUAL);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn alt_symmetric_eigen_closed_form_new_two_equal__op() {
    let a = bb(TWO_EQUAL);
    let _r = bb(EIGEN);
    sink(alt::symmetric_eigen_closed_form(a));
}

#[test]
fn alt_symmetric_eigen_closed_form_new_rod__base() {
    let _a = bb(ROD);
    let r = bb(EIGEN);
    sink(r);
}

#[test]
fn alt_symmetric_eigen_closed_form_new_rod__op() {
    let a = bb(ROD);
    let _r = bb(EIGEN);
    sink(alt::symmetric_eigen_closed_form(a));
}

#[test]
fn alt_symmetric_eigenvalues_closed_form_eigenvalues_generic__base() {
    let _a = bb(GENERIC);
    let r = bb(EIGEN.eigenvalues);
    sink(r);
}

#[test]
fn alt_symmetric_eigenvalues_closed_form_eigenvalues_generic__op() {
    let a = bb(GENERIC);
    let _r = bb(EIGEN.eigenvalues);
    sink(alt::symmetric_eigenvalues_closed_form(a));
}

#[test]
fn eigenvalues_diagonal__base() {
    let _a = bb(DIAGONAL);
    let r = bb(EIGEN.eigenvalues);
    sink(r);
}

#[test]
fn eigenvalues_diagonal__op() {
    let a = bb(DIAGONAL);
    let _r = bb(EIGEN.eigenvalues);
    sink(SymmetricEigen3Trait::eigenvalues(a));
}

#[test]
fn eigenvalues_two_equal__base() {
    let _a = bb(TWO_EQUAL);
    let r = bb(EIGEN.eigenvalues);
    sink(r);
}

#[test]
fn eigenvalues_two_equal__op() {
    let a = bb(TWO_EQUAL);
    let _r = bb(EIGEN.eigenvalues);
    sink(SymmetricEigen3Trait::eigenvalues(a));
}

#[test]
fn eigenvalues_rod__base() {
    let _a = bb(ROD);
    let r = bb(EIGEN.eigenvalues);
    sink(r);
}

#[test]
fn eigenvalues_rod__op() {
    let a = bb(ROD);
    let _r = bb(EIGEN.eigenvalues);
    sink(SymmetricEigen3Trait::eigenvalues(a));
}

#[test]
fn alt_rotation_division__base() {
    let _p = bb(APP);
    let _q = bb(AQQ);
    let _pq = bb(APQ);
    let _v = bb(V);
    sink(bb(APP));
}

#[test]
fn alt_rotation_division__op() {
    let p = bb(APP);
    let q = bb(AQQ);
    let pq = bb(APQ);
    let v = bb(V);
    sink(alt::rotation_division(p, q, pq, q, pq, v, v));
}

#[test]
fn alt_rotation_normalize__base() {
    let _p = bb(APP);
    let _q = bb(AQQ);
    let _pq = bb(APQ);
    let _v = bb(V);
    sink(bb(APP));
}

#[test]
fn alt_rotation_normalize__op() {
    let p = bb(APP);
    let q = bb(AQQ);
    let pq = bb(APQ);
    let v = bb(V);
    sink(alt::rotation_normalize(p, q, pq, q, pq, v, v));
}

#[test]
fn alt_symmetric_eigenvalues_diagonal_read_eigenvalues_generic__base() {
    let _a = bb(GENERIC);
    let r = bb(EIGEN.eigenvalues);
    sink(r);
}

#[test]
fn alt_symmetric_eigenvalues_diagonal_read_eigenvalues_generic__op() {
    let a = bb(GENERIC);
    let _r = bb(EIGEN.eigenvalues);
    sink(alt::symmetric_eigenvalues_diagonal_read(a));
}
