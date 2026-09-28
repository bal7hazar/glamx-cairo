//! Gas benchmarks of `glamx::sdp` and of the alternatives kept in `benches::alt::sdp` (the
//! `alt_*` benches).
//!
//! Sierra gas is charged at the most expensive sibling branch, but steps depend on the path
//! taken: branching functions are measured on one input per branch (`_regular` / `_singular`,
//! `_true` / `_false`). Every input goes through `bb` (otherwise the computation is
//! constant-folded away) and every result through `sink`.

use benches::alt::sdp as alt;
use benches::harness::{bb, sink};
use fixed::fixed::Fixed;
use glam_core::mat2::Mat2;
use glam_core::mat3::Mat3;
use glam_core::quat::Quat;
use glam_core::vec2::Vec2;
use glam_core::vec3::Vec3;
use glamx::sdp::{SdpMatrix2, SdpMatrix2Trait, SdpMatrix3, SdpMatrix3Trait};


/// A symmetric positive-definite matrix (det 17.25).
const S: SdpMatrix3 = SdpMatrix3 {
    m11: Fixed { raw: 0x200000000 },
    m12: Fixed { raw: 0x100000000 },
    m13: Fixed { raw: 0x80000000 },
    m22: Fixed { raw: 0x300000000 },
    m23: Fixed { raw: -0x100000000 },
    m33: Fixed { raw: 0x400000000 },
};

/// A second one, for the binary operators.
const T: SdpMatrix3 = SdpMatrix3 {
    m11: Fixed { raw: 0x180000000 },
    m12: Fixed { raw: -0x40000000 },
    m13: Fixed { raw: 0xc0000000 },
    m22: Fixed { raw: 0x280000000 },
    m23: Fixed { raw: 0x80000000 },
    m33: Fixed { raw: 0x340000000 },
};

/// A singular symmetric matrix (rank 1).
const SING: SdpMatrix3 = SdpMatrix3 {
    m11: Fixed { raw: 0x100000000 },
    m12: Fixed { raw: 0x200000000 },
    m13: Fixed { raw: 0x300000000 },
    m22: Fixed { raw: 0x400000000 },
    m23: Fixed { raw: 0x600000000 },
    m33: Fixed { raw: 0x900000000 },
};

const S2: SdpMatrix2 = SdpMatrix2 {
    m11: Fixed { raw: 0x200000000 },
    m12: Fixed { raw: 0x100000000 },
    m22: Fixed { raw: 0x300000000 },
};

const T2: SdpMatrix2 = SdpMatrix2 {
    m11: Fixed { raw: 0x180000000 },
    m12: Fixed { raw: -0x40000000 },
    m22: Fixed { raw: 0x280000000 },
};

const SING2: SdpMatrix2 = SdpMatrix2 {
    m11: Fixed { raw: 0x100000000 },
    m12: Fixed { raw: 0x200000000 },
    m22: Fixed { raw: 0x400000000 },
};

const M: Mat3 = Mat3 {
    x_axis: Vec3 {
        x: Fixed { raw: 0x280000000 }, y: Fixed { raw: 0xc0000000 }, z: Fixed { raw: -0x80000000 },
    },
    y_axis: Vec3 {
        x: Fixed { raw: 0x40000000 }, y: Fixed { raw: 0x380000000 }, z: Fixed { raw: 0x40000000 },
    },
    z_axis: Vec3 {
        x: Fixed { raw: 0xc0000000 }, y: Fixed { raw: -0x80000000 }, z: Fixed { raw: 0x480000000 },
    },
};

/// The rotation matrix of `ROT`.
const R: Mat3 = Mat3 {
    x_axis: Vec3 {
        x: Fixed { raw: 0xc8198141 }, y: Fixed { raw: 0x8cd47b9b }, z: Fixed { raw: -0x4b40d2d2 },
    },
    y_axis: Vec3 {
        x: Fixed { raw: -0x7ba140ea }, y: Fixed { raw: 0xd4ffed46 }, z: Fixed { raw: 0x45e07775 },
    },
    z_axis: Vec3 {
        x: Fixed { raw: 0x650daadc }, y: Fixed { raw: -0x1246c762 }, z: Fixed { raw: 0xea7ff6a3 },
    },
};

/// A unit quaternion: 0.7 rad about (1, 2, 3).
const ROT: Quat = Quat {
    x: Fixed { raw: 0x1775ef56 },
    y: Fixed { raw: 0x2eebdeac },
    z: Fixed { raw: 0x4661ce02 },
    w: Fixed { raw: 0xf07abae8 },
};

/// A principal inverse inertia.
const D: Vec3 = Vec3 {
    x: Fixed { raw: 0x80000000 }, y: Fixed { raw: 0x40000000 }, z: Fixed { raw: 0x20000000 },
};

const V: Vec3 = Vec3 {
    x: Fixed { raw: 0x180000000 }, y: Fixed { raw: -0x1c0000000 }, z: Fixed { raw: 0x160000000 },
};

const V2: Vec2 = Vec2 { x: Fixed { raw: 0x180000000 }, y: Fixed { raw: -0x1c0000000 } };

const K: Fixed = Fixed { raw: 0x180000000 };

const Q11: Fixed = Fixed { raw: 0x140000000 };

const Q12: Fixed = Fixed { raw: -0x80000000 };

const Q21: Fixed = Fixed { raw: 0xc0000000 };

const Q22: Fixed = Fixed { raw: 0x200000000 };

const Q31: Fixed = Fixed { raw: -0x180000000 };

const Q32: Fixed = Fixed { raw: 0x40000000 };

#[test]
fn add__base() {
    let _a = bb(S);
    let _b = bb(T);
    let r = bb(S);
    sink(r);
}

#[test]
fn add__op() {
    let a = bb(S);
    let b = bb(T);
    let _r = bb(S);
    sink(a + b);
}

#[test]
fn sub__base() {
    let _a = bb(S);
    let _b = bb(T);
    let r = bb(S);
    sink(r);
}

#[test]
fn sub__op() {
    let a = bb(S);
    let b = bb(T);
    let _r = bb(S);
    sink(a - b);
}

#[test]
fn mul_scalar__base() {
    let _a = bb(S);
    let _k = bb(K);
    let r = bb(S);
    sink(r);
}

#[test]
fn mul_scalar__op() {
    let a = bb(S);
    let k = bb(K);
    let _r = bb(S);
    sink(a.mul_scalar(k));
}

#[test]
fn add_diagonal__base() {
    let _a = bb(S);
    let _k = bb(K);
    let r = bb(S);
    sink(r);
}

#[test]
fn add_diagonal__op() {
    let a = bb(S);
    let k = bb(K);
    let _r = bb(S);
    sink(a.add_diagonal(k));
}

#[test]
fn is_zero_false__base() {
    let _a = bb(S);
    let r = bb(false);
    sink(r);
}

#[test]
fn is_zero_false__op() {
    let a = bb(S);
    let _r = bb(false);
    sink(a.is_zero());
}

#[test]
fn is_zero_true__base() {
    let _a = bb(SdpMatrix3Trait::zero());
    let r = bb(false);
    sink(r);
}

#[test]
fn is_zero_true__op() {
    let a = bb(SdpMatrix3Trait::zero());
    let _r = bb(false);
    sink(a.is_zero());
}

#[test]
fn from_sdp_matrix__base() {
    let _m = bb(M);
    let r = bb(S);
    sink(r);
}

#[test]
fn from_sdp_matrix__op() {
    let m = bb(M);
    let _r = bb(S);
    sink(SdpMatrix3Trait::from_sdp_matrix(m));
}

#[test]
fn into_matrix__base() {
    let _a = bb(S);
    let r = bb(M);
    sink(r);
}

#[test]
fn into_matrix__op() {
    let a = bb(S);
    let _r = bb(M);
    sink(a.into_matrix());
}

#[test]
fn mul_vec__base() {
    let _a = bb(S);
    let _v = bb(V);
    let r = bb(V);
    sink(r);
}

#[test]
fn mul_vec__op() {
    let a = bb(S);
    let v = bb(V);
    let _r = bb(V);
    sink(a.mul_vec(v));
}

#[test]
fn transform_vector__base() {
    let _a = bb(S);
    let _v = bb(V);
    let r = bb(V);
    sink(r);
}

#[test]
fn transform_vector__op() {
    let a = bb(S);
    let v = bb(V);
    let _r = bb(V);
    sink(a.transform_vector(v));
}

#[test]
fn mul_mat__base() {
    let _a = bb(S);
    let _m = bb(M);
    let r = bb(M);
    sink(r);
}

#[test]
fn mul_mat__op() {
    let a = bb(S);
    let m = bb(M);
    let _r = bb(M);
    sink(a.mul_mat(m));
}

#[test]
fn quadform__base() {
    let _a = bb(S);
    let _m = bb(M);
    let r = bb(S);
    sink(r);
}

#[test]
fn quadform__op() {
    let a = bb(S);
    let m = bb(M);
    let _r = bb(S);
    sink(a.quadform(m));
}

#[test]
fn quadform3x2__base() {
    let _a = bb(S);
    let _q11 = bb(Q11);
    let _q12 = bb(Q12);
    let _q21 = bb(Q21);
    let _q22 = bb(Q22);
    let _q31 = bb(Q31);
    let _q32 = bb(Q32);
    let r = bb(S2);
    sink(r);
}

#[test]
fn quadform3x2__op() {
    let a = bb(S);
    let q11 = bb(Q11);
    let q12 = bb(Q12);
    let q21 = bb(Q21);
    let q22 = bb(Q22);
    let q31 = bb(Q31);
    let q32 = bb(Q32);
    let _r = bb(S2);
    sink(a.quadform3x2(q11, q12, q21, q22, q31, q32));
}

#[test]
fn inverse_unchecked__base() {
    let _a = bb(S);
    let r = bb(S);
    sink(r);
}

#[test]
fn inverse_unchecked__op() {
    let a = bb(S);
    let _r = bb(S);
    sink(a.inverse_unchecked());
}

#[test]
fn inverse_regular__base() {
    let _a = bb(S);
    let r = bb(S);
    sink(r);
}

#[test]
fn inverse_regular__op() {
    let a = bb(S);
    let _r = bb(S);
    sink(a.inverse());
}

#[test]
fn inverse_singular__base() {
    let _a = bb(SING);
    let r = bb(S);
    sink(r);
}

#[test]
fn inverse_singular__op() {
    let a = bb(SING);
    let _r = bb(S);
    sink(a.inverse());
}

#[test]
fn from_rotated_diagonal__base() {
    let _q = bb(ROT);
    let _d = bb(D);
    let r = bb(S);
    sink(r);
}

#[test]
fn from_rotated_diagonal__op() {
    let q = bb(ROT);
    let d = bb(D);
    let _r = bb(S);
    sink(SdpMatrix3Trait::from_rotated_diagonal(q, d));
}

#[test]
fn from_rotated_diagonal_mat3__base() {
    let _r = bb(R);
    let _d = bb(D);
    let r = bb(S);
    sink(r);
}

#[test]
fn from_rotated_diagonal_mat3__op() {
    let r = bb(R);
    let d = bb(D);
    let _r = bb(S);
    sink(SdpMatrix3Trait::from_rotated_diagonal_mat3(r, d));
}

#[test]
fn add2__base() {
    let _a = bb(S2);
    let _b = bb(T2);
    let r = bb(S2);
    sink(r);
}

#[test]
fn add2__op() {
    let a = bb(S2);
    let b = bb(T2);
    let _r = bb(S2);
    sink(a + b);
}

#[test]
fn sub2__base() {
    let _a = bb(S2);
    let _b = bb(T2);
    let r = bb(S2);
    sink(r);
}

#[test]
fn sub2__op() {
    let a = bb(S2);
    let b = bb(T2);
    let _r = bb(S2);
    sink(a - b);
}

#[test]
fn mul_scalar2__base() {
    let _a = bb(S2);
    let _k = bb(K);
    let r = bb(S2);
    sink(r);
}

#[test]
fn mul_scalar2__op() {
    let a = bb(S2);
    let k = bb(K);
    let _r = bb(S2);
    sink(a.mul_scalar(k));
}

#[test]
fn add_diagonal2__base() {
    let _a = bb(S2);
    let _k = bb(K);
    let r = bb(S2);
    sink(r);
}

#[test]
fn add_diagonal2__op() {
    let a = bb(S2);
    let k = bb(K);
    let _r = bb(S2);
    sink(a.add_diagonal(k));
}

#[test]
fn is_zero2_false__base() {
    let _a = bb(S2);
    let r = bb(false);
    sink(r);
}

#[test]
fn is_zero2_false__op() {
    let a = bb(S2);
    let _r = bb(false);
    sink(a.is_zero());
}

#[test]
fn from_sdp_matrix2__base() {
    let _m = bb(Mat2 { x_axis: V2, y_axis: V2 });
    let r = bb(S2);
    sink(r);
}

#[test]
fn from_sdp_matrix2__op() {
    let m = bb(Mat2 { x_axis: V2, y_axis: V2 });
    let _r = bb(S2);
    sink(SdpMatrix2Trait::from_sdp_matrix(m));
}

#[test]
fn into_matrix2__base() {
    let _a = bb(S2);
    let r = bb(Mat2 { x_axis: V2, y_axis: V2 });
    sink(r);
}

#[test]
fn into_matrix2__op() {
    let a = bb(S2);
    let _r = bb(Mat2 { x_axis: V2, y_axis: V2 });
    sink(a.into_matrix());
}

#[test]
fn mul_vec2__base() {
    let _a = bb(S2);
    let _v = bb(V2);
    let r = bb(V2);
    sink(r);
}

#[test]
fn mul_vec2__op() {
    let a = bb(S2);
    let v = bb(V2);
    let _r = bb(V2);
    sink(a.mul_vec(v));
}

#[test]
fn inverse_unchecked2__base() {
    let _a = bb(S2);
    let r = bb(S2);
    sink(r);
}

#[test]
fn inverse_unchecked2__op() {
    let a = bb(S2);
    let _r = bb(S2);
    sink(a.inverse_unchecked());
}

#[test]
fn inverse_and_get_determinant_unchecked2__base() {
    let _a = bb(S2);
    let r = bb((S2, K));
    sink(r);
}

#[test]
fn inverse_and_get_determinant_unchecked2__op() {
    let a = bb(S2);
    let _r = bb((S2, K));
    sink(a.inverse_and_get_determinant_unchecked());
}

#[test]
fn inverse2_regular__base() {
    let _a = bb(S2);
    let r = bb(S2);
    sink(r);
}

#[test]
fn inverse2_regular__op() {
    let a = bb(S2);
    let _r = bb(S2);
    sink(a.inverse());
}

#[test]
fn inverse2_singular__base() {
    let _a = bb(SING2);
    let r = bb(S2);
    sink(r);
}

#[test]
fn inverse2_singular__op() {
    let a = bb(SING2);
    let _r = bb(S2);
    sink(a.inverse());
}

#[test]
fn alt_from_rotated_diagonal_literal_from_rotated_diagonal__base() {
    let _q = bb(ROT);
    let _d = bb(D);
    let r = bb(S);
    sink(r);
}

#[test]
fn alt_from_rotated_diagonal_literal_from_rotated_diagonal__op() {
    let q = bb(ROT);
    let d = bb(D);
    let _r = bb(S);
    sink(alt::from_rotated_diagonal_literal(q, d));
}

#[test]
fn alt_from_rotated_diagonal_mat3_two_stage_from_rotated_diagonal_mat3__base() {
    let _r = bb(R);
    let _d = bb(D);
    let r = bb(S);
    sink(r);
}

#[test]
fn alt_from_rotated_diagonal_mat3_two_stage_from_rotated_diagonal_mat3__op() {
    let r = bb(R);
    let d = bb(D);
    let _r = bb(S);
    sink(alt::from_rotated_diagonal_mat3_two_stage(r, d));
}

#[test]
fn alt_quadform_literal_quadform__base() {
    let _a = bb(S);
    let _m = bb(M);
    let r = bb(S);
    sink(r);
}

#[test]
fn alt_quadform_literal_quadform__op() {
    let a = bb(S);
    let m = bb(M);
    let _r = bb(S);
    sink(alt::quadform_literal(a, m));
}

#[test]
fn alt_quadform_two_stage_quadform__base() {
    let _a = bb(S);
    let _m = bb(M);
    let r = bb(S);
    sink(r);
}

#[test]
fn alt_quadform_two_stage_quadform__op() {
    let a = bb(S);
    let m = bb(M);
    let _r = bb(S);
    sink(alt::quadform_two_stage(a, m));
}

#[test]
fn alt_inverse_plain_inverse_regular__base() {
    let _a = bb(S);
    let r = bb(S);
    sink(r);
}

#[test]
fn alt_inverse_plain_inverse_regular__op() {
    let a = bb(S);
    let _r = bb(S);
    sink(alt::inverse_plain(a));
}

#[test]
fn alt_inverse_separate_det_inverse_regular__base() {
    let _a = bb(S);
    let r = bb(S);
    sink(r);
}

#[test]
fn alt_inverse_separate_det_inverse_regular__op() {
    let a = bb(S);
    let _r = bb(S);
    sink(alt::inverse_separate_det(a));
}

#[test]
fn alt_inverse_via_mat3_inverse_regular__base() {
    let _a = bb(S);
    let r = bb(S);
    sink(r);
}

#[test]
fn alt_inverse_via_mat3_inverse_regular__op() {
    let a = bb(S);
    let _r = bb(S);
    sink(alt::inverse_via_mat3(a));
}

#[test]
fn alt_mul_vec_unfused_mul_vec__base() {
    let _a = bb(S);
    let _v = bb(V);
    let r = bb(V);
    sink(r);
}

#[test]
fn alt_mul_vec_unfused_mul_vec__op() {
    let a = bb(S);
    let v = bb(V);
    let _r = bb(V);
    sink(alt::mul_vec_unfused(a, v));
}
