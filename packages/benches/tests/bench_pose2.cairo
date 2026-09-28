//! Gas benchmarks of `glamx::pose2` and the composed forms in `benches::alt::pose2`.
//! Every input passes through `bb` and every result through `sink`.

use benches::alt::pose2 as alt;
use benches::harness::{bb, sink};
use fixed::fixed::Fixed;
use glam_core::mat3::Mat3;
use glam_core::vec2::Vec2;
use glam_core::vec3::Vec3;
use glamx::pose2::{Pose2, Pose2Trait, Rot2Pose2Trait};
use glamx::rot2::Rot2;

const RA: Rot2 = Rot2 { re: Fixed { raw: 0xe0a94033 }, im: Fixed { raw: 0x7abba1d1 } };
const RB: Rot2 = Rot2 { re: Fixed { raw: 0x8a51407d }, im: Fixed { raw: 0xd76aa478 } };
const T1: Vec2 = Vec2 { x: Fixed { raw: 0x180000000 }, y: Fixed { raw: -0x1c0000000 } };
const T2: Vec2 = Vec2 { x: Fixed { raw: -0x340000000 }, y: Fixed { raw: 0x280000000 } };
const PA: Pose2 = Pose2 { rotation: RA, translation: T1 };
const PB: Pose2 = Pose2 { rotation: RB, translation: T2 };
const MA: Mat3 = Mat3 {
    x_axis: Vec3 {
        x: Fixed { raw: 0xe0a94033 }, y: Fixed { raw: 0x7abba1d1 }, z: Fixed { raw: 0 },
    },
    y_axis: Vec3 {
        x: Fixed { raw: -0x7abba1d1 }, y: Fixed { raw: 0xe0a94033 }, z: Fixed { raw: 0 },
    },
    z_axis: Vec3 {
        x: Fixed { raw: 0x180000000 },
        y: Fixed { raw: -0x1c0000000 },
        z: Fixed { raw: 0x100000000 },
    },
};
const ANGLE: Fixed = Fixed { raw: 0x80000000 };
const K_THIRD: Fixed = Fixed { raw: 1431655765 };
const K_EPS: Fixed = Fixed { raw: 0x20000 };

#[test]
fn new__base() {
    let _t = bb(T1);
    let _a = bb(ANGLE);
    let r = bb(PA);
    sink(r);
}

#[test]
fn new__op() {
    let t = bb(T1);
    let a = bb(ANGLE);
    let _r = bb(PA);
    sink(Pose2Trait::new(t, a));
}

#[test]
fn rotation__base() {
    let _a = bb(ANGLE);
    let r = bb(PA);
    sink(r);
}

#[test]
fn rotation__op() {
    let a = bb(ANGLE);
    let _r = bb(PA);
    sink(Pose2Trait::rotation(a));
}

#[test]
fn prepend_translation__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(PA);
    sink(r);
}

#[test]
fn prepend_translation__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(PA);
    sink(p.prepend_translation(v));
}

#[test]
fn append_translation__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(PA);
    sink(r);
}

#[test]
fn append_translation__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(PA);
    sink(p.append_translation(v));
}

#[test]
fn inverse__base() {
    let _p = bb(PA);
    let r = bb(PA);
    sink(r);
}

#[test]
fn inverse__op() {
    let p = bb(PA);
    let _r = bb(PA);
    sink(p.inverse());
}

#[test]
fn alt_inverse_composed__base() {
    let _p = bb(PA);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_inverse_composed__op() {
    let p = bb(PA);
    let _r = bb(PA);
    sink(alt::inverse_composed(p));
}

#[test]
fn inv_mul__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn inv_mul__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(a.inv_mul(b));
}

#[test]
fn alt_inv_mul_composed__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_inv_mul_composed__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(alt::inv_mul_composed(a, b));
}

#[test]
fn alt_inv_mul_via_inverse__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_inv_mul_via_inverse__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(alt::inv_mul_via_inverse(a, b));
}

#[test]
fn mul__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn mul__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(a * b);
}

#[test]
fn alt_mul_composed__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_mul_composed__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(alt::mul_composed(a, b));
}

#[test]
fn mul_rot2__base() {
    let _p = bb(PA);
    let _r = bb(RB);
    let out = bb(PA);
    sink(out);
}

#[test]
fn mul_rot2__op() {
    let p = bb(PA);
    let r = bb(RB);
    let _out = bb(PA);
    sink(p.mul_rot2(r));
}

#[test]
fn mul_pose2__base() {
    let _r = bb(RA);
    let _p = bb(PB);
    let out = bb(PA);
    sink(out);
}

#[test]
fn mul_pose2__op() {
    let r = bb(RA);
    let p = bb(PB);
    let _out = bb(PA);
    sink(r.mul_pose2(p));
}

#[test]
fn transform_point__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn transform_point__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(p.transform_point(v));
}

#[test]
fn alt_transform_point_composed__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn alt_transform_point_composed__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(alt::transform_point_composed(p, v));
}

#[test]
fn mul_vec2__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn mul_vec2__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(p.mul_vec2(v));
}

#[test]
fn transform_vector__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn transform_vector__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(p.transform_vector(v));
}

#[test]
fn inverse_transform_point__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn inverse_transform_point__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(p.inverse_transform_point(v));
}

#[test]
fn alt_inverse_transform_point_composed__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn alt_inverse_transform_point_composed__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(alt::inverse_transform_point_composed(p, v));
}

#[test]
fn inverse_transform_vector__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn inverse_transform_vector__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(p.inverse_transform_vector(v));
}

#[test]
fn alt_inverse_transform_vector_composed__base() {
    let _p = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn alt_inverse_transform_vector_composed__op() {
    let p = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(alt::inverse_transform_vector_composed(p, v));
}

#[test]
fn lerp__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let _t = bb(K_THIRD);
    let r = bb(PA);
    sink(r);
}

#[test]
fn lerp__op() {
    let a = bb(PA);
    let b = bb(PB);
    let t = bb(K_THIRD);
    let _r = bb(PA);
    sink(a.lerp(b, t));
}

#[test]
fn to_mat3__base() {
    let _p = bb(PA);
    let r = bb(MA);
    sink(r);
}

#[test]
fn to_mat3__op() {
    let p = bb(PA);
    let _r = bb(MA);
    sink(p.to_mat3());
}

#[test]
fn from_mat3__base() {
    let _m = bb(MA);
    let r = bb(PA);
    sink(r);
}

#[test]
fn from_mat3__op() {
    let m = bb(MA);
    let _r = bb(PA);
    sink(Pose2Trait::from_mat3(m));
}

#[test]
fn abs_diff_eq__true__base() {
    let _a = bb(PA);
    let _b = bb(PA);
    let _e = bb(K_EPS);
    let r = bb(true);
    sink(r);
}

#[test]
fn abs_diff_eq__true__op() {
    let a = bb(PA);
    let b = bb(PA);
    let e = bb(K_EPS);
    let _r = bb(true);
    sink(a.abs_diff_eq(b, e));
}

#[test]
fn abs_diff_eq__false__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let _e = bb(K_EPS);
    let r = bb(true);
    sink(r);
}

#[test]
fn abs_diff_eq__false__op() {
    let a = bb(PA);
    let b = bb(PB);
    let e = bb(K_EPS);
    let _r = bb(true);
    sink(a.abs_diff_eq(b, e));
}
