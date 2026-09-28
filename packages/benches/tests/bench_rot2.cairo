//! Gas benchmarks of `glamx::rot2` and the unfused alternatives in `benches::alt::rot2`.
//!
//! Inputs pass through `bb` and results through `sink`. Branching operations have one benchmark
//! per relevant path; every arithmetic public method has a base/op pair.

use benches::alt::rot2 as alt;
use benches::harness::{bb, sink};
use fixed::fixed::Fixed;
use glam_core::mat2::Mat2;
use glam_core::vec2::Vec2;
use glamx::rot2::{Rot2, Rot2Trait};

/// `new(0.5)`.
const A: Rot2 = Rot2 { re: Fixed { raw: 0xe0a94033 }, im: Fixed { raw: 0x7abba1d1 } };
/// `new(1.0)`.
const B: Rot2 = Rot2 { re: Fixed { raw: 0x8a51407d }, im: Fixed { raw: 0xd76aa478 } };
const WIDE: Rot2 = Rot2 { re: Fixed { raw: 0x300000000 }, im: Fixed { raw: 0x400000000 } };
const V: Vec2 = Vec2 { x: Fixed { raw: 0x180000000 }, y: Fixed { raw: -0x1c0000000 } };
const X: Vec2 = Vec2 { x: Fixed { raw: 0x100000000 }, y: Fixed { raw: 0 } };
const Y: Vec2 = Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0x100000000 } };
const M: Mat2 = Mat2 {
    x_axis: Vec2 { x: Fixed { raw: 0x300000000 }, y: Fixed { raw: 0x400000000 } },
    y_axis: Vec2 { x: Fixed { raw: -0x400000000 }, y: Fixed { raw: 0x300000000 } },
};
const ANGLE: Fixed = Fixed { raw: 0x80000000 };
const HALF: Fixed = Fixed { raw: 0x80000000 };
const SMALL_ANGLE: Fixed = Fixed { raw: 0x33333333 };
const LARGE_ANGLE: Fixed = Fixed { raw: 0x200000000 };
const ZERO: Fixed = Fixed { raw: 0 };
const ONE: Fixed = Fixed { raw: 0x100000000 };

#[test]
fn from_cos_sin_unchecked__base() {
    let _re = bb(A.re);
    let _im = bb(A.im);
    let r = bb(A);
    sink(r);
}

#[test]
fn from_cos_sin_unchecked__op() {
    let re = bb(A.re);
    let im = bb(A.im);
    let _r = bb(A);
    sink(Rot2Trait::from_cos_sin_unchecked(re, im));
}

#[test]
fn new__base() {
    let _a = bb(ANGLE);
    let r = bb(A);
    sink(r);
}

#[test]
fn new__op() {
    let a = bb(ANGLE);
    let _r = bb(A);
    sink(Rot2Trait::new(a));
}

#[test]
fn from_angle__base() {
    let _a = bb(ANGLE);
    let r = bb(A);
    sink(r);
}

#[test]
fn from_angle__op() {
    let a = bb(ANGLE);
    let _r = bb(A);
    sink(Rot2Trait::from_angle(a));
}

#[test]
fn angle__base() {
    let _a = bb(A);
    let r = bb(ANGLE);
    sink(r);
}

#[test]
fn angle__op() {
    let a = bb(A);
    let _r = bb(ANGLE);
    sink(a.angle());
}

#[test]
fn inverse__base() {
    let _a = bb(A);
    let r = bb(A);
    sink(r);
}

#[test]
fn inverse__op() {
    let a = bb(A);
    let _r = bb(A);
    sink(a.inverse());
}

#[test]
fn mul__base() {
    let _a = bb(A);
    let _b = bb(B);
    let r = bb(A);
    sink(r);
}

#[test]
fn mul__op() {
    let a = bb(A);
    let b = bb(B);
    let _r = bb(A);
    sink(a * b);
}

#[test]
fn mul_assign__base() {
    let _a = bb(A);
    let _b = bb(B);
    let r = bb(A);
    sink(r);
}

#[test]
fn mul_assign__op() {
    let mut a = bb(A);
    let b = bb(B);
    let _r = bb(A);
    a *= b;
    sink(a);
}

#[test]
fn alt_mul_unfused__base() {
    let _a = bb(A);
    let _b = bb(B);
    let r = bb(A);
    sink(r);
}

#[test]
fn alt_mul_unfused__op() {
    let a = bb(A);
    let b = bb(B);
    let _r = bb(A);
    sink(alt::mul_unfused(a, b));
}

#[test]
fn transform_vector__base() {
    let _a = bb(A);
    let _v = bb(V);
    let r = bb(V);
    sink(r);
}

#[test]
fn transform_vector__op() {
    let a = bb(A);
    let v = bb(V);
    let _r = bb(V);
    sink(a.transform_vector(v));
}

#[test]
fn inverse_transform_vector__base() {
    let _a = bb(A);
    let _v = bb(V);
    let r = bb(V);
    sink(r);
}

#[test]
fn inverse_transform_vector__op() {
    let a = bb(A);
    let v = bb(V);
    let _r = bb(V);
    sink(a.inverse_transform_vector(v));
}

#[test]
fn mul_vec2__base() {
    let _a = bb(A);
    let _v = bb(V);
    let r = bb(V);
    sink(r);
}

#[test]
fn mul_vec2__op() {
    let a = bb(A);
    let v = bb(V);
    let _r = bb(V);
    sink(a.mul_vec2(v));
}

#[test]
fn alt_mul_vec2_unfused__base() {
    let _a = bb(A);
    let _v = bb(V);
    let r = bb(V);
    sink(r);
}

#[test]
fn alt_mul_vec2_unfused__op() {
    let a = bb(A);
    let v = bb(V);
    let _r = bb(V);
    sink(alt::mul_vec2_unfused(a, v));
}

#[test]
fn to_mat__base() {
    let _a = bb(A);
    let r = bb(M);
    sink(r);
}

#[test]
fn to_mat__op() {
    let a = bb(A);
    let _r = bb(M);
    sink(a.to_mat());
}

#[test]
fn from_mat__base() {
    let _m = bb(M);
    let r = bb(A);
    sink(r);
}

#[test]
fn from_mat__op() {
    let m = bb(M);
    let _r = bb(A);
    sink(Rot2Trait::from_mat(m));
}

#[test]
fn from_mat_unchecked__base() {
    let _m = bb(M);
    let r = bb(A);
    sink(r);
}

#[test]
fn from_mat_unchecked__op() {
    let m = bb(M);
    let _r = bb(A);
    sink(Rot2Trait::from_mat_unchecked(m));
}

#[test]
fn normalize__base() {
    let _a = bb(WIDE);
    let r = bb(A);
    sink(r);
}

#[test]
fn normalize__op() {
    let a = bb(WIDE);
    let _r = bb(A);
    sink(a.normalize());
}

#[test]
fn normalize__near_zero__base() {
    let _a = bb(Rot2 { re: Fixed { raw: 1 }, im: Fixed { raw: 1 } });
    let r = bb(A);
    sink(r);
}

#[test]
fn normalize__near_zero__op() {
    let a = bb(Rot2 { re: Fixed { raw: 1 }, im: Fixed { raw: 1 } });
    let _r = bb(A);
    sink(a.normalize());
}

#[test]
fn normalize_mut__base() {
    let _a = bb(WIDE);
    let r = bb(A);
    sink(r);
}

#[test]
fn normalize_mut__op() {
    let mut a = bb(WIDE);
    let _r = bb(A);
    a.normalize_mut();
    sink(a);
}

#[test]
fn length__base() {
    let _a = bb(WIDE);
    let r = bb(ONE);
    sink(r);
}

#[test]
fn length__op() {
    let a = bb(WIDE);
    let _r = bb(ONE);
    sink(a.length());
}

#[test]
fn length_squared__base() {
    let _a = bb(WIDE);
    let r = bb(ONE);
    sink(r);
}

#[test]
fn length_squared__op() {
    let a = bb(WIDE);
    let _r = bb(ONE);
    sink(a.length_squared());
}

#[test]
fn is_normalized_true__base() {
    let _a = bb(A);
    let r = bb(true);
    sink(r);
}

#[test]
fn is_normalized_true__op() {
    let a = bb(A);
    let _r = bb(true);
    sink(a.is_normalized());
}

#[test]
fn is_normalized_false__base() {
    let _a = bb(WIDE);
    let r = bb(true);
    sink(r);
}

#[test]
fn is_normalized_false__op() {
    let a = bb(WIDE);
    let _r = bb(true);
    sink(a.is_normalized());
}

#[test]
fn dot__base() {
    let _a = bb(A);
    let _b = bb(B);
    let r = bb(ONE);
    sink(r);
}

#[test]
fn dot__op() {
    let a = bb(A);
    let b = bb(B);
    let _r = bb(ONE);
    sink(a.dot(b));
}

#[test]
fn lerp__base() {
    let _a = bb(A);
    let _b = bb(B);
    let _s = bb(HALF);
    let r = bb(A);
    sink(r);
}

#[test]
fn lerp__op() {
    let a = bb(A);
    let b = bb(B);
    let s = bb(HALF);
    let _r = bb(A);
    sink(a.lerp(b, s));
}

#[test]
fn alt_lerp_two_product__base() {
    let _a = bb(A);
    let _b = bb(B);
    let _s = bb(HALF);
    let r = bb(A);
    sink(r);
}

#[test]
fn alt_lerp_two_product__op() {
    let a = bb(A);
    let b = bb(B);
    let s = bb(HALF);
    let _r = bb(A);
    sink(alt::lerp_two_product(a, b, s));
}

#[test]
fn slerp__base() {
    let _a = bb(A);
    let _b = bb(B);
    let _s = bb(HALF);
    let r = bb(A);
    sink(r);
}

#[test]
fn slerp__op() {
    let a = bb(A);
    let b = bb(B);
    let s = bb(HALF);
    let _r = bb(A);
    sink(a.slerp(b, s));
}

#[test]
fn angle_between__base() {
    let _a = bb(A);
    let _b = bb(B);
    let r = bb(ANGLE);
    sink(r);
}

#[test]
fn angle_between__op() {
    let a = bb(A);
    let b = bb(B);
    let _r = bb(ANGLE);
    sink(a.angle_between(b));
}

#[test]
fn rotate_towards__step__base() {
    let _a = bb(A);
    let _b = bb(B);
    let _m = bb(SMALL_ANGLE);
    let r = bb(A);
    sink(r);
}

#[test]
fn rotate_towards__step__op() {
    let a = bb(A);
    let b = bb(B);
    let m = bb(SMALL_ANGLE);
    let _r = bb(A);
    sink(a.rotate_towards(b, m));
}

#[test]
fn rotate_towards__target__base() {
    let _a = bb(A);
    let _b = bb(B);
    let _m = bb(LARGE_ANGLE);
    let r = bb(A);
    sink(r);
}

#[test]
fn rotate_towards__target__op() {
    let a = bb(A);
    let b = bb(B);
    let m = bb(LARGE_ANGLE);
    let _r = bb(A);
    sink(a.rotate_towards(b, m));
}

#[test]
fn from_rotation_arc__base() {
    let _a = bb(X);
    let _b = bb(Y);
    let r = bb(A);
    sink(r);
}

#[test]
fn from_rotation_arc__op() {
    let a = bb(X);
    let b = bb(Y);
    let _r = bb(A);
    sink(Rot2Trait::from_rotation_arc(a, b));
}

// Keep the zero scalar live so the benchmark constants cover every simple result type.
#[test]
fn cos_sin__base() {
    let _a = bb(A);
    let x = bb(ZERO);
    let y = bb(ZERO);
    sink((x, y));
}

#[test]
fn cos_sin__op() {
    let a = bb(A);
    let _x = bb(ZERO);
    let _y = bb(ZERO);
    sink((a.cos(), a.sin()));
}
