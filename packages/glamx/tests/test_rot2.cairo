//! Tests of `glamx::rot2`: exact tables for the fixed-point kernels, edge cases and thresholds,
//! measured composition drift, seeded fuzz properties and every panic path.

use core::ops::MulAssign;
use fixed::fixed::{Fixed, FixedTrait};
use glam_core::mat2::Mat2;
use glam_core::vec2::{Vec2, Vec2Trait};
use glamx::rot2::{Rot2, Rot2Trait};

const ONE_RAW: i64 = 0x100000000;
const TAU_RAW: i64 = 26986075409;

fn f(raw: i64) -> Fixed {
    FixedTrait::from_raw(raw)
}

fn rot(re: i64, im: i64) -> Rot2 {
    Rot2 { re: f(re), im: f(im) }
}

fn vec(x: i64, y: i64) -> Vec2 {
    Vec2 { x: f(x), y: f(y) }
}

fn abs_raw(x: i64) -> i64 {
    if x < 0 {
        -x
    } else {
        x
    }
}

fn close_fixed(a: Fixed, b: Fixed, tolerance: i64) -> bool {
    abs_raw(a.raw - b.raw) <= tolerance
}

fn close_rot(a: Rot2, b: Rot2, tolerance: i64) -> bool {
    close_fixed(a.re, b.re, tolerance) && close_fixed(a.im, b.im, tolerance)
}

fn close_vec(a: Vec2, b: Vec2, tolerance: i64) -> bool {
    close_fixed(a.x, b.x, tolerance) && close_fixed(a.y, b.y, tolerance)
}

fn drift(r: Rot2) -> i64 {
    abs_raw(r.length_squared().raw - ONE_RAW)
}

fn unit(angle_raw: i64) -> Rot2 {
    Rot2Trait::new(f(angle_raw % TAU_RAW))
}

#[test]
fn test_identity_constructors_and_accessors() {
    assert_eq!(Rot2Trait::IDENTITY, rot(ONE_RAW, 0));
    assert_eq!(Rot2Trait::IDENTITY, Default::default());
    let raw = Rot2Trait::from_cos_sin_unchecked(f(3), f(-4));
    assert_eq!(raw, rot(3, -4));
    assert_eq!(raw.cos(), f(3));
    assert_eq!(raw.sin(), f(-4));

    let angles = [0, 1, -1, 0x80000000, 0x100000000, 6746518852, -6746518852];
    for angle in angles.span() {
        let a = f(*angle);
        let r = Rot2Trait::new(a);
        assert_eq!(r, Rot2Trait::from_angle(a));
        assert!(close_fixed(r.angle(), a, 5), "angle round trip at {}", *angle);
    }
}

#[test]
fn test_inverse_mul_and_assign() {
    let r = Rot2Trait::new(f(0x100000000));
    assert_eq!(r.inverse(), Rot2 { re: r.re, im: -r.im });
    assert!(close_rot(r * r.inverse(), Rot2Trait::IDENTITY, 2));
    assert_eq!(Rot2Trait::IDENTITY * r, r);
    assert_eq!(r * Rot2Trait::IDENTITY, r);
    let mut assigned = r;
    assigned.mul_assign(r.inverse());
    assert!(close_rot(assigned, Rot2Trait::IDENTITY, 2));
}

#[test]
fn test_vector_transforms() {
    let quarter = rot(0, ONE_RAW);
    let v = vec(ONE_RAW, 0);
    assert_eq!(quarter.transform_vector(v), vec(0, ONE_RAW));
    assert_eq!(quarter.mul_vec2(v), vec(0, ONE_RAW));
    assert_eq!(quarter.inverse_transform_vector(v), vec(0, -ONE_RAW));

    let r = Rot2Trait::new(f(0x80000000));
    let p = vec(0x180000000, -0x1c0000000);
    assert!(close_vec(r.inverse_transform_vector(r.transform_vector(p)), p, 6));
}

#[test]
fn test_matrix_conversions() {
    let r = Rot2Trait::new(f(0x100000000));
    let m = r.to_mat();
    assert_eq!(m.x_axis, Vec2 { x: r.re, y: r.im });
    assert_eq!(m.y_axis, Vec2 { x: -r.im, y: r.re });
    assert_eq!(Rot2Trait::from_mat_unchecked(m), r);
    assert!(close_rot(Rot2Trait::from_mat(m), r, 2));

    let scaled = Mat2 {
        x_axis: Vec2 { x: f(0x120000000), y: f(0x180000000) },
        y_axis: Vec2 { x: f(-0x180000000), y: f(0x120000000) },
    };
    assert!(drift(Rot2Trait::from_mat(scaled)) <= 2);
}

#[test]
fn test_normalize_threshold_and_mut() {
    for r in [rot(0, 0), rot(1, 0), rot(-1, 1), rot(1, -1)].span() {
        assert_eq!((*r).normalize(), Rot2Trait::IDENTITY);
    }
    assert_eq!(rot(2, 0).normalize(), Rot2Trait::IDENTITY);
    assert_eq!(rot(0, -2).normalize(), rot(0, -ONE_RAW));

    let mut r = rot(0x300000000, 0x400000000);
    let normalized = r.normalize();
    r.normalize_mut();
    assert_eq!(r, normalized);
    assert!(drift(r) <= 2);
}

#[test]
fn test_length_and_dot() {
    let r = rot(0x300000000, 0x400000000);
    assert_eq!(r.length(), f(0x500000000));
    assert_eq!(r.length_squared(), f(0x1900000000));
    assert_eq!(r.dot(rot(0x400000000, -0x300000000)), f(0));
    assert_eq!(r.dot(r), r.length_squared());
}

#[test]
fn test_interpolation() {
    let a = Rot2Trait::new(f(0x33333333));
    let b = Rot2Trait::new(f(0x100000000));
    let zero = f(0);
    let one = f(ONE_RAW);
    let half = f(0x80000000);

    assert_eq!(a.lerp(b, zero), a);
    assert_eq!(a.lerp(b, one), b);
    assert!(a.lerp(b, half).length_squared() < f(ONE_RAW));
    assert!(drift(a.lerp(b, half).normalize()) <= 2);
    assert_eq!(a.slerp(b, zero), a);
    assert!(close_rot(a.slerp(b, one), b, 10));
    assert!(close_fixed(a.slerp(b, half).angle(), f(0x9999999a), 12));

    // Not normalized, as upstream: the midpoint of `(1, 0)` and `(0, 1)` is `(0.5, 0.5)` and the
    // midpoint of opposite rotations is the zero complex number, not a fallback.
    let quarter = rot(0, ONE_RAW);
    assert_eq!(Rot2Trait::IDENTITY.lerp(quarter, half), rot(0x80000000, 0x80000000));
    assert_eq!(Rot2Trait::IDENTITY.lerp(rot(-ONE_RAW, 0), half), rot(0, 0));
    assert_eq!(quarter.lerp(rot(0, -ONE_RAW), half), rot(0, 0));
    // A normalized blend is spelled `lerp(..).normalize()`; the floored length puts it 1 ULP
    // above `round(sqrt(0.5) * 2^32) = 3037000500`.
    let mid = Rot2Trait::IDENTITY.lerp(quarter, half).normalize();
    assert_eq!(mid, rot(3037000501, 3037000501));
    // Extrapolation is not clamped.
    assert_eq!(Rot2Trait::IDENTITY.lerp(quarter, f(2 * ONE_RAW)), rot(-ONE_RAW, 2 * ONE_RAW));
}

#[test]
fn test_is_normalized_threshold_and_long_rotations() {
    assert!(Rot2Trait::IDENTITY.is_normalized());
    assert!(rot(0, -ONE_RAW).is_normalized());
    assert!(Rot2Trait::new(f(0x33333333)).is_normalized());
    assert!(!rot(0, 0).is_normalized());
    assert!(!rot(0x80000000, 0x80000000).is_normalized());
    // The floored squared lengths are 1 - 1025, 1 - 1024, 1 + 1024 and 1 + 1025 ULP.
    assert!(!rot(4294966783, 65536).is_normalized());
    assert!(rot(4294966784, 0).is_normalized());
    assert!(rot(4294967808, 0).is_normalized());
    assert!(!rot(4294967808, 65536).is_normalized());
    // Total on long rotations: the wide sum of squares is compared without narrowing.
    assert!(!MAX_ROT.is_normalized());
    assert!(!rot(-0x8000000000000000, -0x8000000000000000).is_normalized());
    assert!(!rot(0x7fffffffffffffff, 0).is_normalized());
    assert!(!rot(0, -0x8000000000000000).is_normalized());
}

#[test]
fn test_angle_between_slerp_branch_cut_and_rotate_towards() {
    let a = Rot2Trait::new(f(0));
    let b = Rot2Trait::new(f(0x1921fb544));
    assert!(close_fixed(a.angle_between(b), f(6746518852), 6));

    // 0 -> 3 pi / 2 takes the short path through -pi / 2.
    let long = Rot2Trait::new(f(20239556556));
    assert!(close_fixed(a.slerp(long, f(0x80000000)).angle(), f(-3373259426), 12));

    let target = Rot2Trait::new(f(0x100000000));
    let moved = a.rotate_towards(target, f(0x4ccccccd));
    assert!(close_fixed(moved.angle(), f(0x4ccccccd), 10));
    assert_eq!(a.rotate_towards(target, f(0x200000000)), target);
}

#[test]
fn test_from_rotation_arc() {
    assert_eq!(Rot2Trait::from_rotation_arc(Vec2Trait::X, Vec2Trait::Y), rot(0, ONE_RAW));
    assert_eq!(Rot2Trait::from_rotation_arc(Vec2Trait::Y, Vec2Trait::X), rot(0, -ONE_RAW));
    assert_eq!(Rot2Trait::from_rotation_arc(Vec2Trait::X, -Vec2Trait::X), rot(-ONE_RAW, 0));
}
/// `step(re, im)`, exact absolute squared-norm drift after 100 and 1,000 products. The steps
/// are values emitted by `fixed::trig::sin_cos` (angles 1, 0.5, pi/4, 1,000 tau and 0.0287 rad).
/// The largest measured drift in this table is 198 ULP after 100 and 1,998 after 1,000.
#[cairofmt::skip]
const DRIFT_CASES: [(i64, i64, i64, i64); 5] = [
    (0x8a51407d, 0xd76aa478, 117, 1189),
    (0xe0a94033, 0x7abba1d1, 14, 74),
    (0xb504f334, 0xb504f334, 2, 4),
    (0x100000000, -44, 198, 1998),
    (0xffe4ed69, 0x75b8aad, 48, 217),
];

#[test]
fn test_composition_norm_drift() {
    for case in DRIFT_CASES.span() {
        let (re, im, after_100, after_1000) = *case;
        let step = rot(re, im);
        let mut r = Rot2Trait::IDENTITY;
        for _ in 0_u32..100 {
            r *= step;
        }
        assert_eq!(drift(r), after_100, "100 compositions of ({}, {})", re, im);
        for _ in 100_u32..1000 {
            r *= step;
        }
        assert_eq!(drift(r), after_1000, "1000 compositions of ({}, {})", re, im);
    }
}

// Seeded fuzz properties. At most six per module; these five use rotations produced by `new`.

#[test]
#[fuzzer(runs: 128, seed: 1201)]
fn fuzz_mul(a: i64, b: i64, c: i64) {
    let p = unit(a);
    let q = unit(b);
    let r = unit(c);
    assert_eq!(Rot2Trait::IDENTITY * p, p);
    assert_eq!(p * Rot2Trait::IDENTITY, p);
    assert!(close_rot((p * q) * r, p * (q * r), 4));
    assert!(close_rot((p * q).inverse(), q.inverse() * p.inverse(), 2));
    assert!(drift(p * q) <= 16);
}

#[test]
#[fuzzer(runs: 128, seed: 1202)]
fn fuzz_transform_round_trip(a: i64, x: i64, y: i64) {
    let r = unit(a);
    let v = vec(x % 0x400000000, y % 0x400000000);
    assert!(close_vec(r.inverse_transform_vector(r.mul_vec2(v)), v, 32));
    assert_eq!(r.mul_vec2(v), r.transform_vector(v));
}

#[test]
#[fuzzer(runs: 128, seed: 1203)]
fn fuzz_normalize(re: i64, im: i64) {
    let x = re % 0x1000000000;
    let y = im % 0x1000000000;
    // Keep the input length at least one: below that, the one-ULP floor of the length is a large
    // relative error by design, and the dedicated threshold table covers that region.
    let r = if abs_raw(x) < ONE_RAW && abs_raw(y) < ONE_RAW {
        rot(ONE_RAW, y)
    } else {
        rot(x, y)
    };
    let n = r.normalize();
    assert!(drift(n) <= 4);
    assert!(close_rot(n.normalize(), n, 2));
}

#[test]
#[fuzzer(runs: 128, seed: 1204)]
fn fuzz_interpolation(a: i64, b: i64, t: u64) {
    let p = unit(a);
    let candidate = unit(b);
    // The raw blend of two unit rotations stays inside the unit circle (convexity). Its
    // normalization is ill-conditioned at the zero midpoint of opposite rotations: keep that
    // property away from the region; the exact opposite case is pinned in the table.
    let s = f((t % 4294967297_u64).try_into().unwrap());
    assert!(p.lerp(candidate, s).length_squared() <= f(ONE_RAW + 8));
    let q = if p.dot(candidate) < f(-0xf0000000) {
        p
    } else {
        candidate
    };
    assert!(drift(p.lerp(q, s).normalize()) <= 32);
    assert!(drift(p.slerp(q, s)) <= 32);
}

#[test]
#[fuzzer(runs: 128, seed: 1205)]
fn fuzz_rotation_arc(a: i64, b: i64) {
    let from = Vec2Trait::from_angle(f(a % TAU_RAW));
    let to = Vec2Trait::from_angle(f(b % TAU_RAW));
    let r = Rot2Trait::from_rotation_arc(from, to);
    assert!(drift(r) <= 16);
    assert!(close_vec(r.mul_vec2(from), to, 16));
}

const MAX_ROT: Rot2 = Rot2 {
    re: Fixed { raw: 0x7fffffffffffffff }, im: Fixed { raw: 0x7fffffffffffffff },
};
const MIN_IM: Rot2 = Rot2 { re: Fixed { raw: 0 }, im: Fixed { raw: -0x8000000000000000 } };
const MAX_VEC: Vec2 = Vec2 {
    x: Fixed { raw: 0x7fffffffffffffff }, y: Fixed { raw: 0x7fffffffffffffff },
};

#[test]
#[should_panic(expected: 'i64_neg Underflow')]
fn test_inverse_panics_min() {
    let _ = MIN_IM.inverse();
}

#[test]
#[should_panic(expected: 'i64_neg Underflow')]
fn test_to_mat_panics_min() {
    let _ = MIN_IM.to_mat();
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_mul_panics_overflow() {
    let _ = MAX_ROT * MAX_ROT;
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_transform_vector_panics_overflow() {
    let _ = MAX_ROT.transform_vector(MAX_VEC);
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_inverse_transform_vector_panics_overflow() {
    let _ = MAX_ROT.inverse_transform_vector(MAX_VEC);
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_length_panics_overflow() {
    let _ = MAX_ROT.length();
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_length_squared_panics_overflow() {
    let _ = MAX_ROT.length_squared();
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_dot_panics_overflow() {
    let _ = MAX_ROT.dot(MAX_ROT);
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_lerp_panics_overflow() {
    let _ = Rot2Trait::IDENTITY.lerp(rot(-ONE_RAW, 0), f(0x7fffffffffffffff));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_slerp_panics_angle_scale_overflow() {
    let target = rot(0, ONE_RAW);
    let _ = Rot2Trait::IDENTITY.slerp(target, f(0x7fffffffffffffff));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_angle_between_panics_overflow() {
    let _ = MAX_ROT.angle_between(MAX_ROT);
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_rotate_towards_panics_min_budget() {
    let target = rot(0, ONE_RAW);
    let _ = Rot2Trait::IDENTITY.rotate_towards(target, f(-0x8000000000000000));
}

// panics: Rot2::from_rotation_arc
#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_rotation_arc_panics_overflow() {
    let _ = Rot2Trait::from_rotation_arc(MAX_VEC, MAX_VEC);
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_mul_vec2_panics_overflow() {
    let _ = MAX_ROT.mul_vec2(MAX_VEC);
}
