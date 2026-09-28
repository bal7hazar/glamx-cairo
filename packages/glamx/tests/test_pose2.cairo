//! Tests of `glamx::pose2`: constructors and conversions, exact transforms, fused/composed
//! equivalence, matrix conversion, measured composition drift, seeded properties and panic paths.
//!
//! The f64 glamx oracle is checked separately by generated `golden_pose2.cairo`.

use fixed::fixed::{Fixed, FixedTrait};
use glam_core::mat3::Mat3Trait;
use glam_core::vec2::{Vec2, Vec2Trait, vec2};
use glamx::pose2::{Pose2, Pose2Trait, Rot2Pose2Trait};
use glamx::rot2::{Rot2, Rot2Trait};

const ONE: i64 = 0x100000000;
const TAU: i64 = 26986075409;
const FRAC_PI_2: i64 = 6746518852;

fn f(raw: i64) -> Fixed {
    FixedTrait::from_raw(raw)
}

fn n(v: i64) -> Fixed {
    f(v * ONE)
}

fn v2(x: i64, y: i64) -> Vec2 {
    vec2(n(x), n(y))
}

fn unit(angle: i64) -> Rot2 {
    Rot2Trait::new(f(angle % TAU))
}

fn pose(a: i64, x: i64, y: i64) -> Pose2 {
    Pose2Trait::from_parts(vec2(f(x % 0x10000000000), f(y % 0x10000000000)), unit(a))
}

fn abs_raw(x: i64) -> i64 {
    if x < 0 {
        -x
    } else {
        x
    }
}

fn drift(r: Rot2) -> i64 {
    abs_raw(r.length_squared().raw - ONE)
}

fn composed_transform_point(p: Pose2, v: Vec2) -> Vec2 {
    p.rotation.transform_vector(v) + p.translation
}

fn composed_inverse_transform_point(p: Pose2, v: Vec2) -> Vec2 {
    p.rotation.inverse().transform_vector(v - p.translation)
}

fn composed_inverse(p: Pose2) -> Pose2 {
    let inv = p.rotation.inverse();
    Pose2 { rotation: inv, translation: inv.transform_vector(-p.translation) }
}

fn composed_inv_mul(a: Pose2, b: Pose2) -> Pose2 {
    let inv = a.rotation.inverse();
    Pose2 {
        rotation: inv * b.rotation,
        translation: inv.transform_vector(b.translation - a.translation),
    }
}

fn composed_mul(a: Pose2, b: Pose2) -> Pose2 {
    Pose2 {
        rotation: a.rotation * b.rotation,
        translation: a.translation + a.rotation.transform_vector(b.translation),
    }
}

fn assert_fused_is_composed(a: Pose2, b: Pose2, v: Vec2) {
    assert_eq!(a.transform_point(v), composed_transform_point(a, v));
    assert_eq!(a.mul_vec2(v), composed_transform_point(a, v));
    assert_eq!(a.inverse_transform_point(v), composed_inverse_transform_point(a, v));
    assert_eq!(a.inverse_transform_vector(v), a.rotation.inverse().transform_vector(v));
    assert_eq!(a.transform_vector(v), a.rotation.transform_vector(v));
    assert_eq!(a.inverse(), composed_inverse(a));
    assert_eq!(a.inv_mul(b), composed_inv_mul(a, b));
    assert_eq!(a * b, composed_mul(a, b));
    assert_eq!(
        a.prepend_translation(v),
        Pose2 { rotation: a.rotation, translation: a.translation + a.rotation.transform_vector(v) },
    );
}

#[test]
fn test_identity_and_constructors() {
    let id = Pose2 { rotation: Rot2Trait::IDENTITY, translation: Vec2Trait::ZERO };
    assert_eq!(Pose2Trait::IDENTITY, id);
    assert_eq!(Pose2Trait::identity(), id);
    assert_eq!(Default::<Pose2>::default(), id);
    let t = v2(1, 2);
    let r = Rot2 { re: f(0), im: f(ONE) };
    assert_eq!(
        Pose2Trait::from_translation(t), Pose2 { rotation: Rot2Trait::IDENTITY, translation: t },
    );
    assert_eq!(Pose2Trait::translation(n(1), n(2)), Pose2Trait::from_translation(t));
    assert_eq!(Pose2Trait::from_rotation(r), Pose2 { rotation: r, translation: Vec2Trait::ZERO });
    assert_eq!(Pose2Trait::from_parts(t, r), Pose2 { rotation: r, translation: t });
    assert_eq!(Pose2Trait::new(t, f(0)), Pose2Trait::from_translation(t));
    assert_eq!(Pose2Trait::rotation(f(0)), id);
    assert_eq!(Pose2Trait::rotation(f(FRAC_PI_2)).rotation, Rot2Trait::new(f(FRAC_PI_2)));
    assert_eq!(
        Pose2Trait::new(t, f(FRAC_PI_2)), Pose2Trait::from_parts(t, Rot2Trait::new(f(FRAC_PI_2))),
    );
}

#[test]
fn test_conversions() {
    let t = v2(1, -2);
    let r = Rot2 { re: f(0), im: f(ONE) };
    let p: Pose2 = r.into();
    assert_eq!(p, Pose2Trait::from_rotation(r));
    let p: Pose2 = (t, r).into();
    assert_eq!(p, Pose2Trait::from_parts(t, r));
    let (t2, r2): (Vec2, Rot2) = p.into();
    assert_eq!(t2, t);
    assert_eq!(r2, r);
}

#[test]
fn test_exact_rotations() {
    let quarter = Rot2 { re: f(0), im: f(ONE) };
    let p = Pose2Trait::from_parts(v2(1, 2), quarter);
    assert_eq!(p.transform_point(v2(1, 2)), v2(-1, 3));
    assert_eq!(p.transform_vector(v2(1, 2)), v2(-2, 1));
    assert_eq!(p.inverse_transform_point(v2(-1, 3)), v2(1, 2));
    assert_eq!(p.inverse_transform_vector(v2(-2, 1)), v2(1, 2));
    assert_eq!(p.inverse(), Pose2Trait::from_parts(v2(-2, 1), quarter.inverse()));
    assert_eq!(p * p.inverse(), Pose2Trait::IDENTITY);
    assert_eq!(p.inv_mul(p), Pose2Trait::IDENTITY);
    let raw = vec2(f(123456789), f(-987654321));
    assert_eq!(Pose2Trait::IDENTITY.transform_point(raw), raw);
    assert_eq!(Pose2Trait::IDENTITY.inverse_transform_point(raw), raw);
    assert_eq!(Pose2Trait::from_translation(v2(1, 1)).transform_point(v2(1, 2)), v2(2, 3));
}

/// The upstream glamx unit-test examples, with `1e-6` converted to raw Q32.32 ULP.
#[test]
fn test_glamx_examples() {
    let eps = f(4295);
    let p = Pose2Trait::new(v2(1, 0), f(FRAC_PI_2));
    assert!(p.transform_point(v2(1, 0)).abs_diff_eq(v2(1, 1), eps));
    assert!(p.mul_vec2(v2(1, 0)).abs_diff_eq(v2(1, 1), eps));
    let p = Pose2Trait::new(v2(100, 200), f(FRAC_PI_2));
    assert!(p.transform_vector(v2(1, 0)).abs_diff_eq(v2(0, 1), eps));
    let p = Pose2Trait::new(v2(1, 2), f(0x80000000));
    let id = p * p.inverse();
    assert!(id.rotation.re.abs_diff_eq(n(1), eps));
    assert!(id.rotation.im.abs_diff_eq(f(0), eps));
    assert!(id.translation.abs_diff_eq(Vec2Trait::ZERO, eps));
    let back = Pose2Trait::from_mat3(p.to_mat3());
    assert!(back.abs_diff_eq(p, eps));
}

#[test]
fn test_fused_is_composed() {
    let a = Pose2Trait::from_parts(
        vec2(f(0x180000000), f(-0x1c0000000)), Rot2 { re: f(0xc3a5c85c), im: f(0xa4f3e27a) },
    );
    let b = Pose2Trait::from_parts(
        vec2(f(-0x340000000), f(0x280000000)), Rot2 { re: f(-0xbd736074), im: f(0xad7ac4e7) },
    );
    assert_fused_is_composed(a, b, v2(7, -11));
    assert_fused_is_composed(b, a, vec2(f(-1), f(0x12345678)));
    assert_fused_is_composed(Pose2Trait::IDENTITY, a, Vec2Trait::ZERO);
    let c = Pose2Trait::from_parts(v2(5, 6), Rot2 { re: n(2), im: n(3) });
    assert_fused_is_composed(c, a, v2(1, -2));
}

#[test]
fn test_append_prepend_rot2_products_and_assign() {
    let r = Rot2 { re: f(0), im: f(ONE) };
    let p = Pose2Trait::from_parts(v2(1, 2), r);
    assert_eq!(p.append_translation(v2(1, 1)), Pose2Trait::from_parts(v2(2, 3), r));
    assert_eq!(p.prepend_translation(v2(1, 1)), Pose2Trait::from_parts(v2(0, 3), r));
    assert_eq!(p.prepend_translation(v2(1, 1)), p * Pose2Trait::from_translation(v2(1, 1)));
    assert_eq!(p.append_translation(v2(1, 1)), Pose2Trait::from_translation(v2(1, 1)) * p);
    let s = Rot2 { re: f(-ONE), im: f(0) };
    assert_eq!(p.mul_rot2(s), p * Pose2Trait::from_rotation(s));
    assert_eq!(s.mul_pose2(p), Pose2Trait::from_rotation(s) * p);
    assert_eq!(s.mul_pose2(p).translation, v2(-1, -2));
    let mut q = p;
    q *= p;
    assert_eq!(q, p * p);
    let mut q = p;
    q *= s;
    assert_eq!(q, p.mul_rot2(s));
}

#[test]
fn test_abs_diff_eq_and_lerp() {
    let a = Pose2Trait::from_parts(Vec2Trait::ZERO, Rot2Trait::IDENTITY);
    let mut q = a;
    q.translation.x = f(5);
    assert!(a.abs_diff_eq(q, f(5)));
    assert!(!a.abs_diff_eq(q, f(4)));
    q = a;
    q.rotation.im = f(-5);
    assert!(a.abs_diff_eq(q, f(5)));
    assert!(!a.abs_diff_eq(q, f(4)));

    let b = Pose2Trait::from_parts(v2(2, 4), Rot2 { re: f(-ONE), im: f(0) });
    assert_eq!(a.lerp(b, f(0)).translation, a.translation);
    assert_eq!(a.lerp(b, n(1)).translation, b.translation);
    assert_eq!(a.lerp(b, f(ONE / 2)).translation, v2(1, 2));
    assert_eq!(a.lerp(b, f(ONE / 2)).rotation, a.rotation.slerp(b.rotation, f(ONE / 2)));

    // Across the +/-pi branch cut, the midpoint stays near pi rather than crossing through zero.
    let p = Pose2Trait::new(Vec2Trait::ZERO, f(13443288064)); // 3.13
    let q = Pose2Trait::new(v2(2, 4), f(13529187410)); // 3.15
    assert!(p.lerp(q, f(ONE / 2)).rotation.angle().abs_diff_eq(f(13486237737), f(16)));
}

#[test]
fn test_to_from_mat3() {
    let p = Pose2Trait::from_parts(v2(1, 2), Rot2 { re: f(0), im: f(ONE) });
    let m = p.to_mat3();
    assert_eq!(
        m,
        Mat3Trait::from_cols(
            glam_core::vec3::Vec3 { x: f(0), y: f(ONE), z: f(0) },
            glam_core::vec3::Vec3 { x: f(-ONE), y: f(0), z: f(0) },
            glam_core::vec3::Vec3 { x: n(1), y: n(2), z: f(ONE) },
        ),
    );
    assert_eq!(Pose2Trait::from_mat3(m), p);
    assert_eq!(Pose2Trait::from_mat3(Mat3Trait::IDENTITY), Pose2Trait::IDENTITY);
}

/// Pose composition has exactly the `Rot2` product's measured norm drift: the worst committed
/// representative reaches 198 ULP after 100 products and 1,998 ULP after 1,000.
#[test]
fn test_composition_drift() {
    let step = Pose2Trait::from_parts(
        vec2(f(0x10000000), f(0)), Rot2 { re: f(0x100000000), im: f(-44) },
    );
    let mut p = Pose2Trait::IDENTITY;
    for _ in 0_u32..100 {
        p *= step;
    }
    assert_eq!(drift(p.rotation), 198);
    for _ in 100_u32..1000 {
        p *= step;
    }
    assert_eq!(drift(p.rotation), 1998);
    assert!(drift(p.rotation.normalize()) <= 4);
}

#[test]
#[fuzzer(runs: 128, seed: 2101)]
fn fuzz_fused_is_composed(a: i64, b: i64, c: i64) {
    let p = pose(a, b, c);
    let q = pose(c, a, b);
    let v = vec2(f(c % 0x10000000000), f(a % 0x10000000000));
    assert_fused_is_composed(p, q, v);
}

#[test]
#[fuzzer(runs: 128, seed: 2102)]
fn fuzz_inverse(a: i64, b: i64, c: i64) {
    let p = pose(a, b, c);
    let v = vec2(f(c % 0x10000000000), f(a % 0x10000000000));
    let id = p * p.inverse();
    assert!(id.abs_diff_eq(Pose2Trait::IDENTITY, f(4096)));
    assert!(p.inv_mul(p).abs_diff_eq(Pose2Trait::IDENTITY, f(16)));
    assert!(p.inverse_transform_point(p.transform_point(v)).abs_diff_eq(v, f(8192)));
    assert!(p.inverse_transform_vector(p.transform_vector(v)).abs_diff_eq(v, f(8192)));
    assert_eq!(p.inverse().inverse().rotation, p.rotation);
}

#[test]
#[fuzzer(runs: 128, seed: 2103)]
fn fuzz_composition(a: i64, b: i64, c: i64) {
    let p = pose(a, b, c);
    let q = pose(c, a, b);
    let r = pose(b, c, a);
    let v = vec2(f(c % 0x10000000000), f(a % 0x10000000000));
    assert_eq!(Pose2Trait::IDENTITY * p, p);
    assert_eq!(p * Pose2Trait::IDENTITY, p);
    assert!(((p * q) * r).abs_diff_eq(p * (q * r), f(8192)));
    assert!(
        (p * q).transform_point(v).abs_diff_eq(p.transform_point(q.transform_point(v)), f(8192)),
    );
    // The upstream two-rotation `inverse() * rhs` may differ in its last bits from fused `inv_mul`.
    assert!(p.inv_mul(q).abs_diff_eq(p.inverse() * q, f(8192)));
    assert!(r.rotation.mul_pose2(q).abs_diff_eq(Pose2Trait::from_rotation(r.rotation) * q, f(0)));
    assert_eq!(p.mul_rot2(r.rotation), p * Pose2Trait::from_rotation(r.rotation));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_transform_point_overflow() {
    Pose2Trait::from_translation(v2(0x40000000, 0)).transform_point(v2(0x40000000, 0));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_mul_overflow() {
    let p = Pose2Trait::from_translation(v2(0x40000000, 0));
    let _ = p * p;
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_inverse_overflow() {
    Pose2Trait::from_parts(v2(-0x40000000, 0), Rot2 { re: n(2), im: f(0) }).inverse();
}

#[test]
#[should_panic(expected: ('i64_sub Overflow',))]
fn test_inverse_transform_point_sub_overflow() {
    let p = Pose2Trait::from_translation(vec2(f(-0x7fffffffffffffff), f(0)));
    p.inverse_transform_point(vec2(f(0x7fffffffffffffff), f(0)));
}

#[test]
#[should_panic(expected: ('i64_sub Overflow',))]
fn test_inv_mul_sub_overflow() {
    let a = Pose2Trait::from_translation(vec2(f(-0x7fffffffffffffff), f(0)));
    let b = Pose2Trait::from_translation(vec2(f(0x7fffffffffffffff), f(0)));
    a.inv_mul(b);
}

#[test]
#[should_panic(expected: ('i64_add Overflow',))]
fn test_append_translation_overflow() {
    Pose2Trait::from_translation(vec2(f(0x7fffffffffffffff), f(0)))
        .append_translation(vec2(f(1), f(0)));
}

#[test]
#[should_panic(expected: ('i64_neg Underflow',))]
fn test_to_mat3_neg_overflow() {
    Pose2Trait::from_rotation(Rot2 { re: f(0), im: f(-0x8000000000000000) }).to_mat3();
}

#[test]
#[should_panic(expected: ('i64_neg Underflow',))]
fn test_inverse_neg_underflow() {
    Pose2Trait::from_rotation(Rot2 { re: f(0), im: f(-0x8000000000000000) }).inverse();
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_prepend_translation_overflow() {
    Pose2Trait::from_translation(v2(0x40000000, 0)).prepend_translation(v2(0x40000000, 0));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_inv_mul_overflow() {
    let a = Pose2Trait::from_rotation(Rot2 { re: n(2), im: f(0) });
    a.inv_mul(Pose2Trait::from_translation(v2(0x40000000, 0)));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_transform_vector_overflow() {
    Pose2Trait::from_rotation(Rot2 { re: n(2), im: f(0) }).transform_vector(v2(0x40000000, 0));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_inverse_transform_point_overflow() {
    let p = Pose2Trait::from_rotation(Rot2 { re: n(2), im: f(0) });
    p.inverse_transform_point(v2(0x40000000, 0));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_inverse_transform_vector_overflow() {
    let p = Pose2Trait::from_rotation(Rot2 { re: n(2), im: f(0) });
    p.inverse_transform_vector(v2(0x40000000, 0));
}

#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_mul_rot2_overflow() {
    let big = Rot2 { re: n(0x40000000), im: f(0) };
    Pose2Trait::from_rotation(big).mul_rot2(big);
}

// panics: Rot2Pose2::mul_pose2
#[test]
#[should_panic(expected: 'Fixed: overflow')]
fn test_rot2_mul_pose2_overflow() {
    let big = Rot2 { re: n(0x40000000), im: f(0) };
    big.mul_pose2(Pose2Trait::from_rotation(big));
}
