//! Tests of `glamx::rot3`: `Rot3` is a type alias of `glam_core::quat::Quat`, so the checks below
//! only confirm that the `Quat` API (trait methods, operator impls, conversions, `Default`,
//! derives) resolves through the alias. The rotation arithmetic itself is tested by `glam`.

use core::hash::{HashStateExTrait, HashStateTrait};
use core::poseidon::PoseidonTrait;
use fixed::fixed::{Fixed, FixedTrait};
use glam_core::quat::{Quat, QuatTrait, quat};
use glam_core::vec3::{Vec3, vec3};
use glam_core::vec4::{Vec4, vec4};
use glamx::rot3::Rot3;

const ONE: i64 = 0x100000000;

fn f(raw: i64) -> Fixed {
    FixedTrait::from_raw(raw)
}

fn hash(q: Quat) -> felt252 {
    PoseidonTrait::new().update_with(q).finalize()
}

#[test]
fn test_rot3_is_quat() {
    // Literal, constants and `Default` through the alias.
    let r: Rot3 = Rot3 { x: f(0), y: f(0), z: f(ONE), w: f(0) };
    let q: Quat = r;
    assert_eq!(q, quat(f(0), f(0), f(ONE), f(0)));
    let id: Rot3 = QuatTrait::IDENTITY;
    assert_eq!(id, Default::<Rot3>::default());
    assert_eq!(id, Default::<Quat>::default());
    // Trait methods.
    let v: Vec3 = vec3(f(ONE), f(2 * ONE), f(3 * ONE));
    assert_eq!(r.mul_vec3(v), vec3(f(-ONE), f(-2 * ONE), f(3 * ONE)));
    assert_eq!(r.conjugate(), quat(f(0), f(0), f(-ONE), f(0)));
    assert_eq!(r.inverse().mul_quat(r), id);
    assert!(r.is_normalized());
    assert_eq!(r.normalize(), r);
    // Operators.
    let s: Rot3 = r * r;
    assert_eq!(s, -id);
    assert_eq!(r + r - r, r);
    // Conversions and derives.
    let w: Vec4 = r.into();
    assert_eq!(w, vec4(f(0), f(0), f(ONE), f(0)));
    let back: Rot3 = w.into();
    assert_eq!(back, r);
    assert_eq!(hash(r), hash(q));
}
