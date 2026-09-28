//! Port of glamx `rot3` @ 0.3.1: the 3D rotation type, a unit quaternion.
//!
//! glamx adds no method of its own here: `Rot3` is the name parry and rapier use for
//! `glam::Quat`, so every `QuatTrait` method, operator impl (`QuatMul`, `QuatNeg`, ...),
//! conversion and constant (`QuatTrait::IDENTITY`) applies to a `Rot3` unchanged.

/// A 3D rotation represented as a unit quaternion.
///
/// A type alias: `Rot3` and `glam_core::quat::Quat` are the same type, the trait impls of `Quat`
/// resolve through the alias (`QuatTrait::IDENTITY` is a `Rot3`, `r.mul_vec3(v)` and `r * s`
/// compile on `Rot3` values) and a `Rot3` literal is written `Rot3 { x, y, z, w }`.
///
/// Mirrors `glamx::Rot3`.
/// #### Panics
/// * Never.
/// #### Deviations
/// * The f64 alias `DRot3` does not exist: there is one scalar (docs/DESIGN.md section 1).
pub type Rot3 = glam_core::quat::Quat;
