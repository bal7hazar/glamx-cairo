//! Port of glamx `pose3.rs` @ 0.3.1 on the Q32.32 scalar: a rigid-body transformation
//! (rotation + translation), the `Pose` of parry and rapier in 3D.
//!
//! Every rotation of a vector goes through one fused kernel: the translation added by
//! [`Pose3Trait::transform_point`] and by the composition `Pose3 * Pose3` is lifted into the exact
//! Q96.96 sum of the rotation instead of being added after its rescale, and the rotations by the
//! conjugate of [`Pose3Trait::inverse`], [`Pose3Trait::inv_mul`] and
//! [`Pose3Trait::inverse_transform_point`] flip the sign of the cross-product term inside the
//! kernel instead of building the conjugate quaternion first. The exact value is the same, so the
//! results are bit-identical to the composed forms of glamx (`q.conjugate().mul_vec3(v) + t`,
//! ...), which live in `benches::alt::pose3` with their benches (`gas/pose3.snap`).
//!
//! Rotations are not renormalized by composition, as upstream: see [`Pose3`] for the measured
//! drift and the renormalization policy it implies.

use core::ops::MulAssign;
use fixed::fixed::Fixed;
use fixed::wide::{WideAdd, WideLift, WideMul, WideNarrow, WideSub, wide_from, wide_mul};
use glam_core::mat4::{Mat4, Mat4Trait};
use glam_core::quat::{Quat, QuatTrait};
use glam_core::vec3::{Vec3, Vec3Trait};
use glam_core::vec4::Vec4Trait;
use crate::rot3::Rot3;

/// A 3D pose (rotation + translation), representing a rigid-body transformation: a point `p` is
/// mapped to `rotation * p + translation`.
///
/// The rotation is intended to be of unit length but is not renormalized by composition, as in
/// glamx: every `Pose3 * Pose3`, `inv_mul`, `mul_rot3` and `mul_pose3` multiplies the rotations
/// with `QuatTrait::mul_quat`, whose floor rescale makes the squared length of the rotation
/// drift. Measured by `test_composition_drift` on a chain `p = p * step`: 58 ULP after 100
/// compositions, 278 after 400, 697 after 1 000 (about 0.7 ULP per composition); a chain of
/// `inv_mul`, whose conjugated terms floor in both directions, stays within 36 ULP over 1 000
/// steps. `glam_core::quat` measures up to 2.5 ULP per product on random chains (271 ULP after 100,
/// 2 400 after 1 000). A rotation therefore leaves the `QuatTrait::is_normalized` band
/// (1 024 ULP) after 400 to 1 400 compositions: renormalizing the rotation of a body every few
/// hundred steps (`pose.rotation = pose.rotation.normalize()`, 4 ULP from unit afterwards) is
/// enough, and rapier renormalizes after each integration anyway.
///
/// Mirrors `glamx::Pose3`.
/// #### Deviations
/// * No `padding: u32` field (it exists for `bytemuck` / spirv only): the struct is the two
///   public fields `rotation` and `translation`.
/// * `Debug` is the derived Cairo formatting. `Default` is the identity, as upstream.
/// * One type for the three flavours of glamx (`Pose3`, `Pose3A`, `DPose3`): there is one scalar
///   and one `Vec3` (docs/DESIGN.md section 1).
/// * Not ported (no consumer in parry / rapier, docs/research/06-glamx-scope.md section 6.1):
///   `look_at_rh`, `face_towards`, `is_finite` / `is_nan` (those values do not exist), the
///   by-reference operator overloads, the f32 / f64 / nalgebra conversions and
///   `approx::RelativeEq` (`abs_diff_eq` is ported).
#[derive(Copy, Drop, Serde, PartialEq, Debug, Hash)]
pub struct Pose3 {
    /// The rotational part of the pose.
    pub rotation: Rot3,
    /// The translational part of the pose.
    pub translation: Vec3,
}

pub trait Pose3Trait {
    /// The identity pose (no rotation, no translation).
    ///
    /// Mirrors `glamx::Pose3::IDENTITY`.
    const IDENTITY: Pose3;
    /// Creates the identity pose.
    ///
    /// Mirrors `glamx::Pose3::identity`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn identity() -> Pose3;
    /// Creates a pose from a translation vector (identity rotation).
    ///
    /// Mirrors `glamx::Pose3::from_translation`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_translation(translation: Vec3) -> Pose3;
    /// Creates a pose from translation components (identity rotation).
    ///
    /// Called as `Pose3Trait::translation(x, y, z)`: the associated function and the public field
    /// `pose.translation` share their name, which Cairo accepts as in Rust.
    ///
    /// Mirrors `glamx::Pose3::translation`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn translation(x: Fixed, y: Fixed, z: Fixed) -> Pose3;
    /// Creates a pose from a rotation (no translation).
    ///
    /// Mirrors `glamx::Pose3::from_rotation`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_rotation(rotation: Rot3) -> Pose3;
    /// Creates a pose from its translation and rotation parts.
    ///
    /// Mirrors `glamx::Pose3::from_parts`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_parts(translation: Vec3, rotation: Rot3) -> Pose3;
    /// Creates a pose from a translation and an axis-angle rotation: the rotation of
    /// `|axisangle|` radians around `axisangle / |axisangle|` (`QuatTrait::from_scaled_axis`,
    /// the identity when `axisangle` is zero).
    ///
    /// Mirrors `glamx::Pose3::new`.
    /// #### Panics
    /// * As `QuatTrait::from_scaled_axis`.
    /// #### Deviations
    /// * The rotation carries the error of `QuatTrait::from_scaled_axis` (floored halving,
    ///   `sin_cos`, rounded normalization of the axis: a few ULP).
    fn new(translation: Vec3, axisangle: Vec3) -> Pose3;
    /// Creates a pose from an axis-angle rotation only (no translation): see [`Pose3Trait::new`].
    ///
    /// Called as `Pose3Trait::rotation(axisangle)`: the associated function and the public field
    /// `pose.rotation` share their name, which Cairo accepts as in Rust.
    ///
    /// Implementation notes:
    /// * As [`Pose3Trait::new`].
    ///
    /// Mirrors `glamx::Pose3::rotation`.
    /// #### Panics
    /// * As `QuatTrait::from_scaled_axis`.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn rotation(axisangle: Vec3) -> Pose3;
    /// Prepends a translation to this pose, i.e. applies `translation` in the local frame first:
    /// the translation becomes `self.translation + self.rotation * translation`.
    ///
    /// Mirrors `glamx::Pose3::prepend_translation`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the new translation does not fit the scalar range.
    /// #### Deviations
    /// * The rotated vector and the translation are summed exactly and floored once (the same
    ///   kernel as [`Pose3Trait::transform_point`]): at most 1 ULP below the exact result.
    fn prepend_translation(self: Pose3, translation: Vec3) -> Pose3;
    /// Appends a translation to this pose, i.e. applies `translation` in the world frame last:
    /// the translation becomes `self.translation + translation`. Exact.
    ///
    /// Mirrors `glamx::Pose3::append_translation`.
    /// #### Panics
    /// * `'i64_add Overflow'` / `'i64_add Underflow'` if a component leaves the scalar range.
    /// #### Deviations
    /// * Overflow panics with the native message where floating-point arithmetic returns infinity
    ///   or a larger finite value: docs/DESIGN.md section 3, "overflow".
    fn append_translation(self: Pose3, translation: Vec3) -> Pose3;
    /// Returns the inverse of this pose: the rotation is conjugated (no division) and the
    /// translation becomes `conjugate(rotation) * -translation`.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized for the result to be the inverse; it is not checked.
    ///
    /// Implementation notes:
    /// * The rotation by the conjugate is the fused kernel of `QuatTrait::mul_vec3` with the sign
    ///   of the cross-product term flipped and the whole sum negated in place: one floor rescale
    ///   per component, bit-identical to `rotation.conjugate().mul_vec3(-translation)`:
    ///   10 360 gas against 11 560.
    ///
    /// Mirrors `glamx::Pose3::inverse`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the translation does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inverse(self: Pose3) -> Pose3;
    /// Computes `self.inverse() * rhs`, the pose of `rhs` in the frame of `self` (the relative
    /// pose of collision detection): `(conj(qa) * qb, conj(qa) * (tb - ta))`.
    ///
    /// #### Preconditions
    /// * The rotation of `self` must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * Fused: the conjugate of `self.rotation` is never built. The Hamilton product carries its
    ///   signs in the add / sub chain of the kernel and the rotation of the translation flips the
    ///   cross-product term: bit-identical to the composed glamx expression, one floor rescale
    ///   per output scalar: 20 700 gas, against 21 600 for the composed glamx body and 31 240
    ///   for `self.inverse() * rhs` (`gas/pose3.snap`).
    ///
    /// Mirrors `glamx::Pose3::inv_mul`.
    /// #### Panics
    /// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `rhs.translation - self.translation`
    ///   leaves the scalar range.
    /// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inv_mul(self: Pose3, rhs: Pose3) -> Pose3;
    /// Transforms a point by this pose: `rotation * p + translation`.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized for the result to be a rigid motion; it is not checked.
    ///
    /// Implementation notes:
    /// * The translation is lifted into the exact Q96.96 sum of the rotation and the component
    ///   is floored once: bit-identical to `rotation.mul_vec3(p) + translation`, but a rotated
    ///   vector outside the scalar range does not panic when the sum fits. 10 260 gas against
    ///   11 880 for the composed form.
    ///
    /// Mirrors `glamx::Pose3::transform_point`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn transform_point(self: Pose3, p: Vec3) -> Vec3;
    /// Transforms a vector by this pose: the rotation only, the translation is ignored.
    ///
    /// Implementation notes:
    /// * As `QuatTrait::mul_vec3`: at most 1 ULP below the exact rotation.
    ///
    /// Mirrors `glamx::Pose3::transform_vector`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn transform_vector(self: Pose3, v: Vec3) -> Vec3;
    /// Transforms a point by the inverse of this pose: `conjugate(rotation) * (p - translation)`.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * The conjugate is not built: the cross-product term of the fused rotation changes sign.
    ///   Bit-identical to the composed form, at most 1 ULP below the exact result: 11 580 gas
    ///   against 12 480.
    ///
    /// Mirrors `glamx::Pose3::inverse_transform_point`.
    /// #### Panics
    /// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `p - translation` leaves the scalar
    ///   range.
    /// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inverse_transform_point(self: Pose3, p: Vec3) -> Vec3;
    /// Transforms a vector by the inverse of this pose: `conjugate(rotation) * v`.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * As [`Pose3Trait::inverse_transform_point`]: 9 360 gas (the cost of
    ///   `QuatTrait::mul_vec3`) against 10 260 with the conjugate built first.
    ///
    /// Mirrors `glamx::Pose3::inverse_transform_vector`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inverse_transform_vector(self: Pose3, v: Vec3) -> Vec3;
    /// Interpolates between two poses: `QuatTrait::slerp` of the rotations and `Vec3Trait::lerp`
    /// of the translations. When `t` is zero the result is `self`, when `t` is one it is `other`.
    ///
    /// #### Preconditions
    /// * Both rotations must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * The rotation carries the re-derived thresholds and the error of `QuatTrait::slerp`
    ///   (132 970 gas in all): [`Pose3Trait::nlerp`] is the cheap alternative.
    ///
    /// Mirrors `glamx::Pose3::lerp`.
    /// #### Panics
    /// * As `QuatTrait::slerp` and `Vec3Trait::lerp`.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn lerp(self: Pose3, other: Pose3, t: Fixed) -> Pose3;
    /// Interpolates between two poses with a normalized linear interpolation of the rotations
    /// (`QuatTrait::lerp`, shortest path) and `Vec3Trait::lerp` of the translations.
    ///
    /// The rotation follows the same great arc as [`Pose3Trait::lerp`] but not at a constant
    /// angular speed; both agree at `t = 0`, `t = 1/2` and `t = 1`. 34 910 gas, 3.8 times
    /// cheaper than the slerp of [`Pose3Trait::lerp`] (132 970).
    ///
    /// #### Preconditions
    /// * Both rotations must be normalized; it is not checked.
    ///
    /// Mirrors nothing in glamx: an addition of glamx.cairo (docs/research/06-glamx-scope.md
    /// section 6.1 item 6).
    /// #### Panics
    /// * As `QuatTrait::lerp` and `Vec3Trait::lerp`.
    /// #### Deviations
    /// * Not in glamx.
    fn nlerp(self: Pose3, other: Pose3, t: Fixed) -> Pose3;
    /// Converts this pose to a homogeneous 4x4 matrix (`Mat4Trait::from_rotation_translation`).
    ///
    /// #### Preconditions
    /// * The rotation must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * As `Mat4Trait::from_rotation_translation`.
    ///
    /// Mirrors `glamx::Pose3::to_mat4`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn to_mat4(self: Pose3) -> Mat4;
    /// Creates a pose from a homogeneous 4x4 matrix: the rotation is `QuatTrait::from_mat4` of
    /// the linear part and the translation is `w_axis.xyz`.
    ///
    /// #### Preconditions
    /// * The matrix must be rigid: its linear part a pure rotation (orthonormal, determinant
    ///   one), no scale and no shear, as glamx documents. It is not checked; a scaled matrix
    ///   gives a rotation that is not normalized.
    ///
    /// Implementation notes:
    /// * glamx decomposes the matrix with `Mat4::to_scale_rotation_translation` (three lengths, a
    ///   determinant and a division per axis) and drops the scale; the rigid-only precondition
    ///   makes that scale one, so the linear part is converted directly with
    ///   `QuatTrait::from_mat4` (19 770 gas). On a rigid matrix both give the same rotation up to
    ///   rounding.
    ///
    /// Mirrors `glamx::Pose3::from_mat4`.
    /// #### Panics
    /// * As `QuatTrait::from_mat4`.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn from_mat4(mat: Mat4) -> Pose3;
    /// Composes `self` with a rotation applied first: `(self.translation, self.rotation * rhs)`.
    ///
    /// Implementation notes:
    /// * As `QuatTrait::mul_quat`: at most 1 ULP below the exact product.
    ///
    /// Mirrors `impl Mul<glamx::Rot3> for glamx::Pose3`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the rotation does not fit the scalar range.
    /// #### Deviations
    /// * glamx spells this with an operator; the core operator traits of Cairo are homogeneous
    ///   (docs/DESIGN.md section 3).
    fn mul_rot3(self: Pose3, rhs: Rot3) -> Pose3;
    /// Transforms the point `rhs`: the same as [`Pose3Trait::transform_point`].
    ///
    /// Mirrors `impl Mul<glam_core::Vec3> for glamx::Pose3`.
    /// #### Panics
    /// * As [`Pose3Trait::transform_point`].
    /// #### Deviations
    /// * glamx spells this with an operator (docs/DESIGN.md section 3).
    fn mul_vec3(self: Pose3, rhs: Vec3) -> Vec3;
    /// Returns `true` if the absolute difference of every component of the rotations and of the
    /// translations is less than or equal to `max_abs_diff`.
    ///
    /// Mirrors `approx::AbsDiffEq for glamx::Pose3`.
    /// #### Panics
    /// * Never: built on `Fixed::abs_diff_eq`, an `i128` difference that never panics (R1
    ///   panic-coverage audit, escalation 1).
    /// #### Deviations
    /// * An inherent method with an explicit tolerance, as `glam_core::quat::QuatTrait::abs_diff_eq`
    ///   (there is no `approx` crate and no default epsilon).
    fn abs_diff_eq(self: Pose3, rhs: Pose3, max_abs_diff: Fixed) -> bool;
}

pub impl Pose3Impl of Pose3Trait {
    const IDENTITY: Pose3 = Pose3 {
        rotation: Quat {
            x: Fixed { raw: 0 }, y: Fixed { raw: 0 }, z: Fixed { raw: 0 }, w: Fixed { raw: ONE },
        },
        translation: Vec3 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 }, z: Fixed { raw: 0 } },
    };

    #[inline(always)]
    fn identity() -> Pose3 {
        Self::IDENTITY
    }

    #[inline(always)]
    fn from_translation(translation: Vec3) -> Pose3 {
        Pose3 { rotation: QuatTrait::IDENTITY, translation }
    }

    #[inline(always)]
    fn translation(x: Fixed, y: Fixed, z: Fixed) -> Pose3 {
        Pose3 { rotation: QuatTrait::IDENTITY, translation: Vec3 { x, y, z } }
    }

    #[inline(always)]
    fn from_rotation(rotation: Rot3) -> Pose3 {
        Pose3 { rotation, translation: Vec3Trait::ZERO }
    }

    #[inline(always)]
    fn from_parts(translation: Vec3, rotation: Rot3) -> Pose3 {
        Pose3 { rotation, translation }
    }

    fn new(translation: Vec3, axisangle: Vec3) -> Pose3 {
        Pose3 { rotation: QuatTrait::from_scaled_axis(axisangle), translation }
    }

    fn rotation(axisangle: Vec3) -> Pose3 {
        Pose3 { rotation: QuatTrait::from_scaled_axis(axisangle), translation: Vec3Trait::ZERO }
    }

    #[inline(always)]
    fn prepend_translation(self: Pose3, translation: Vec3) -> Pose3 {
        Pose3 {
            rotation: self.rotation,
            translation: rotate_add(self.rotation, translation, self.translation),
        }
    }

    #[inline(always)]
    fn append_translation(self: Pose3, translation: Vec3) -> Pose3 {
        Pose3 { rotation: self.rotation, translation: self.translation + translation }
    }

    #[inline(always)]
    fn inverse(self: Pose3) -> Pose3 {
        Pose3 {
            rotation: self.rotation.conjugate(),
            translation: rotate_conj_neg(self.rotation, self.translation),
        }
    }

    #[inline(always)]
    fn inv_mul(self: Pose3, rhs: Pose3) -> Pose3 {
        Pose3 {
            rotation: conj_mul_quat(self.rotation, rhs.rotation),
            translation: rotate_conj(self.rotation, rhs.translation - self.translation),
        }
    }

    #[inline(always)]
    fn transform_point(self: Pose3, p: Vec3) -> Vec3 {
        rotate_add(self.rotation, p, self.translation)
    }

    #[inline(always)]
    fn transform_vector(self: Pose3, v: Vec3) -> Vec3 {
        self.rotation.mul_vec3(v)
    }

    #[inline(always)]
    fn inverse_transform_point(self: Pose3, p: Vec3) -> Vec3 {
        rotate_conj(self.rotation, p - self.translation)
    }

    #[inline(always)]
    fn inverse_transform_vector(self: Pose3, v: Vec3) -> Vec3 {
        rotate_conj(self.rotation, v)
    }

    fn lerp(self: Pose3, other: Pose3, t: Fixed) -> Pose3 {
        Pose3 {
            rotation: self.rotation.slerp(other.rotation, t),
            translation: self.translation.lerp(other.translation, t),
        }
    }

    fn nlerp(self: Pose3, other: Pose3, t: Fixed) -> Pose3 {
        Pose3 {
            rotation: self.rotation.lerp(other.rotation, t),
            translation: self.translation.lerp(other.translation, t),
        }
    }

    #[inline(always)]
    fn to_mat4(self: Pose3) -> Mat4 {
        Mat4Trait::from_rotation_translation(self.rotation, self.translation)
    }

    #[inline(always)]
    fn from_mat4(mat: Mat4) -> Pose3 {
        Pose3 { rotation: QuatTrait::from_mat4(mat), translation: mat.w_axis.truncate() }
    }

    #[inline(always)]
    fn mul_rot3(self: Pose3, rhs: Rot3) -> Pose3 {
        Pose3 { rotation: self.rotation.mul_quat(rhs), translation: self.translation }
    }

    #[inline(always)]
    fn mul_vec3(self: Pose3, rhs: Vec3) -> Vec3 {
        rotate_add(self.rotation, rhs, self.translation)
    }

    #[inline(always)]
    fn abs_diff_eq(self: Pose3, rhs: Pose3, max_abs_diff: Fixed) -> bool {
        self.rotation.abs_diff_eq(rhs.rotation, max_abs_diff)
            && self.translation.abs_diff_eq(rhs.translation, max_abs_diff)
    }
}

/// The heterogeneous product `Rot3 * Pose3` of glamx, as a method of the rotation.
pub trait Rot3Pose3Trait {
    /// Applies the rotation `self` after the pose `rhs`:
    /// `(self * rhs.translation, self * rhs.rotation)`.
    ///
    /// #### Preconditions
    /// * `self` must be normalized; it is not checked.
    ///
    /// Mirrors `impl Mul<glamx::Pose3> for glamx::Rot3`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
    /// #### Deviations
    /// * glamx spells this with an operator; the core operator traits of Cairo are homogeneous
    ///   (docs/DESIGN.md section 3).
    fn mul_pose3(self: Rot3, rhs: Pose3) -> Pose3;
}

pub impl Rot3Pose3Impl of Rot3Pose3Trait {
    #[inline(always)]
    fn mul_pose3(self: Rot3, rhs: Pose3) -> Pose3 {
        Pose3 { rotation: self.mul_quat(rhs.rotation), translation: self.mul_vec3(rhs.translation) }
    }
}

/// Composes two poses: `lhs` applied after `rhs`,
/// `(lhs.rotation * rhs.rotation, lhs.translation + lhs.rotation * rhs.translation)`.
///
/// #### Preconditions
/// * The rotation of `lhs` must be normalized; it is not checked. The rotation of the result is
///   not renormalized (see [`Pose3`]).
///
/// Implementation notes:
/// * The translation is the fused kernel of [`Pose3Trait::transform_point`] (one floor rescale
///   per component, bit-identical to the composed form); the rotation is `QuatTrait::mul_quat`.
///   19 380 gas against 21 000 for the composed glamx body.
///
/// Mirrors `impl Mul for glamx::Pose3`.
/// #### Panics
/// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
/// #### Deviations
/// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
///   value: docs/DESIGN.md section 3, "overflow".
pub impl Pose3Mul of Mul<Pose3> {
    #[inline(always)]
    fn mul(lhs: Pose3, rhs: Pose3) -> Pose3 {
        Pose3 {
            rotation: lhs.rotation.mul_quat(rhs.rotation),
            translation: rotate_add(lhs.rotation, rhs.translation, lhs.translation),
        }
    }
}

/// Mirrors `impl MulAssign for glamx::Pose3`.
pub impl Pose3MulAssign of MulAssign<Pose3, Pose3> {
    #[inline(always)]
    fn mul_assign(ref self: Pose3, rhs: Pose3) {
        self = self * rhs;
    }
}

/// Mirrors `impl MulAssign<glamx::Rot3> for glamx::Pose3` (see [`Pose3Trait::mul_rot3`]).
pub impl Pose3MulAssignRot3 of MulAssign<Pose3, Rot3> {
    #[inline(always)]
    fn mul_assign(ref self: Pose3, rhs: Rot3) {
        self = self.mul_rot3(rhs);
    }
}

/// The identity pose.
///
/// Mirrors `impl Default for glamx::Pose3`.
pub impl Pose3Default of Default<Pose3> {
    #[inline(always)]
    fn default() -> Pose3 {
        Pose3Impl::IDENTITY
    }
}

/// A pure rotation ([`Pose3Trait::from_rotation`]).
///
/// Mirrors `impl From<glamx::Rot3> for glamx::Pose3`.
pub impl Rot3IntoPose3 of Into<Rot3, Pose3> {
    #[inline(always)]
    fn into(self: Rot3) -> Pose3 {
        Pose3Impl::from_rotation(self)
    }
}

/// `(translation, rotation)` ([`Pose3Trait::from_parts`]).
///
/// Mirrors `impl From<(glam_core::Vec3, glamx::Rot3)> for glamx::Pose3`.
pub impl Vec3Rot3IntoPose3 of Into<(Vec3, Rot3), Pose3> {
    #[inline(always)]
    fn into(self: (Vec3, Rot3)) -> Pose3 {
        let (translation, rotation) = self;
        Pose3 { rotation, translation }
    }
}

/// `(translation, rotation)`.
///
/// Mirrors `impl From<glamx::Pose3> for (glam_core::Vec3, glamx::Rot3)`.
pub impl Pose3IntoVec3Rot3 of Into<Pose3, (Vec3, Rot3)> {
    #[inline(always)]
    fn into(self: Pose3) -> (Vec3, Rot3) {
        (self.translation, self.rotation)
    }
}

/// Raw value of `1`.
const ONE: i64 = 0x100000000;

/// `q * v + t`: the kernel of `QuatTrait::mul_vec3`
/// (`v (w^2 - b.b) + b (2 v.b) + (b x v) 2w` with `b = q.xyz()`, exact Q96.96 triple products)
/// with the translation lifted into the same exact sum, one floor rescale per component.
#[inline(always)]
fn rotate_add(q: Quat, v: Vec3, t: Vec3) -> Vec3 {
    let s1 = wide_mul(q.w, q.w)
        .sub(wide_mul(q.x, q.x))
        .sub(wide_mul(q.y, q.y))
        .sub(wide_mul(q.z, q.z));
    let d = wide_mul(v.x, q.x).add(wide_mul(v.y, q.y)).add(wide_mul(v.z, q.z));
    let s2 = d.add(d);
    let cx = wide_mul(q.y, v.z).sub(wide_mul(v.y, q.z));
    let cy = wide_mul(q.z, v.x).sub(wide_mul(v.z, q.x));
    let cz = wide_mul(q.x, v.y).sub(wide_mul(v.x, q.y));
    Vec3 {
        x: s1
            .mul(v.x)
            .add(s2.mul(q.x))
            .add(cx.add(cx).mul(q.w))
            .add(wide_from(t.x).lift())
            .narrow(),
        y: s1
            .mul(v.y)
            .add(s2.mul(q.y))
            .add(cy.add(cy).mul(q.w))
            .add(wide_from(t.y).lift())
            .narrow(),
        z: s1
            .mul(v.z)
            .add(s2.mul(q.z))
            .add(cz.add(cz).mul(q.w))
            .add(wide_from(t.z).lift())
            .narrow(),
    }
}

/// `conjugate(q) * v`: negating `b = q.xyz()` leaves `v (w^2 - b.b)` and `b (2 v.b)` unchanged
/// and negates `(b x v) 2w`, so the kernel of `QuatTrait::mul_vec3` subtracts that term.
#[inline(always)]
fn rotate_conj(q: Quat, v: Vec3) -> Vec3 {
    let s1 = wide_mul(q.w, q.w)
        .sub(wide_mul(q.x, q.x))
        .sub(wide_mul(q.y, q.y))
        .sub(wide_mul(q.z, q.z));
    let d = wide_mul(v.x, q.x).add(wide_mul(v.y, q.y)).add(wide_mul(v.z, q.z));
    let s2 = d.add(d);
    let cx = wide_mul(q.y, v.z).sub(wide_mul(v.y, q.z));
    let cy = wide_mul(q.z, v.x).sub(wide_mul(v.z, q.x));
    let cz = wide_mul(q.x, v.y).sub(wide_mul(v.x, q.y));
    Vec3 {
        x: s1.mul(v.x).add(s2.mul(q.x)).sub(cx.add(cx).mul(q.w)).narrow(),
        y: s1.mul(v.y).add(s2.mul(q.y)).sub(cy.add(cy).mul(q.w)).narrow(),
        z: s1.mul(v.z).add(s2.mul(q.z)).sub(cz.add(cz).mul(q.w)).narrow(),
    }
}

/// `-(conjugate(q) * v)`, i.e. `conjugate(q) * -v`: the sum of [`rotate_conj`] negated term by
/// term, so that the single floor rescale applies to the negated value.
#[inline(always)]
fn rotate_conj_neg(q: Quat, v: Vec3) -> Vec3 {
    let s1 = wide_mul(q.w, q.w)
        .sub(wide_mul(q.x, q.x))
        .sub(wide_mul(q.y, q.y))
        .sub(wide_mul(q.z, q.z));
    let d = wide_mul(v.x, q.x).add(wide_mul(v.y, q.y)).add(wide_mul(v.z, q.z));
    let s2 = d.add(d);
    let cx = wide_mul(q.y, v.z).sub(wide_mul(v.y, q.z));
    let cy = wide_mul(q.z, v.x).sub(wide_mul(v.z, q.x));
    let cz = wide_mul(q.x, v.y).sub(wide_mul(v.x, q.y));
    Vec3 {
        x: cx.add(cx).mul(q.w).sub(s1.mul(v.x)).sub(s2.mul(q.x)).narrow(),
        y: cy.add(cy).mul(q.w).sub(s1.mul(v.y)).sub(s2.mul(q.y)).narrow(),
        z: cz.add(cz).mul(q.w).sub(s1.mul(v.z)).sub(s2.mul(q.z)).narrow(),
    }
}

/// `conjugate(a) * b`: the Hamilton product of `QuatTrait::mul_quat` with the signs of the
/// terms in `a.x`, `a.y`, `a.z` flipped (free: `add` and `sub` cost the same).
#[inline(always)]
fn conj_mul_quat(a: Quat, b: Quat) -> Quat {
    Quat {
        x: wide_mul(a.w, b.x)
            .sub(wide_mul(a.x, b.w))
            .sub(wide_mul(a.y, b.z))
            .add(wide_mul(a.z, b.y))
            .narrow(),
        y: wide_mul(a.w, b.y)
            .add(wide_mul(a.x, b.z))
            .sub(wide_mul(a.y, b.w))
            .sub(wide_mul(a.z, b.x))
            .narrow(),
        z: wide_mul(a.w, b.z)
            .sub(wide_mul(a.x, b.y))
            .add(wide_mul(a.y, b.x))
            .sub(wide_mul(a.z, b.w))
            .narrow(),
        w: wide_mul(a.w, b.w)
            .add(wide_mul(a.x, b.x))
            .add(wide_mul(a.y, b.y))
            .add(wide_mul(a.z, b.z))
            .narrow(),
    }
}
