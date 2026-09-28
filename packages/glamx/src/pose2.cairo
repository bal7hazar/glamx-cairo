//! Port of glamx `pose2.rs` @ 0.3.1 on the Q32.32 scalar: a rigid-body transformation
//! (rotation + translation), the `Pose` of parry and rapier in 2D.
//!
//! Point transforms and pose composition fold the translation into the same exact Q64.64 sum
//! as the two rotation products, with one floor rescale per output component. Inverse paths carry
//! the conjugate signs directly in those sums instead of constructing an intermediate rotation.
//! The literal composed glamx forms live in `benches::alt::pose2` for reproducible gas comparison.

use core::ops::MulAssign;
use fixed::fixed::{Fixed, FixedTrait};
use fixed::wide::{WideAdd, WideNarrow, WideNeg, WideSub, dot2, mul_sub, wide_from, wide_mul};
use glam_core::mat2::Mat2;
use glam_core::mat3::{Mat3, Mat3Trait};
use glam_core::vec2::{Vec2, Vec2Trait};
use glam_core::vec3::Vec3;
use crate::rot2::{Rot2, Rot2Trait};

/// A 2D pose (rotation + translation), representing a rigid-body transformation: a point `p` is
/// mapped to `rotation * p + translation`.
///
/// The rotation is intended to be unit length and is not renormalized by composition, matching
/// glamx. Its `Rot2` product's floor rounding drifts by up to 198 raw ULP in squared norm after
/// 100 products and 1,998 ULP after 1,000 across the committed representative chains. Physics
/// integration may conservatively normalize every 100 compositions for a 200-ULP norm budget;
/// rapier normalizes after each integration step.
///
/// Mirrors `glamx::Pose2`.
/// #### Deviations
/// * `Debug` is the derived Cairo formatting. `Default` is the identity, as upstream.
/// * One type replaces the f32 `Pose2` and f64 `DPose2` flavours (there is one scalar).
/// * Not ported: `is_finite` / `is_nan` (those values do not exist), reference operator
///   overloads, f32/f64 conversions, serde-feature/bytemuck/rkyv/nalgebra glue and
///   `approx::RelativeEq` (`abs_diff_eq` is ported).
/// * Heterogeneous operators use named methods because Cairo's core `Mul<T>` is homogeneous.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Hash)]
pub struct Pose2 {
    /// The rotational part of the pose.
    pub rotation: Rot2,
    /// The translational part of the pose.
    pub translation: Vec2,
}

pub trait Pose2Trait {
    /// The identity pose (no rotation, no translation).
    ///
    /// Mirrors `glamx::Pose2::IDENTITY`.
    const IDENTITY: Pose2;

    /// Creates the identity pose.
    ///
    /// Mirrors `glamx::Pose2::identity`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn identity() -> Pose2;

    /// Creates a pose from a translation vector (identity rotation).
    ///
    /// Mirrors `glamx::Pose2::from_translation`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_translation(translation: Vec2) -> Pose2;

    /// Creates a pose from translation components (identity rotation).
    ///
    /// Called as `Pose2Trait::translation(x, y)`; Cairo accepts the same name as the public
    /// `pose.translation` field.
    ///
    /// Mirrors `glamx::Pose2::translation`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn translation(x: Fixed, y: Fixed) -> Pose2;

    /// Creates a pose from a rotation (zero translation).
    ///
    /// Mirrors `glamx::Pose2::from_rotation`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_rotation(rotation: Rot2) -> Pose2;

    /// Creates a pose that is a pure rotation by `angle` radians.
    ///
    /// Called as `Pose2Trait::rotation(angle)`; Cairo accepts the same name as the public
    /// `pose.rotation` field.
    ///
    /// Implementation notes:
    /// * As [`Rot2Trait::new`]: the deterministic `sin_cos` is accurate to 1.02 raw ULP.
    ///
    /// Mirrors `glamx::Pose2::rotation`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn rotation(angle: Fixed) -> Pose2;

    /// Creates a pose from its translation and rotation parts.
    ///
    /// Mirrors `glamx::Pose2::from_parts`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_parts(translation: Vec2, rotation: Rot2) -> Pose2;

    /// Creates a pose from a translation and an angle in radians.
    ///
    /// Implementation notes:
    /// * As [`Rot2Trait::new`].
    ///
    /// Mirrors `glamx::Pose2::new`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn new(translation: Vec2, angle: Fixed) -> Pose2;

    /// Prepends a translation in the local frame: `self.translation + rotation * translation`.
    ///
    /// Implementation notes:
    /// * The rotation products and translation are summed exactly and floored once: 5,280 gas
    ///   (`gas/pose2.snap`).
    ///
    /// Mirrors `glamx::Pose2::prepend_translation`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of the result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn prepend_translation(self: Pose2, translation: Vec2) -> Pose2;

    /// Appends a translation in the world frame.
    ///
    /// Mirrors `glamx::Pose2::append_translation`.
    /// #### Panics
    /// * `'i64_add Overflow'` / `'i64_add Underflow'` if a component leaves the scalar range.
    /// #### Deviations
    /// * Overflow panics with the native message where floating-point arithmetic returns infinity
    ///   or a larger finite value: docs/DESIGN.md section 3, "overflow".
    fn append_translation(self: Pose2, translation: Vec2) -> Pose2;

    /// Returns the inverse pose: `(conjugate(rotation), conjugate(rotation) * -translation)`.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * The translation is negated inside the fused rotation sum, avoiding an intermediate
    ///   `Vec2` negation and one constructed conjugate: 5,180 gas against 5,780 for the literal
    ///   composed form (`gas/pose2.snap`).
    ///
    /// Mirrors `glamx::Pose2::inverse`.
    /// #### Panics
    /// * `'i64_neg Underflow'` if `rotation.im` is `Fixed::MIN`.
    /// * `'Fixed: overflow'` if a translation component does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inverse(self: Pose2) -> Pose2;

    /// Computes `self.inverse() * rhs`, the relative pose of `rhs` in `self`'s frame.
    ///
    /// #### Preconditions
    /// * `self.rotation` must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * Fused as `(conj(ra) * rb, conj(ra) * (tb - ta))`; 9,920 gas against 10,220 for the
    ///   one-rotation composed form and 14,780 for upstream's `self.inverse() * rhs`, which
    ///   rotates a translation twice (`gas/pose2.snap`).
    ///
    /// Mirrors `glamx::Pose2::inv_mul`.
    /// #### Panics
    /// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if the translation difference leaves the
    ///   scalar range.
    /// * `'Fixed: overflow'` if a result component does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inv_mul(self: Pose2, rhs: Pose2) -> Pose2;

    /// Transforms a point by this pose: `rotation * p + translation`.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized for a rigid motion; it is not checked.
    ///
    /// Implementation notes:
    /// * The two rotation products and translation are summed exactly and floored once: 5,080
    ///   gas against 6,360 for the composed form (`gas/pose2.snap`).
    ///
    /// Mirrors `glamx::Pose2::transform_point`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result component does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn transform_point(self: Pose2, p: Vec2) -> Vec2;

    /// Transforms a vector by the rotation only.
    ///
    /// Implementation notes:
    /// * As [`Rot2Trait::transform_vector`].
    ///
    /// Mirrors `glamx::Pose2::transform_vector`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result component does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn transform_vector(self: Pose2, v: Vec2) -> Vec2;

    /// Transforms a point by the inverse pose.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * The conjugate signs are carried directly in the fused sums: 6,160 gas against 6,460 for
    ///   the composed form (`gas/pose2.snap`).
    ///
    /// Mirrors `glamx::Pose2::inverse_transform_point`.
    /// #### Panics
    /// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if `p - translation` leaves the scalar
    ///   range.
    /// * `'Fixed: overflow'` if a result component does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inverse_transform_point(self: Pose2, p: Vec2) -> Vec2;

    /// Transforms a vector by the inverse rotation.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized; it is not checked.
    ///
    /// Implementation notes:
    /// * The conjugate is not constructed; its signs are carried in the fused sums: 4,680 gas
    ///   against 4,980 for the composed form (`gas/pose2.snap`).
    ///
    /// Mirrors `glamx::Pose2::inverse_transform_vector`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result component does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inverse_transform_vector(self: Pose2, v: Vec2) -> Vec2;

    /// Interpolates rotations spherically and translations linearly.
    ///
    /// #### Preconditions
    /// * Both rotations must be normalized; it is not checked.
    ///
    /// Mirrors `glamx::Pose2::lerp`.
    /// #### Panics
    /// * As [`Rot2Trait::slerp`] and [`Vec2Trait::lerp`].
    /// #### Deviations
    /// * Carries the deterministic fixed-point error of `Rot2Trait::slerp`.
    fn lerp(self: Pose2, other: Pose2, t: Fixed) -> Pose2;

    /// Converts this pose to a homogeneous 3x3 matrix.
    ///
    /// #### Preconditions
    /// * The rotation must be normalized; it is not checked.
    ///
    /// Mirrors `glamx::Pose2::to_mat3`.
    /// #### Panics
    /// * `'i64_neg Underflow'` if `rotation.im` is `Fixed::MIN`.
    /// #### Deviations
    /// * Overflow panics with the native message where floating-point negation returns the finite
    ///   value `2^31`: docs/DESIGN.md section 3, "overflow".
    fn to_mat3(self: Pose2) -> Mat3;

    /// Creates a pose from a homogeneous rigid 3x3 matrix.
    ///
    /// #### Preconditions
    /// * The matrix must contain only a rotation and translation (no scale or shear). It is not
    ///   checked; the first column is normalized as in upstream.
    ///
    /// Mirrors `glamx::Pose2::from_mat3`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * `Rot2Trait::from_mat` uses the Q32.32 one-ULP near-zero threshold.
    fn from_mat3(mat: Mat3) -> Pose2;

    /// Composes `self` with a rotation applied first.
    ///
    /// Mirrors `impl Mul<glamx::Rot2> for glamx::Pose2`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a rotation component does not fit the scalar range.
    /// #### Deviations
    /// * A named method because Cairo's core `Mul<T>` is homogeneous.
    fn mul_rot2(self: Pose2, rhs: Rot2) -> Pose2;

    /// Transforms the point `rhs`.
    ///
    /// Mirrors `impl Mul<glam_core::Vec2> for glamx::Pose2`.
    /// #### Panics
    /// * As [`Pose2Trait::transform_point`].
    /// #### Deviations
    /// * A named method because Cairo's core `Mul<T>` is homogeneous.
    fn mul_vec2(self: Pose2, rhs: Vec2) -> Vec2;

    /// Returns whether every rotation and translation component differs by at most the tolerance.
    ///
    /// Mirrors `approx::AbsDiffEq for glamx::Pose2`.
    /// #### Panics
    /// * Never: built on `Fixed::abs_diff_eq`, an `i128` difference that never panics (R1
    ///   panic-coverage audit, escalation 1).
    /// #### Deviations
    /// * An inherent method with an explicit tolerance; there is no `approx` crate or default
    ///   epsilon.
    fn abs_diff_eq(self: Pose2, rhs: Pose2, max_abs_diff: Fixed) -> bool;
}

pub impl Pose2Impl of Pose2Trait {
    const IDENTITY: Pose2 = Pose2 {
        rotation: Rot2 { re: Fixed { raw: ONE }, im: Fixed { raw: 0 } },
        translation: Vec2 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 } },
    };

    #[inline(always)]
    fn identity() -> Pose2 {
        Self::IDENTITY
    }

    #[inline(always)]
    fn from_translation(translation: Vec2) -> Pose2 {
        Pose2 { rotation: Rot2Trait::IDENTITY, translation }
    }

    #[inline(always)]
    fn translation(x: Fixed, y: Fixed) -> Pose2 {
        Pose2 { rotation: Rot2Trait::IDENTITY, translation: Vec2 { x, y } }
    }

    #[inline(always)]
    fn from_rotation(rotation: Rot2) -> Pose2 {
        Pose2 { rotation, translation: Vec2Trait::ZERO }
    }

    #[inline(always)]
    fn rotation(angle: Fixed) -> Pose2 {
        Pose2 { rotation: Rot2Trait::new(angle), translation: Vec2Trait::ZERO }
    }

    #[inline(always)]
    fn from_parts(translation: Vec2, rotation: Rot2) -> Pose2 {
        Pose2 { rotation, translation }
    }

    #[inline(always)]
    fn new(translation: Vec2, angle: Fixed) -> Pose2 {
        Pose2 { rotation: Rot2Trait::new(angle), translation }
    }

    #[inline(always)]
    fn prepend_translation(self: Pose2, translation: Vec2) -> Pose2 {
        Pose2 {
            rotation: self.rotation,
            translation: rotate_add(self.rotation, translation, self.translation),
        }
    }

    #[inline(always)]
    fn append_translation(self: Pose2, translation: Vec2) -> Pose2 {
        Pose2 { rotation: self.rotation, translation: self.translation + translation }
    }

    #[inline(always)]
    fn inverse(self: Pose2) -> Pose2 {
        Pose2 {
            rotation: self.rotation.inverse(),
            translation: rotate_conj_neg(self.rotation, self.translation),
        }
    }

    #[inline(always)]
    fn inv_mul(self: Pose2, rhs: Pose2) -> Pose2 {
        Pose2 {
            rotation: conj_mul_rot2(self.rotation, rhs.rotation),
            translation: rotate_conj(self.rotation, rhs.translation - self.translation),
        }
    }

    #[inline(always)]
    fn transform_point(self: Pose2, p: Vec2) -> Vec2 {
        rotate_add(self.rotation, p, self.translation)
    }

    #[inline(always)]
    fn transform_vector(self: Pose2, v: Vec2) -> Vec2 {
        self.rotation.transform_vector(v)
    }

    #[inline(always)]
    fn inverse_transform_point(self: Pose2, p: Vec2) -> Vec2 {
        rotate_conj(self.rotation, p - self.translation)
    }

    #[inline(always)]
    fn inverse_transform_vector(self: Pose2, v: Vec2) -> Vec2 {
        rotate_conj(self.rotation, v)
    }

    fn lerp(self: Pose2, other: Pose2, t: Fixed) -> Pose2 {
        Pose2 {
            rotation: self.rotation.slerp(other.rotation, t),
            translation: self.translation.lerp(other.translation, t),
        }
    }

    #[inline(always)]
    fn to_mat3(self: Pose2) -> Mat3 {
        Mat3Trait::from_cols(
            Vec3 { x: self.rotation.re, y: self.rotation.im, z: F_ZERO },
            Vec3 { x: -self.rotation.im, y: self.rotation.re, z: F_ZERO },
            Vec3 { x: self.translation.x, y: self.translation.y, z: F_ONE },
        )
    }

    #[inline(always)]
    fn from_mat3(mat: Mat3) -> Pose2 {
        Pose2 {
            rotation: Rot2Trait::from_mat(
                Mat2 {
                    x_axis: Vec2 { x: mat.x_axis.x, y: mat.x_axis.y },
                    y_axis: Vec2 { x: mat.y_axis.x, y: mat.y_axis.y },
                },
            ),
            translation: Vec2 { x: mat.z_axis.x, y: mat.z_axis.y },
        }
    }

    #[inline(always)]
    fn mul_rot2(self: Pose2, rhs: Rot2) -> Pose2 {
        Pose2 { rotation: self.rotation * rhs, translation: self.translation }
    }

    #[inline(always)]
    fn mul_vec2(self: Pose2, rhs: Vec2) -> Vec2 {
        rotate_add(self.rotation, rhs, self.translation)
    }

    #[inline(always)]
    fn abs_diff_eq(self: Pose2, rhs: Pose2, max_abs_diff: Fixed) -> bool {
        self.rotation.re.abs_diff_eq(rhs.rotation.re, max_abs_diff)
            && self.rotation.im.abs_diff_eq(rhs.rotation.im, max_abs_diff)
            && self.translation.abs_diff_eq(rhs.translation, max_abs_diff)
    }
}

/// The heterogeneous product `Rot2 * Pose2` of glamx, as a method of the rotation.
pub trait Rot2Pose2Trait {
    /// Applies `self` after `rhs`, rotating both its translation and rotation.
    ///
    /// #### Preconditions
    /// * Both rotations should be normalized; it is not checked.
    ///
    /// Mirrors `impl Mul<glamx::Pose2> for glamx::Rot2`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result component does not fit the scalar range.
    /// #### Deviations
    /// * A named method because Cairo's core `Mul<T>` is homogeneous.
    fn mul_pose2(self: Rot2, rhs: Pose2) -> Pose2;
}

pub impl Rot2Pose2Impl of Rot2Pose2Trait {
    #[inline(always)]
    fn mul_pose2(self: Rot2, rhs: Pose2) -> Pose2 {
        Pose2 { rotation: self * rhs.rotation, translation: self.mul_vec2(rhs.translation) }
    }
}

/// Composes two poses: `lhs` applied after `rhs`.
///
/// #### Preconditions
/// * Both rotations should be normalized; they are not checked or renormalized.
///
/// Implementation notes:
/// * The translation uses one fused rescale per component rather than a rotated `Vec2` followed
///   by a separate addition: 8,840 gas against 10,120 (`gas/pose2.snap`).
///
/// Mirrors `impl Mul for glamx::Pose2`.
/// #### Panics
/// * `'Fixed: overflow'` if a result component does not fit the scalar range.
/// #### Deviations
/// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
///   value: docs/DESIGN.md section 3, "overflow".
pub impl Pose2Mul of Mul<Pose2> {
    #[inline(always)]
    fn mul(lhs: Pose2, rhs: Pose2) -> Pose2 {
        Pose2 {
            rotation: lhs.rotation * rhs.rotation,
            translation: rotate_add(lhs.rotation, rhs.translation, lhs.translation),
        }
    }
}

/// Mirrors `impl MulAssign for glamx::Pose2`.
pub impl Pose2MulAssign of MulAssign<Pose2, Pose2> {
    #[inline(always)]
    fn mul_assign(ref self: Pose2, rhs: Pose2) {
        self = self * rhs;
    }
}

/// Mirrors `impl MulAssign<glamx::Rot2> for glamx::Pose2`.
pub impl Pose2MulAssignRot2 of MulAssign<Pose2, Rot2> {
    #[inline(always)]
    fn mul_assign(ref self: Pose2, rhs: Rot2) {
        self = self.mul_rot2(rhs);
    }
}

/// The identity pose.
///
/// Mirrors `impl Default for glamx::Pose2`.
pub impl Pose2Default of Default<Pose2> {
    #[inline(always)]
    fn default() -> Pose2 {
        Pose2Impl::IDENTITY
    }
}

/// A pure rotation.
///
/// Mirrors `impl From<glamx::Rot2> for glamx::Pose2`.
pub impl Rot2IntoPose2 of Into<Rot2, Pose2> {
    #[inline(always)]
    fn into(self: Rot2) -> Pose2 {
        Pose2Impl::from_rotation(self)
    }
}

/// `(translation, rotation)` converted with [`Pose2Trait::from_parts`].
///
/// Mirrors the parts conversion pattern of glamx; upstream's tuple stores an angle scalar.
/// #### Deviations
/// * The tuple stores `Rot2` rather than recomputing it from an angle, matching `Pose3` and the
///   public-field representation required by the port brief.
pub impl Vec2Rot2IntoPose2 of Into<(Vec2, Rot2), Pose2> {
    #[inline(always)]
    fn into(self: (Vec2, Rot2)) -> Pose2 {
        let (translation, rotation) = self;
        Pose2 { rotation, translation }
    }
}

/// Converts a pose into `(translation, rotation)`.
///
/// Mirrors the parts conversion pattern of glamx; see [`Vec2Rot2IntoPose2`].
pub impl Pose2IntoVec2Rot2 of Into<Pose2, (Vec2, Rot2)> {
    #[inline(always)]
    fn into(self: Pose2) -> (Vec2, Rot2) {
        (self.translation, self.rotation)
    }
}

const ONE: i64 = 0x100000000;
const F_ZERO: Fixed = Fixed { raw: 0 };
const F_ONE: Fixed = Fixed { raw: ONE };

/// `r * v + t`, with the translation lifted into the exact Q64.64 product sum.
#[inline(always)]
fn rotate_add(r: Rot2, v: Vec2, t: Vec2) -> Vec2 {
    Vec2 {
        x: wide_mul(r.re, v.x).sub(wide_mul(r.im, v.y)).add(wide_from(t.x)).narrow(),
        y: wide_mul(r.im, v.x).add(wide_mul(r.re, v.y)).add(wide_from(t.y)).narrow(),
    }
}

/// `conjugate(r) * v`, with the conjugate signs carried by the add/sub chain.
#[inline(always)]
fn rotate_conj(r: Rot2, v: Vec2) -> Vec2 {
    Vec2 { x: dot2(r.re, v.x, r.im, v.y), y: mul_sub(r.re, v.y, r.im, v.x) }
}

/// `conjugate(r) * -v`, negated inside the exact sums before their single floor rescale.
#[inline(always)]
fn rotate_conj_neg(r: Rot2, v: Vec2) -> Vec2 {
    Vec2 {
        x: wide_mul(r.re, v.x).add(wide_mul(r.im, v.y)).neg().narrow(),
        y: wide_mul(r.im, v.x).sub(wide_mul(r.re, v.y)).narrow(),
    }
}

/// `conjugate(a) * b`, without constructing the conjugate.
#[inline(always)]
fn conj_mul_rot2(a: Rot2, b: Rot2) -> Rot2 {
    Rot2 { re: dot2(a.re, b.re, a.im, b.im), im: mul_sub(a.re, b.im, a.im, b.re) }
}
