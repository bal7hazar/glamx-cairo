//! Port of Dimforge glamx `rot2.rs` @ 0.3.1 on the Q32.32 `fixed::Fixed` scalar.
//!
//! Every product is a fused `fixed::wide` kernel: complex products, vector transforms and dot
//! products accumulate exactly at Q64.64 and rescale once per output scalar. The normalization
//! path uses the shared integer square root and reciprocal of `normalize2`.
//!
//! There is no NaN or infinity. Overflow panics, and a rotation at or below the one-ULP
//! near-zero threshold normalizes to the identity.

use core::ops::MulAssign;
use fixed::fixed::{Fixed, FixedTrait};
use fixed::trig::TrigTrait;
use fixed::wide::{dot2, is_unit2, mul_sub, norm2, norm2_squared, normalize2};
use glam_core::mat2::Mat2;
use glam_core::vec2::Vec2;

/// A 2D rotation represented as the unit complex number `re + im * i`, where `re` is the cosine
/// and `im` the sine of its angle.
///
/// Composition does not renormalize, matching glamx. Floor rounding makes its squared norm drift:
/// across the five representative chains in `test_composition_norm_drift`, the largest measured
/// absolute drift is 198 raw ULP after 100 products (`4.61e-8`) and 1,998 ULP after 1,000
/// (`4.65e-7`). Physics integrations should call [`Rot2Trait::normalize`] or
/// [`Rot2Trait::normalize_mut`] periodically. Once per integration step is conservative; the
/// measured table supports every 100 compositions when a 200-ULP squared-norm budget is
/// acceptable.
///
/// Mirrors `glamx::Rot2`.
/// #### Deviations
/// * `Debug` is the derived Cairo formatting.
/// * No `identity()` function (use [`Rot2Trait::IDENTITY`]), `length_recip`, `is_finite` /
///   `is_nan`, `powf`, reference operator overloads, f32/f64 conversions,
///   `approx`, serde-feature, bytemuck, rkyv or nalgebra glue.
/// * `Rot2 * Vec2` is [`Rot2Trait::mul_vec2`] because Cairo's core `Mul<T>` is homogeneous.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Hash)]
pub struct Rot2 {
    pub re: Fixed,
    pub im: Fixed,
}

pub trait Rot2Trait {
    /// The identity rotation (no rotation).
    ///
    /// Mirrors `glamx::Rot2::IDENTITY`.
    const IDENTITY: Rot2;

    /// Creates a unit complex number from cosine and sine values without checking its length.
    ///
    /// #### Preconditions
    /// * `re * re + im * im` should equal one.
    ///
    /// Mirrors `glamx::Rot2::from_cos_sin_unchecked`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * The precondition is not checked, as upstream.
    fn from_cos_sin_unchecked(re: Fixed, im: Fixed) -> Rot2;

    /// Creates a rotation from an angle in radians with one shared `sin_cos` reduction.
    ///
    /// Mirrors `glamx::Rot2::new`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * `fixed::trig::sin_cos` is deterministic and accurate to 1.02 raw ULP.
    fn new(angle: Fixed) -> Rot2;

    /// Creates a rotation from an angle in radians.
    ///
    /// Implementation notes:
    /// * As [`Rot2Trait::new`].
    ///
    /// Mirrors `glamx::Rot2::from_angle`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_angle(angle: Fixed) -> Rot2;

    /// Returns the rotation angle in radians.
    ///
    /// Mirrors `glamx::Rot2::angle`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * `fixed::trig::atan2` is deterministic and accurate to 3.22 raw ULP.
    fn angle(self: Rot2) -> Fixed;

    /// Returns the cosine of the rotation angle (the real part).
    ///
    /// Mirrors `glamx::Rot2::cos`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn cos(self: Rot2) -> Fixed;

    /// Returns the sine of the rotation angle (the imaginary part).
    ///
    /// Mirrors `glamx::Rot2::sin`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn sin(self: Rot2) -> Fixed;

    /// Returns the inverse rotation (the complex conjugate).
    ///
    /// #### Preconditions
    /// * `self` should be normalized.
    ///
    /// Mirrors `glamx::Rot2::inverse`.
    /// #### Panics
    /// * `'i64_neg Underflow'` if `im` is `Fixed::MIN`.
    /// #### Deviations
    /// * Overflow panics with the native message where floating-point negation returns the finite
    ///   value `2^31`: docs/DESIGN.md section 3, "overflow".
    fn inverse(self: Rot2) -> Rot2;

    /// Rotates a 2D vector by this rotation.
    ///
    /// Mirrors `glamx::Rot2::transform_vector`.
    /// #### Panics
    /// * `'Fixed: overflow'` if an output component does not fit the scalar range.
    /// #### Deviations
    /// * Each component is one exact two-product Q64.64 sum, floored once.
    fn transform_vector(self: Rot2, v: Vec2) -> Vec2;

    /// Rotates a 2D vector by the inverse of this rotation.
    ///
    /// #### Preconditions
    /// * `self` should be normalized.
    ///
    /// Mirrors `glamx::Rot2::inverse_transform_vector`.
    /// #### Panics
    /// * `'Fixed: overflow'` if an output component does not fit the scalar range.
    /// #### Deviations
    /// * Each component is one exact two-product Q64.64 sum, floored once.
    fn inverse_transform_vector(self: Rot2, v: Vec2) -> Vec2;

    /// Rotates a 2D vector by this rotation.
    ///
    /// Mirrors `impl Mul<Vec2> for glamx::Rot2`.
    /// #### Panics
    /// * `'Fixed: overflow'` if an output component does not fit the scalar range.
    /// #### Deviations
    /// * Named method because Cairo's core `Mul<T>` is homogeneous.
    fn mul_vec2(self: Rot2, rhs: Vec2) -> Vec2;

    /// Returns the rotation matrix equivalent to this rotation.
    ///
    /// Mirrors `glamx::Rot2::to_mat`.
    /// #### Panics
    /// * `'i64_neg Underflow'` if `im` is `Fixed::MIN`.
    /// #### Deviations
    /// * Overflow panics with the native message where floating-point negation returns the finite
    ///   value `2^31`: docs/DESIGN.md section 3, "overflow".
    fn to_mat(self: Rot2) -> Mat2;

    /// Creates a normalized rotation from the first column of a 2x2 rotation matrix.
    ///
    /// Mirrors `glamx::Rot2::from_mat`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * A first column whose floored length is at most one raw ULP returns `IDENTITY`; upstream
    ///   compares its floating-point length with the scalar epsilon.
    fn from_mat(mat: Mat2) -> Rot2;

    /// Creates a rotation from the first column of a 2x2 matrix without normalization.
    ///
    /// Mirrors `glamx::Rot2::from_mat_unchecked` and `from_matrix_unchecked`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * The upstream alias `from_matrix_unchecked` is not ported; consumers use this name.
    fn from_mat_unchecked(mat: Mat2) -> Rot2;

    /// Returns `self` normalized to length one.
    ///
    /// Mirrors `glamx::Rot2::normalize`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * A floored length at most one raw ULP (`2^-32`) returns `IDENTITY`; this is the Q32.32
    ///   re-derivation of upstream's `len <= f64::EPSILON` near-zero threshold.
    /// * Otherwise uses `fixed::wide::normalize2`: one integer square root, one shared division
    ///   and one round-to-nearest multiplication per component.
    fn normalize(self: Rot2) -> Rot2;

    /// Normalizes `self` in place.
    ///
    /// Mirrors `glamx::Rot2::normalize_mut`.
    /// #### Panics
    /// * As [`Rot2Trait::normalize`].
    /// #### Deviations
    /// * Uses Cairo's `ref self` in place of Rust's `&mut self`.
    fn normalize_mut(ref self: Rot2);

    /// Computes the length (magnitude) of `self`.
    ///
    /// Mirrors `glamx::Rot2::length`.
    /// #### Panics
    /// * `'Fixed: overflow'` if the length does not fit the scalar range.
    /// #### Deviations
    /// * The integer square root returns the floor of the exact length.
    fn length(self: Rot2) -> Fixed;

    /// Computes the squared length of `self`.
    ///
    /// Mirrors `glamx::Rot2::length_squared`.
    /// #### Panics
    /// * `'Fixed: overflow'` if the result does not fit the scalar range.
    /// #### Deviations
    /// * The exact sum of the two products is rescaled once (floored).
    fn length_squared(self: Rot2) -> Fixed;

    /// Returns whether `self` is of length `1` or not.
    ///
    /// Uses a precision threshold of `1024` raw ULP (`2^-22`) on the squared length.
    ///
    /// Mirrors `glamx::Rot2::is_normalized`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * The threshold is re-derived for Q32.32: `|length_squared - 1| <= 1024` raw ULP
    ///   (inclusive), about `2.4e-7`, where glamx uses `< 2e-4`, as `glam_core::Vec2::is_normalized`.
    ///   The squared length of the output of `normalize` is within 5 ULP of 1, so the margin is a
    ///   hundredfold.
    /// * The exact wide sum of squares is compared without narrowing, so long rotations return
    ///   `false` instead of panicking.
    fn is_normalized(self: Rot2) -> bool;

    /// Computes the dot product of `self` and `rhs`.
    ///
    /// Mirrors `glamx::Rot2::dot`.
    /// #### Panics
    /// * `'Fixed: overflow'` if the result does not fit the scalar range.
    /// #### Deviations
    /// * The exact sum of the two products is rescaled once (floored).
    fn dot(self: Rot2, rhs: Rot2) -> Fixed;

    /// Performs a linear interpolation between `self` and `rhs` based on the value `s`.
    ///
    /// When `s` is `0`, the result is `self`; when `s` is `1`, the result is `rhs`. The result is
    /// the component-wise blend and is **not** normalized: between distinct unit rotations its
    /// length is below one, and the midpoint of opposite rotations is the zero complex number
    /// `(0, 0)`. For rotations, [`Rot2Trait::slerp`] is usually preferred; a normalized linear
    /// blend is spelled `a.lerp(b, s).normalize()`.
    ///
    /// Mirrors `glamx::Rot2::lerp`.
    /// #### Panics
    /// * `'Fixed: overflow'` if an interpolated component does not fit the scalar range.
    /// #### Deviations
    /// * None in semantics. Each component is `Fixed::lerp`: `re + (rhs.re - re) * s` (upstream's
    ///   formula) evaluated exactly and floored once, so `s = 0` and `s = 1` return `self` and
    ///   `rhs` exactly.
    fn lerp(self: Rot2, rhs: Rot2, s: Fixed) -> Rot2;

    /// Spherically interpolates between two rotations along the shortest signed arc.
    ///
    /// Mirrors `glamx::Rot2::slerp`.
    /// #### Panics
    /// * `'Fixed: overflow'` if the angle scaling or complex product leaves the scalar range.
    /// #### Deviations
    /// * `angle_between` uses fixed-point fused kernels and `atan2`; `new` uses one `sin_cos`.
    fn slerp(self: Rot2, other: Rot2, t: Fixed) -> Rot2;

    /// Returns the signed angle in radians from `self` to `rhs`.
    ///
    /// Mirrors `glamx::Rot2::angle_between`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component of `self.inverse() * rhs` does not fit the scalar
    ///   range.
    /// #### Deviations
    /// * Computes the two components directly with one fused kernel each.
    fn angle_between(self: Rot2, rhs: Rot2) -> Fixed;

    /// Rotates `self` towards `rhs` by at most `max_angle` radians without passing the target.
    ///
    /// A negative `max_angle` rotates towards the exact opposite of `rhs`.
    ///
    /// Mirrors `glamx::Rot2::rotate_towards`.
    /// #### Panics
    /// * `'Fixed: overflow'` if `max_angle` is `Fixed::MIN` (its absolute value is outside the
    ///   positive scalar range).
    /// * As [`Rot2Trait::slerp`].
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn rotate_towards(self: Rot2, rhs: Rot2, max_angle: Fixed) -> Rot2;

    /// Gets the minimal planar rotation for transforming normalized `from` to normalized `to`.
    ///
    /// #### Preconditions
    /// * Both vectors should be normalized.
    ///
    /// Mirrors `glamx::Rot2::from_rotation_arc`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a component does not fit the scalar range.
    /// #### Deviations
    /// * The cosine and sine are each one fused two-product kernel, floored once.
    fn from_rotation_arc(from: Vec2, to: Vec2) -> Rot2;
}

pub impl Rot2Impl of Rot2Trait {
    const IDENTITY: Rot2 = Rot2 { re: F_ONE, im: F_ZERO };

    #[inline(always)]
    fn from_cos_sin_unchecked(re: Fixed, im: Fixed) -> Rot2 {
        Rot2 { re, im }
    }

    #[inline(always)]
    fn new(angle: Fixed) -> Rot2 {
        let (im, re) = angle.sin_cos();
        Rot2 { re, im }
    }

    #[inline(always)]
    fn from_angle(angle: Fixed) -> Rot2 {
        Self::new(angle)
    }

    #[inline(always)]
    fn angle(self: Rot2) -> Fixed {
        self.im.atan2(self.re)
    }

    #[inline(always)]
    fn cos(self: Rot2) -> Fixed {
        self.re
    }

    #[inline(always)]
    fn sin(self: Rot2) -> Fixed {
        self.im
    }

    #[inline(always)]
    fn inverse(self: Rot2) -> Rot2 {
        Rot2 { re: self.re, im: -self.im }
    }

    #[inline(always)]
    fn transform_vector(self: Rot2, v: Vec2) -> Vec2 {
        Vec2 { x: mul_sub(self.re, v.x, self.im, v.y), y: dot2(self.im, v.x, self.re, v.y) }
    }

    #[inline(always)]
    fn inverse_transform_vector(self: Rot2, v: Vec2) -> Vec2 {
        Vec2 { x: dot2(self.re, v.x, self.im, v.y), y: mul_sub(self.re, v.y, self.im, v.x) }
    }

    #[inline(always)]
    fn mul_vec2(self: Rot2, rhs: Vec2) -> Vec2 {
        Self::transform_vector(self, rhs)
    }

    #[inline(always)]
    fn to_mat(self: Rot2) -> Mat2 {
        Mat2 { x_axis: Vec2 { x: self.re, y: self.im }, y_axis: Vec2 { x: -self.im, y: self.re } }
    }

    #[inline(always)]
    fn from_mat(mat: Mat2) -> Rot2 {
        Self::normalize(Self::from_mat_unchecked(mat))
    }

    #[inline(always)]
    fn from_mat_unchecked(mat: Mat2) -> Rot2 {
        Rot2 { re: mat.x_axis.x, im: mat.x_axis.y }
    }

    #[inline(always)]
    fn normalize(self: Rot2) -> Rot2 {
        if near_zero(self) {
            Self::IDENTITY
        } else {
            let (re, im) = normalize2(self.re, self.im);
            Rot2 { re, im }
        }
    }

    #[inline(always)]
    fn normalize_mut(ref self: Rot2) {
        self = Self::normalize(self);
    }

    #[inline(always)]
    fn length(self: Rot2) -> Fixed {
        norm2(self.re, self.im)
    }

    #[inline(always)]
    fn length_squared(self: Rot2) -> Fixed {
        norm2_squared(self.re, self.im)
    }

    #[inline(always)]
    fn is_normalized(self: Rot2) -> bool {
        is_unit2(self.re, self.im, 1024)
    }

    #[inline(always)]
    fn dot(self: Rot2, rhs: Rot2) -> Fixed {
        dot2(self.re, rhs.re, self.im, rhs.im)
    }

    #[inline(always)]
    fn lerp(self: Rot2, rhs: Rot2, s: Fixed) -> Rot2 {
        Rot2 { re: self.re.lerp(rhs.re, s), im: self.im.lerp(rhs.im, s) }
    }

    fn slerp(self: Rot2, other: Rot2, t: Fixed) -> Rot2 {
        self * Self::new(Self::angle_between(self, other) * t)
    }

    #[inline(always)]
    fn angle_between(self: Rot2, rhs: Rot2) -> Fixed {
        let re = dot2(self.re, rhs.re, self.im, rhs.im);
        let im = mul_sub(self.re, rhs.im, self.im, rhs.re);
        im.atan2(re)
    }

    fn rotate_towards(self: Rot2, rhs: Rot2, max_angle: Fixed) -> Rot2 {
        let angle = Self::angle_between(self, rhs);
        if angle.abs() <= max_angle.abs() {
            rhs
        } else {
            Self::slerp(self, rhs, max_angle / angle)
        }
    }

    #[inline(always)]
    fn from_rotation_arc(from: Vec2, to: Vec2) -> Rot2 {
        Rot2 { re: dot2(from.x, to.x, from.y, to.y), im: mul_sub(from.x, to.y, from.y, to.x) }
    }
}

/// Complex multiplication of two rotations.
///
/// Mirrors `impl Mul for glamx::Rot2`.
/// #### Panics
/// * `'Fixed: overflow'` if a component does not fit the scalar range.
/// #### Deviations
/// * Each component is one exact two-product Q64.64 sum, floored once. The result is not
///   renormalized, matching upstream; see [`Rot2`] for the measured drift policy.
pub impl Rot2Mul of Mul<Rot2> {
    #[inline(always)]
    fn mul(lhs: Rot2, rhs: Rot2) -> Rot2 {
        Rot2 {
            re: mul_sub(lhs.re, rhs.re, lhs.im, rhs.im), im: dot2(lhs.re, rhs.im, lhs.im, rhs.re),
        }
    }
}

/// Multiplies `self` by `rhs` in place.
///
/// Mirrors `impl MulAssign for glamx::Rot2`.
/// #### Panics
/// * As [`Rot2Mul`].
/// #### Deviations
/// * Overflow panics where floating-point arithmetic returns infinity or a larger finite value:
///   docs/DESIGN.md section 3, "overflow".
pub impl Rot2MulAssign of MulAssign<Rot2, Rot2> {
    #[inline(always)]
    fn mul_assign(ref self: Rot2, rhs: Rot2) {
        self = self * rhs;
    }
}

/// The identity rotation.
///
/// Mirrors `impl Default for glamx::Rot2`.
/// #### Panics
/// * Never.
/// #### Deviations
/// * Not derived: Cairo's derived default would be the zero complex number.
pub impl Rot2Default of Default<Rot2> {
    #[inline(always)]
    fn default() -> Rot2 {
        Rot2Impl::IDENTITY
    }
}

/// Whether the floored magnitude of `r` is at most one raw Q32.32 ULP.
#[inline(always)]
fn near_zero(r: Rot2) -> bool {
    r.re.raw >= -1 && r.re.raw <= 1 && r.im.raw >= -1 && r.im.raw <= 1
}

const F_ZERO: Fixed = Fixed { raw: 0 };
const F_ONE: Fixed = Fixed { raw: 0x100000000 };
