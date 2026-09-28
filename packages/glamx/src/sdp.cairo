//! Symmetric matrices of parry (`parry::utils::SdpMatrix2` / `SdpMatrix3`, `utils/sdp_matrix.rs`)
//! with the angular-inertia operations of rapier (`AngularInertiaOps`,
//! `utils/angular_inertia_ops.rs`) and the world-inertia kernel `R diag(d) R^T` of parry
//! (`MassProperties::world_inv_inertia`, `mass_properties/mass_properties.rs`).
//!
//! These types are not part of Dimforge's glamx: they are the per-body, per-step matrix work of a
//! rigid-body step (docs/research/06-glamx-scope.md sections 3.4 and 6.1 item 4). A symmetric
//! matrix stores its 6 (3D) or 3 (2D) unique entries, so every operation computes only those.
//!
//! Every product goes through the fused kernels of `fixed::wide` (one rescale per output scalar,
//! docs/DESIGN.md section 2.1). In particular [`SdpMatrix3Trait::quadform`] and
//! [`SdpMatrix3Trait::from_rotated_diagonal_mat3`] keep their inner products un-rescaled
//! (Q64.64) and rescale each of the 6 outputs once from an exact sum of triple products (Q96.96):
//! their result is the floor of the exact value. The literal parry / rapier formulations live in
//! `benches::alt::sdp` and the measurements in `gas/sdp.snap`.
//!
//! Positive-definiteness is a documented precondition of the names, never checked: every
//! function is defined on any symmetric matrix. There is no NaN and no infinity: overflow and the
//! unchecked inversion of a singular matrix panic.

use fixed::fixed::Fixed;
use fixed::wide::{
    RecipTrait, W2, WideAdd, WideMul, WideNarrow, WideSub, dot2, dot3, mul_sub, wide_mul,
};
use glam_core::mat2::Mat2;
use glam_core::mat3::{Mat3, Mat3Trait};
use glam_core::quat::Quat;
use glam_core::vec2::Vec2;
use glam_core::vec3::Vec3;

/// A 2x2 symmetric matrix, stored as its 3 unique entries (row, column indices from 1).
///
/// Intended to be symmetric positive-definite (an inertia or an effective-mass matrix); nothing
/// checks it.
///
/// Mirrors `parry::utils::SdpMatrix2<Real>`.
/// #### Deviations
/// * Not generic over the scalar: `Fixed` only (the SIMD lane types of parry do not exist here).
/// * `Default` is the zero matrix, as in parry.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default, Hash)]
pub struct SdpMatrix2 {
    /// The component at the first row and first column.
    pub m11: Fixed,
    /// The component at the first row and second column (and second row, first column).
    pub m12: Fixed,
    /// The component at the second row and second column.
    pub m22: Fixed,
}

/// A 3x3 symmetric matrix, stored as its 6 unique entries (row, column indices from 1).
///
/// Intended to be symmetric positive-definite (an angular inertia tensor or its inverse); nothing
/// checks it.
///
/// Mirrors `parry::utils::SdpMatrix3<Real>` (rapier's `AngularInertia` in 3D).
/// #### Deviations
/// * Not generic over the scalar: `Fixed` only (the SIMD lane types of parry do not exist here).
/// * `Default` is the zero matrix, as in parry.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default, Hash)]
pub struct SdpMatrix3 {
    /// The component at the first row and first column.
    pub m11: Fixed,
    /// The component at the first row and second column (and second row, first column).
    pub m12: Fixed,
    /// The component at the first row and third column (and third row, first column).
    pub m13: Fixed,
    /// The component at the second row and second column.
    pub m22: Fixed,
    /// The component at the second row and third column (and third row, second column).
    pub m23: Fixed,
    /// The component at the third row and third column.
    pub m33: Fixed,
}

pub trait SdpMatrix2Trait {
    /// Creates a symmetric matrix from its 3 unique components.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::new`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn new(m11: Fixed, m12: Fixed, m22: Fixed) -> SdpMatrix2;
    /// The matrix with all components set to `0`.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::zero`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn zero() -> SdpMatrix2;
    /// The identity matrix.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::diagonal(1.0)`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * Not a parry function: the identity of `num_traits::One`, spelled as a constructor.
    fn identity() -> SdpMatrix2;
    /// The matrix with `val` on its diagonal and zeros elsewhere.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::diagonal`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn diagonal(val: Fixed) -> SdpMatrix2;
    /// Returns `true` if every component of `self` is zero.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::is_zero`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * parry defines `is_zero` on `SdpMatrix3` only; provided here for symmetry.
    fn is_zero(self: SdpMatrix2) -> bool;
    /// Returns `self` with `elt` added to its diagonal components.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::add_diagonal`.
    /// #### Panics
    /// * `'i64_add Overflow'` / `'i64_add Underflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * A plain `Fixed` sum, not `'Fixed: overflow'` as previously documented (R1
    ///   panic-coverage audit, escalation 2).
    /// * Takes `self` by value and does not mutate it (parry takes `&mut self` and returns a
    ///   copy without writing it back).
    fn add_diagonal(self: SdpMatrix2, elt: Fixed) -> SdpMatrix2;
    /// Builds a symmetric matrix from a full matrix assumed symmetric: reads the diagonal and the
    /// upper triangle (`m12` = column 1, row 0), ignores the lower triangle.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::from_sdp_matrix`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_sdp_matrix(mat: Mat2) -> SdpMatrix2;
    /// Converts `self` into a full (symmetric) matrix.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::into_matrix`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn into_matrix(self: SdpMatrix2) -> Mat2;
    /// Multiplies each component of `self` by `rhs`.
    ///
    /// Mirrors `Mul<Real> for parry::utils::SdpMatrix2`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * A named method: core `Mul` is homogeneous (docs/DESIGN.md section 3).
    fn mul_scalar(self: SdpMatrix2, rhs: Fixed) -> SdpMatrix2;
    /// Multiplies `self` by the vector `rhs`.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::mul_vec` (and `Mul<Vector2>`).
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * One fused `dot2` per component: the floor of the exact result.
    fn mul_vec(self: SdpMatrix2, rhs: Vec2) -> Vec2;
    /// Returns the inverse of `self`, without any invertibility check in parry.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::inverse_unchecked`.
    /// #### Panics
    /// * `'SdpMatrix2: singular'` if the determinant of `self` is zero (parry divides by zero
    ///   and returns infinities / NaN).
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * One division shared by the 3 components (`Recip`, rounded to nearest) instead of one
    ///   per component: `x / det` is exact whenever it is representable.
    /// * The determinant is the floor of the exact `m11 m22 - m12^2` (one fused `mul_sub`), and
    ///   singular means that floored determinant is zero.
    fn inverse_unchecked(self: SdpMatrix2) -> SdpMatrix2;
    /// Returns the inverse of `self` and the determinant of `self`.
    ///
    /// Implementation notes:
    /// * As `inverse_unchecked`.
    ///
    /// Mirrors `parry::utils::SdpMatrix2::inverse_and_get_determinant_unchecked`.
    /// #### Panics
    /// * `'SdpMatrix2: singular'` if the determinant of `self` is zero.
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn inverse_and_get_determinant_unchecked(self: SdpMatrix2) -> (SdpMatrix2, Fixed);
    /// Returns the inverse of `self`, or the zero matrix if the determinant is zero.
    ///
    /// Mirrors rapier's `AngularInertiaOps::inverse` (3D) on the 2x2 matrix.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * rapier defines it on `SdpMatrix3` only (its 2D angular inertia is a scalar); provided
    ///   here for symmetry. Same rounding as `inverse_unchecked`.
    fn inverse(self: SdpMatrix2) -> SdpMatrix2;
}

pub impl SdpMatrix2Impl of SdpMatrix2Trait {
    #[inline(always)]
    fn new(m11: Fixed, m12: Fixed, m22: Fixed) -> SdpMatrix2 {
        SdpMatrix2 { m11, m12, m22 }
    }

    #[inline(always)]
    fn zero() -> SdpMatrix2 {
        SdpMatrix2 { m11: F_ZERO, m12: F_ZERO, m22: F_ZERO }
    }

    #[inline(always)]
    fn identity() -> SdpMatrix2 {
        SdpMatrix2 { m11: F_ONE, m12: F_ZERO, m22: F_ONE }
    }

    #[inline(always)]
    fn diagonal(val: Fixed) -> SdpMatrix2 {
        SdpMatrix2 { m11: val, m12: F_ZERO, m22: val }
    }

    #[inline(always)]
    fn is_zero(self: SdpMatrix2) -> bool {
        self.m11 == F_ZERO && self.m12 == F_ZERO && self.m22 == F_ZERO
    }

    #[inline(always)]
    fn add_diagonal(self: SdpMatrix2, elt: Fixed) -> SdpMatrix2 {
        SdpMatrix2 { m11: self.m11 + elt, m12: self.m12, m22: self.m22 + elt }
    }

    #[inline(always)]
    fn from_sdp_matrix(mat: Mat2) -> SdpMatrix2 {
        SdpMatrix2 { m11: mat.x_axis.x, m12: mat.y_axis.x, m22: mat.y_axis.y }
    }

    #[inline(always)]
    fn into_matrix(self: SdpMatrix2) -> Mat2 {
        Mat2 {
            x_axis: Vec2 { x: self.m11, y: self.m12 }, y_axis: Vec2 { x: self.m12, y: self.m22 },
        }
    }

    #[inline(always)]
    fn mul_scalar(self: SdpMatrix2, rhs: Fixed) -> SdpMatrix2 {
        SdpMatrix2 { m11: self.m11 * rhs, m12: self.m12 * rhs, m22: self.m22 * rhs }
    }

    #[inline(always)]
    fn mul_vec(self: SdpMatrix2, rhs: Vec2) -> Vec2 {
        Vec2 {
            x: dot2(self.m11, rhs.x, self.m12, rhs.y), y: dot2(self.m12, rhs.x, self.m22, rhs.y),
        }
    }

    fn inverse_unchecked(self: SdpMatrix2) -> SdpMatrix2 {
        let det = mul_sub(self.m11, self.m22, self.m12, self.m12);
        if det == F_ZERO {
            core::panic_with_felt252('SdpMatrix2: singular');
        }
        adjugate_div2(self, det)
    }

    fn inverse_and_get_determinant_unchecked(self: SdpMatrix2) -> (SdpMatrix2, Fixed) {
        let det = mul_sub(self.m11, self.m22, self.m12, self.m12);
        if det == F_ZERO {
            core::panic_with_felt252('SdpMatrix2: singular');
        }
        (adjugate_div2(self, det), det)
    }

    fn inverse(self: SdpMatrix2) -> SdpMatrix2 {
        let det = mul_sub(self.m11, self.m22, self.m12, self.m12);
        if det == F_ZERO {
            return Self::zero();
        }
        adjugate_div2(self, det)
    }
}

/// Adds two symmetric matrices component-wise.
///
/// Mirrors `Add for parry::utils::SdpMatrix2`.
/// #### Panics
/// * `'i64_add Overflow'` / `'i64_add Underflow'` if a result does not fit the scalar range.
/// #### Deviations
/// * A plain `Fixed` sum, not `'Fixed: overflow'` as previously documented (R1 panic-coverage
///   audit, escalation 2).
/// * Overflow panics where floating-point arithmetic returns infinity or a larger finite value:
///   docs/DESIGN.md section 3, "overflow".
pub impl SdpMatrix2Add of Add<SdpMatrix2> {
    #[inline(always)]
    fn add(lhs: SdpMatrix2, rhs: SdpMatrix2) -> SdpMatrix2 {
        SdpMatrix2 { m11: lhs.m11 + rhs.m11, m12: lhs.m12 + rhs.m12, m22: lhs.m22 + rhs.m22 }
    }
}

/// Subtracts two symmetric matrices component-wise.
///
/// Mirrors nothing in parry (which only implements `Add`).
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if a result does not fit the scalar range.
/// #### Deviations
/// * A plain `Fixed` difference, not `'Fixed: overflow'` as previously documented (R1
///   panic-coverage audit, escalation 2).
/// * Not in parry; the component-wise difference.
pub impl SdpMatrix2Sub of Sub<SdpMatrix2> {
    #[inline(always)]
    fn sub(lhs: SdpMatrix2, rhs: SdpMatrix2) -> SdpMatrix2 {
        SdpMatrix2 { m11: lhs.m11 - rhs.m11, m12: lhs.m12 - rhs.m12, m22: lhs.m22 - rhs.m22 }
    }
}

pub trait SdpMatrix3Trait {
    /// Creates a symmetric matrix from its 6 unique components.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::new`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn new(m11: Fixed, m12: Fixed, m13: Fixed, m22: Fixed, m23: Fixed, m33: Fixed) -> SdpMatrix3;
    /// The matrix with all components set to `0`.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::zero`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn zero() -> SdpMatrix3;
    /// The identity matrix.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::diagonal(1.0)`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * Not a parry function: the identity of `num_traits::One`, spelled as a constructor.
    fn identity() -> SdpMatrix3;
    /// The matrix with `val` on its diagonal and zeros elsewhere.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::diagonal`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn diagonal(val: Fixed) -> SdpMatrix3;
    /// Returns `true` if every component of `self` is zero.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::is_zero`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn is_zero(self: SdpMatrix3) -> bool;
    /// Returns `self` with `elt` added to its diagonal components.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::add_diagonal`.
    /// #### Panics
    /// * `'i64_add Overflow'` / `'i64_add Underflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * A plain `Fixed` sum, not `'Fixed: overflow'` as previously documented (R1
    ///   panic-coverage audit, escalation 2).
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn add_diagonal(self: SdpMatrix3, elt: Fixed) -> SdpMatrix3;
    /// Builds a symmetric matrix from a full matrix assumed symmetric: reads the diagonal and the
    /// upper triangle (`m12` = column 1, row 0; `m13` = column 2, row 0; `m23` = column 2, row
    /// 1), ignores the lower triangle.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::from_sdp_matrix`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn from_sdp_matrix(mat: Mat3) -> SdpMatrix3;
    /// Converts `self` into a full (symmetric) matrix.
    ///
    /// Mirrors rapier's `AngularInertiaOps::into_matrix` for `SdpMatrix3<Real>`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None.
    fn into_matrix(self: SdpMatrix3) -> Mat3;
    /// Multiplies each component of `self` by `rhs`.
    ///
    /// Mirrors `Mul<Real> for parry::utils::SdpMatrix3`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * A named method: core `Mul` is homogeneous (docs/DESIGN.md section 3).
    fn mul_scalar(self: SdpMatrix3, rhs: Fixed) -> SdpMatrix3;
    /// Multiplies `self` by the vector `rhs` (angular inertia times angular velocity, inverse
    /// inertia times torque).
    ///
    /// Mirrors `parry::utils::SdpMatrix3::mul_vec` (and `Mul<Vector3>`).
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * One fused `dot3` per component: the floor of the exact result.
    fn mul_vec(self: SdpMatrix3, rhs: Vec3) -> Vec3;
    /// Multiplies `self` by the vector `v`: the same function as `mul_vec`, under the name the
    /// rapier solver uses (`ii.transform_vector(torque_dir)`).
    ///
    /// Implementation notes:
    /// * As `mul_vec`.
    ///
    /// Mirrors rapier's `AngularInertiaOps::transform_vector` for `SdpMatrix3<Real>`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * Overflow panics where floating-point arithmetic returns infinity or a larger finite
    ///   value: docs/DESIGN.md section 3, "overflow".
    fn transform_vector(self: SdpMatrix3, v: Vec3) -> Vec3;
    /// Multiplies `self` by the matrix `rhs`.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::mul_mat` (and `Mul<Matrix3>`).
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * One fused `dot3` per element: the floor of the exact product.
    fn mul_mat(self: SdpMatrix3, rhs: Mat3) -> Mat3;
    /// Computes the quadratic form `m^T * self * m`.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::quadform`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * Fully fused: `self * m` is kept exact (Q64.64) and each of the 6 unique outputs is the
    ///   floor of the exact sum of 9 triple products (one rescale), where parry rounds `self * m`
    ///   before the second product and computes the 3 redundant entries. The result is exactly
    ///   symmetric by construction.
    fn quadform(self: SdpMatrix3, m: Mat3) -> SdpMatrix3;
    /// Computes the quadratic form `m^T * self * m` for the 3x2 matrix `m` given by its entries
    /// (`mij` = row `i`, column `j`, from 1).
    ///
    /// Mirrors `parry::utils::SdpMatrix3::quadform3x2`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * Fully fused as `quadform`: each of the 3 outputs is the floor of the exact value.
    fn quadform3x2(
        self: SdpMatrix3, m11: Fixed, m12: Fixed, m21: Fixed, m22: Fixed, m31: Fixed, m32: Fixed,
    ) -> SdpMatrix2;
    /// Returns the inverse of `self`, without any invertibility check in parry.
    ///
    /// Mirrors `parry::utils::SdpMatrix3::inverse_unchecked`.
    /// #### Panics
    /// * `'SdpMatrix3: singular'` if the determinant of `self` is zero (parry divides by zero
    ///   and returns infinities / NaN).
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * The 6 cofactors of the symmetric adjugate are fused `mul_sub` (one floor rescale each);
    ///   the determinant is the floor of the exact expansion along the first row (the same
    ///   value as `Mat3::determinant` of `into_matrix()`), and singular means that floored
    ///   determinant is zero.
    /// * One division shared by the 6 cofactors (`Recip`, rounded to nearest) instead of
    ///   `1 / det` rounded then 6 products: `x / det` is exact whenever it is representable.
    fn inverse_unchecked(self: SdpMatrix3) -> SdpMatrix3;
    /// Returns the inverse of `self`, or the zero matrix if the determinant is zero (a body
    /// whose rotations are all locked has a zero inverse inertia).
    ///
    /// Mirrors rapier's `AngularInertiaOps::inverse` for `SdpMatrix3<Real>`.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * Same rounding and same singularity test as `inverse_unchecked`.
    fn inverse(self: SdpMatrix3) -> SdpMatrix3;
    /// The world-inertia kernel: returns `R * diag(d) * R^T` where `R` is the rotation matrix of
    /// the unit quaternion `rotation`, i.e. a principal inertia (or inverse inertia) `d` expressed
    /// in the frame rotated by `rotation`.
    ///
    /// Mirrors the matrix part of `parry::mass_properties::MassProperties::world_inv_inertia`
    /// (`from_quat`, `transpose`, 3 column scalings, `lhs * rhs`, `from_sdp_matrix`).
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * `R` is `Mat3::from_quat(rotation)` (one floor rescale per element); then each of the 6
    ///   unique outputs is the floor of the exact `sum_k r_ik d_k r_jk` (one rescale), instead of
    ///   rounding `R diag(d)` and computing the 9 entries of the full product. The result is
    ///   exactly symmetric by construction.
    /// * `rotation` is assumed normalized (not checked, as in parry).
    fn from_rotated_diagonal(rotation: Quat, d: Vec3) -> SdpMatrix3;
    /// The world-inertia kernel on a rotation matrix: returns `rotation * diag(d) * rotation^T`,
    /// each of the 6 unique outputs the floor of the exact `sum_k r_ik d_k r_jk` (one rescale).
    ///
    /// Mirrors the matrix part of `parry::mass_properties::MassProperties::world_inv_inertia`,
    /// starting from the rotation matrix.
    /// #### Panics
    /// * `'Fixed: overflow'` if a result does not fit the scalar range.
    /// #### Deviations
    /// * Defined for any matrix `rotation` (`M diag(d) M^T`); a rotation is not checked. A
    ///   permutation matrix (a rotation by a multiple of 90 degrees about an axis) permutes `d`
    ///   exactly.
    fn from_rotated_diagonal_mat3(rotation: Mat3, d: Vec3) -> SdpMatrix3;
}

pub impl SdpMatrix3Impl of SdpMatrix3Trait {
    #[inline(always)]
    fn new(m11: Fixed, m12: Fixed, m13: Fixed, m22: Fixed, m23: Fixed, m33: Fixed) -> SdpMatrix3 {
        SdpMatrix3 { m11, m12, m13, m22, m23, m33 }
    }

    #[inline(always)]
    fn zero() -> SdpMatrix3 {
        SdpMatrix3 { m11: F_ZERO, m12: F_ZERO, m13: F_ZERO, m22: F_ZERO, m23: F_ZERO, m33: F_ZERO }
    }

    #[inline(always)]
    fn identity() -> SdpMatrix3 {
        SdpMatrix3 { m11: F_ONE, m12: F_ZERO, m13: F_ZERO, m22: F_ONE, m23: F_ZERO, m33: F_ONE }
    }

    #[inline(always)]
    fn diagonal(val: Fixed) -> SdpMatrix3 {
        SdpMatrix3 { m11: val, m12: F_ZERO, m13: F_ZERO, m22: val, m23: F_ZERO, m33: val }
    }

    #[inline(always)]
    fn is_zero(self: SdpMatrix3) -> bool {
        self.m11 == F_ZERO
            && self.m12 == F_ZERO
            && self.m13 == F_ZERO
            && self.m22 == F_ZERO
            && self.m23 == F_ZERO
            && self.m33 == F_ZERO
    }

    #[inline(always)]
    fn add_diagonal(self: SdpMatrix3, elt: Fixed) -> SdpMatrix3 {
        SdpMatrix3 {
            m11: self.m11 + elt,
            m12: self.m12,
            m13: self.m13,
            m22: self.m22 + elt,
            m23: self.m23,
            m33: self.m33 + elt,
        }
    }

    #[inline(always)]
    fn from_sdp_matrix(mat: Mat3) -> SdpMatrix3 {
        SdpMatrix3 {
            m11: mat.x_axis.x,
            m12: mat.y_axis.x,
            m13: mat.z_axis.x,
            m22: mat.y_axis.y,
            m23: mat.z_axis.y,
            m33: mat.z_axis.z,
        }
    }

    #[inline(always)]
    fn into_matrix(self: SdpMatrix3) -> Mat3 {
        Mat3 {
            x_axis: Vec3 { x: self.m11, y: self.m12, z: self.m13 },
            y_axis: Vec3 { x: self.m12, y: self.m22, z: self.m23 },
            z_axis: Vec3 { x: self.m13, y: self.m23, z: self.m33 },
        }
    }

    #[inline(always)]
    fn mul_scalar(self: SdpMatrix3, rhs: Fixed) -> SdpMatrix3 {
        SdpMatrix3 {
            m11: self.m11 * rhs,
            m12: self.m12 * rhs,
            m13: self.m13 * rhs,
            m22: self.m22 * rhs,
            m23: self.m23 * rhs,
            m33: self.m33 * rhs,
        }
    }

    #[inline(always)]
    fn mul_vec(self: SdpMatrix3, rhs: Vec3) -> Vec3 {
        Vec3 {
            x: dot3(self.m11, rhs.x, self.m12, rhs.y, self.m13, rhs.z),
            y: dot3(self.m12, rhs.x, self.m22, rhs.y, self.m23, rhs.z),
            z: dot3(self.m13, rhs.x, self.m23, rhs.y, self.m33, rhs.z),
        }
    }

    #[inline(always)]
    fn transform_vector(self: SdpMatrix3, v: Vec3) -> Vec3 {
        Self::mul_vec(self, v)
    }

    #[inline(always)]
    fn mul_mat(self: SdpMatrix3, rhs: Mat3) -> Mat3 {
        Mat3 {
            x_axis: Self::mul_vec(self, rhs.x_axis),
            y_axis: Self::mul_vec(self, rhs.y_axis),
            z_axis: Self::mul_vec(self, rhs.z_axis),
        }
    }

    fn quadform(self: SdpMatrix3, m: Mat3) -> SdpMatrix3 {
        // `s * m` column by column, kept exact at Q64.64 (`sjk` = row `j`, column `k`).
        let c0 = m.x_axis;
        let c1 = m.y_axis;
        let c2 = m.z_axis;
        let s00 = wide_mul(self.m11, c0.x)
            .add(wide_mul(self.m12, c0.y))
            .add(wide_mul(self.m13, c0.z));
        let s10 = wide_mul(self.m12, c0.x)
            .add(wide_mul(self.m22, c0.y))
            .add(wide_mul(self.m23, c0.z));
        let s20 = wide_mul(self.m13, c0.x)
            .add(wide_mul(self.m23, c0.y))
            .add(wide_mul(self.m33, c0.z));
        let s01 = wide_mul(self.m11, c1.x)
            .add(wide_mul(self.m12, c1.y))
            .add(wide_mul(self.m13, c1.z));
        let s11 = wide_mul(self.m12, c1.x)
            .add(wide_mul(self.m22, c1.y))
            .add(wide_mul(self.m23, c1.z));
        let s21 = wide_mul(self.m13, c1.x)
            .add(wide_mul(self.m23, c1.y))
            .add(wide_mul(self.m33, c1.z));
        let s02 = wide_mul(self.m11, c2.x)
            .add(wide_mul(self.m12, c2.y))
            .add(wide_mul(self.m13, c2.z));
        let s12 = wide_mul(self.m12, c2.x)
            .add(wide_mul(self.m22, c2.y))
            .add(wide_mul(self.m23, c2.z));
        let s22 = wide_mul(self.m13, c2.x)
            .add(wide_mul(self.m23, c2.y))
            .add(wide_mul(self.m33, c2.z));
        // `(m^T s m)_ij = column_i(m) . (s m)_column_j`: 9 exact triple products, one rescale.
        SdpMatrix3 {
            m11: s00.mul(c0.x).add(s10.mul(c0.y)).add(s20.mul(c0.z)).narrow(),
            m12: s01.mul(c0.x).add(s11.mul(c0.y)).add(s21.mul(c0.z)).narrow(),
            m13: s02.mul(c0.x).add(s12.mul(c0.y)).add(s22.mul(c0.z)).narrow(),
            m22: s01.mul(c1.x).add(s11.mul(c1.y)).add(s21.mul(c1.z)).narrow(),
            m23: s02.mul(c1.x).add(s12.mul(c1.y)).add(s22.mul(c1.z)).narrow(),
            m33: s02.mul(c2.x).add(s12.mul(c2.y)).add(s22.mul(c2.z)).narrow(),
        }
    }

    fn quadform3x2(
        self: SdpMatrix3, m11: Fixed, m12: Fixed, m21: Fixed, m22: Fixed, m31: Fixed, m32: Fixed,
    ) -> SdpMatrix2 {
        let x0 = wide_mul(self.m11, m11).add(wide_mul(self.m12, m21)).add(wide_mul(self.m13, m31));
        let y0 = wide_mul(self.m12, m11).add(wide_mul(self.m22, m21)).add(wide_mul(self.m23, m31));
        let z0 = wide_mul(self.m13, m11).add(wide_mul(self.m23, m21)).add(wide_mul(self.m33, m31));
        let x1 = wide_mul(self.m11, m12).add(wide_mul(self.m12, m22)).add(wide_mul(self.m13, m32));
        let y1 = wide_mul(self.m12, m12).add(wide_mul(self.m22, m22)).add(wide_mul(self.m23, m32));
        let z1 = wide_mul(self.m13, m12).add(wide_mul(self.m23, m22)).add(wide_mul(self.m33, m32));
        SdpMatrix2 {
            m11: x0.mul(m11).add(y0.mul(m21)).add(z0.mul(m31)).narrow(),
            m12: x1.mul(m11).add(y1.mul(m21)).add(z1.mul(m31)).narrow(),
            m22: x1.mul(m12).add(y1.mul(m22)).add(z1.mul(m32)).narrow(),
        }
    }

    fn inverse_unchecked(self: SdpMatrix3) -> SdpMatrix3 {
        let (det, minor_a, minor_b, minor_c) = det_minors3(self);
        if det == F_ZERO {
            core::panic_with_felt252('SdpMatrix3: singular');
        }
        adjugate_div3(self, det, minor_a, minor_b, minor_c)
    }

    fn inverse(self: SdpMatrix3) -> SdpMatrix3 {
        let (det, minor_a, minor_b, minor_c) = det_minors3(self);
        if det == F_ZERO {
            return Self::zero();
        }
        adjugate_div3(self, det, minor_a, minor_b, minor_c)
    }

    fn from_rotated_diagonal(rotation: Quat, d: Vec3) -> SdpMatrix3 {
        rotated_diagonal(Mat3Trait::from_quat(rotation), d)
    }

    fn from_rotated_diagonal_mat3(rotation: Mat3, d: Vec3) -> SdpMatrix3 {
        rotated_diagonal(rotation, d)
    }
}

/// Adds two symmetric matrices component-wise.
///
/// Mirrors `Add for parry::utils::SdpMatrix3`.
/// #### Panics
/// * `'i64_add Overflow'` / `'i64_add Underflow'` if a result does not fit the scalar range.
/// #### Deviations
/// * A plain `Fixed` sum, not `'Fixed: overflow'` as previously documented (R1 panic-coverage
///   audit, escalation 2).
/// * Overflow panics where floating-point arithmetic returns infinity or a larger finite value:
///   docs/DESIGN.md section 3, "overflow".
pub impl SdpMatrix3Add of Add<SdpMatrix3> {
    #[inline(always)]
    fn add(lhs: SdpMatrix3, rhs: SdpMatrix3) -> SdpMatrix3 {
        SdpMatrix3 {
            m11: lhs.m11 + rhs.m11,
            m12: lhs.m12 + rhs.m12,
            m13: lhs.m13 + rhs.m13,
            m22: lhs.m22 + rhs.m22,
            m23: lhs.m23 + rhs.m23,
            m33: lhs.m33 + rhs.m33,
        }
    }
}

/// Subtracts two symmetric matrices component-wise.
///
/// Mirrors nothing in parry (which only implements `Add`).
/// #### Panics
/// * `'i64_sub Overflow'` / `'i64_sub Underflow'` if a result does not fit the scalar range.
/// #### Deviations
/// * A plain `Fixed` difference, not `'Fixed: overflow'` as previously documented (R1
///   panic-coverage audit, escalation 2).
/// * Not in parry; the component-wise difference.
pub impl SdpMatrix3Sub of Sub<SdpMatrix3> {
    #[inline(always)]
    fn sub(lhs: SdpMatrix3, rhs: SdpMatrix3) -> SdpMatrix3 {
        SdpMatrix3 {
            m11: lhs.m11 - rhs.m11,
            m12: lhs.m12 - rhs.m12,
            m13: lhs.m13 - rhs.m13,
            m22: lhs.m22 - rhs.m22,
            m23: lhs.m23 - rhs.m23,
            m33: lhs.m33 - rhs.m33,
        }
    }
}

/// The adjugate of `s` divided once (`Recip`) by its non-zero determinant `det`.
#[inline(always)]
fn adjugate_div2(s: SdpMatrix2, det: Fixed) -> SdpMatrix2 {
    let r = RecipTrait::new(det);
    SdpMatrix2 { m11: r.mul(s.m22), m12: r.mul(-s.m12), m22: r.mul(s.m11) }
}

/// The floored exact determinant of `s` and its three exact (Q64.64) first-row cofactors
/// (parry's `minor_m12_m23`, `-minor_m11_m23`, `minor_m11_m22`), shared by the determinant
/// expansion (triple products, one rescale) and the adjugate.
///
/// Sharing them measured cheaper than the separate `det3` kernel
/// (`alt_inverse_separate_det`).
#[inline(always)]
fn det_minors3(s: SdpMatrix3) -> (Fixed, W2, W2, W2) {
    let minor_a = wide_mul(s.m22, s.m33).sub(wide_mul(s.m23, s.m23));
    let minor_b = wide_mul(s.m13, s.m23).sub(wide_mul(s.m12, s.m33));
    let minor_c = wide_mul(s.m12, s.m23).sub(wide_mul(s.m13, s.m22));
    let det = minor_a.mul(s.m11).add(minor_b.mul(s.m12)).add(minor_c.mul(s.m13)).narrow();
    (det, minor_a, minor_b, minor_c)
}

/// The symmetric adjugate of `s` (6 unique cofactors, one floor rescale each) divided once
/// (`Recip`) by its non-zero determinant `det`.
#[inline(always)]
fn adjugate_div3(s: SdpMatrix3, det: Fixed, minor_a: W2, minor_b: W2, minor_c: W2) -> SdpMatrix3 {
    let r = RecipTrait::new(det);
    SdpMatrix3 {
        m11: r.mul(minor_a.narrow()),
        m12: r.mul(minor_b.narrow()),
        m13: r.mul(minor_c.narrow()),
        m22: r.mul(mul_sub(s.m11, s.m33, s.m13, s.m13)),
        m23: r.mul(mul_sub(s.m13, s.m12, s.m23, s.m11)),
        m33: r.mul(mul_sub(s.m11, s.m22, s.m12, s.m12)),
    }
}

/// `r * diag(d) * r^T`, 6 unique outputs: `r_ik * d_k` is kept exact (Q64.64), then each output
/// is the exact sum of 3 triple products `r_ik d_k r_jk` rescaled once.
#[inline(always)]
fn rotated_diagonal(r: Mat3, d: Vec3) -> SdpMatrix3 {
    // Column `k` of `r` is scaled by `d_k`: `aik` = `r_ik * d_k`.
    let a00 = wide_mul(r.x_axis.x, d.x);
    let a10 = wide_mul(r.x_axis.y, d.x);
    let a20 = wide_mul(r.x_axis.z, d.x);
    let a01 = wide_mul(r.y_axis.x, d.y);
    let a11 = wide_mul(r.y_axis.y, d.y);
    let a21 = wide_mul(r.y_axis.z, d.y);
    let a02 = wide_mul(r.z_axis.x, d.z);
    let a12 = wide_mul(r.z_axis.y, d.z);
    let a22 = wide_mul(r.z_axis.z, d.z);
    SdpMatrix3 {
        m11: a00.mul(r.x_axis.x).add(a01.mul(r.y_axis.x)).add(a02.mul(r.z_axis.x)).narrow(),
        m12: a00.mul(r.x_axis.y).add(a01.mul(r.y_axis.y)).add(a02.mul(r.z_axis.y)).narrow(),
        m13: a00.mul(r.x_axis.z).add(a01.mul(r.y_axis.z)).add(a02.mul(r.z_axis.z)).narrow(),
        m22: a10.mul(r.x_axis.y).add(a11.mul(r.y_axis.y)).add(a12.mul(r.z_axis.y)).narrow(),
        m23: a10.mul(r.x_axis.z).add(a11.mul(r.y_axis.z)).add(a12.mul(r.z_axis.z)).narrow(),
        m33: a20.mul(r.x_axis.z).add(a21.mul(r.y_axis.z)).add(a22.mul(r.z_axis.z)).narrow(),
    }
}

/// `0`.
const F_ZERO: Fixed = Fixed { raw: 0 };

/// `1`.
const F_ONE: Fixed = Fixed { raw: 0x100000000 };
