//! Alternative implementations benchmarked against `glamx::sdp` (the `alt_*` rows of
//! `gas/sdp.snap`). The library ships the cheapest formulation; the literal parry / rapier
//! formulations and the partially fused ones stay here so that the comparison is reproducible
//! across compiler upgrades.

use fixed::fixed::Fixed;
use fixed::wide::{RecipTrait, det3, dot3, mul_sub};
use glam_core::mat3::{Mat3, Mat3Trait};
use glam_core::quat::Quat;
use glam_core::vec3::{Vec3, Vec3Trait};
use glamx::sdp::{SdpMatrix3, SdpMatrix3Trait};

/// Alternative to `SdpMatrix3::from_rotated_diagonal`. The literal parry
/// `MassProperties::world_inv_inertia`: `Mat3::from_quat`, `transpose`, the 3 columns scaled by
/// `d` (`Vec3::mul_scalar`), the full `Mat3 * Mat3` product (9 outputs), `from_sdp_matrix`.
#[inline(always)]
pub fn from_rotated_diagonal_literal(rotation: Quat, d: Vec3) -> SdpMatrix3 {
    let lhs = Mat3Trait::from_quat(rotation);
    let rhs = lhs.transpose();
    let lhs = Mat3 {
        x_axis: lhs.x_axis.mul_scalar(d.x),
        y_axis: lhs.y_axis.mul_scalar(d.y),
        z_axis: lhs.z_axis.mul_scalar(d.z),
    };
    SdpMatrix3Trait::from_sdp_matrix(lhs * rhs)
}

/// Alternative to `SdpMatrix3::from_rotated_diagonal_mat3`. Two stages: `R diag(d)` rounded
/// (9 rescales), then one `dot3` per unique output (6 rescales) instead of 6 fused triple sums.
#[inline(always)]
pub fn from_rotated_diagonal_mat3_two_stage(r: Mat3, d: Vec3) -> SdpMatrix3 {
    let a = r.mul_diagonal_scale(d);
    SdpMatrix3 {
        m11: dot3(a.x_axis.x, r.x_axis.x, a.y_axis.x, r.y_axis.x, a.z_axis.x, r.z_axis.x),
        m12: dot3(a.x_axis.x, r.x_axis.y, a.y_axis.x, r.y_axis.y, a.z_axis.x, r.z_axis.y),
        m13: dot3(a.x_axis.x, r.x_axis.z, a.y_axis.x, r.y_axis.z, a.z_axis.x, r.z_axis.z),
        m22: dot3(a.x_axis.y, r.x_axis.y, a.y_axis.y, r.y_axis.y, a.z_axis.y, r.z_axis.y),
        m23: dot3(a.x_axis.y, r.x_axis.z, a.y_axis.y, r.y_axis.z, a.z_axis.y, r.z_axis.z),
        m33: dot3(a.x_axis.z, r.x_axis.z, a.y_axis.z, r.y_axis.z, a.z_axis.z, r.z_axis.z),
    }
}

/// Alternative to `SdpMatrix3::quadform`. The literal parry formula: `self.mul_mat(m)` (9
/// rescales), the full `m^T * sm` product (9 more), `from_sdp_matrix`.
#[inline(always)]
pub fn quadform_literal(s: SdpMatrix3, m: Mat3) -> SdpMatrix3 {
    let sm = s.mul_mat(m);
    SdpMatrix3Trait::from_sdp_matrix(m.transpose() * sm)
}

/// Alternative to `SdpMatrix3::quadform`. Two stages: `self.mul_mat(m)` rounded (9 rescales),
/// then one `dot3` per unique output (6 rescales).
#[inline(always)]
pub fn quadform_two_stage(s: SdpMatrix3, m: Mat3) -> SdpMatrix3 {
    let sm = s.mul_mat(m);
    let c0 = m.x_axis;
    let c1 = m.y_axis;
    let c2 = m.z_axis;
    SdpMatrix3 {
        m11: dot3(c0.x, sm.x_axis.x, c0.y, sm.x_axis.y, c0.z, sm.x_axis.z),
        m12: dot3(c0.x, sm.y_axis.x, c0.y, sm.y_axis.y, c0.z, sm.y_axis.z),
        m13: dot3(c0.x, sm.z_axis.x, c0.y, sm.z_axis.y, c0.z, sm.z_axis.z),
        m22: dot3(c1.x, sm.y_axis.x, c1.y, sm.y_axis.y, c1.z, sm.y_axis.z),
        m23: dot3(c1.x, sm.z_axis.x, c1.y, sm.z_axis.y, c1.z, sm.z_axis.z),
        m33: dot3(c2.x, sm.z_axis.x, c2.y, sm.z_axis.y, c2.z, sm.z_axis.z),
    }
}

/// Alternative to `SdpMatrix3::inverse`. The literal rapier `AngularInertiaOps::inverse`: plain
/// `Fixed` products (one rescale each) for the cofactors and the determinant, one rounded `Fixed
/// / Fixed` per output (6 divisions) instead of the shared `Recip`.
#[inline(always)]
pub fn inverse_plain(s: SdpMatrix3) -> SdpMatrix3 {
    let minor_a = s.m22 * s.m33 - s.m23 * s.m23;
    let minor_b = s.m12 * s.m33 - s.m13 * s.m23;
    let minor_c = s.m12 * s.m23 - s.m13 * s.m22;
    let det = s.m11 * minor_a - s.m12 * minor_b + s.m13 * minor_c;
    if det == F_ZERO {
        return SdpMatrix3Trait::zero();
    }
    SdpMatrix3 {
        m11: minor_a / det,
        m12: -minor_b / det,
        m13: minor_c / det,
        m22: (s.m11 * s.m33 - s.m13 * s.m13) / det,
        m23: (s.m13 * s.m12 - s.m23 * s.m11) / det,
        m33: (s.m11 * s.m22 - s.m12 * s.m12) / det,
    }
}

/// Alternative to `SdpMatrix3::inverse`. The exact `det3` kernel on the full matrix (the
/// cofactors are recomputed instead of shared with the determinant), then the shared `Recip`.
#[inline(always)]
pub fn inverse_separate_det(s: SdpMatrix3) -> SdpMatrix3 {
    let det = det3(s.m11, s.m12, s.m13, s.m12, s.m22, s.m23, s.m13, s.m23, s.m33);
    if det == F_ZERO {
        return SdpMatrix3Trait::zero();
    }
    let r = RecipTrait::new(det);
    SdpMatrix3 {
        m11: r.mul(mul_sub(s.m22, s.m33, s.m23, s.m23)),
        m12: r.mul(mul_sub(s.m13, s.m23, s.m12, s.m33)),
        m13: r.mul(mul_sub(s.m12, s.m23, s.m13, s.m22)),
        m22: r.mul(mul_sub(s.m11, s.m33, s.m13, s.m13)),
        m23: r.mul(mul_sub(s.m13, s.m12, s.m23, s.m11)),
        m33: r.mul(mul_sub(s.m11, s.m22, s.m12, s.m12)),
    }
}

/// Alternative to `SdpMatrix3::inverse`. Through the full matrix: `into_matrix`,
/// `Mat3::inverse_or_zero` (9 cofactors, 9 outputs), `from_sdp_matrix`.
#[inline(always)]
pub fn inverse_via_mat3(s: SdpMatrix3) -> SdpMatrix3 {
    SdpMatrix3Trait::from_sdp_matrix(s.into_matrix().inverse_or_zero())
}

/// Alternative to `SdpMatrix3::mul_vec`. The literal parry sum of products: one rescale per
/// product (9) instead of one per component (3).
#[inline(always)]
pub fn mul_vec_unfused(s: SdpMatrix3, v: Vec3) -> Vec3 {
    Vec3 {
        x: s.m11 * v.x + s.m12 * v.y + s.m13 * v.z,
        y: s.m12 * v.x + s.m22 * v.y + s.m23 * v.z,
        z: s.m13 * v.x + s.m23 * v.y + s.m33 * v.z,
    }
}

/// `0`.
const F_ZERO: Fixed = Fixed { raw: 0 };
