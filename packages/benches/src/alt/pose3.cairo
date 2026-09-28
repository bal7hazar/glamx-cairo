//! Alternative implementations benchmarked against `glamx::pose3` (the `alt_*` rows of
//! `gas/pose3.snap`). The library ships the fused formulations it documents; the composed forms
//! below are the literal glamx expressions on top of the public `glam_core::quat` API. They give
//! bit-identical results (the exact value inside each kernel is the same) and stay here so that
//! the comparison is reproducible across compiler upgrades.

use glam_core::quat::QuatTrait;
use glam_core::vec3::Vec3;
use glamx::pose3::Pose3;

/// Alternative to `Pose3Trait::transform_point`: `rotation.mul_vec3(p) + translation`, a rescale
/// per component and then a separate `Vec3` addition.
#[inline(always)]
pub fn transform_point_composed(pose: Pose3, p: Vec3) -> Vec3 {
    pose.rotation.mul_vec3(p) + pose.translation
}

/// Alternative to `Pose3Trait::inverse_transform_point`: the conjugate is built, then rotated
/// by `QuatTrait::mul_vec3`.
#[inline(always)]
pub fn inverse_transform_point_composed(pose: Pose3, p: Vec3) -> Vec3 {
    pose.rotation.conjugate().mul_vec3(p - pose.translation)
}

/// Alternative to `Pose3Trait::inverse_transform_vector`: the conjugate is built first.
#[inline(always)]
pub fn inverse_transform_vector_composed(pose: Pose3, v: Vec3) -> Vec3 {
    pose.rotation.conjugate().mul_vec3(v)
}

/// Alternative to `Pose3Trait::inverse`, the literal glamx body:
/// `inv_rot = rotation.inverse(); (inv_rot, inv_rot * -translation)`.
#[inline(always)]
pub fn inverse_composed(pose: Pose3) -> Pose3 {
    let inv_rot = pose.rotation.conjugate();
    Pose3 { rotation: inv_rot, translation: inv_rot.mul_vec3(-pose.translation) }
}

/// Alternative to `Pose3Trait::inv_mul`, the literal glamx body: the conjugate of
/// `lhs.rotation` is built once and shared by the Hamilton product and the rotation.
#[inline(always)]
pub fn inv_mul_composed(lhs: Pose3, rhs: Pose3) -> Pose3 {
    let inv_rot = lhs.rotation.conjugate();
    Pose3 {
        rotation: inv_rot.mul_quat(rhs.rotation),
        translation: inv_rot.mul_vec3(rhs.translation - lhs.translation),
    }
}

/// Alternative to `Pose3Trait::inv_mul`: `inverse(lhs) * rhs`, the unfused form of glamx's
/// `Pose2::inv_mul` (two rotations of a vector instead of one).
#[inline(always)]
pub fn inv_mul_via_inverse(lhs: Pose3, rhs: Pose3) -> Pose3 {
    mul_composed(inverse_composed(lhs), rhs)
}

/// Alternative to `Pose3 * Pose3`: the translation `lhs.translation + lhs.rotation * rhs.t` is a
/// `mul_vec3` followed by a separate `Vec3` addition.
#[inline(always)]
pub fn mul_composed(lhs: Pose3, rhs: Pose3) -> Pose3 {
    Pose3 {
        rotation: lhs.rotation.mul_quat(rhs.rotation),
        translation: lhs.translation + lhs.rotation.mul_vec3(rhs.translation),
    }
}
