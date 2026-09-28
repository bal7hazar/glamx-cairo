//! Alternative implementations benchmarked against `glamx::pose2` (the `alt_*` rows of
//! `gas/pose2.snap`). The library ships fused formulations; these literal composed forms keep
//! the comparisons reproducible across compiler upgrades.

use glam_core::vec2::Vec2;
use glamx::pose2::Pose2;
use glamx::rot2::Rot2Trait;

/// `rotation.transform_vector(p) + translation`.
#[inline(always)]
pub fn transform_point_composed(pose: Pose2, p: Vec2) -> Vec2 {
    pose.rotation.transform_vector(p) + pose.translation
}

/// Constructs the conjugate and then rotates `p - translation`.
#[inline(always)]
pub fn inverse_transform_point_composed(pose: Pose2, p: Vec2) -> Vec2 {
    pose.rotation.inverse().transform_vector(p - pose.translation)
}

/// Constructs the conjugate before rotating the vector.
#[inline(always)]
pub fn inverse_transform_vector_composed(pose: Pose2, v: Vec2) -> Vec2 {
    pose.rotation.inverse().transform_vector(v)
}

/// Literal glamx `inverse` body.
#[inline(always)]
pub fn inverse_composed(pose: Pose2) -> Pose2 {
    let inv_rot = pose.rotation.inverse();
    Pose2 { rotation: inv_rot, translation: inv_rot.transform_vector(-pose.translation) }
}

/// The relative-pose expression with a single conjugate rotation of the translation difference.
#[inline(always)]
pub fn inv_mul_composed(lhs: Pose2, rhs: Pose2) -> Pose2 {
    let inv_rot = lhs.rotation.inverse();
    Pose2 {
        rotation: inv_rot * rhs.rotation,
        translation: inv_rot.transform_vector(rhs.translation - lhs.translation),
    }
}

/// Literal upstream glamx 0.3.1 body: `lhs.inverse() * rhs`.
#[inline(always)]
pub fn inv_mul_via_inverse(lhs: Pose2, rhs: Pose2) -> Pose2 {
    mul_composed(inverse_composed(lhs), rhs)
}

/// Literal glamx composition with a separately rounded rotation and vector addition.
#[inline(always)]
pub fn mul_composed(lhs: Pose2, rhs: Pose2) -> Pose2 {
    Pose2 {
        rotation: lhs.rotation * rhs.rotation,
        translation: lhs.translation + lhs.rotation.transform_vector(rhs.translation),
    }
}
