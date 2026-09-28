//! Type identity of `glamx` and the `glam` facade: every value below is built and typed through
//! `glam::...` paths and crosses the `glamx` API without conversion. A `glam` / `glam_core` type
//! mismatch is a compile error of this file.

use fixed::fixed::{Fixed, FixedTrait};
use glam::mat2::Mat2;
use glam::mat3::{Mat3, mat3};
use glam::mat4::{Mat4, Mat4Trait};
use glam::quat::{Quat, QuatTrait};
use glam::vec2::{Vec2, Vec2Trait, vec2};
use glam::vec3::{Vec3, Vec3Trait, vec3};
use glam::vec4::Vec4;
use glamx::eigen3::{Mat3ExtTrait, SymmetricEigen3, SymmetricEigen3Trait};
use glamx::pose2::{Pose2, Pose2Trait};
use glamx::pose3::{Pose3, Pose3Trait};
use glamx::rot2::{Rot2, Rot2Trait};
use glamx::rot3::Rot3;
use glamx::sdp::{SdpMatrix2, SdpMatrix2Trait, SdpMatrix3, SdpMatrix3Trait};

const ONE: i64 = 0x100000000;
const FRAC_PI_2: i64 = 6746518852;

fn f(raw: i64) -> Fixed {
    FixedTrait::from_raw(raw)
}

fn n(v: i64) -> Fixed {
    f(v * ONE)
}

#[test]
fn test_rot3_is_the_facade_quat() {
    let q: Quat = QuatTrait::IDENTITY;
    let r: Rot3 = q;
    let back: Quat = r;
    assert_eq!(back, QuatTrait::IDENTITY);
}

#[test]
fn test_pose3_takes_and_returns_facade_values() {
    // A quarter turn about Z, built as a facade `Quat`, used as the `Rot3` of a `Pose3`.
    let rotation: Quat = QuatTrait::from_rotation_z(f(FRAC_PI_2));
    let translation: Vec3 = vec3(n(1), n(2), n(3));
    let pose: Pose3 = Pose3Trait::from_parts(translation, rotation);
    assert_eq!(pose.rotation, rotation);
    assert_eq!(pose.translation, translation);

    let p: Vec3 = vec3(n(1), n(0), n(0));
    let moved: Vec3 = pose.transform_point(p);
    assert_eq!(moved, rotation.mul_vec3(p) + translation);
    let round_trip: Vec3 = pose.inverse_transform_point(moved);
    assert!(round_trip.abs_diff_eq(p, f(1024)));

    // The pose as a facade `Mat4` and back.
    let m: Mat4 = pose.to_mat4();
    let w_axis: Vec4 = m.w_axis;
    assert_eq!(w_axis.x, n(1));
    assert_eq!(w_axis.y, n(2));
    assert_eq!(w_axis.z, n(3));
    let from_m: Pose3 = Pose3Trait::from_mat4(m);
    assert!(from_m.abs_diff_eq(pose, f(1024)));
    let identity: Mat4 = Mat4Trait::IDENTITY;
    assert_eq!(Pose3Trait::from_mat4(identity), Pose3Trait::identity());
}

#[test]
fn test_rot2_and_pose2_take_and_return_facade_values() {
    let v: Vec2 = vec2(n(1), n(0));
    let r: Rot2 = Rot2Trait::from_angle(f(FRAC_PI_2));
    let turned: Vec2 = r.transform_vector(v);
    assert!(turned.abs_diff_eq(vec2(n(0), n(1)), f(8)));

    let mat: Mat2 = Rot2Trait::to_mat(r);
    assert_eq!(Rot2Trait::from_mat_unchecked(mat), r);

    let translation: Vec2 = vec2(n(5), n(-7));
    let pose: Pose2 = Pose2Trait::from_parts(translation, r);
    assert_eq!(pose.translation, translation);
    assert_eq!(pose.transform_point(v), turned + translation);
    assert!(pose.inverse_transform_point(pose.transform_point(v)).abs_diff_eq(v, f(1024)));

    // The pose as a facade `Mat3` (affine) and back.
    let m: Mat3 = pose.to_mat3();
    assert_eq!(m.z_axis.x, n(5));
    assert_eq!(m.z_axis.y, n(-7));
    let from_m: Pose2 = Pose2Trait::from_mat3(m);
    assert!(from_m.abs_diff_eq(pose, f(1024)));
}

#[test]
fn test_sdp_matrices_take_and_return_facade_values() {
    let d: Vec3 = vec3(n(1), n(2), n(3));
    let rotation: Quat = QuatTrait::IDENTITY;
    let s: SdpMatrix3 = SdpMatrix3Trait::from_rotated_diagonal(rotation, d);
    let m: Mat3 = s.into_matrix();
    assert_eq!(m, mat3(vec3(n(1), n(0), n(0)), vec3(n(0), n(2), n(0)), vec3(n(0), n(0), n(3))));
    assert_eq!(SdpMatrix3Trait::from_sdp_matrix(m), s);
    let v: Vec3 = vec3(n(1), n(1), n(1));
    let sv: Vec3 = s.mul_vec(v);
    assert_eq!(sv, d);
    assert_eq!(
        s.mul_mat(m), mat3(vec3(n(1), n(0), n(0)), vec3(n(0), n(4), n(0)), vec3(n(0), n(0), n(9))),
    );

    let s2: SdpMatrix2 = SdpMatrix2Trait::diagonal(n(2));
    let u: Vec2 = vec2(n(3), n(-4));
    let s2u: Vec2 = s2.mul_vec(u);
    assert_eq!(s2u, vec2(n(6), n(-8)));
    let m2: Mat2 = s2.into_matrix();
    assert_eq!(SdpMatrix2Trait::from_sdp_matrix(m2), s2);
}

#[test]
fn test_symmetric_eigen3_takes_and_returns_facade_values() {
    // diag(3, 1, 2): eigenvalues are returned in ascending order.
    let a: Mat3 = mat3(vec3(n(3), n(0), n(0)), vec3(n(0), n(1), n(0)), vec3(n(0), n(0), n(2)));
    let values: Vec3 = a.symmetric_eigenvalues();
    assert!(values.abs_diff_eq(vec3(n(1), n(2), n(3)), f(1024)));
    let e: SymmetricEigen3 = SymmetricEigen3Trait::new(a);
    let eigenvalues: Vec3 = e.eigenvalues;
    let eigenvectors: Mat3 = e.eigenvectors;
    assert!(eigenvalues.abs_diff_eq(values, f(1024)));
    // A * V = V * diag(lambda): the first eigenvector of diag(3, 1, 2) is the Y axis, up to sign.
    let first: Vec3 = eigenvectors.x_axis;
    assert!(
        first.abs_diff_eq(vec3(n(0), n(1), n(0)), f(1024))
            || first.abs_diff_eq(vec3(n(0), n(-1), n(0)), f(1024)),
    );
    let s: SdpMatrix3 = SdpMatrix3Trait::from_sdp_matrix(a);
    let from_sdp: SymmetricEigen3 = SymmetricEigen3Trait::from_sdp(s);
    assert_eq!(from_sdp, e);
}
