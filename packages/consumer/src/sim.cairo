//! The simulation logic shared by the contracts: plain functions over the library types.

use fixed::{Fixed, FixedTrait, ONE, TrigTrait, ZERO};
use glam_core::camera::rh::proj::opengl;
use glam_core::camera::rh::view::look_at_mat4;
use glam_core::{
    EulerRot, Mat3, Mat3Trait, Mat4, Mat4Trait, Quat, QuatEulerTrait, QuatTrait, Vec2, Vec2Trait,
    Vec3, Vec3Trait,
};
use glamx::{
    Pose2, Pose2Trait, Pose3, Pose3Trait, Rot2, Rot2Trait, SdpMatrix3Trait, SymmetricEigen3,
    SymmetricEigen3Trait,
};

/// A 2D particle, stored as raw Q32.32 fields (`Fixed` is not `starknet::Store`).
#[derive(Copy, Drop, Serde, starknet::Store)]
pub struct Particle {
    pub px: i64,
    pub py: i64,
    pub vx: i64,
    pub vy: i64,
    pub re: i64,
    pub im: i64,
    pub omega: i64,
}

/// A 3D rigid body, stored as raw Q32.32 fields.
#[derive(Copy, Drop, Serde, starknet::Store)]
pub struct Body {
    pub px: i64,
    pub py: i64,
    pub pz: i64,
    pub qx: i64,
    pub qy: i64,
    pub qz: i64,
    pub qw: i64,
    pub vx: i64,
    pub vy: i64,
    pub vz: i64,
    pub wx: i64,
    pub wy: i64,
    pub wz: i64,
    pub inv_mass: i64,
    pub ix: i64,
    pub iy: i64,
    pub iz: i64,
}

fn f(raw: i64) -> Fixed {
    FixedTrait::from_raw(raw)
}

fn v3(x: i64, y: i64, z: i64) -> Vec3 {
    Vec3Trait::new(f(x), f(y), f(z))
}

// ---------------------------------------------------------------------------------------------
// 2D

/// Semi-implicit Euler step of a particle under gravity, with its orientation integrated from
/// its angular velocity.
pub fn particle_step(p: Particle, gravity: Vec2, dt: Fixed) -> Particle {
    let vel = Vec2Trait::new(f(p.vx), f(p.vy)) + gravity.mul_scalar(dt);
    let pos = Vec2Trait::new(f(p.px), f(p.py)) + vel.mul_scalar(dt);
    let rot = Rot2 { re: f(p.re), im: f(p.im) };
    let rot = (Rot2Trait::from_angle(f(p.omega) * dt) * rot).normalize();
    Particle {
        px: pos.x.raw,
        py: pos.y.raw,
        vx: vel.x.raw,
        vy: vel.y.raw,
        re: rot.re.raw,
        im: rot.im.raw,
        omega: p.omega,
    }
}

/// Circle-circle contact: the normal from `a` to `b` and the penetration depth, if any.
pub fn circle_contact(a: Vec2, ra: Fixed, b: Vec2, rb: Fixed) -> Option<(Vec2, Fixed)> {
    let d = b - a;
    let r = ra + rb;
    if d.length_squared() >= r * r {
        return None;
    }
    Some((d.normalize_or_zero(), r - d.length()))
}

/// A point of `rel` expressed in the frame of `pose`, then moved by the relative pose.
pub fn pose2_relative(pose: Pose2, rel: Pose2, p: Vec2) -> Vec2 {
    pose.inv_mul(rel).transform_point(p) + (pose * rel).transform_point(p)
}

// ---------------------------------------------------------------------------------------------
// 3D

/// Symplectic Euler step of a rigid body: world-space inverse inertia, velocity update from a
/// force and a torque, position update, orientation integrated from the angular velocity.
pub fn body_step(b: Body, force: Vec3, torque: Vec3, dt: Fixed) -> Body {
    let rot = QuatTrait::from_xyzw(f(b.qx), f(b.qy), f(b.qz), f(b.qw));
    let inv_inertia = SdpMatrix3Trait::from_rotated_diagonal(rot, v3(b.ix, b.iy, b.iz));
    let angvel = v3(b.wx, b.wy, b.wz) + inv_inertia.mul_vec(torque).mul_scalar(dt);
    let linvel = v3(b.vx, b.vy, b.vz) + force.mul_scalar(f(b.inv_mass) * dt);
    let pos = v3(b.px, b.py, b.pz) + linvel.mul_scalar(dt);
    let rot = (QuatTrait::from_scaled_axis(angvel.mul_scalar(dt)) * rot).normalize();
    Body {
        px: pos.x.raw,
        py: pos.y.raw,
        pz: pos.z.raw,
        qx: rot.x.raw,
        qy: rot.y.raw,
        qz: rot.z.raw,
        qw: rot.w.raw,
        vx: linvel.x.raw,
        vy: linvel.y.raw,
        vz: linvel.z.raw,
        wx: angvel.x.raw,
        wy: angvel.y.raw,
        wz: angvel.z.raw,
        inv_mass: b.inv_mass,
        ix: b.ix,
        iy: b.iy,
        iz: b.iz,
    }
}

/// Contact frame between two bodies: the world normal, a tangent and the signed distance of a
/// contact point given in the local frame of `b`.
pub fn contact_frame(
    a: Pose3, b: Pose3, local_point: Vec3, local_normal: Vec3,
) -> (Vec3, Vec3, Fixed) {
    let point_in_a = a.inv_mul(b).transform_point(local_point);
    let world_point = (a * b).transform_point(local_point);
    let n = a.rotation.mul_vec3(local_normal).normalize();
    let t = n.cross(Vec3Trait::X).normalize_or_zero();
    (n, t, (world_point - point_in_a).dot(n))
}

/// Rotated and inverted inertia tensor through `Mat3` products: `(R M R^T)^-1`.
pub fn inertia_basis(m: Mat3, q: Quat) -> Mat3 {
    let r = Mat3Trait::from_quat(q);
    r.mul_mat3(m).mul_mat3(r.transpose()).inverse()
}

// ---------------------------------------------------------------------------------------------
// Heavy remaining items

/// Inverse of a `Mat4` applied to a point.
pub fn mat4_inverse_point(m: Mat4, p: Vec3) -> Vec3 {
    m.inverse().transform_point3(p)
}

/// Spherical interpolation between two orientations, then its Euler angles, rebuilt as a quat.
pub fn slerp_euler(a: Quat, b: Quat, s: Fixed) -> Quat {
    let (y, x, z) = a.slerp(b, s).to_euler(EulerRot::YXZ);
    QuatEulerTrait::from_euler(EulerRot::YXZ, y, x, z)
}

/// Projects a world point through a right-handed OpenGL camera.
pub fn camera_project(eye: Vec3, center: Vec3, fov: Fixed, aspect: Fixed, p: Vec3) -> Vec3 {
    let up = Vec3Trait::new(ZERO, ONE, ZERO);
    let proj = opengl::perspective(fov, aspect, ONE, f(100 * 0x100000000));
    proj.mul_mat4(look_at_mat4(eye, center, up)).project_point3(p)
}

/// Principal axes of a symmetric inertia tensor.
pub fn principal_axes(m: Mat3) -> SymmetricEigen3 {
    SymmetricEigen3Trait::new(m)
}

/// Heading of a 2D direction, and back.
pub fn heading(v: Vec2) -> Vec2 {
    let (s, c) = v.y.atan2(v.x).sin_cos();
    Vec2Trait::new(c, s)
}
