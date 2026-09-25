//! Everything used by `Particles2d` and `Rigid3d`, plus scalar operations, `Mat4` inverse,
//! `slerp`, Euler conversions, a camera projection and `SymmetricEigen3`.

#[starknet::contract]
pub mod KitchenSink {
    use fixed::{ExpTrait, Fixed, FixedTrait, TrigTrait};
    use glam::{Mat3, Mat4, Quat, Vec2, Vec2Trait, Vec3};
    use glamx::{Pose2, Pose3, SymmetricEigen3};
    use starknet::storage::{Map, StorageMapReadAccess, StorageMapWriteAccess};
    use crate::sim::{self, Body, Particle};

    #[storage]
    struct Storage {
        particles: Map<u64, Particle>,
        bodies: Map<u64, Body>,
    }

    // Scalar

    #[external(v0)]
    fn mul(self: @ContractState, a: Fixed, b: Fixed) -> Fixed {
        a * b
    }

    #[external(v0)]
    fn div(self: @ContractState, a: Fixed, b: Fixed) -> Fixed {
        a / b
    }

    #[external(v0)]
    fn sqrt(self: @ContractState, a: Fixed) -> Fixed {
        a.sqrt()
    }

    #[external(v0)]
    fn sin_cos(self: @ContractState, a: Fixed) -> (Fixed, Fixed) {
        a.sin_cos()
    }

    #[external(v0)]
    fn atan2(self: @ContractState, y: Fixed, x: Fixed) -> Fixed {
        y.atan2(x)
    }

    #[external(v0)]
    fn exp(self: @ContractState, a: Fixed) -> Fixed {
        a.exp()
    }

    #[external(v0)]
    fn ln(self: @ContractState, a: Fixed) -> Fixed {
        a.ln()
    }

    #[external(v0)]
    fn powf(self: @ContractState, a: Fixed, n: Fixed) -> Fixed {
        a.powf(n)
    }

    // Particles2d

    #[external(v0)]
    fn set_particle(ref self: ContractState, id: u64, p: Particle) {
        self.particles.write(id, p);
    }

    #[external(v0)]
    fn step_particle(ref self: ContractState, id: u64, gravity: Vec2, dt: Fixed) {
        self.particles.write(id, sim::particle_step(self.particles.read(id), gravity, dt));
    }

    #[external(v0)]
    fn contact(
        self: @ContractState, a: u64, b: u64, ra: Fixed, rb: Fixed,
    ) -> Option<(Vec2, Fixed)> {
        let pa = self.particles.read(a);
        let pb = self.particles.read(b);
        let ca = Vec2Trait::new(FixedTrait::from_raw(pa.px), FixedTrait::from_raw(pa.py));
        let cb = Vec2Trait::new(FixedTrait::from_raw(pb.px), FixedTrait::from_raw(pb.py));
        sim::circle_contact(ca, ra, cb, rb)
    }

    #[external(v0)]
    fn relative(self: @ContractState, pose: Pose2, rel: Pose2, p: Vec2) -> Vec2 {
        sim::pose2_relative(pose, rel, p)
    }

    // Rigid3d

    #[external(v0)]
    fn set_body(ref self: ContractState, id: u64, b: Body) {
        self.bodies.write(id, b);
    }

    #[external(v0)]
    fn step_body(ref self: ContractState, id: u64, force: Vec3, torque: Vec3, dt: Fixed) {
        self.bodies.write(id, sim::body_step(self.bodies.read(id), force, torque, dt));
    }

    #[external(v0)]
    fn contact_frame(
        self: @ContractState, a: Pose3, b: Pose3, local_point: Vec3, local_normal: Vec3,
    ) -> (Vec3, Vec3, Fixed) {
        sim::contact_frame(a, b, local_point, local_normal)
    }

    #[external(v0)]
    fn inertia_basis(self: @ContractState, m: Mat3, q: Quat) -> Mat3 {
        sim::inertia_basis(m, q)
    }

    // Heavy remaining items

    #[external(v0)]
    fn mat4_inverse_point(self: @ContractState, m: Mat4, p: Vec3) -> Vec3 {
        sim::mat4_inverse_point(m, p)
    }

    #[external(v0)]
    fn slerp_euler(self: @ContractState, a: Quat, b: Quat, s: Fixed) -> Quat {
        sim::slerp_euler(a, b, s)
    }

    #[external(v0)]
    fn camera_project(
        self: @ContractState, eye: Vec3, center: Vec3, fov: Fixed, aspect: Fixed, p: Vec3,
    ) -> Vec3 {
        sim::camera_project(eye, center, fov, aspect, p)
    }

    #[external(v0)]
    fn principal_axes(self: @ContractState, m: Mat3) -> SymmetricEigen3 {
        sim::principal_axes(m)
    }

    #[external(v0)]
    fn heading(self: @ContractState, v: Vec2) -> Vec2 {
        sim::heading(v)
    }
}
