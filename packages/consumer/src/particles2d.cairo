//! A 2D integrator step on `Vec2` / `Rot2` / `Pose2` over particles kept in storage.

#[starknet::contract]
pub mod Particles2d {
    use fixed::{Fixed, FixedTrait};
    use glam_core::{Vec2, Vec2Trait};
    use glamx::Pose2;
    use starknet::storage::{Map, StorageMapReadAccess, StorageMapWriteAccess};
    use crate::sim::{self, Particle};

    #[storage]
    struct Storage {
        particles: Map<u64, Particle>,
    }

    #[external(v0)]
    fn set_particle(ref self: ContractState, id: u64, p: Particle) {
        self.particles.write(id, p);
    }

    #[external(v0)]
    fn step(ref self: ContractState, id: u64, gravity: Vec2, dt: Fixed) {
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
}
