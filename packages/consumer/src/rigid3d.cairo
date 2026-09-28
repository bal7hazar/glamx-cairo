//! What one rapier-style 3D step touches: `Vec3`, `Quat`, `Mat3`, `Pose3`, `SdpMatrix3`.

#[starknet::contract]
pub mod Rigid3d {
    use fixed::Fixed;
    use glam_core::{Mat3, Quat, Vec3};
    use glamx::Pose3;
    use starknet::storage::{Map, StorageMapReadAccess, StorageMapWriteAccess};
    use crate::sim::{self, Body};

    #[storage]
    struct Storage {
        bodies: Map<u64, Body>,
    }

    #[external(v0)]
    fn set_body(ref self: ContractState, id: u64, b: Body) {
        self.bodies.write(id, b);
    }

    #[external(v0)]
    fn step(ref self: ContractState, id: u64, force: Vec3, torque: Vec3, dt: Fixed) {
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
}
