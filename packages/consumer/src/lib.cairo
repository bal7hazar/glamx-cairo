//! Starknet contract fixtures that link `glamx` and its dependencies into deployable classes, so
//! that the size of a realistic consumer can be tracked against the network limits.
//!
//! Measurement fixtures, not a product: `scripts/bytecode_size.py` builds this package in the
//! release profile and reports the Sierra and CASM sizes of every contract
//! (`gas/bytecode.size`). Every input comes from calldata or storage, so that nothing is
//! constant-folded.
//!
//! The contracts grow in weight: `Particles2d` (a 2D integrator step), `Rigid3d` (what one
//! rapier-style 3D step touches) and `KitchenSink` (everything above plus the heaviest remaining
//! items). The simulation logic lives in `sim` and is shared by the contracts, like it would be
//! in a real consumer.

pub mod kitchen_sink;
pub mod particles2d;
pub mod rigid3d;
pub mod sim;
