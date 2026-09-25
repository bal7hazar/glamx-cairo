//! Alternative implementations (math / bitwise / loop / table variants) kept for gas comparison.
//! The winner lives in the library; the losers stay here with their benches.

pub mod eigen3;
pub mod pose2;
pub mod pose3;
pub mod rot2;
pub mod rot3;
pub mod sdp;
