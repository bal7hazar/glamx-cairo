# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versioning policy:
`docs/DESIGN.md` section 6.

## [Unreleased]

Nothing yet.

## [0.5.0] - 2026-10-05

No glamx result changes: every gas snapshot and golden file is identical to 0.4.1. `glam_core` 0.5.0
changes the result of `Quat::to_axis_angle` and `to_scaled_axis`; `glamx` calls neither (it only
uses `Quat::from_scaled_axis`) and re-exports neither.

### Changed

- Depends on `fixed` 0.5.0 and `glam_core` 0.5.0 (was 0.4.x); `facade_check` on `glam` 0.5.0.

## [0.4.1] - 2026-09-28

No API change and no numeric change: every gas snapshot and golden file is identical to 0.4.0.

### Changed

- Depends on `glam_core` (0.4.1) instead of the `glam` facade (closure -0.45 s / -0.25 GB); no
  API change. `glamx` values interoperate with `glam` facade users: the facade re-exports the
  `glam_core` modules, so `glam::vec3::Vec3`, `glam::quat::Quat`, `glam::mat4::Mat4` and the other
  facade types are the types `glamx` takes and returns.

### Added

- `packages/facade_check` (unpublished): a fixture that depends on the registry `glam` 0.4.1 and
  on `glamx`, builds its values through the facade paths and passes them to `glamx`; its `snforge`
  tests are the proof of the type identity.

## [0.4.0] - 2026-09-25

First release of `glamx` cut from `glamx-cairo`: depends on the published `fixed` 0.4.0 (adds the
hyperbolic functions) and `glam` 0.4.0. No numeric result of `glamx` changes; every gas snapshot
and golden file is identical. Released so that `glamx` shares one `Fixed` type with the other
consumers of `fixed` 0.4.0 (pre-1.0, `^0.3.0` excludes `0.4.0`).

### Changed

- Repository split: `glamx` now lives in `bal7hazar/glamx-cairo` and consumes the published
  `fixed` and `glam` from the Scarb registry (`fixed = "0.4.0"`, `glam = "0.4.0"`).

## [0.3.0] - 2026-09-23

This version was released from the former monorepo at `bal7hazar/glam-cairo`.

### Changed

- Division inherited from `fixed` now rounds to nearest, ties to even. Every glamx result that
  goes through `/`, `recip` or `from_ratio` may move by 1 ULP; the symmetric-eigenvalue error
  measured 0.64 ULP times the matrix scale, with residual tolerance 16 ULP.

## [0.2.0] - 2026-09-23

This version was released from the former monorepo at `bal7hazar/glam-cairo`.

### Changed

- Re-released unchanged so that `glamx`, `glam` and `fixed` use the same pre-1.0 dependency
  version after `fixed::wide::Acc` was added.

## [0.1.0] - 2026-09-22

This version was released from the former monorepo at `bal7hazar/glam-cairo`.

### Added

- Initial `glamx` package mirroring Dimforge glamx 0.3.1.
- `Pose3`, the `Rot3 = Quat` alias, fused `inv_mul`, point and vector transforms, and `nlerp`.
- `SdpMatrix2`, `SdpMatrix3` and the fused world-inertia kernel
  `from_rotated_diagonal` (`R D R^T`).
- `Rot2`, including fused composition and explicit normalization policy.
- `Pose2`, including fused composition, `inv_mul`, and point and vector transforms.
- `SymmetricEigen3` using scaled cyclic Jacobi rotations with Rayleigh refinement.
- Golden-vector generation against glamx 0.3.1, gas snapshots, generated gas tables, panic
  coverage and Starknet consumer bytecode-size fixtures.

### Changed

- `SymmetricEigen3` runs one Jacobi rotation per metered loop iteration, short-circuits diagonal
  inputs and lets `eigenvalues` skip eigenvector polishing. Generic `new` fell from 586,320 to
  535,850 gas at the time; rotation order changed to `(1,2), (2,3), (3,1)` and improved all
  measured error bounds.
- `Rot2::lerp` now matches upstream's unnormalized component-wise blend; `Rot2::is_normalized`
  was added using `fixed::wide::is_unit2`.
- Deviation documentation and exact panic-path coverage were completed for all glamx items.
