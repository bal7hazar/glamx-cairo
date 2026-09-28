# Design

Decisions that every module must follow. Changing a numeric rule is a breaking change and is
decided by the cross-repository orchestrator in
[`glam-cairo`](https://github.com/bal7hazar/glam-cairo), never by a porting agent.

## 1. Scope

`glamx-cairo` ports [Dimforge glamx](https://github.com/dimforge/glamx) **0.3.1** to pure Cairo
without a `starknet` dependency. It is the physics-oriented extension layer used by the Cairo
ports of Parry and Rapier:

| package | role |
|---|---|
| `glamx` | `Rot2`, `Rot3 = Quat`, `Pose2`, `Pose3`, `SdpMatrix2`, `SdpMatrix3`, `SymmetricEigen3` |

The package deliberately depends on the registry releases `fixed = "0.4.0"` and
`glam_core = "0.4.1"` (not the `glam` facade: glamx only uses core types, and the closure of an
empty consumer is 0.45 s / 0.25 GB lighter, 0.4.1). The `glam` facade re-exports `glam_core`'s
modules, so `glam::vec3::Vec3` and the other facade types are the types glamx takes and returns;
`packages/facade_check` proves it. General vector, matrix, quaternion and scalar APIs belong to
those repositories, not here.

## 2. Scalar and arithmetic foundation

The scalar is `fixed::Fixed`, a signed Q32.32 `i64`, and all base geometry types come from
`glam`. Their representation, rounding, overflow rules, fused kernels and transcendental choices
are defined by [`fixed-cairo`](https://github.com/bal7hazar/fixed-cairo/blob/main/docs/DESIGN.md)
and [`glam-cairo`](https://github.com/bal7hazar/glam-cairo/blob/main/docs/DESIGN.md).

Every product in this repository uses `fixed::wide` fused kernels: raw products are accumulated
exactly and rescaled once per output scalar. Missing scalar kernels are requested from
`fixed-cairo`; they are not recreated locally as chains of `Fixed * Fixed`.

One glamx-specific rounding exception is part of the public numeric contract: Jacobi rotations in
`SymmetricEigen3` round to nearest. Repeated floor rounding across roughly twelve rotations had a
measured worst residual of 14.0 ULP; round-to-nearest reduces it to 6.7 ULP. The scaled cyclic
Jacobi solver and Rayleigh refinement are deterministic, and their results are bit-exact API.

## 3. Semantics that differ from upstream glamx

The finite Q32.32 domain and inherited `glam` semantics apply throughout: there are no NaN or
infinity values, overflow panics, products are fused, and invalid operations that would produce a
non-finite float panic instead. Each affected public item documents its precise behaviour under
`#### Deviations`.

| topic | glamx 0.3.1 | glamx-cairo |
|---|---|---|
| interpolation | `Rot2::lerp` is a component-wise blend and is not normalized | Same-name `lerp` preserves that behaviour; normalization is explicit with `.normalize()` |
| alternative algorithms | platform floating-point formulas, including closed-form symmetric eigen decomposition | deterministic `fixed` transcendentals and a scaled cyclic Jacobi `SymmetricEigen3`; measured error bounds are documented on the items |
| algebraic reassociation | floating-point operations round after every source operation | products are accumulated through `fixed::wide` and rescaled once, so an algebraically equivalent expression may differ bitwise |
| API surface | inherent methods plus Rust reference and iterator glue | Cairo extension traits, named heterogeneous operations, value parameters and derived `Debug`; omissions and additions are documented on the type |

## 4. Code conventions

Naming mirrors glamx exactly. A type uses `pub trait XTrait` plus `pub impl XImpl of XTrait`;
operator implementations are named `XAdd`, `XMul`, and so on. Panic messages have the form
`'<Type>: <reason>'` and fit within 31 characters.

Hard rules:

1. Value types are
   `#[derive(Copy, Drop, Serde, PartialEq, Debug, Default, Hash)]` structs with named public scalar
   fields and are passed by value. Fixed-size math uses no `Array`, `Span`, `Felt252Dict` or loop.
2. Products go through `fixed::wide` fused kernels, with one rescale per output component.
3. `#[inline(always)]` belongs on scalar operators, constructors, accessors and kernel helpers.
   Large bodies such as the eigen solver are not forced inline. Any hot-path inlining decision is
   backed by a snapshot delta. Consumers that need smaller bytecode can wrap call sites in their
   own `#[inline(never)]` functions; the library does not ship duplicate APIs.
4. Bitwise operators are forbidden. Masks and shifts use `DivRem` by a constant power of two.
   Powers of two are never computed at runtime.
5. Tables use `const [T; N]` plus `.span()`. Small dispatch uses `match`, not if-chains.
6. Multiplication operands stay at 64 bits or below. `u128` multiplication, `u256` and
   felt-to-integer conversions are excluded from hot paths unless measured.
7. Operators are the plain panicking variants, never wrapping, checked or saturating substitutes.
8. Use one `DivRem::div_rem` instead of separate division and remainder, and do not recompute
   derived values.
9. When the cheapest formulation is ambiguous, implement and benchmark the candidates, ship the
   winner and retain losing variants under `benches::alt` so the comparison survives compiler
   upgrades.
10. No stubbed success: an unported function does not exist.

Gas accounting has one important wrinkle: inside a function Sierra gas is charged for the most
expensive branch even when a cheaper branch executes, while steps reflect the branch actually
taken. A loop saves both when it exits early. Branching benchmarks therefore carry multiple
inputs and report both gas and steps.

Every public item uses this documentation template:

```cairo
/// Computes the relative pose.
///
/// Mirrors `glamx::Pose3::inv_mul`.
/// #### Panics
/// * `'Fixed: overflow'` if the result does not fit the scalar range.
/// #### Deviations
/// * None.
```

The `#### Deviations` section is semantic only. Gas notes and inlining choices do not belong
there.

## 5. Tests and gas tracking

- Correctness tests live in `packages/glamx/tests/test_<module>.cairo`. Golden vectors come from
  glamx 0.3.1 and glam-rs 0.33.8 through `tools/refgen`, with inputs quantized to Q32.32 before
  both implementations run. Tolerances are explicit raw-ULP budgets and are justified in the
  spec; a mismatch is investigated rather than hidden by widening a tolerance.
- Tests include zero, one, negative and extreme magnitudes, deterministic seeded properties and
  `#[should_panic(expected: ...)]` cases with exact messages for every panic path.
- Arithmetic public functions have paired `X__base` / `X__op` tests in
  `packages/benches/tests/bench_<module>.cairo`. Inputs pass through `bb` and outputs through
  `sink`; otherwise the compiler can fold the operation away.
- `scripts/bench.py snapshot` writes `gas/<module>.snap` with `l2_gas`, steps and builtin counts.
  `scripts/bench.py check` uses exact equality and CI rejects uncommitted changes.
- `packages/consumer` contains realistic Starknet fixtures; `scripts/bytecode_size.py` tracks
  Sierra and CASM class sizes in `gas/bytecode.size`.
- `scripts/check.sh` is the complete local gate: format, lint, build, glamx tests, gas snapshots,
  bytecode sizes, panic coverage, README gas tables, golden vectors, docs and eigen generator.

## 6. Versioning

Before 1.0, PATCH releases are fixes and performance changes with identical API and identical
numeric results. A public API change or any changed numeric result requires a MINOR release,
because downstream simulation determinism depends on exact outputs.

This repository releases only `glamx`. It consumes published `fixed` and `glam` versions from the
Scarb registry; a MINOR bump in either dependency is reviewed and released here separately, as in
the Rust ecosystem. Cross-repository release sequencing is owned by `glam-cairo`.
