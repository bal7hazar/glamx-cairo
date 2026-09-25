# AGENTS.md - canonical agent instructions

## Mission

Port [Dimforge glamx](https://github.com/dimforge/glamx) 0.3.1 to pure Cairo as a deterministic,
gas-efficient, provable physics-math extension of the `fixed` and `glam` packages. Parry and
Rapier depend on this API.

Read `docs/DESIGN.md` before writing code. Cross-repository sequencing and porter briefs live in
[`glam-cairo`](https://github.com/bal7hazar/glam-cairo/tree/main/docs), not this repository.

## Repository map

| path | content |
|---|---|
| `packages/glamx` | `Rot2`, `Rot3`, `Pose2/3`, `SdpMatrix2/3`, `SymmetricEigen3` |
| `packages/benches` | unpublished benchmark harness and losing glamx variants |
| `packages/consumer` | unpublished Starknet bytecode-size fixtures using glamx |
| `gas/*.snap` | committed gas/step snapshots, one per glamx benchmark module |
| `tools/refgen` | glamx 0.3.1 golden-vector specs and Rust oracles |
| `scripts/check.sh` | the full quality gate |
| `docs/DESIGN.md` | numeric, API and implementation decisions |

`fixed = "0.3.0"` and `glam = "0.3.0"` come from the Scarb registry. Their source and design
documents live in `fixed-cairo` and `glam-cairo`.

## Commands

| task | command |
|---|---|
| Full gate | `scripts/check.sh` |
| Format | `scarb fmt --workspace` |
| Lint | `scarb lint --workspace --test --deny-warnings` |
| Test glamx | `snforge test -p glamx` |
| Generate one golden module | `cargo run --manifest-path tools/refgen/Cargo.toml -- gen <module>` |
| Bench one module | `scripts/bench.py run bench_pose3` |
| Update one snapshot | `scripts/bench.py snapshot bench_pose3` |
| Check all snapshots | `scripts/bench.py check` |
| Check bytecode fixtures | `scripts/bytecode_size.py check` |

Toolchain versions live in `.tool-versions` only.

## Principles

1. Correctness first, then gas. Never optimize untested code.
2. Measure every performance claim with both `l2_gas` and steps from `gas/*.snap`.
3. When the cheapest formulation is ambiguous, bench the variants, ship the winner and retain
   the losers in `benches::alt`.
4. Determinism is API: numeric results are bit-exact and numeric changes require a MINOR bump.
5. Names mirror glamx 0.3.1. Deviations are documented, never silent.
6. No stubbed success: an unported function does not exist.
7. Keep tasks scoped to one module; do not make drive-by refactors.

## Work protocol

1. Read the task brief in `glam-cairo`, this file, `docs/DESIGN.md`, the upstream source and the
   neighbouring glamx modules.
2. Plan the API, tests, `fixed::wide` kernels and open questions. Escalate contradictions rather
   than guessing.
3. Implement source, tests, docs, benches and snapshots in that order.
4. Run `scripts/check.sh` until green.
5. Use a conventional commit scoped to the module and open a pull request; do not merge it.

## Definition of done

- Every public item has the glamx name and the documentation template (`Mirrors`, `#### Panics`,
  `#### Deviations`).
- Tests include golden vectors, zero/one/negative/extreme edge cases, seeded properties and exact
  `#[should_panic(expected: ...)]` messages for every panic path.
- Every public arithmetic function has an `X__base` / `X__op` benchmark with `bb` inputs and a
  `sink` result; affected snapshots are regenerated.
- `scripts/check.sh` is green.
- Any semantic deviation is documented on the item; changes to `docs/DESIGN.md` are escalated.

## Cairo rules

- Value types are `Copy` structs of named scalar fields, passed by value. Do not use `Array`,
  `Span`, dictionaries or loops for fixed-size math.
- Products use `fixed::wide` fused kernels with one rescale per output scalar. Request a missing
  kernel from `fixed-cairo`; do not emulate it with chains of `Fixed * Fixed`.
- Put `#[inline(always)]` on scalar operators, constructors, accessors and small kernel helpers,
  not large bodies or hot generic free functions.
- Do not use bitwise operators. Use `DivRem` by a constant power of two and never compute powers
  of two at runtime.
- Use `const [T; N]` plus `.span()` for tables and `match` for dispatch, never if-chains.
- Keep multiplication operands at 64 bits or below; avoid `u128` multiplication, `u256` and
  felt-to-integer conversions in hot paths unless a benchmark proves the choice.
- Use plain panicking operators, and one `DivRem::div_rem` instead of separate `/` and `%`.
- Bench inputs go through `bb` and results through `sink`; otherwise constants fold away.

## Escalation

Escalate when a brief contradicts glamx or `docs/DESIGN.md`, an API cannot be expressed in Cairo,
a shared file must change, or `fixed` lacks a required kernel. Include the blocker, options and a
recommendation. Scalar issues belong in `fixed-cairo`; cross-repository sequencing belongs in
`glam-cairo`.
