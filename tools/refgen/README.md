# refgen - glamx golden vectors

`refgen` generates `packages/glamx/tests/golden_<module>.cairo`. It runs the pinned Rust crates
glamx 0.3.1 and glam-rs 0.33.8 as f64 oracles, after quantizing every input to Q32.32, then emits
deterministic Cairo cases with explicit raw-ULP tolerances.

It is a standalone Cargo workspace with a pinned `Cargo.lock`.

```sh
cargo run --manifest-path tools/refgen/Cargo.toml -- gen [module...]
cargo run --manifest-path tools/refgen/Cargo.toml -- check [module...]
cargo run --manifest-path tools/refgen/Cargo.toml -- list
```

The retained modules are `eigen3`, `pose2`, `pose3`, `rot2` and `sdp`. `rot3` is a type alias and
its small hand-authored golden file has no independent Rust oracle.

## How it works

1. An integer-only xoshiro256** PRNG draws raw Q32.32 inputs from domains in
   `specs/<module>.toml`; hand-picked edges are emitted first.
2. Raw inputs convert exactly to f64, the registered oracle in
   `src/oracles/<module>.rs` runs, and representable results are quantized to the nearest raw
   value.
3. NaN, infinity, overflow, failed preconditions and results too large for half-ULP f64 resolution
   are skipped and redrawn. Panic paths are declared explicitly in the spec.
4. The emitter writes formatted, table-driven Cairo tests. A mismatch reports the module,
   function, case, actual raw value, expected raw value and tolerance.

Output is byte-identical for the same spec and oracle on every host.

## Editing a module

Only edit the module's pair:

| file | role |
|---|---|
| `specs/<module>.toml` | Cairo call, argument domains, edge cases and tolerance justification |
| `src/oracles/<module>.rs` | Rust reference computation and custom generators |

Then run:

```sh
cargo run --manifest-path tools/refgen/Cargo.toml -- gen <module>
snforge test -p glamx golden_<module>
scripts/check.sh
```

Do not hand-edit a generated `golden_*.cairo` file. Changes to the shared PRNG, sampling,
quantization or emitter can change every golden file and require orchestrator review.

## Tolerances

Tolerance is measured in raw Q32.32 ULPs and always has a justification in the spec. Exact
constructors and integer-defined operations use 0. A single fused floor rescale normally differs
from the round-to-nearest f64 oracle by at most 1. Chained and conditioned operations derive a
bound from every rescale and from amplification by the input domain. Transcendental bounds include
the documented `fixed` error.

Never widen a tolerance merely to make a mismatch pass. Investigate the numeric path and update
the public item's bound if the result is intentionally changed. See `docs/DESIGN.md` section 5.
