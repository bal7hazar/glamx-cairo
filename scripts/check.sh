#!/usr/bin/env bash
# Full quality gate. Must be green before any task is reported as done.
set -euo pipefail
cd "$(dirname "$0")/.."

scarb fmt --check --workspace
scarb lint --workspace --test --deny-warnings
scarb build --workspace
# The benches package is not run here: `bench.py check` below runs every bench twice and fails on
# any failing test. The consumer package has no tests; `bytecode_size.py` builds it in release mode.
snforge test -p glamx
snforge test -p facade_check
python3 scripts/bench.py check
# Class size of the packages/consumer contract fixtures (gas/bytecode.size, release build).
python3 scripts/bytecode_size.py check
python3 scripts/panic_coverage.py --check
python3 scripts/gas_tables.py --check
# Golden vectors are up to date with tools/refgen (skipped when the Rust toolchain is absent; CI
# always runs it in the `golden` job).
if command -v cargo >/dev/null 2>&1; then
  cargo run --quiet --locked --manifest-path tools/refgen/Cargo.toml -- check
else
  echo "cargo not found: skipping the golden vector check"
fi
scarb doc --workspace --disable-remote-linking >/dev/null
python3 scripts/gen_eigen3.py emit --check
echo "all checks passed"
