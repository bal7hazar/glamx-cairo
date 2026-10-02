#!/usr/bin/env bash
# Pre-push check: the cheap part of the gate, so that a push does not turn CI red for a reason a
# local run finds in seconds. Usage: scripts/prepush.sh [<sha> [<base>]]
#   <sha>   the commit to check (default HEAD; .githooks/pre-push passes the pushed sha)
#   <base>  the commit changes are measured from (default: merge base of <sha> and origin/main)
#
# Always: formatting, the unit tests of the Python scripts, the cheap `--check` modes of the
# document/generator scripts. Only when their inputs changed since the base: the compile and the
# lint of the touched packages and of the workspace packages that depend on them, the generated
# artefacts (`gen_eigen3.py emit --check`, the bytecode size, the golden vectors).
#
# Left to CI (scripts/check.sh is the full gate, run by CI): the snforge test suites (`snforge
# test -p glamx`, `-p facade_check`), the gas snapshots (`bench.py check`), `scarb doc`, the Rust
# unit tests of tools/refgen and the consumer-cost measure.
#
# `scarb` is always called through PATH: on the shared VPS the shim ~/.local/bin/scarb serialises
# build/lint/check on ~/orchestrator/heavy-build.lock. The wait on that lock is printed apart from
# the work of each step. Never bypass the hook (`--no-verify`) nor the lock.
set -euo pipefail
cd "$(dirname "$0")/.."

sha_arg="${1:-HEAD}"
if ! sha=$(git rev-parse --verify --quiet "${sha_arg}^{commit}"); then
  echo "prepush: '$sha_arg' is not a commit" >&2
  exit 1
fi
head=$(git rev-parse HEAD)
if [[ -n "${2:-}" ]]; then
  base=$(git rev-parse --verify "$2^{commit}")
else
  base=$(git merge-base "$sha" origin/main 2>/dev/null || git rev-parse "$sha")
fi

# The checks run on the working tree but a push sends commits: refuse when they differ.
status=$(git status --porcelain)
if [[ "$head" != "$sha" || -n "$status" ]]; then
  {
    echo "prepush: refusing to check: the working tree is not the commit to push."
    echo "  checked sha: $sha"
    echo "  HEAD:        $head"
    if [[ -n "$status" ]]; then
      echo "  uncommitted or untracked files:"
      sed 's/^/    /' <<<"$status"
    fi
    echo "Commit the changes (or remove them), and check out the commit to push."
    echo "Never use 'git stash' (the stash stack is shared by every worktree and session) and"
    echo "never --no-verify."
  } >&2
  exit 1
fi

changed=$(git diff --name-only "$base" "$sha")
echo "prepush: checking ${sha:0:12} (base ${base:0:12}, $(grep -c . <<<"$changed" || true) changed files)"

LOCK="${HEAVY_BUILD_LOCK:-$HOME/orchestrator/heavy-build.lock}"
total_start=$SECONDS
lock_wait_total=0

# step <name> <cmd...>: unlocked step, prints its duration.
step() {
  local name=$1
  shift
  local t0=$SECONDS
  echo "==> $name"
  "$@"
  echo "    ok: $name ($((SECONDS - t0)) s)"
}

# lstep <name> <cmd...>: step that goes through the shim's heavy-build lock (scarb build, lint,
# check; bytecode_size.py). Where the lock file's directory exists (the VPS) the step runs under
# `flock` so the wait is measured; the shim sees the ancestor holding the lock and does not wait
# again. Elsewhere the command runs as is.
lstep() {
  local name=$1
  shift
  local t0=$SECONDS t_start
  t_start=$(date +%s.%N)
  echo "==> $name"
  if [[ -d "$(dirname "$LOCK")" ]] && command -v flock >/dev/null 2>&1; then
    local acq
    acq=$(mktemp)
    flock "$LOCK" bash -c 'date +%s.%N >"$1"; shift; exec "$@"' _ "$acq" "$@"
    local t_end t_acq
    t_end=$(date +%s.%N)
    t_acq=$(<"$acq")
    rm -f "$acq"
    # wait = acquisition - start; work = end - acquisition
    local wait work
    wait=$(awk -v a="$t_acq" -v s="$t_start" 'BEGIN { w = a - s; if (w < 0) w = 0; printf "%.1f", w }')
    work=$(awk -v a="$t_acq" -v e="$t_end" 'BEGIN { printf "%.1f", e - a }')
    lock_wait_total=$(awk -v t="$lock_wait_total" -v w="$wait" 'BEGIN { printf "%.1f", t + w }')
    echo "    ok: $name (lock wait ${wait} s, work ${work} s)"
  else
    "$@"
    echo "    ok: $name ($((SECONDS - t0)) s, no lock)"
  fi
}

# --- changed-input detection -------------------------------------------------------------------

touches() { grep -Eq "$1" <<<"$changed"; }

# Packages (directories of packages/) with a changed Cairo source, manifest or test file.
touched_pkgs=""
workspace_wide=0
if touches '^(Scarb\.toml|Scarb\.lock|\.tool-versions)$'; then
  workspace_wide=1
fi
while IFS= read -r p; do
  [[ -n "$p" ]] && touched_pkgs+="$p "
done < <(grep -E '^packages/[^/]+/.*(\.cairo|Scarb\.toml|Scarb\.lock)$' <<<"$changed" |
  cut -d/ -f2 | sort -u || true)

# Workspace package names that depend (transitively, by workspace path) on the touched ones, plus
# the touched ones. Computed from the Scarb.toml files, so a new package needs no edit here.
affected_pkgs=""
if [[ $workspace_wide -eq 0 && -n "$touched_pkgs" ]]; then
  affected_pkgs=$(python3 - $touched_pkgs <<'PY'
import sys, tomllib, pathlib
root = pathlib.Path("packages")
names, deps = {}, {}
for m in sorted(root.glob("*/Scarb.toml")):
    d = tomllib.loads(m.read_text())
    names[m.parent.name] = d["package"]["name"]
    deps[m.parent.name] = set(d.get("dependencies", {}))
    deps[m.parent.name] |= set(d.get("dev-dependencies", {}))
touched = {names[p] for p in sys.argv[1:] if p in names}
out = set(touched)
changed = True
while changed:
    changed = False
    for p, ds in deps.items():
        if names[p] not in out and ds & out:
            out.add(names[p])
            changed = True
print(" ".join(sorted(out)))
PY
  )
fi

# --- always ------------------------------------------------------------------------------------

step "scarb fmt --check --workspace" scarb fmt --check --workspace
step "consumer_cost.py --self-test" python3 scripts/consumer_cost.py --self-test
step "packages_table.py --self-test" python3 scripts/packages_table.py --self-test
if [[ -d scripts/tests ]]; then
  step "unittest scripts/tests" python3 -m unittest discover -s scripts/tests -p 'test_*.py'
fi
step "gas_tables.py --check" python3 scripts/gas_tables.py --check
step "panic_coverage.py --check" python3 scripts/panic_coverage.py --check

# --- only when their inputs changed ------------------------------------------------------------

if touches '^(scripts/gen_eigen3\.py|packages/glamx/)'; then
  step "gen_eigen3.py emit --check" python3 scripts/gen_eigen3.py emit --check
fi

if touches '^(tools/refgen/|packages/glamx/tests/golden_)'; then
  if command -v cargo >/dev/null 2>&1; then
    step "refgen check (golden vectors)" \
      cargo run --quiet --locked --manifest-path tools/refgen/Cargo.toml -- check
  else
    echo "==> cargo not found: skipping the golden vector check"
  fi
fi

compile_args=()
if [[ $workspace_wide -eq 1 ]]; then
  compile_args=(--workspace)
elif [[ -n "$affected_pkgs" ]]; then
  for p in $affected_pkgs; do compile_args+=(-p "$p"); done
fi

if [[ ${#compile_args[@]} -gt 0 ]]; then
  echo "prepush: Cairo inputs changed; compiling: ${compile_args[*]}"
  # `scarb build` with several -p flags builds them in one invocation (one lock acquisition).
  lstep "scarb ${PREPUSH_COMPILE:-build} ${compile_args[*]}" \
    scarb "${PREPUSH_COMPILE:-build}" "${compile_args[@]}"
  lstep "scarb lint ${compile_args[*]} --test --deny-warnings" \
    scarb lint "${compile_args[@]}" --test --deny-warnings
else
  echo "prepush: no Cairo source, manifest or toolchain file changed: skipping compile and lint"
fi

if touches '^(packages/(glamx|consumer)/|Scarb\.(toml|lock)|\.tool-versions|scripts/bytecode_size\.py|gas/bytecode\.size)'; then
  lstep "bytecode_size.py check" python3 scripts/bytecode_size.py check
fi

echo "prepush: all checks passed in $((SECONDS - total_start)) s (lock wait ${lock_wait_total} s)"
