#!/usr/bin/env bash
# Pre-push check: the cheap part of the gate, so that a push does not turn CI red for a reason a
# local run finds in seconds. Usage: scripts/prepush.sh [<sha> [<base>]]
#   <sha>   the commit to check (default HEAD; .githooks/pre-push passes the pushed sha)
#   <base>  the commit changes are measured from (default: merge base of <sha> and origin/main)
#
# Always: formatting, the unit tests of the Python scripts, the cheap `--check` modes of the
# document/generator scripts (including `gen_eigen3.py emit --check`). Only when their inputs
# changed since the base: the golden vectors, and the "heavy" steps (the compile of the touched
# packages and of the workspace packages that depend on them, the lint of the touched packages only,
# the bytecode size). Needs Python >= 3.11.
#
# The heavy steps need the shared heavy-build lock: they wait for it at most 90 s (once, for the
# whole group, `flock -w 90` on the lock file the shim uses). Not obtained: they are skipped with
# one line "heavy lock busy: Cairo compile left to CI" and the script still passes. Where there is
# no lock (no flock or no lock directory, as on the Mac) they always run. fmt, the self-tests and the
# generated-artefact checks that need no build always run.
#
# Left to CI (scripts/check.sh is the full gate, run by CI): the snforge test suites (`snforge
# test -p glamx`, `-p facade_check`), the gas snapshots (`bench.py check`), `scarb doc`, the Rust
# unit tests of tools/refgen, the consumer-cost measure, and a lint seen only in a package that
# depends on a touched one (`scarb lint --workspace` in CI), and the heavy steps when the lock is
# busy for 90 s.
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
  if ! base=$(git merge-base "$sha" origin/main 2>/dev/null); then
    echo "prepush: no merge base of ${sha:0:12} with origin/main (run 'git fetch origin', or pass" >&2
    echo "  the base explicitly: scripts/prepush.sh <sha> <base>): refusing to guess an empty change set." >&2
    exit 1
  fi
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

# heavy_group: the steps that need the heavy-build lock (the Cairo compile, the lint of the touched
# packages, bytecode_size.py). Runs with the lock held (see below) or, where there is no lock, as is.
# Inputs, exported as strings: H_COMPILE_CMD (build|check), H_COMPILE_ARGS, H_LINT_ARGS, H_BYTECODE.
heavy_group() {
  # shellcheck disable=SC2086
  if [[ -n "$H_COMPILE_ARGS" ]]; then
    echo "prepush: Cairo inputs changed; compiling: $H_COMPILE_ARGS"
    step "scarb $H_COMPILE_CMD $H_COMPILE_ARGS" scarb "$H_COMPILE_CMD" $H_COMPILE_ARGS
    step "scarb lint $H_LINT_ARGS --test --deny-warnings" \
      scarb lint $H_LINT_ARGS --test --deny-warnings
  fi
  if [[ "$H_BYTECODE" == 1 ]]; then
    step "bytecode_size.py check" python3 scripts/bytecode_size.py check
  fi
}
export -f step heavy_group

# run_heavy: where the lock file's directory and `flock` exist (the VPS), take the lock ONCE for
# the whole group with `flock -w 90` (the shim, called through PATH, sees the ancestor holding it and
# does not wait again). Not obtained in 90 s: skip the group, print one line, pass. Where there is
# no lock (no flock or no lock directory, as on the Mac) the group always runs.
ancestor_holds_lock() { # as the shim checks it
  local p=$PPID
  while [[ -n "$p" && "$p" -gt 1 ]] 2>/dev/null; do
    if ls -l /proc/"$p"/fd 2>/dev/null | grep -qF -- "$LOCK"; then return 0; fi
    p=$(awk '{print $4}' /proc/"$p"/stat 2>/dev/null) || return 1
  done
  return 1
}

heavy_skipped=0
run_heavy() {
  if [[ -n "${HEAVY_BUILD_LOCK_HELD:-}" ]] || ancestor_holds_lock; then
    heavy_group # the lock is already held by a caller: no wait
  elif [[ -d "$(dirname "$LOCK")" ]] && command -v flock >/dev/null 2>&1; then
    local acq t_start t_end t_acq rc=0
    acq=$(mktemp)
    t_start=$(date +%s.%N)
    flock -w "${PREPUSH_LOCK_WAIT:-90}" -E 75 "$LOCK" \
      bash -c 'date +%s.%N >"$1"; set -euo pipefail; heavy_group' _ "$acq" || rc=$?
    t_end=$(date +%s.%N)
    if [[ $rc -eq 75 && ! -s "$acq" ]]; then
      lock_wait_total=$(awk -v a="$t_start" -v e="$t_end" 'BEGIN { printf "%.1f", e - a }')
      echo "heavy lock busy: Cairo compile left to CI (waited ${lock_wait_total} s)"
      heavy_skipped=1
      rm -f "$acq"
      return 0
    fi
    t_acq=$(<"$acq")
    rm -f "$acq"
    lock_wait_total=$(awk -v a="$t_acq" -v s="$t_start" 'BEGIN { printf "%.1f", a - s }')
    echo "prepush: lock wait ${lock_wait_total} s; work $(awk -v a="$t_acq" -v e="$t_end" 'BEGIN { printf "%.1f", e - a }') s (lock taken at $(date -d "@${t_acq%.*}" +%H:%M:%S))"
    return "$rc"
  fi
  heavy_group
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

# Package names of the touched packages (lint) and of those plus the workspace packages that
# depend on them transitively (compile). Computed from the Scarb.toml files, so a new package needs no edit here.
affected_pkgs=""
touched_names=""
if [[ $workspace_wide -eq 0 && -n "$touched_pkgs" ]]; then
  pyout=$(python3 - $touched_pkgs <<'PY'
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
print(" ".join(sorted(touched)))
print(" ".join(sorted(out)))
PY
  )
  touched_names=$(sed -n 1p <<<"$pyout")
  affected_pkgs=$(sed -n 2p <<<"$pyout")
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

# Always run (about 1 s): its inputs span packages/glamx/ and packages/benches/src/alt/eigen3.cairo.
step "gen_eigen3.py emit --check" python3 scripts/gen_eigen3.py emit --check

if touches '^(tools/refgen/|packages/glamx/tests/golden_)'; then
  if command -v cargo >/dev/null 2>&1; then
    step "refgen check (golden vectors)" \
      cargo run --quiet --locked --manifest-path tools/refgen/Cargo.toml -- check
  else
    echo "==> cargo not found: skipping the golden vector check"
  fi
fi

compile_args=()
lint_args=()
if [[ $workspace_wide -eq 1 ]]; then
  compile_args=(--workspace)
  lint_args=(--workspace)
elif [[ -n "$affected_pkgs" ]]; then
  for p in $affected_pkgs; do compile_args+=(-p "$p"); done
  for p in $touched_names; do lint_args+=(-p "$p"); done
fi

export H_COMPILE_CMD="${PREPUSH_COMPILE:-build}"
export H_COMPILE_ARGS="${compile_args[*]:-}" H_LINT_ARGS="${lint_args[*]:-}" H_BYTECODE=0
if touches '^(packages/(glamx|consumer)/|Scarb\.(toml|lock)|\.tool-versions|scripts/bytecode_size\.py|gas/bytecode\.size)'; then
  H_BYTECODE=1
fi
if [[ -n "$H_COMPILE_ARGS" || "$H_BYTECODE" == 1 ]]; then
  run_heavy
else
  echo "prepush: no Cairo source, manifest or toolchain file changed: skipping compile and lint"
fi

if [[ $heavy_skipped -eq 1 ]]; then
  echo "prepush: passed (heavy steps left to CI) in $((SECONDS - total_start)) s"
else
  echo "prepush: all checks passed in $((SECONDS - total_start)) s (lock wait ${lock_wait_total} s)"
fi
