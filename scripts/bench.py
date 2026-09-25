#!/usr/bin/env python3
"""Runs the benchmark suite (packages/benches) and maintains the gas snapshots (gas/*.snap).

usage:
  scripts/bench.py run      [FILTER]  run the benches, print a markdown table of net costs
  scripts/bench.py snapshot [FILTER]  same, then (re)write gas/<module>.snap for the modules that ran
  scripts/bench.py check    [FILTER]  same, then diff against gas/*.snap; exit 1 on ANY difference
                                      (with FILTER: only the snapshot entries matching it)

Convention: a benchmark `X` is a pair of tests `X__base` / `X__op` sharing the same prelude; its
cost is op - base for every metric, which removes the test overhead. Inputs MUST go through
`benches::harness::bb` and results through `sink`, otherwise the compiler constant-folds the work.

Two snforge runs are needed: `--tracked-resource sierra-gas` gives l2_gas (what users pay, the
north-star metric) and `--tracked-resource cairo-steps` gives steps and builtins (what provers
pay). Results are deterministic, so the check uses exact equality.

Every bench file `tests/bench_<m>.cairo` is its own snforge test crate (`[[test]]` target in
packages/benches/Scarb.toml, checked by `check_targets`) so unrelated modules are not compiled
into each benchmark program.

One snapshot file per bench module (`bench_pose3.cairo` -> `gas/pose3.snap`) so that parallel pull
requests touching different modules never conflict.
"""
import argparse
import re
import subprocess
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
GAS = ROOT / "gas"
BENCHES = ROOT / "packages" / "benches"
METRICS = ["l2_gas", "steps", "range_check", "bitwise", "other_builtins"]
HEADER = "# bench: " + " ".join(METRICS)
PASS_RE = re.compile(r"^\[(PASS|FAIL)\] (\S+)")


def check_targets():
    """Every tests/bench_*.cairo must be a [[test]] target of packages/benches/Scarb.toml (and
    vice versa), otherwise snforge would silently not run it and its snapshot would be REMOVED."""
    files = {f.stem for f in (BENCHES / "tests").glob("bench_*.cairo")}
    manifest = tomllib.loads((BENCHES / "Scarb.toml").read_text())
    targets = {t["name"]: t for t in manifest.get("test", [])}
    bad = [
        f"bench file without a [[test]] target: tests/{n}.cairo"
        for n in sorted(files - targets.keys())
    ]
    bad += [f"[[test]] target without a bench file: {n}" for n in sorted(targets.keys() - files)]
    bad += [
        f"[[test]] {n}: expected source-path = \"tests/{n}.cairo\" and test-type = \"integration\""
        for n, t in sorted(targets.items())
        if t.get("source-path") != f"tests/{n}.cairo" or t.get("test-type") != "integration"
    ]
    if bad:
        sys.exit("packages/benches/Scarb.toml is out of sync with tests/:\n  " + "\n  ".join(bad))


def run_snforge(mode, flt):
    cmd = ["snforge", "test", "-p", "benches", "--detailed-resources", "--tracked-resource", mode]
    if flt:
        cmd.insert(2, flt)
    print("$", " ".join(cmd), file=sys.stderr)
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    out = p.stdout + p.stderr
    if p.returncode != 0:
        sys.exit(f"snforge failed (mode={mode}):\n" + "\n".join(out.splitlines()[-40:]))
    return out


def parse(out):
    """-> {test_name: {metric: int}}"""
    res, cur = {}, None
    for line in out.splitlines():
        m = PASS_RE.match(line)
        if m:
            cur = res.setdefault(m.group(2).split("::", 1)[1], {})
            continue
        if cur is None:
            continue
        s = line.strip()
        if s.startswith("sierra gas:"):
            cur["l2_gas"] = int(s.split(":")[1])
        elif s.startswith("steps:"):
            cur["steps"] = int(s.split(":")[1])
        elif s.startswith("builtins:"):
            cur["range_check"] = cur["bitwise"] = cur["other_builtins"] = 0
            for name, n in re.findall(r"Builtin\((\w+)\): (\d+)", s):
                key = name if name in ("range_check", "bitwise") else "other_builtins"
                cur[key] += int(n)
    return res


def collect(flt):
    check_targets()
    gas = parse(run_snforge("sierra-gas", flt))
    steps = parse(run_snforge("cairo-steps", flt))
    tests = {}
    for name, g in gas.items():
        s = steps.get(name, {})
        tests[name] = {"l2_gas": g.get("l2_gas", 0)}
        tests[name].update({k: s.get(k, 0) for k in METRICS[1:]})
    rows = {}
    for name, t in tests.items():
        if name.endswith("__op"):
            stem = name[: -len("__op")]
            base = tests.get(stem + "__base")
            if base is None:
                sys.exit(f"missing baseline test for {name}")
            rows[stem] = {k: t[k] - base[k] for k in METRICS}
        elif not name.endswith("__base"):
            sys.exit(f"{name}: bench tests must be named X__base / X__op")
    return rows


def module_of(name):
    return name.split("::", 1)[0].removeprefix("bench_")


def read_snapshots():
    snap = {}
    for path in sorted(GAS.glob("*.snap")):
        for line in path.read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            name, vals = line.rsplit(":", 1)
            snap[name] = dict(zip(METRICS, map(int, vals.split())))
    return snap


def write_snapshots(rows, partial):
    GAS.mkdir(exist_ok=True)
    modules = {}
    for name, r in rows.items():
        modules.setdefault(module_of(name), {})[name] = r
    if not partial:
        for path in GAS.glob("*.snap"):
            if path.stem not in modules:
                path.unlink()
    for mod, entries in modules.items():
        lines = [HEADER] + [
            f"{n}: " + " ".join(str(entries[n][k]) for k in METRICS) for n in sorted(entries)
        ]
        (GAS / f"{mod}.snap").write_text("\n".join(lines) + "\n")
    print(f"wrote {len(modules)} snapshot file(s) in gas/", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "snapshot", "check"])
    ap.add_argument("filter", nargs="?", default="")
    a = ap.parse_args()
    rows = collect(a.filter)
    print("| bench | " + " | ".join(METRICS) + " |\n|---|" + "---:|" * len(METRICS))
    for name in sorted(rows):
        print(f"| `{name}` | " + " | ".join(str(rows[name][k]) for k in METRICS) + " |")
    if a.cmd == "snapshot":
        write_snapshots(rows, partial=bool(a.filter))
    elif a.cmd == "check":
        snap = {n: v for n, v in read_snapshots().items() if a.filter in n}
        bad = []
        for name in sorted(set(rows) | set(snap)):
            new, old = rows.get(name), snap.get(name)
            if old is None:
                bad.append(f"ADDED   {name}")
            elif new is None:
                bad.append(f"REMOVED {name}")
            else:
                bad += [
                    f"CHANGED {name}: {k} {old[k]} -> {new[k]}" for k in METRICS if new[k] != old[k]
                ]
        if bad:
            print("\n" + "\n".join(bad))
            sys.exit(f"gas snapshot mismatch ({len(bad)}). Run `scripts/bench.py snapshot` and commit gas/.")
        print(f"\ngas snapshot OK ({len(rows)} benches)", file=sys.stderr)


if __name__ == "__main__":
    main()
