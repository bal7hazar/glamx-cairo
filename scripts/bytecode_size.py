#!/usr/bin/env python3
"""Compiled class size of the `consumer` contract fixtures (packages/consumer) against the Starknet
limits, and the bytecode cost of the heaviest library items per call site.

usage:
  scripts/bytecode_size.py [table]    build packages/consumer (release), print the size table
  scripts/bytecode_size.py snapshot   same, then write gas/bytecode.size
  scripts/bytecode_size.py check      same, then diff against gas/bytecode.size; exit 1 on ANY difference
  scripts/bytecode_size.py attribution [--strategy S ...] [--only ITEM ...]
      builds, in a temporary package outside the workspace, the consumer fixtures plus one probe
      contract per (item, number of call sites) and prints the marginal CASM / Sierra felts of the
      first and of each further call site. `--strategy` (repeatable: `default`, `avoid` or a
      number) sets the compiler's `inlining-strategy` of the temporary package; the fixture table
      is printed for every strategy as well.

Measured quantities, per contract (see the `LIMITS` block for their source):
  sierra_felts  length of `sierra_program` in `*.contract_class.json`
  casm_felts    length of `bytecode` in `*.compiled_contract_class.json`
  sierra_bytes  compact JSON of the class as the gateway serializes it (`sierra_program`,
                `contract_class_version`, `entry_points_by_type`, `abi` as a string; without the
                debug info, which a declare transaction does not carry)
  casm_bytes    compact JSON of the compiled class
The build is deterministic for a given toolchain (`.tool-versions`), so the check uses equality.

Attribution protocol: a probe contract has two entry points `a` and `b` with the same signature.
Variant 0 returns a free expression of the inputs from both, variant 1 calls the item in `a` only,
variant 2 in `a` and `b`. `1st use = v1 - v0`, `each further use = v2 - v1`. An inlined body is
paid at every call site; a non-inlined one once, plus the call.

Dependency free (Python 3 standard library only).
"""
import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACKAGE = "consumer"
SNAPSHOT = ROOT / "gas" / "bytecode.size"
METRICS = ["sierra_felts", "casm_felts", "sierra_bytes", "casm_bytes"]
HEADER = "# contract: " + " ".join(METRICS)

# Starknet limits. Source: https://docs.starknet.io/learn/cheatsheets/chain-info (Starknet v0.14.2
# on Mainnet, v0.14.3 on Sepolia, read 2026-09-21) and the sequencer that enforces them,
# https://github.com/starkware-libs/sequencer at 1c4fa0261847f403fa0e2da81d412a30d56481d9:
# * apollo_gateway `stateless_transaction_validator.rs`: `sierra_program.len()` <=
#   `max_contract_bytecode_size` (81920) and `serde_json::to_string(&contract_class).len()` <=
#   `max_contract_class_object_size` (4089446);
# * apollo_sierra_compilation_config: the Sierra -> CASM compilation fails above
#   `max_bytecode_size` = 80 * 1024 = 81920 CASM felts;
# * apollo_class_manager_config: `max_compiled_contract_class_object_size` = 4089446 bytes.
# The limits change between Starknet versions: they are parameters, update them here.
LIMITS = {
    "sierra_felts": 81920,
    "casm_felts": 81920,
    "sierra_bytes": 4089446,
    "casm_bytes": 4089446,
}

# ---------------------------------------------------------------------------------------------
# Measurement


def build(cwd, args, env_note=""):
    cmd = ["scarb", "--release", "build"] + args
    print(f"$ {' '.join(cmd)}{env_note}", file=sys.stderr)
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if p.returncode != 0:
        out = (p.stdout + p.stderr).splitlines()
        sys.exit("scarb build failed:\n" + "\n".join(out[-60:]))


def compact(obj):
    return json.dumps(obj, separators=(",", ":"))


def measure(target, package):
    """-> {contract_name: {metric: int}} from target/release/<package>.starknet_artifacts.json"""
    artifacts = json.loads((target / f"{package}.starknet_artifacts.json").read_text())
    res = {}
    for c in artifacts["contracts"]:
        sierra = json.loads((target / c["artifacts"]["sierra"]).read_text())
        casm = json.loads((target / c["artifacts"]["casm"]).read_text())
        declared = {
            "sierra_program": sierra["sierra_program"],
            "contract_class_version": sierra["contract_class_version"],
            "entry_points_by_type": sierra["entry_points_by_type"],
            "abi": compact(sierra["abi"]),
        }
        res[c["contract_name"]] = {
            "sierra_felts": len(sierra["sierra_program"]),
            "casm_felts": len(casm["bytecode"]),
            "sierra_bytes": len(compact(declared)),
            "casm_bytes": len(compact(casm)),
        }
    return res


def print_table(rows, title):
    print(f"\n### {title}\n")
    print("| contract | Sierra felts | CASM felts | CASM / limit | Sierra class bytes "
          "| CASM class bytes | bytes / limit |")
    print("|---|--:|--:|--:|--:|--:|--:|")
    for name, r in sorted(rows.items(), key=lambda kv: kv[1]["casm_felts"]):
        felts = max(r["casm_felts"] / LIMITS["casm_felts"], r["sierra_felts"] / LIMITS["sierra_felts"])
        size = max(r["sierra_bytes"] / LIMITS["sierra_bytes"], r["casm_bytes"] / LIMITS["casm_bytes"])
        print(f"| `{name}` | {r['sierra_felts']:,} | {r['casm_felts']:,} | {100 * felts:.1f} % "
              f"| {r['sierra_bytes']:,} | {r['casm_bytes']:,} | {100 * size:.1f} % |")
    print(f"\nLimits: {LIMITS['sierra_felts']:,} Sierra felts, {LIMITS['casm_felts']:,} CASM felts, "
          f"{LIMITS['sierra_bytes']:,} bytes per class object (`scripts/bytecode_size.py`, `LIMITS`).")


def read_snapshot():
    snap = {}
    if not SNAPSHOT.exists():
        return snap
    for line in SNAPSHOT.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        name, vals = line.split(":", 1)
        snap[name] = dict(zip(METRICS, map(int, vals.split())))
    return snap


def write_snapshot(rows):
    lines = [HEADER] + [f"{n}: " + " ".join(str(rows[n][m]) for m in METRICS) for n in sorted(rows)]
    SNAPSHOT.write_text("\n".join(lines) + "\n")
    print(f"wrote {SNAPSHOT.relative_to(ROOT)}", file=sys.stderr)


def fixtures():
    build(ROOT, ["-p", PACKAGE])
    return measure(ROOT / "target" / "release", PACKAGE)


# ---------------------------------------------------------------------------------------------
# Attribution probes
#
# (name, inline(always) today, params, return type, free expression, call expression)
# `inline` is the attribute on the public item itself (its helpers may differ); verify it in the
# source when an item moves. The free expression builds the return type from the inputs without
# any library arithmetic, so that it cancels out of the difference.

ITEMS = [
    ("Rot2 from_angle", True, "a: Fixed", "Rot2", "Rot2 { re: a, im: a }", "Rot2Trait::from_angle(a)"),
    ("Pose2 inv_mul", True, "p: Pose2, r: Pose2", "Pose2", "p", "p.inv_mul(r)"),
    ("Pose3 mul", True, "p: Pose3, r: Pose3", "Pose3", "p", "p * r"),
    ("Pose3 inv_mul", True, "p: Pose3, r: Pose3", "Pose3", "p", "p.inv_mul(r)"),
    ("Pose3 transform_point", True, "p: Pose3, v: Vec3", "Vec3", "v", "p.transform_point(v)"),
    ("SdpMatrix3 from_rotated_diagonal", False, "q: Quat, d: Vec3", "SdpMatrix3",
     "SdpMatrix3Trait::new(d.x, d.y, d.z, q.x, q.y, q.z)",
     "SdpMatrix3Trait::from_rotated_diagonal(q, d)"),
    ("SymmetricEigen3 new", False, "m: Mat3", "SymmetricEigen3",
     "SymmetricEigen3 { eigenvalues: m.x_axis, eigenvectors: m }", "SymmetricEigen3Trait::new(m)"),
]

PROBE_PRELUDE = """\
    use fixed::Fixed;
    use glam_core::{Mat3, Quat, Vec3};
    use glamx::{
        Pose2, Pose2Trait, Pose3, Pose3Trait, Rot2, Rot2Trait, SdpMatrix3, SdpMatrix3Trait,
        SymmetricEigen3, SymmetricEigen3Trait,
    };
"""


def probe_id(i, uses):
    return f"Probe{i:02d}x{uses}"


def probe_module(i, item, uses):
    _, _, params, ret, free, call = item
    body_a = call if uses >= 1 else free
    body_b = call if uses >= 2 else free
    return (
        "#[starknet::contract]\n"
        f"pub mod {probe_id(i, uses)} {{\n{PROBE_PRELUDE}\n"
        "    #[storage]\n    struct Storage {}\n\n"
        f"    #[external(v0)]\n    fn a(self: @ContractState, {params}) -> {ret} {{\n        {body_a}\n    }}\n\n"
        f"    #[external(v0)]\n    fn b(self: @ContractState, {params}) -> {ret} {{\n        {body_b}\n    }}\n"
        "}\n"
    )


def temp_package(work, strategy, items):
    """Consumer fixtures + probes as a standalone package (its own workspace) in `work`."""
    src = work / "src"
    shutil.copytree(ROOT / "packages" / PACKAGE / "src", src)
    deps = (
        'fixed = "0.4.0"\n'
        'glam_core = "0.4.1"\n'
        f'glamx = {{ path = "{(ROOT / "packages" / "glamx").as_posix()}" }}'
    )
    (work / "Scarb.toml").write_text(
        f'[package]\nname = "{PACKAGE}"\nversion = "0.1.0"\nedition = "2024_07"\n\n'
        f'[dependencies]\n{deps}\nstarknet = "2.20.0"\n\n'
        "[[target.starknet-contract]]\nsierra = true\ncasm = true\n\n"
        f"[profile.release.cairo]\ninlining-strategy = {strategy}\n"
    )
    probes = "".join(probe_module(i, item, u) + "\n" for i, item in items for u in range(3))
    (src / "probes.cairo").write_text(probes)
    with (src / "lib.cairo").open("a") as f:
        f.write("pub mod probes;\n")


def attribution(strategies, only):
    items = [(i, it) for i, it in enumerate(ITEMS) if not only or any(o in it[0] for o in only)]
    for strategy in strategies:
        toml_value = strategy if strategy.isdigit() else f'"{strategy}"'
        with tempfile.TemporaryDirectory(prefix="bytecode_size_") as tmp:
            work = Path(tmp)
            temp_package(work, toml_value, items)
            build(work, [], f"  (inlining-strategy = {toml_value}, {len(items) * 3} probes)")
            rows = measure(work / "target" / "release", PACKAGE)
        fixtures_rows = {n: r for n, r in rows.items() if not n.startswith("Probe")}
        print_table(fixtures_rows, f"Fixtures, inlining-strategy = {toml_value}")
        print(f"\n### Marginal cost per call site, inlining-strategy = {toml_value}\n")
        print("| item | CASM felts, 1st use | CASM felts, each further use | Sierra felts, 1st use "
              "| Sierra felts, each further use | `inline(always)` today |")
        print("|---|--:|--:|--:|--:|:-:|")
        for i, item in items:
            v = [rows[probe_id(i, u)] for u in range(3)]
            c1, c2 = (v[1]["casm_felts"] - v[0]["casm_felts"], v[2]["casm_felts"] - v[1]["casm_felts"])
            s1, s2 = (v[1]["sierra_felts"] - v[0]["sierra_felts"],
                      v[2]["sierra_felts"] - v[1]["sierra_felts"])
            print(f"| `{item[0]}` | {c1:,} | {c2:,} | {s1:,} | {s2:,} | {'yes' if item[1] else 'no'} |")


# ---------------------------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", nargs="?", default="table",
                    choices=["table", "snapshot", "check", "attribution"])
    ap.add_argument("--strategy", action="append",
                    help="attribution: inlining-strategy (default, avoid or a number); repeatable")
    ap.add_argument("--only", action="append", help="attribution: items whose name contains this")
    a = ap.parse_args()

    if a.cmd == "attribution":
        attribution(a.strategy or ["default"], a.only)
        return

    rows = fixtures()
    print_table(rows, "Consumer fixtures (release profile)")
    if a.cmd == "snapshot":
        write_snapshot(rows)
    elif a.cmd == "check":
        snap, bad = read_snapshot(), []
        for name in sorted(set(rows) | set(snap)):
            new, old = rows.get(name), snap.get(name)
            if new != old:
                bad.append(f"{name}: {old} -> {new}")
        if bad:
            print("\n".join(bad), file=sys.stderr)
            sys.exit(f"bytecode size mismatch ({len(bad)}). Run `scripts/bytecode_size.py snapshot` "
                     "and commit gas/bytecode.size.")
        print(f"\nbytecode size snapshot OK ({len(rows)} contracts)", file=sys.stderr)


if __name__ == "__main__":
    main()
