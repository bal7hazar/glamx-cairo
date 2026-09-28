#!/usr/bin/env python3
"""What a published Cairo crate costs an empty consumer, checked against size gates.

The rule (package granularity, programme decision 2026-09-28): a published crate has

  * at most 40,000 library lines (inline tests excluded);
  * an EMPTY consumer of the crate alone (one trivial function that does not name the
    dependency) that adds at most 5 s and 1 GB (peak RSS, cold build) to the no-dependency build;
  * a documented "typical closure" per product (the crates a consumer of that product pulls in
    together) that stays under 15 s and 3 GB added.

The script is repository-agnostic (no crate name, no path): it only needs `scarb` on the PATH and
a workspace (or `--manifest-path`). Python 3.8+, standard library only.

Definitions
  published    a workspace package whose manifest does not say `publish = false` (or the packages
               named with `--package NAME`, repeatable).
  lines        physical lines (`wc -l`: blank lines and comments COUNT, the gate uses this number) of
               the `.cairo` files reachable from the `lib` target's root by following the `mod x;`
               declarations, minus test-only code: files declared under a test-only `#[cfg(...)]`
               (`test`, `and(test, ..)`, `or(test-only, ..)`), test-only items and
               `#[cfg(test)] mod x { .. }` blocks inside files (with the doc comments and
               attributes right above). Code behind Scarb features (`#[cfg(feature: 'x')]`) is
               counted: the gate is about the crate, not one configuration. The non-blank count is
               reported too (`--lines-metric nonblank` gates on it instead).
  added cost   wall seconds and peak RSS (GB = 10^9 bytes) of `scarb build` of a consumer crate
               minus the same for a baseline consumer with no dependency; each one cold
               (`SCARB_INCREMENTAL=false`, fresh `target/`, dependencies fetched beforehand) in a
               temporary directory, one after the other, `--repeat N` takes the median. The consumer
               has the same `[lib]` targets as the crate (read from its manifest) and depends on it
               by path, with default features, or with `--features a,b` / `--no-default-features`.
               The wall time includes any wait of a wrapper around `scarb` (a shared-build lock,
               for instance): run under that lock instead (`flock LOCK python3 ...`).
  closure      `--closure NAME=crate1,crate2,..` (repeatable) or a `[closures]` table of the
               config file: one consumer depending on all the listed crates, checked against the
               closure budget.

Usage
  python3 scripts/consumer_cost.py --lines-only --report-only        # fast proxy, no build
  python3 scripts/consumer_cost.py --json consumer_cost.json         # gates, exit 1 on failure
  python3 scripts/consumer_cost.py --package a --features fast --repeat 3
  python3 scripts/consumer_cost.py --closure product=a,b --report-only --modules

Config (`consumer_cost.toml` at the workspace root, or `--config PATH`; flags win; every key optional)
  [gates]     max_lines = 40000, max_seconds = 5, max_gb = 1, closure_seconds = 15, closure_gb = 3
  [closures]  product = ["crate1", "crate2"]
  [crates.NAME]   features = ["a"], default_features = false   # configuration to measure

Exit status: 1 if any gate fails (unless `--report-only`), 2 on a usage or build error.
Output: a markdown table on stdout (crate, version, lines, added s, added GB, verdict), the JSON
with `--json PATH` (raw measures, medians, per top-level module lines, gates, verdicts).
"""

import argparse
import json
import os
import re
import resource
import shutil
import statistics
import subprocess
import sys
import tempfile
import time

try:
    import tomllib
except ImportError:  # Python < 3.11: the manifests are read with a minimal fallback
    tomllib = None

DEFAULT_GATES = {
    "max_lines": 40_000,
    "max_seconds": 5.0,
    "max_gb": 1.0,
    "closure_seconds": 15.0,
    "closure_gb": 3.0,
}
GB = 1e9


# --------------------------------------------------------------------------------------------
# Manifests
# --------------------------------------------------------------------------------------------


def read_toml(path):
    """The manifest as a dict; without `tomllib`, only what the script needs (best effort)."""
    with open(path, "rb") as f:
        raw = f.read()
    if tomllib is not None:
        return tomllib.loads(raw.decode())
    text = raw.decode()
    out, table = {}, None
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        m = re.match(r"^\[([\w.-]+)\]$", line)
        if m:
            table = out.setdefault(m.group(1), {})
        elif table is not None and "=" in line:
            k, v = (s.strip() for s in line.split("=", 1))
            table[k] = v.strip('"')
    return out


def run(cmd, cwd=None, env=None, check=True):
    p = subprocess.run(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if check and p.returncode != 0:
        sys.exit(f"error: {' '.join(cmd)} failed ({p.returncode}):\n{p.stderr[-2000:]}")
    return p


def workspace_packages(manifest_path):
    cmd = ["scarb", "metadata", "--format-version", "1", "--no-deps"]
    if manifest_path:
        cmd += ["--manifest-path", manifest_path]
    meta = json.loads(run(cmd).stdout)
    pkgs = []
    for p in meta["packages"]:
        manifest = read_toml(p["manifest_path"])
        lib = [t for t in p["targets"] if t["kind"] == "lib"]
        pkgs.append(
            {
                "name": p["name"],
                "version": p["version"],
                "edition": p.get("edition"),
                "root": p["root"],
                "manifest": manifest,
                "publish": manifest.get("package", {}).get("publish", True) is not False,
                "features": sorted(manifest.get("features", {})),
                "lib_source": lib[0]["source_path"] if lib else None,
                "lib_table": manifest.get("lib"),
            }
        )
    return meta["workspace"]["root"], pkgs


# --------------------------------------------------------------------------------------------
# Lines
# --------------------------------------------------------------------------------------------


def mask(text):
    """`text` with comments and string / short-string contents blanked (newlines kept)."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c == "/" and text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            if j < n and text[j] == c:
                j += 1
            out.append(c + "".join("\n" if ch == "\n" else " " for ch in text[i + 1 : j - 1]) + c)
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def split_args(s):
    args, depth, cur = [], 0, ""
    for c in s:
        if c == "," and depth == 0:
            args.append(cur.strip())
            cur = ""
            continue
        depth += c == "("
        depth -= c == ")"
        cur += c
    if cur.strip():
        args.append(cur.strip())
    return args


def test_only(expr):
    """Is the cfg expression false whenever `test` is off (whatever the features)?"""
    expr = expr.strip()
    if expr == "test":
        return True
    m = re.match(r"^(and|or|not)\s*\((.*)\)$", expr, re.S)
    if not m:
        return False
    args = [test_only(a) for a in split_args(m.group(2))]
    return {"and": any(args), "or": bool(args) and all(args), "not": False}[m.group(1)]


def match_close(text, i, open_c, close_c):
    """Index of the bracket closing the one at `text[i]`, or -1."""
    depth = 0
    for j in range(i, len(text)):
        depth += text[j] == open_c
        depth -= text[j] == close_c
        if depth == 0:
            return j
    return -1


def item_end(m, i):
    """Index just after the item starting at `m[i]` (`;` or the closing `}` of its body)."""
    while i < len(m) and (m.startswith("#[", i) or m[i].isspace()):
        i = match_close(m, i + 1, "[", "]") + 1 if m[i] == "#" else i + 1
    is_use = re.match(r"(pub(\([\w:]+\))?\s+)?use\b", m[i:i + 32]) is not None
    depth = 0
    for j in range(i, len(m)):
        c = m[j]
        if c in "([{":
            depth += 1
        elif c in ")]":
            depth -= 1
        elif c == "}":
            depth -= 1
            if depth == 0 and not is_use:
                return j + 1
        elif c == ";" and depth == 0:
            return j + 1
    return len(m)


def analyse_file(path):
    """(raw lines, non-blank lines, child module names, excluded line count, non-blank excluded)."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    m = mask(text)
    lines = text.split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    excluded = set()
    starts = [0]
    for k, c in enumerate(m):
        if c == "\n":
            starts.append(k + 1)

    def line_of(pos):
        lo, hi = 0, len(starts) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if starts[mid] <= pos:
                lo = mid
            else:
                hi = mid - 1
        return lo

    for a in re.finditer(r"#\[cfg\(", m):
        close = match_close(m, a.start() + 1, "[", "]")
        if close < 0 or not test_only(m[a.end() : close - 1]):
            continue
        first = line_of(a.start())
        while first > 0 and re.match(r"\s*(///|#\[)", lines[first - 1]):  # docs and attributes above
            first -= 1
        last = line_of(max(item_end(m, close + 1) - 1, close))
        excluded.update(range(first, last + 1))
    kept = "\n".join(l for k, l in enumerate(m.split("\n")) if k not in excluded)
    mods = [
        (x.group(1))
        for x in re.finditer(r"^[ \t]*(?:pub(?:\([\w:]+\))?[ \t]+)?mod[ \t]+(\w+)[ \t]*;", kept, re.M)
    ]
    nonblank = lambda ls: sum(1 for l in ls if l.strip())
    kept_lines = [l for k, l in enumerate(lines) if k not in excluded]
    return len(kept_lines), nonblank(kept_lines), mods, len(excluded), nonblank(
        l for k, l in enumerate(lines) if k in excluded
    )


def module_file(parent, name):
    """The file of `mod name;` declared in `parent` (Cairo: `parent_dir/[parent_stem/]name.cairo`)."""
    d, base = os.path.split(parent)
    stem = os.path.splitext(base)[0]
    dirs = [d] if base == "lib.cairo" or base == "mod.cairo" else [os.path.join(d, stem)]
    for dd in dirs:
        for cand in (f"{name}.cairo", os.path.join(name, "lib.cairo"), os.path.join(name, "mod.cairo")):
            p = os.path.join(dd, cand)
            if os.path.isfile(p):
                return p
    return None


def count_lines(root_file):
    """{'raw', 'nonblank', 'test_excluded', 'modules': {top-level module: raw lines}, 'files'}."""
    total = {"raw": 0, "nonblank": 0, "test_excluded": 0, "modules": {}, "files": 0, "missing": []}

    def walk(path, top):
        raw, nonblank, mods, ex, _ = analyse_file(path)
        total["raw"] += raw
        total["nonblank"] += nonblank
        total["test_excluded"] += ex
        total["files"] += 1
        total["modules"][top] = total["modules"].get(top, 0) + raw
        for name in mods:
            child = module_file(path, name)
            if child is None:
                total["missing"].append(f"{name} (declared in {path})")
                continue
            walk(child, top if top != "(root)" or path != root_file else name)

    walk(root_file, "(root)")
    return total


# --------------------------------------------------------------------------------------------
# Measured cost
# --------------------------------------------------------------------------------------------


def toml_str(s):
    return json.dumps(s)


def consumer_manifest(deps, edition, lib_table, name="cost_consumer"):
    """A consumer crate: one trivial function, the dependencies by path (`deps`: list of dicts)."""
    lines = ["[package]", f'name = "{name}"', 'version = "0.1.0"']
    if edition:
        lines.append(f"edition = {toml_str(edition)}")
    if lib_table:
        lines.append("\n[lib]")
        lines += [f"{k} = {json.dumps(v)}" for k, v in lib_table.items()]
    lines.append("\n[dependencies]")
    for d in deps:
        opts = [f"path = {toml_str(d['root'])}"]
        if not d["default_features"]:
            opts.append("default-features = false")
        if d["features"]:
            opts.append("features = " + json.dumps(d["features"]))
        lines.append(f"{d['name']} = {{ {', '.join(opts)} }}")
    return "\n".join(lines) + "\n"


def timed_build(cmd, cwd, env):
    """(wall seconds, peak RSS bytes, returncode, stderr tail) of one command."""
    tfile = os.path.join(cwd, "time.out")
    gnu = os.path.exists("/usr/bin/time")
    full = ["/usr/bin/time", "-v", "-o", tfile] + cmd if gnu else cmd
    t0 = time.monotonic()
    p = subprocess.run(full, cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    wall = time.monotonic() - t0
    rss = None
    if gnu and os.path.exists(tfile):
        with open(tfile) as f:
            m = re.search(r"Maximum resident set size \(kbytes\): (\d+)", f.read())
        rss = int(m.group(1)) * 1024 if m else None
    if rss is None:  # fallback: the peak of the waited-for children (a monotonic maximum)
        rss = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss * 1024
    return wall, rss, p.returncode, p.stderr[-2000:]


def measure(label, manifest, repeat, scarb):
    """Cold `scarb build` of the consumer manifest: (median seconds, median peak bytes, samples)."""
    env = dict(os.environ, SCARB_INCREMENTAL="false")
    secs, rsss = [], []
    with tempfile.TemporaryDirectory(prefix="consumer_cost_") as tmp:
        os.makedirs(os.path.join(tmp, "src"))
        with open(os.path.join(tmp, "Scarb.toml"), "w") as f:
            f.write(manifest)
        with open(os.path.join(tmp, "src", "lib.cairo"), "w") as f:
            f.write("pub fn consumer_cost_answer() -> u32 {\n    42\n}\n")
        p = subprocess.run([scarb, "fetch"], cwd=tmp, env=env, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, text=True)
        if p.returncode != 0:
            sys.exit(f"error: `scarb fetch` of the {label} consumer failed:\n{p.stderr[-2000:]}")
        for k in range(repeat):
            shutil.rmtree(os.path.join(tmp, "target"), ignore_errors=True)
            wall, rss, rc, err = timed_build([scarb, "build"], tmp, env)
            if rc != 0:
                sys.exit(f"error: `scarb build` of the {label} consumer failed:\n{err}")
            print(f"  {label} [{k + 1}/{repeat}]: {wall:.1f} s, "
                  f"{(rss or 0) / GB:.2f} GB", file=sys.stderr)
            secs.append(wall)
            rsss.append(rss if rss is not None else 0)
    return statistics.median(secs), statistics.median(rsss), {"seconds": secs, "rss_bytes": rsss}


# --------------------------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------------------------


def verdict(fails):
    return "FAIL: " + ", ".join(fails) if fails else "ok"


def main():
    ap = argparse.ArgumentParser(description="Consumer cost and size gates of published crates.")
    ap.add_argument("--manifest-path", help="workspace manifest (default: from the current directory)")
    ap.add_argument("--package", action="append", default=[], metavar="NAME",
                    help="crate to check (repeatable); default: the published packages")
    ap.add_argument("--closure", action="append", default=[], metavar="NAME=a,b",
                    help="consumer of several crates against the closure budget (repeatable)")
    ap.add_argument("--features", help="comma-separated features to enable on every measured crate")
    ap.add_argument("--no-default-features", action="store_true")
    ap.add_argument("--repeat", type=int, default=1, help="builds per consumer, the median is kept")
    ap.add_argument("--lines-only", action="store_true", help="count lines only (no build)")
    ap.add_argument("--report-only", action="store_true", help="never fail on a gate")
    ap.add_argument("--modules", action="store_true", help="print the lines per top-level module")
    ap.add_argument("--json", metavar="PATH", help="write the results as JSON")
    ap.add_argument("--config", metavar="PATH", help="config file (default: consumer_cost.toml)")
    ap.add_argument("--scarb", default="scarb", help="scarb executable")
    ap.add_argument("--lines-metric", choices=["raw", "nonblank"], default="raw")
    for k, v in DEFAULT_GATES.items():
        ap.add_argument("--" + k.replace("_", "-"), type=float, default=None,
                        help=f"gate (default {v:g})")
    args = ap.parse_args()

    root, packages = workspace_packages(args.manifest_path)
    cfg_path = args.config or os.path.join(root, "consumer_cost.toml")
    cfg = read_toml(cfg_path) if os.path.exists(cfg_path) and tomllib else {}
    if args.config and not cfg:
        sys.exit(f"error: cannot read the config {args.config}")
    gates = dict(DEFAULT_GATES)
    gates.update(cfg.get("gates", {}))
    gates.update({k: getattr(args, k) for k in DEFAULT_GATES if getattr(args, k) is not None})

    by_name = {p["name"]: p for p in packages}
    names = args.package or [p["name"] for p in packages if p["publish"]]
    for n in names:
        if n not in by_name:
            sys.exit(f"error: no package `{n}` in the workspace")
    closures = {k: list(v) for k, v in cfg.get("closures", {}).items()}
    for spec in args.closure:
        name, _, members = spec.partition("=")
        closures[name] = [m for m in members.split(",") if m]
    for name, members in closures.items():
        for m in members:
            if m not in by_name:
                sys.exit(f"error: closure `{name}`: no package `{m}` in the workspace")

    def dep(name):
        conf = cfg.get("crates", {}).get(name, {})
        feats = args.features.split(",") if args.features else list(conf.get("features", []))
        return {
            "name": name,
            "root": by_name[name]["root"],
            "features": feats,
            "default_features": not args.no_default_features and conf.get("default_features", True),
        }

    metric = args.lines_metric
    line_counts = {}

    def lines_of(name):
        if name not in line_counts:
            line_counts[name] = count_lines(by_name[name]["lib_source"]) if by_name[name]["lib_source"] else None
        return line_counts[name]

    results = []
    for n in names:
        p = by_name[n]
        if not p["lib_source"]:
            print(f"warning: `{n}` has no lib target, skipped", file=sys.stderr)
            continue
        c = lines_of(n)
        for m in c["missing"]:
            print(f"warning: module file not found: {m}", file=sys.stderr)
        results.append({"crate": n, "version": p["version"], "features": p["features"],
                        "lines": c["raw"], "lines_nonblank": c["nonblank"],
                        "test_lines_excluded": c["test_excluded"], "files": c["files"],
                        "modules": c["modules"],
                        "gated_lines": c["raw"] if metric == "raw" else c["nonblank"]})

    baseline = None
    if not args.lines_only:
        lib_tables = [by_name[n]["lib_table"] for n in names]
        lib0 = lib_tables[0] if lib_tables else None
        edition = by_name[names[0]]["edition"] if names else None
        print("baseline consumer (no dependency):", file=sys.stderr)
        bs, br, bsamples = measure("baseline", consumer_manifest([], edition, lib0), args.repeat,
                                   args.scarb)
        baseline = {"seconds": bs, "rss_bytes": br, **bsamples}
        for r in results:
            p = by_name[r["crate"]]
            print(f"consumer of {r['crate']}:", file=sys.stderr)
            s, rss, samples = measure(r["crate"], consumer_manifest([dep(r["crate"])], p["edition"],
                                      p["lib_table"]), args.repeat, args.scarb)
            r.update(added_seconds=s - bs, added_gb=max(rss - br, 0) / GB, samples=samples)
        for cname, members in closures.items():
            print(f"closure {cname}:", file=sys.stderr)
            first = by_name[members[0]]
            s, rss, samples = measure(cname, consumer_manifest([dep(m) for m in members],
                                      first["edition"], first["lib_table"]), args.repeat, args.scarb)
            results.append({"crate": f"closure:{cname}", "version": "-", "members": members,
                            "lines": sum(lines_of(m)["raw"] for m in members if lines_of(m)),
                            "added_seconds": s - bs, "added_gb": max(rss - br, 0) / GB,
                            "samples": samples, "closure": True})

    failed = False
    for r in results:
        fails = []
        if r.get("closure"):
            if r["added_seconds"] > gates["closure_seconds"]:
                fails.append(f"time > {gates['closure_seconds']:g} s")
            if r["added_gb"] > gates["closure_gb"]:
                fails.append(f"memory > {gates['closure_gb']:g} GB")
        else:
            if r["gated_lines"] > gates["max_lines"]:
                fails.append(f"lines > {gates['max_lines']:,.0f}")
            if "added_seconds" in r:
                if r["added_seconds"] > gates["max_seconds"]:
                    fails.append(f"time > {gates['max_seconds']:g} s")
                if r["added_gb"] > gates["max_gb"]:
                    fails.append(f"memory > {gates['max_gb']:g} GB")
        r["verdict"] = verdict(fails)
        failed |= bool(fails)

    table = ["| crate | version | lines | added s | added GB | verdict |", "|---|---|---:|---:|---:|---|"]
    for r in results:
        s = f"{r['added_seconds']:.1f}" if "added_seconds" in r else "-"
        g = f"{r['added_gb']:.2f}" if "added_gb" in r else "-"
        table.append(f"| {r['crate']} | {r['version']} | {r['lines']:,} | {s} | {g} | {r['verdict']} |")
    print("\n".join(table))
    print(f"\nLines: physical lines of the library files, test-only code excluded "
          f"(gate: {metric}, max {gates['max_lines']:,.0f}); added cost = consumer - baseline "
          f"(gates: {gates['max_seconds']:g} s / {gates['max_gb']:g} GB, closures "
          f"{gates['closure_seconds']:g} s / {gates['closure_gb']:g} GB).")
    if args.modules:
        for r in results:
            if "modules" in r:
                print(f"\n{r['crate']}: lines per top-level module")
                for mod, n in sorted(r["modules"].items(), key=lambda kv: -kv[1]):
                    print(f"  {mod:<24}{n:>8,}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"gates": gates, "baseline": baseline, "results": results,
                       "passed": not failed}, f, indent=2)
            f.write("\n")
    return 1 if failed and not args.report_only else 0


if __name__ == "__main__":
    sys.exit(main())
