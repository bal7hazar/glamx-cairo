#!/usr/bin/env python3
"""What a published Cairo crate costs an empty consumer, checked against size gates.

The rule (package granularity, programme decision 2026-09-28): a published crate has

  * at most 40,000 library lines (inline tests excluded);
  * a MARGINAL cost of at most 5 s and 1 GB (peak RSS, cold build): an EMPTY consumer of the crate
    (one trivial function that does not name the dependency) minus an empty consumer of the
    crate's DIRECT dependencies together;
  * a declared "typical closure" per product (the crates a consumer of that product pulls in
    together, workspace or registry crates) that stays under 15 s and 3 GB over the no-dependency
    build;
  * a FACADE (a crate that only re-exports others) or a product is judged on the closure budget
    only: its own consumer is checked against 15 s / 3 GB, not against lines and marginal cost.

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
  cost         wall seconds and peak RSS (GB = 10^9 bytes) of `scarb build` of a consumer crate; each one
               cold (`SCARB_INCREMENTAL=false`, fresh `target/`, dependencies fetched beforehand) in
               a temporary directory, one after the other. `--repeat N` builds every consumer N
               times (N rounds; `--interleave`: round k builds every consumer once, in a rotated
               order, so a slow period of the machine hits every consumer alike) and every
               difference below is the MEDIAN OF THE PER-ROUND DIFFERENCES (interquartile range
               shown), memory differences clamped at 0 per round. The consumer
               has the same `[lib]` targets as the crate (read from its manifest) and depends on it
               by path, with default features, or with `--features a,b` / `--no-default-features`.
               The wall time includes any wait of a wrapper around `scarb` (a shared-build lock,
               for instance): run under that lock instead (`flock LOCK python3 ...`). A manifest
               that two consumers share is built once.
  over baseline  cost(consumer of the crate) - cost(baseline consumer with no dependency).
  direct deps  the crate's `[dependencies]` as `scarb metadata` resolves them (workspace
               inheritance applied): workspace path dependencies by path, registry ones by the same
               version requirement, both with the features the crate declares; dev-dependencies,
               `core` and `starknet` excluded.
  marginal     cost(consumer of the crate) - cost(consumer of its direct deps together): what the
               crate adds on top of what it pulls in. Time in seconds, memory in GB (never below 0).
               A crate without dependency: marginal = over baseline. GATE 2 USES THIS FIGURE; the
               over-baseline figures stay in the table and in the JSON.
  facade       a crate judged on the closure budget only (`--facade NAME`, repeatable, or `facades`
               in the config): reported with its lines and marginal cost, its over-baseline cost is
               checked against the closure budget, lines and marginal cost are not gated.
  --report-only-marginals  gate 2 (marginal cost) reported, never gated; lines and closures stay
               gated (a transition state, e.g. while a crate map is being re-cut).
  report only  a crate (`[crates.NAME] report_only = true`) or closure (table form, below) that is
               measured and shown but never gated (verdict `reported (...)`): e.g. a facade that
               fails the closure budget by construction and was accepted as such.
  shard        `--shard K/N` measures the K-th of N shards: the crates and closures are spread by
               a deterministic longest-first assignment weighted by the library lines each one
               builds (its closure plus its direct deps' closure), every shard with its own
               baseline; `--merge a.json b.json ..` combines the shards' `--json` files into one
               table and ONE verdict (CI: one job per shard, one enforcing job).
  closure      `--closure NAME=m1,m2,..` (repeatable) or a `[closures]` table of the config file:
               one consumer depending on all the members, checked against the closure budget. A
               member is a workspace crate by name (`nalgebra_glam`, features from `[crates.NAME]`)
               or a registry crate with its version requirement (`glam@0.4.1`, `fixed@0.4.0`,
               `simba@^0.2`), written as `name = "req"` in the consumer manifest.
  budget       a closure (or facade) is checked against `closure_seconds` / `closure_gb` unless it
               declares its own budget: `budget = { seconds = 20, gb = 4.5 }` in the table form of a
               `[closures]` entry (either key may be omitted: the default applies), or on the command
               line `--closure NAME=a,b::20s,4.5gb` (members, then `::`, then any of `<n>s`, `<n>gb`).
               The budget used is shown per closure in the table (`budget` column) and in the JSON
               (`budget: {seconds, gb}`); a configuration without a budget behaves as before.

Usage
  python3 scripts/consumer_cost.py --lines-only --report-only        # fast proxy, no build
  python3 scripts/consumer_cost.py --json consumer_cost.json         # gates, exit 1 on failure
  python3 scripts/consumer_cost.py --package a --features fast --repeat 3
  python3 scripts/consumer_cost.py --closure product=a,glam@0.4.1 --facade umbrella --report-only
  python3 scripts/consumer_cost.py --repeat 5 --interleave --shard 2/6 --json s2.json
  python3 scripts/consumer_cost.py --merge s1.json s2.json ... --json all.json  # one verdict
  python3 scripts/consumer_cost.py --package a --no-closures --repeat 9 --interleave
  python3 scripts/consumer_cost.py --dry-run          # print the consumer manifests, build nothing
  python3 scripts/consumer_cost.py --self-test        # checks of the script's own logic, no scarb

Config (`consumer_cost.toml` at the workspace root, or `--config PATH`; flags win; every key optional)
  [gates]     max_lines = 40000, max_seconds = 5, max_gb = 1, closure_seconds = 15, closure_gb = 3
  facades = ["umbrella"]                                    # top-level key: judged on the closure budget
  [closures]  product = ["crate1", "registry_crate@1.2.3"]
              lean = { members = ["crate1"], default_features = false, features = [], report_only = true }
              big = { members = ["crate1", "crate2"], budget = { seconds = 20, gb = 4.5 } }
  [crates.NAME]   features = ["a"], default_features = false, report_only = true

Exit status: 1 if any gated row fails (unless `--report-only`), 2 on a usage or build error.
Every `scarb` call passes `--manifest-path` BEFORE the subcommand (scarb 2.19 rejects it after).
Output: a markdown table on stdout (crate, version, lines, over-baseline s / GB, marginal s / GB,
verdict), the JSON with `--json PATH` (raw measures, medians, direct deps, per top-level module
lines, gates, verdicts).
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


def scarb_cmd(scarb, manifest_path, *sub):
    """`scarb [--manifest-path X] SUB..`: the global option goes BEFORE the subcommand."""
    return [scarb] + (["--manifest-path", manifest_path] if manifest_path else []) + list(sub)


NOT_DIRECT = {"core", "starknet"}  # the corelib and the platform crate are in every consumer


def direct_dependencies(p):
    """The `[dependencies]` of a metadata package (no dev-dependencies, no corelib / starknet)."""
    out = []
    for d in p["dependencies"]:
        if d.get("kind") is not None or d["name"] in NOT_DIRECT:
            continue
        dep = {"name": d["name"], "features": sorted(d.get("features") or []),
               "default_features": d.get("default_features", True)}
        source = d["source"]
        if source.startswith("path+file://"):
            dep["root"] = os.path.dirname(source[len("path+file://"):])
        elif source.startswith("registry+"):
            dep["version"] = d["version_req"]
        else:
            sys.exit(f"error: `{p['name']}`: dependency `{d['name']}` from `{source}` is not supported")
        out.append(dep)
    return sorted(out, key=lambda d: d["name"])


def workspace_packages(scarb, manifest_path):
    cmd = scarb_cmd(scarb, manifest_path, "metadata", "--format-version", "1", "--no-deps")
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
                "deps": direct_dependencies(p),
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
    """A consumer crate: one trivial function, the dependencies (`deps`: list of dicts with a
    `root` (path dependency) or a `version` (registry requirement), features, default_features)."""
    lines = ["[package]", f'name = "{name}"', 'version = "0.1.0"']
    if edition:
        lines.append(f"edition = {toml_str(edition)}")
    if lib_table:
        lines.append("\n[lib]")
        lines += [f"{k} = {json.dumps(v)}" for k, v in lib_table.items()]
    lines.append("\n[dependencies]")
    for d in deps:
        if "root" in d:
            opts = [f"path = {toml_str(d['root'])}"]
        elif d["default_features"] and not d["features"]:
            lines.append(f"{d['name']} = {toml_str(d['version'])}")
            continue
        else:
            opts = [f"version = {toml_str(d['version'])}"]
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


def prepare_consumer(tmp, label, manifest, scarb, env):
    """Write the consumer crate in `tmp` and fetch its dependencies (fetching is not measured)."""
    os.makedirs(os.path.join(tmp, "src"))
    with open(os.path.join(tmp, "Scarb.toml"), "w") as f:
        f.write(manifest)
    with open(os.path.join(tmp, "src", "lib.cairo"), "w") as f:
        f.write("pub fn consumer_cost_answer() -> u32 {\n    42\n}\n")
    p = subprocess.run([scarb, "fetch"], cwd=tmp, env=env, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, text=True)
    if p.returncode != 0:
        sys.exit(f"error: `scarb fetch` of the {label} consumer failed:\n{p.stderr[-2000:]}")


def cold_build(tmp, label, scarb, env, tag):
    shutil.rmtree(os.path.join(tmp, "target"), ignore_errors=True)
    wall, rss, rc, err = timed_build([scarb, "build"], tmp, env)
    if rc != 0:
        sys.exit(f"error: `scarb build` of the {label} consumer failed:\n{err}")
    print(f"  {label} [{tag}]: {wall:.1f} s, {(rss or 0) / GB:.2f} GB", file=sys.stderr)
    return wall, rss if rss is not None else 0


def round_order(n, k):
    """The build order of round `k` (0-based) over `n` consumers: rotated by `k`, reversed on odd
    rounds, so every consumer runs early and late and next to different neighbours."""
    order = [(i + k) % n for i in range(n)]
    return order[::-1] if k % 2 else order


def measure_all(planned, repeat, scarb, interleave):
    """Cold `scarb build` of every consumer manifest of `planned` ({manifest: label}), `repeat`
    times each: {manifest: {"seconds": [..], "rss_bytes": [..]}}, sample k of every consumer from
    round k. Consecutive (a consumer's builds one after the other) or `interleave`d (round k builds
    every consumer once, in `round_order`, so a slow period of the machine hits every consumer)."""
    env = dict(os.environ, SCARB_INCREMENTAL="false")
    texts = list(planned)
    out = {t: {"seconds": [], "rss_bytes": []} for t in texts}
    with tempfile.TemporaryDirectory(prefix="consumer_cost_") as root:
        dirs = {}
        for i, t in enumerate(texts):
            dirs[t] = os.path.join(root, f"c{i}")
            print(f"consumer {planned[t]}: fetch", file=sys.stderr)
            prepare_consumer(dirs[t], planned[t], t, scarb, env)
        schedule = ([(k, i) for k in range(repeat) for i in round_order(len(texts), k)] if interleave
                    else [(k, i) for i in range(len(texts)) for k in range(repeat)])
        for k, i in schedule:
            t = texts[i]
            wall, rss = cold_build(dirs[t], planned[t], scarb, env, f"{k + 1}/{repeat}")
            out[t]["seconds"].append(wall)
            out[t]["rss_bytes"].append(rss)
    return out


def quartiles(xs):
    """(first quartile, median, third quartile) of a non-empty list (inclusive method)."""
    if len(xs) == 1:
        return xs[0], xs[0], xs[0]
    q = statistics.quantiles(xs, n=4, method="inclusive")
    return q[0], q[1], q[2]


def paired(a, b, clamp=False):
    """Per-round differences `a[k] - b[k]` (both consumers were built in round k), clamped at 0 for
    memory, and their (Q1, median, Q3)."""
    d = [x - y for x, y in zip(a, b)]
    if clamp:
        d = [max(x, 0) for x in d]
    return d, quartiles(d)


# --------------------------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------------------------


def verdict(fails, report_only=False):
    v = "FAIL: " + ", ".join(fails) if fails else "ok"
    return f"reported ({v})" if report_only else v


def parse_member(spec):
    """A closure member: `name` (workspace crate) or `name@requirement` (registry crate)."""
    name, at, req = spec.partition("@")
    if not name or (at and not req):
        sys.exit(f"error: bad closure member `{spec}` (expected `name` or `name@version`)")
    return name, (req if at else None)


def budget_of(r, gates):
    """The (seconds, GB) budget of a closure / facade row: its own `budget`, else the closure gates."""
    b = r.get("budget") or {}
    return b.get("seconds", gates["closure_seconds"]), b.get("gb", gates["closure_gb"])


def parse_budget(text):
    """`20s,4.5gb` (either part optional) -> {"seconds": 20.0, "gb": 4.5}."""
    out = {}
    for tok in (t.strip().lower() for t in text.split(",") if t.strip()):
        m = re.match(r"^(\d+(?:\.\d+)?)(s|gb)$", tok)
        if not m:
            sys.exit(f"error: bad budget `{tok}` (expected `<n>s` or `<n>gb`)")
        out["seconds" if m.group(2) == "s" else "gb"] = float(m.group(1))
    return out


def budget_config(value):
    """The `budget` of a `[closures]` table: {seconds, gb} (floats) or None."""
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) - {"seconds", "gb"}:
        sys.exit("error: a closure `budget` is a table `{ seconds = N, gb = N }`")
    return {k: float(v) for k, v in value.items()}


def judge(r, gates, facade):
    """The failed gates of one result row."""
    fails = []

    def over(seconds, gb, label_s, label_gb, tag=""):
        if seconds > label_s:
            fails.append(f"{tag}time > {label_s:g} s")
        if gb > label_gb:
            fails.append(f"{tag}memory > {label_gb:g} GB")

    if r.get("closure"):
        if "added_seconds" in r:
            over(r["added_seconds"], r["added_gb"], *budget_of(r, gates))
    elif facade:  # gate 3 only: the consumer of the facade is its closure
        if "added_seconds" in r:
            over(r["added_seconds"], r["added_gb"], *budget_of(r, gates), "closure ")
    else:
        if r["gated_lines"] > gates["max_lines"]:
            fails.append(f"lines > {gates['max_lines']:,.0f}")
        if "marginal_seconds" in r:
            over(r["marginal_seconds"], r["marginal_gb"], gates["max_seconds"], gates["max_gb"],
                 "marginal ")
    return fails


def self_test():
    """Checks of the pure logic (no scarb, no build); exit status 0 when everything holds."""
    assert parse_member("nalgebra_glam") == ("nalgebra_glam", None)
    assert parse_member("glam@0.4.1") == ("glam", "0.4.1")
    assert parse_member("simba@^0.2") == ("simba", "^0.2")
    assert scarb_cmd("scarb", "X/Scarb.toml", "metadata", "--no-deps") == [
        "scarb", "--manifest-path", "X/Scarb.toml", "metadata", "--no-deps"]
    assert scarb_cmd("scarb", None, "fetch") == ["scarb", "fetch"]
    reg = {"name": "glam", "version": "0.4.1", "features": [], "default_features": True}
    reg_f = {"name": "fixed", "version": "^0.4.0", "features": ["x"], "default_features": False}
    loc = {"name": "dep", "root": "/w/dep", "features": [], "default_features": False}
    m = consumer_manifest([reg, reg_f, loc], "2024_07", None)
    assert 'glam = "0.4.1"' in m, m
    assert 'fixed = { version = "^0.4.0", default-features = false, features = ["x"] }' in m, m
    assert 'dep = { path = "/w/dep", default-features = false }' in m, m
    assert "[dependencies]" not in consumer_manifest([], None, None).split("[package]")[0]
    meta = {"name": "c", "dependencies": [
        {"name": "core", "kind": None, "source": "registry+x", "version_req": "=1"},
        {"name": "starknet", "kind": None, "source": "registry+x", "version_req": "=1"},
        {"name": "t", "kind": "dev", "source": "registry+x", "version_req": "^1"},
        {"name": "b", "kind": None, "source": "registry+x", "version_req": "^0.4.0"},
        {"name": "a", "kind": None, "source": "path+file:///w/a/Scarb.toml", "version_req": "^0.1.0",
         "default_features": False, "features": ["f"]}]}
    deps = direct_dependencies(meta)
    assert [d["name"] for d in deps] == ["a", "b"], deps
    assert deps[0] == {"name": "a", "features": ["f"], "default_features": False, "root": "/w/a"}
    assert deps[1]["version"] == "^0.4.0" and deps[1]["default_features"], deps
    g = dict(DEFAULT_GATES)
    row = {"gated_lines": 10, "added_seconds": 20.0, "added_gb": 0.5, "marginal_seconds": 2.0,
           "marginal_gb": 0.2}
    assert judge(row, g, False) == []  # over-baseline is 20 s but the marginal cost is what counts
    assert judge(dict(row, marginal_seconds=6.0), g, False) == ["marginal time > 5 s"]
    assert judge(dict(row, gated_lines=40_001), g, False) == ["lines > 40,000"]
    assert judge(dict(row, gated_lines=10**6, marginal_seconds=99.0), g, True) == ["closure time > 15 s"]
    assert judge({"closure": True, "added_seconds": 1.0, "added_gb": 3.5}, g, False) == [
        "memory > 3 GB"]
    assert judge({"closure": True, "added_seconds": 18.0, "added_gb": 4.0,
                  "budget": {"seconds": 20.0, "gb": 4.5}}, g, False) == []
    assert judge({"closure": True, "added_seconds": 21.0, "added_gb": 4.0,
                  "budget": {"seconds": 20.0}}, g, False) == ["time > 20 s", "memory > 3 GB"]
    assert judge(dict(row, gated_lines=1, added_seconds=16.0, budget={"seconds": 20.0, "gb": 4.5}),
                 g, True) == []
    assert parse_budget("20s,4.5gb") == {"seconds": 20.0, "gb": 4.5}
    assert parse_budget("3GB") == {"gb": 3.0} and parse_budget("") == {}
    assert budget_config(None) is None and budget_config({"gb": 4}) == {"gb": 4.0}
    assert split_closure_spec("p=a,b@1::20s,4.5gb") == ("p", ["a", "b@1"], {"seconds": 20.0, "gb": 4.5})
    assert split_closure_spec("p=a,b") == ("p", ["a", "b"], None)
    assert budget_label({"closure": True, "budget": {"seconds": 20.0, "gb": 4.5}}, g) == "20 s / 4.5 GB"
    assert budget_label({"closure": True}, g) == "15 s / 3 GB"
    assert budget_label({"crate": "x"}, g) == "-"
    assert verdict([], True) == "reported (ok)" and verdict(["x"]) == "FAIL: x"
    assert assign_shards({"a": 10, "b": 6, "c": 5, "d": 1}, 2) == {"a": 1, "b": 2, "c": 2, "d": 1}
    assert assign_shards({"a": 1}, 3) == {"a": 1}
    assert parse_shard("2/5") == (2, 5)
    assert round_order(3, 0) == [0, 1, 2] and round_order(3, 1) == [0, 2, 1]
    assert sorted(round_order(4, 3)) == [0, 1, 2, 3]
    d, (q1, med, q3) = paired([5.0, 7.0, 6.0], [1.0, 2.0, 1.5])
    assert d == [4.0, 5.0, 4.5] and med == 4.5 and q1 <= med <= q3, (d, q1, med, q3)
    assert paired([1, 2], [3, 1], clamp=True)[0] == [0, 1]
    assert closure_config(["a", "b@1"]) == {"members": ["a", "b@1"], "features": None,
                                            "default_features": None, "report_only": False,
                                            "budget": None}
    assert closure_config({"members": ["a"], "budget": {"seconds": 20, "gb": 4.5}})["budget"] == {
        "seconds": 20.0, "gb": 4.5}
    assert closure_config({"members": ["a"], "default_features": False, "report_only": True})[
        "report_only"]
    print("self-test: ok")
    return 0


def parse_shard(spec):
    """`K/N` (1-based shard K of N) -> (K, N)."""
    m = re.match(r"^(\d+)/(\d+)$", spec or "")
    if not m or not 1 <= int(m.group(1)) <= int(m.group(2)):
        sys.exit(f"error: bad shard `{spec}` (expected K/N with 1 <= K <= N)")
    return int(m.group(1)), int(m.group(2))


def assign_shards(weights, n):
    """Longest-processing-time assignment of the items ({key: weight}) to `n` shards: the heaviest
    item first, each to the least loaded shard (ties: the lowest shard, then the key order), so the
    assignment is deterministic. -> {key: shard index 1..n}."""
    load = [0] * n
    out = {}
    for key, w in sorted(weights.items(), key=lambda kv: (-kv[1], kv[0])):
        s = min(range(n), key=lambda i: (load[i], i))
        load[s] += w
        out[key] = s + 1
    return out


def closure_config(value):
    """A `[closures]` entry: a list of members, or a table `{ members = [..], features = [..],
    default_features = bool, report_only = bool }` (features apply to its workspace members)."""
    if isinstance(value, dict):
        if "members" not in value:
            sys.exit("error: a `[closures]` table needs `members`")
        return {"members": list(value["members"]), "features": value.get("features"),
                "default_features": value.get("default_features"),
                "report_only": bool(value.get("report_only", False)),
                "budget": budget_config(value.get("budget"))}
    return {"members": list(value), "features": None, "default_features": None, "report_only": False,
            "budget": None}


def split_closure_spec(spec):
    """`NAME=a,b@1::20s,4.5gb` -> (NAME, ["a", "b@1"], budget or None)."""
    name, _, rest = spec.partition("=")
    members, sep, budget = rest.partition("::")
    return name, [m for m in members.split(",") if m], parse_budget(budget) if sep else None


def budget_label(r, gates):
    """The budget of a row as shown in the table: `20 s / 4.5 GB` for closures and facades, else `-`."""
    if not (r.get("closure") or r.get("facade")):
        return "-"
    s, gb = budget_of(r, gates)
    return f"{s:g} s / {gb:g} GB"


def fmt_iqr(value, iqr, spec, show):
    """`2.6` or, with several rounds, `2.6 (2.4-2.8)` (the interquartile range)."""
    if value is None:
        return "-"
    if show and iqr:
        return f"{format(value, spec)} ({format(iqr[0], spec)}–{format(iqr[1], spec)})"
    return format(value, spec)


def render(results, gates, metric, rounds, modules=False):
    """The markdown table (and the notes) of a list of result rows."""
    show = rounds > 1

    def cell(r, key, spec, iqr_key=None):
        return fmt_iqr(r.get(key), r.get(iqr_key) if iqr_key else None, spec, show)

    lines = ["| crate | version | lines | over baseline s | over baseline GB | marginal s "
             "| marginal GB | budget | verdict |", "|---|---|---:|---:|---:|---:|---:|---|---|"]
    for r in results:
        name = r["crate"] + (" (facade)" if r.get("facade") else "")
        lines.append(f"| {name} | {r['version']} | {r['lines']:,} "
                     f"| {cell(r, 'added_seconds', '.1f', 'added_iqr_seconds')} "
                     f"| {cell(r, 'added_gb', '.2f')} "
                     f"| {cell(r, 'marginal_seconds', '.1f', 'marginal_iqr_seconds')} "
                     f"| {cell(r, 'marginal_gb', '.2f')} | {budget_label(r, gates)} | {r['verdict']} |")
    lines.append(f"\nLines: physical lines of the library files, test-only code excluded "
                 f"(gate: {metric}, max {gates['max_lines']:,.0f}). Over baseline = consumer of the crate - "
                 f"baseline consumer (no dependency). Marginal = consumer of the crate - consumer of its "
                 f"direct dependencies (gate 2: {gates['max_seconds']:g} s / {gates['max_gb']:g} GB). "
                 f"Closures and facades (gate 3): over baseline, against the budget of the row "
                 f"(default {gates['closure_seconds']:g} s / {gates['closure_gb']:g} GB). `reported`: measured, never gated (config `report_only`).")
    if show:
        lines.append(f"Medians of the per-round differences over {rounds} rounds (interquartile range "
                     f"in parentheses); memory: median of the per-round peak differences.")
    if modules:
        for r in results:
            if "modules" in r:
                lines.append(f"\n{r['crate']}: lines per top-level module")
                for mod, n in sorted(r["modules"].items(), key=lambda kv: -kv[1]):
                    lines.append(f"  {mod:<24}{n:>8,}")
    return "\n".join(lines)


def merge(paths, json_out, report_only_marginals=False):
    """Combine the JSON files of several shards: one table, one verdict (exit 1 when a gated row
    fails in any shard, 2 when a file is missing or unreadable)."""
    rows, gates, metric, rounds, baselines = [], None, "raw", 1, []
    for path in paths:
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            print(f"error: cannot read {path}: {e}", file=sys.stderr)
            return 2
        gates = gates or data["gates"]
        metric = data.get("lines_metric", metric)
        rounds = max(rounds, data.get("rounds", 1))
        baselines.append({"file": path, "shard": data.get("shard"), "baseline": data.get("baseline")})
        rows += data["results"]
    if report_only_marginals:  # re-judge: gate 2 reported only (the shards may have gated it)
        for r in rows:
            moved = [f for f in r.get("fails", []) if f.startswith("marginal ")]
            if moved:
                r["fails"] = [f for f in r["fails"] if f not in moved]
                r["reported_fails"] = r.get("reported_fails", []) + moved
                r["verdict"] += " (marginal reported, not gated)"
    names = [r["crate"] for r in rows]
    dup = sorted({n for n in names if names.count(n) > 1})
    if dup:
        print(f"error: measured in several shards: {', '.join(dup)}", file=sys.stderr)
        return 2
    rows.sort(key=lambda r: (bool(r.get("closure")), r["crate"]))
    print(render(rows, gates, metric, rounds))
    failed = [r["crate"] for r in rows if r.get("fails") and not r.get("report_only")]
    reported = [r["crate"] for r in rows if r.get("reported_fails") and not r.get("report_only")]
    if reported:
        print(f"\nOver a gate but reported only (marginals not gated in this run): {', '.join(reported)}")
    print(f"\n{len(rows)} rows from {len(paths)} shard file(s): "
          + (f"FAIL ({', '.join(failed)})" if failed else "every gated row passes"))
    if json_out:
        with open(json_out, "w") as f:
            json.dump({"gates": gates, "rounds": rounds, "shards": baselines, "results": rows,
                       "passed": not failed}, f, indent=2)
            f.write("\n")
    return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description="Consumer cost and size gates of published crates.")
    ap.add_argument("--manifest-path", help="workspace manifest (default: from the current directory)")
    ap.add_argument("--package", action="append", default=[], metavar="NAME",
                    help="crate to check (repeatable); default: the published packages")
    ap.add_argument("--no-packages", action="store_true", help="measure the closures only")
    ap.add_argument("--facade", action="append", default=[], metavar="NAME",
                    help="crate judged on the closure budget only (repeatable)")
    ap.add_argument("--closure", action="append", default=[], metavar="NAME=a,b@1.2",
                    help="consumer of several crates (workspace `name` or registry `name@version`) "
                         "against the closure budget (repeatable)")
    ap.add_argument("--no-closures", action="store_true",
                    help="skip the closures of the config file (`--closure` ones still run)")
    ap.add_argument("--features", help="comma-separated features to enable on every measured crate")
    ap.add_argument("--no-default-features", action="store_true")
    ap.add_argument("--repeat", type=int, default=1, help="builds per consumer (rounds), the median is kept")
    ap.add_argument("--interleave", action="store_true",
                    help="round k builds every consumer once (rotated order) instead of each "
                         "consumer's builds one after the other")
    ap.add_argument("--shard", metavar="K/N",
                    help="measure only the K-th of N balanced shards of the crates and closures")
    ap.add_argument("--merge", nargs="+", metavar="JSON",
                    help="combine the `--json` files of several shards into one table and verdict")
    ap.add_argument("--lines-only", action="store_true", help="count lines only (no build)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the consumer manifests that would be built, build nothing")
    ap.add_argument("--self-test", action="store_true", help="check the script's own logic and exit")
    ap.add_argument("--report-only", action="store_true", help="never fail on a gate")
    ap.add_argument("--report-only-marginals", action="store_true",
                    help="gate 2's marginal cost is reported, never gated (lines and closures stay gated)")
    ap.add_argument("--modules", action="store_true", help="print the lines per top-level module")
    ap.add_argument("--json", metavar="PATH", help="write the results as JSON")
    ap.add_argument("--config", metavar="PATH", help="config file (default: consumer_cost.toml)")
    ap.add_argument("--scarb", default="scarb", help="scarb executable")
    ap.add_argument("--lines-metric", choices=["raw", "nonblank"], default="raw")
    for k, v in DEFAULT_GATES.items():
        ap.add_argument("--" + k.replace("_", "-"), type=float, default=None,
                        help=f"gate (default {v:g})")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.merge:
        return merge(args.merge, args.json, args.report_only_marginals)
    if args.repeat < 1:
        sys.exit("error: --repeat must be at least 1")

    root, packages = workspace_packages(args.scarb, args.manifest_path)
    cfg_path = args.config or os.path.join(root, "consumer_cost.toml")
    cfg = read_toml(cfg_path) if os.path.exists(cfg_path) and tomllib else {}
    if args.config and not cfg:
        sys.exit(f"error: cannot read the config {args.config}")
    gates = dict(DEFAULT_GATES)
    gates.update(cfg.get("gates", {}))
    gates.update({k: getattr(args, k) for k in DEFAULT_GATES if getattr(args, k) is not None})

    by_name = {p["name"]: p for p in packages}
    by_root = {os.path.normpath(p["root"]): p["name"] for p in packages}
    names = [] if args.no_packages else (args.package or [p["name"] for p in packages if p["publish"]])
    for n in names:
        if n not in by_name:
            sys.exit(f"error: no package `{n}` in the workspace")
    facades = set(cfg.get("facades", [])) | set(args.facade)
    for n in facades:
        if n not in by_name:
            sys.exit(f"error: facade `{n}`: no package in the workspace")
    crate_cfg = cfg.get("crates", {})
    closures = {} if args.no_closures else {k: closure_config(v) for k, v in cfg.get("closures", {}).items()}
    for spec in args.closure:
        name, members, budget = split_closure_spec(spec)
        closures[name] = dict(closure_config(members), budget=budget)
    for name, c in closures.items():
        for m in c["members"]:
            crate, req = parse_member(m)
            if req is None and crate not in by_name:
                sys.exit(f"error: closure `{name}`: no package `{crate}` in the workspace "
                         f"(a registry crate is written `{crate}@VERSION`)")

    def dep(name, override=None):
        conf = dict(crate_cfg.get(name, {}))
        if override:
            conf.update({k: v for k, v in override.items() if v is not None})
        feats = args.features.split(",") if args.features else list(conf.get("features") or [])
        return {
            "name": name,
            "root": by_name[name]["root"],
            "features": feats,
            "default_features": not args.no_default_features and conf.get("default_features", True),
        }

    def member(spec, c):
        crate, req = parse_member(spec)
        if req is None:
            return dep(crate, {"features": c["features"], "default_features": c["default_features"]})
        return {"name": crate, "version": req, "features": [], "default_features": True}

    metric = args.lines_metric
    line_counts = {}

    def lines_of(name):
        if name not in line_counts:
            line_counts[name] = count_lines(by_name[name]["lib_source"]) if by_name[name]["lib_source"] else None
        return line_counts[name]

    def closure_lines(crates):
        """Library lines of the workspace crates `crates` and everything they pull in (a proxy of
        the cost of building them, used to balance the shards)."""
        seen, todo = set(), list(crates)
        while todo:
            n = todo.pop()
            if n in seen or n not in by_name:
                continue
            seen.add(n)
            todo += [by_root.get(os.path.normpath(d["root"])) for d in by_name[n]["deps"] if "root" in d]
        return sum((lines_of(n) or {"raw": 0})["raw"] for n in seen)

    shard = None
    if args.shard:
        k, n = parse_shard(args.shard)
        weights = {("crate", c): closure_lines([c]) + closure_lines(
            [by_root.get(os.path.normpath(d["root"])) for d in by_name[c]["deps"] if "root" in d])
            for c in names if by_name[c]["lib_source"]}
        weights.update({("closure", c): closure_lines(
            [parse_member(m)[0] for m in cl["members"] if parse_member(m)[1] is None])
            for c, cl in closures.items()})
        where = assign_shards(weights, n)
        names = [c for c in names if where.get(("crate", c)) == k]
        closures = {c: cl for c, cl in closures.items() if where[("closure", c)] == k}
        shard = {"index": k, "count": n, "crates": names, "closures": list(closures)}
        print(f"shard {k}/{n}: crates {', '.join(names) or '-'}; closures {', '.join(closures) or '-'}",
              file=sys.stderr)

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
                        "facade": n in facades,
                        "report_only": bool(crate_cfg.get(n, {}).get("report_only", False)),
                        "direct_deps": [d["name"] for d in p["deps"]],
                        "lines": c["raw"], "lines_nonblank": c["nonblank"],
                        "test_lines_excluded": c["test_excluded"], "files": c["files"],
                        "modules": c["modules"],
                        "gated_lines": c["raw"] if metric == "raw" else c["nonblank"]})

    baseline = None
    planned = {}  # consumer manifest -> label (a manifest shared by several rows is built once)

    def consumer(label, deps, edition, lib_table):
        text = consumer_manifest(deps, edition, lib_table)
        if text in planned:
            print(f"consumer {label}: same manifest as {planned[text]}, reused", file=sys.stderr)
        else:
            planned[text] = label
        return text

    if not args.lines_only:
        # Plan every consumer, measure them all (consecutive or interleaved rounds), then compute.
        ref = by_name[names[0]] if names else next(iter(by_name.values()))
        base_text = consumer("baseline (no dependency)", [], ref["edition"], ref["lib_table"])
        rows = []
        for r in results:
            p = by_name[r["crate"]]
            own = consumer(r["crate"], [dep(r["crate"])], p["edition"], p["lib_table"])
            deps = consumer(f"{r['crate']} direct deps ({', '.join(r['direct_deps']) or 'none'})",
                            p["deps"], p["edition"], p["lib_table"])
            rows.append((r, own, deps))
        closure_rows = []
        for cname, c in closures.items():
            workspace = [by_name[parse_member(m)[0]] for m in c["members"] if parse_member(m)[1] is None]
            first = workspace[0] if workspace else ref
            text = consumer(f"closure {cname}", [member(m, c) for m in c["members"]],
                            first["edition"], first["lib_table"])
            row = {"crate": f"closure:{cname}", "version": "-", "members": c["members"],
                   "lines": sum(lines_of(p["name"])["raw"] for p in workspace if lines_of(p["name"])),
                   "closure": True, "report_only": c["report_only"]}
            if c["budget"]:
                row["budget"] = c["budget"]
            if c["default_features"] is not None or c["features"]:
                row["configuration"] = {"default_features": c["default_features"],
                                        "features": c["features"] or []}
            results.append(row)
            closure_rows.append((row, text))

        if args.dry_run:
            print(f"# Consumer manifests that would be built ({len(planned)}, cold `scarb build` each)")
            for text, label in planned.items():
                print(f"\n## {label}\n```toml\n{text}```")
            print()
        else:
            measured = measure_all(planned, args.repeat, args.scarb, args.interleave)
            b = measured[base_text]
            baseline = {"seconds": statistics.median(b["seconds"]),
                        "rss_bytes": statistics.median(b["rss_bytes"]), **b}

            def over_baseline(row, samples):
                _, (q1, med, q3) = paired(samples["seconds"], b["seconds"])
                _, (_, gmed, _) = paired(samples["rss_bytes"], b["rss_bytes"], clamp=True)
                row.update(added_seconds=med, added_iqr_seconds=[q1, q3], added_gb=gmed / GB,
                           samples=samples)

            for r, own, deps in rows:
                over_baseline(r, measured[own])
                _, (q1, med, q3) = paired(measured[own]["seconds"], measured[deps]["seconds"])
                _, (_, gmed, _) = paired(measured[own]["rss_bytes"], measured[deps]["rss_bytes"], clamp=True)
                r.update(marginal_seconds=med, marginal_iqr_seconds=[q1, q3], marginal_gb=gmed / GB,
                         deps_samples=measured[deps])
            for row, text in closure_rows:
                over_baseline(row, measured[text])

    failed = False
    for r in results:
        if r.get("closure") or r.get("facade"):
            r["budget"] = dict(zip(("seconds", "gb"), budget_of(r, gates)))
        fails = judge(r, gates, r.get("facade", False))
        if r.get("report_only"):
            r["fails"], r["reported_fails"] = [], fails
        elif args.report_only_marginals:
            r["fails"] = [f for f in fails if not f.startswith("marginal ")]
            r["reported_fails"] = [f for f in fails if f.startswith("marginal ")]
        else:
            r["fails"], r["reported_fails"] = fails, []
        r["verdict"] = verdict(fails, r.get("report_only", False))
        if r["reported_fails"] and not r.get("report_only"):
            r["verdict"] += " (marginal reported, not gated)"
        failed |= bool(r["fails"])

    print(render(results, gates, metric, 1 if args.dry_run or args.lines_only else args.repeat,
                 modules=args.modules))
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"gates": gates, "lines_metric": metric,
                       "marginals_gated": not args.report_only_marginals,
                       "rounds": 0 if args.lines_only or args.dry_run else args.repeat,
                       "interleaved": args.interleave, "shard": shard, "baseline": baseline,
                       "results": results, "passed": not failed}, f, indent=2)
            f.write("\n")
    return 1 if failed and not args.report_only else 0


if __name__ == "__main__":
    sys.exit(main())
