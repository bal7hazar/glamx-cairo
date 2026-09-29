#!/usr/bin/env python3
"""The package table (`docs/PACKAGES.md`) from a `scripts/consumer_cost.py --json` / `--merge --json` file.

Repository-agnostic (no crate name, no path), Python 3.8+, standard library only.

Per published package: its library lines against the line gate, its marginal time and memory
against gate 2, each with its margin (`(max - value) / max`, in %, negative = over the gate);
a facade is marked "gate 3 only" (its lines and marginal cost are not gated, its over-baseline cost
is judged as a closure). Per declared closure: its time and memory over the no-dependency build
against its budget (its own `budget`, else the closure gates), with the margins. A row the
config marks `report_only` is flagged "(reported)". The header says which run produced the table
(date, runner, rounds) and how to regenerate it.

Usage
  python3 scripts/consumer_cost.py --merge shards/*/consumer_cost_*.json --json consumer_cost.json
  python3 scripts/packages_table.py consumer_cost.json --runner "ubuntu-latest, 4 vCPU" \
      --output docs/PACKAGES.md
  python3 scripts/packages_table.py --self-test        # synthetic JSON, no scarb
  python3 scripts/packages_table.py --example          # print the table of the synthetic JSON
"""

import argparse
import datetime
import json
import sys

REGENERATE = (
    "Regenerate: run `python3 scripts/consumer_cost.py --repeat 5 --interleave --json consumer_cost.json` "
    "(CI: the `Consumer cost` job uploads `consumer_cost.json` and this table as the artifact "
    "`consumer-cost`), then `python3 scripts/packages_table.py consumer_cost.json --runner RUNNER "
    "--output docs/PACKAGES.md`."
)


def margin(value, limit):
    """`+37 %` (room left below the limit) or `-12 %` (over it); `-` when a figure is missing."""
    if value is None or not limit:
        return "-"
    return f"{(limit - value) / limit * 100:+.0f} %"


def figure(value, limit, spec, unit):
    """`3.2 s / 5 s (+36 %)`."""
    if value is None:
        return "-"
    return f"{format(value, spec)} {unit} / {limit:g} {unit} ({margin(value, limit)})"


def budget_of(r, gates):
    b = r.get("budget") or {}
    return b.get("seconds", gates["closure_seconds"]), b.get("gb", gates["closure_gb"])


def render(data, runner="unknown", date=None):
    gates, rows = data["gates"], data["results"]
    rounds = data.get("rounds", 0)
    date = date or datetime.date.today().isoformat()
    pkgs = sorted((r for r in rows if not r.get("closure")), key=lambda r: r["crate"])
    closures = sorted((r for r in rows if r.get("closure")), key=lambda r: r["crate"])
    out = [
        "# Packages",
        "",
        "Generated file, do not edit by hand. "
        f"Run: {date}, runner: {runner}, {rounds} cold build round(s) per consumer "
        "(medians of the per-round differences). " + REGENERATE,
        "",
        f"Gates: at most {gates['max_lines']:,.0f} library lines; marginal cost at most "
        f"{gates['max_seconds']:g} s / {gates['max_gb']:g} GB (gate 2); closures "
        f"{gates['closure_seconds']:g} s / {gates['closure_gb']:g} GB unless they declare a budget "
        "(gate 3). Each figure is `value / limit (margin)`; the margin is the room left below the "
        "limit, negative when over it.",
        "",
        "## Published packages",
        "",
        "| package | version | lines | marginal time | marginal memory | verdict |",
        "|---|---|---:|---:|---:|---|",
    ]
    for r in pkgs:
        flag = " (reported)" if r.get("report_only") else ""
        if r.get("facade"):
            lines = f"{r['lines']:,} (gate 3 only)"
            t = m = "gate 3 only"
            if r.get("added_seconds") is not None:
                bs, bg = budget_of(r, gates)
                t = f"closure {figure(r['added_seconds'], bs, '.1f', 's')}"
                m = f"closure {figure(r['added_gb'], bg, '.2f', 'GB')}"
        else:
            lines = f"{r['lines']:,} / {gates['max_lines']:,.0f} ({margin(r['lines'], gates['max_lines'])})"
            t = figure(r.get("marginal_seconds"), gates["max_seconds"], ".1f", "s")
            m = figure(r.get("marginal_gb"), gates["max_gb"], ".2f", "GB")
        out.append(f"| {r['crate']}{flag} | {r['version']} | {lines} | {t} | {m} | {r['verdict']} |")
    out += ["", "## Declared closures", "",
            "| closure | members | time | memory | budget | verdict |", "|---|---|---:|---:|---|---|"]
    for r in closures:
        bs, bg = budget_of(r, gates)
        name = r["crate"].split(":", 1)[-1] + (" (reported)" if r.get("report_only") else "")
        out.append(f"| {name} | {', '.join(r.get('members', []))} "
                   f"| {figure(r.get('added_seconds'), bs, '.1f', 's')} "
                   f"| {figure(r.get('added_gb'), bg, '.2f', 'GB')} | {bs:g} s / {bg:g} GB "
                   f"| {r['verdict']} |")
    return "\n".join(out) + "\n"


EXAMPLE = {
    "gates": {"max_lines": 40000, "max_seconds": 5.0, "max_gb": 1.0, "closure_seconds": 15.0,
              "closure_gb": 3.0},
    "rounds": 5,
    "results": [
        {"crate": "core_a", "version": "0.1.1", "lines": 30000, "marginal_seconds": 3.2,
         "marginal_gb": 0.8, "added_seconds": 3.2, "added_gb": 0.8, "verdict": "ok"},
        {"crate": "big_b", "version": "0.1.1", "lines": 41000, "marginal_seconds": 5.5,
         "marginal_gb": 0.9, "verdict": "FAIL: lines > 40,000, marginal time > 5 s"},
        {"crate": "umbrella", "version": "0.1.1", "lines": 50, "facade": True, "report_only": True,
         "added_seconds": 16.0, "added_gb": 3.5, "verdict": "reported (FAIL: closure time > 15 s)"},
        {"crate": "closure:product", "version": "-", "closure": True, "members": ["core_a", "glam@0.4.1"],
         "added_seconds": 12.0, "added_gb": 2.4, "verdict": "ok"},
        {"crate": "closure:wide", "version": "-", "closure": True, "members": ["core_a", "big_b"],
         "budget": {"seconds": 20.0, "gb": 4.5}, "added_seconds": 18.0, "added_gb": 4.0,
         "verdict": "ok"},
    ],
}


def self_test():
    assert margin(3.0, 5.0) == "+40 %" and margin(6.0, 5.0) == "-20 %" and margin(None, 5) == "-"
    t = render(EXAMPLE, runner="test runner", date="2026-01-02")
    assert "Run: 2026-01-02, runner: test runner, 5 cold build round(s)" in t, t
    assert "| core_a | 0.1.1 | 30,000 / 40,000 (+25 %) | 3.2 s / 5 s (+36 %) | 0.80 GB / 1 GB (+20 %) | ok |" in t, t
    assert "| big_b | 0.1.1 | 41,000 / 40,000 (-2 %)" in t and "5.5 s / 5 s (-10 %)" in t, t
    assert "| umbrella (reported) | 0.1.1 | 50 (gate 3 only) | closure 16.0 s / 15 s (-7 %)" in t, t
    assert "| product | core_a, glam@0.4.1 | 12.0 s / 15 s (+20 %) | 2.40 GB / 3 GB (+20 %) | 15 s / 3 GB | ok |" in t, t
    assert "| wide | core_a, big_b | 18.0 s / 20 s (+10 %) | 4.00 GB / 4.5 GB (+11 %) | 20 s / 4.5 GB | ok |" in t, t
    print("self-test: ok")
    return 0


def main():
    ap = argparse.ArgumentParser(description="Markdown package table from a consumer_cost.py JSON.")
    ap.add_argument("json", nargs="?", help="`consumer_cost.py --json` (or `--merge --json`) file")
    ap.add_argument("--runner", default="unknown", help="runner description for the header")
    ap.add_argument("--date", help="run date for the header (default: today)")
    ap.add_argument("--output", metavar="PATH", help="write the table here (default: stdout)")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--example", action="store_true", help="print the table of a synthetic JSON")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.example:
        sys.stdout.write(render(EXAMPLE, "example runner", "2026-01-02"))
        return 0
    if not args.json:
        ap.error("a JSON file is required")
    with open(args.json) as f:
        text = render(json.load(f), args.runner, args.date)
    if args.output:
        with open(args.output, "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
