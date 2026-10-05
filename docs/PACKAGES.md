# Packages

Generated file, do not edit by hand. Run: 2026-10-05, runner: ubuntu-latest, 4 vCPU / 15 GB, 3 cold build round(s) per consumer (medians of the per-round differences). Regenerate: run `python3 scripts/consumer_cost.py --repeat 5 --interleave --json consumer_cost.json` (CI: the `Consumer cost` job uploads `consumer_cost.json` and this table as the artifact `consumer-cost`), then `python3 scripts/packages_table.py consumer_cost.json --runner RUNNER --output docs/PACKAGES.md`.

Gates: at most 40,000 library lines; marginal cost at most 5 s / 1 GB (gate 2); closures 15 s / 3 GB unless they declare a budget (gate 3). Each figure is `value / limit (margin)`; the margin is the room left below the limit, negative when over it.

## Published packages

| package | version | lines | marginal time | marginal memory | verdict |
|---|---|---:|---:|---:|---|
| glamx | 0.5.0 | 3,292 / 40,000 (+92 %) | 0.9 s / 5 s (+82 %) | 0.03 GB / 1 GB (+97 %) | ok |

## Declared closures

| closure | members | time | memory | budget | verdict |
|---|---|---:|---:|---|---|
| glamx | glamx, glam_core@0.5.0, fixed@0.5.0 | 2.0 s / 15 s (+87 %) | 0.48 GB / 3 GB (+84 %) | 15 s / 3 GB | ok |
