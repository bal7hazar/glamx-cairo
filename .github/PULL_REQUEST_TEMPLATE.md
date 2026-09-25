## Summary

<!-- What does this PR port / change? Reference the glamx 0.3.1 source file(s). -->

## Type

- [ ] feat (new port) - [ ] perf - [ ] fix - [ ] test - [ ] docs - [ ] chore

## Gas delta

<!-- Paste the relevant `gas/*.snap` delta as a table. Justify any increase. -->

| bench | before | after | delta |
|---|---|---|---|

## Checklist

- [ ] `scripts/check.sh` is green locally
- [ ] Tests: golden vectors, edge cases, properties, `should_panic` with exact messages
- [ ] A bench with non-constant inputs exists for each hot operation
- [ ] Deviations from glamx documented (`#### Deviations` + `docs/DESIGN.md`)
- [ ] `CHANGELOG.md` updated when the change is user-visible
- [ ] Breaking change (API or numeric results): yes / no
