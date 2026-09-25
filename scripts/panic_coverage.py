#!/usr/bin/env python3
"""Item-level panic coverage: every documented panic of a public item has a test on that item.

The documented side comes from `scripts/deviations.py` (the doc-template parser): every public
item with a `#### Panics` section other than `Never` yields one requirement per bullet that quotes
a message. Messages of one bullet joined by `/` (`'i64_add Overflow'` / `'i64_add Underflow'`)
are the two directions of one branch: one test with either message covers it. A bullet with no
quoted message that refers to another item (`As [`QuatTrait::slerp`]`) is an inherited
requirement, covered by any panic test on the item or by the allowlist.

The tested side is every `#[should_panic(expected: '<msg>')]` test of
`packages/glamx/tests/*.cairo`, attributed to the item it exercises by, in order:
  (a) a marker comment in the lines above the test, `// panics: <Owner>::<item>` (for the
      generated golden files, which cannot carry one, the `MARKERS` table below);
  (b) its name, `test_<item>_<reason>` (or `golden_<module>_<item>_<reason>`), where `<item>` is
      the longest public item of the module under test that prefixes the rest of the name
      (operator impls answer to their operator: `Vec3Add` to `add`, `Vec3IndexView` to `index`);
  (c) otherwise the test is unattributed.

`--check` exits 1 when a documented (item, message) requirement has no test, when a panic test is
unattributed or attributed to an unknown item, when a test panics with a message that the doc of
its item does not list, or when an allowlist entry is stale.
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deviations  # noqa: E402  (the doc-template parser; reused, not forked)

ROOT = deviations.ROOT
PACKAGES = ("glamx",)
INHERITED = "(inherited)"

# (owner, item, message) -> reason. `message` is one message of the requirement (any of its
# `/` alternatives) or INHERITED. Every entry must still match an uncovered requirement.
ALLOWED_MISSING: dict[tuple[str, str, str], str] = {
    ("Pose2", "lerp", "(inherited)"):
        "inherited: panics as `Rot2Trait::slerp` and `Vec2Trait::lerp`, exercised there",
    ("Pose2", "mul_vec2", "(inherited)"):
        "inherited: panics as `Pose2Trait::transform_point`, exercised there",
    ("Pose3", "new", "(inherited)"):
        "inherited: panics as `QuatTrait::from_scaled_axis`, exercised there",
    ("Pose3", "rotation", "(inherited)"):
        "inherited: panics as `QuatTrait::from_scaled_axis`, exercised there",
    ("Pose3", "lerp", "(inherited)"):
        "inherited: panics as `QuatTrait::slerp` and `Vec3Trait::lerp`, exercised there",
    ("Pose3", "from_mat4", "(inherited)"):
        "inherited: panics as `QuatTrait::from_mat4`, exercised there",
    ("Pose3", "mul_vec3", "(inherited)"):
        "inherited: panics as `Pose3Trait::transform_point`, exercised there",
    ("Rot2", "normalize_mut", "(inherited)"):
        "inherited: panics as `Rot2Trait::normalize`, exercised there",
    ("Rot2", "Rot2MulAssign", "(inherited)"):
        "inherited: panics as `Rot2Mul`, exercised there",
}

# (test file relative to the repository, test function) -> reason.
ALLOWED_UNATTRIBUTED: dict[tuple[str, str], str] = {}

# Markers of the tests of generated files that cannot carry a comment (`tools/refgen` output):
# (test file, test function) -> `<Owner>::<item>`, the same contract as a `// panics:` marker.
MARKERS: dict[tuple[str, str], str] = {
    ("packages/glamx/tests/golden_sdp.cairo", "golden_sdp_inverse_unchecked2_panics_singular"):
        "SdpMatrix2::inverse_unchecked",
}

# (test file, test function) -> reason: the test panics with a message that the doc of its item
# does not list.
ALLOWED_UNDOCUMENTED: dict[tuple[str, str], str] = {}

# Test module stem -> source modules whose items it tests (default: the same stem).
TEST_MODULES = {}

MESSAGE_RE = re.compile(r"`'([^'`]+)'`")
ALTERNATIVES_RE = re.compile(r"`'[^'`]+'`(?:\s*/\s*`'[^'`]+'`)+")
REFERENCE_RE = re.compile(r"\bAs \[?`")
PANIC_RE = re.compile(r"#\[should_panic\(expected:\s*\(?'([^']*)',?\)?\)\]")
FN_RE = re.compile(r"^\s*fn\s+([A-Za-z_][A-Za-z0-9_]*)")
MARKER_RE = re.compile(r"//\s*panics:\s*([A-Za-z_][A-Za-z0-9_:]*)")
MOD_RE = re.compile(r"^(\s*)(?:pub\s+)?mod\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{")
PUB_FN_RE = re.compile(r"^\s*(?:pub\s+)?fn\s+([A-Za-z_][A-Za-z0-9_]*)")
PUB_IMPL_RE = re.compile(
    r"^\s*pub\s+impl\s+([A-Za-z_][A-Za-z0-9_]*)\s+of\s+([A-Za-z_][A-Za-z0-9_]*)"
)
OPERATORS = {
    "Add": "add", "Sub": "sub", "Mul": "mul", "Div": "div", "Rem": "rem", "Neg": "neg",
    "AddAssign": "add_assign", "SubAssign": "sub_assign", "MulAssign": "mul_assign",
    "DivAssign": "div_assign", "RemAssign": "rem_assign", "IndexView": "index",
    "Index": "index", "Into": "into", "TryInto": "try_into", "PartialEq": "eq",
}


@dataclass
class Requirement:
    owner: str
    item: str
    messages: tuple[str, ...]  # alternatives
    path: str
    line: int
    tests: list[str] = field(default_factory=list)
    allowed: str = ""

    @property
    def label(self) -> str:
        return " / ".join(f"'{m}'" for m in self.messages) if self.messages[0] != INHERITED \
            else INHERITED


@dataclass
class PanicTest:
    path: str
    line: int
    name: str
    message: str
    target: tuple[str, str] | None = None
    how: str = ""


def package_of(path: str) -> str:
    return path.split("/")[1]


def source_module(path: str) -> str:
    """`packages/glamx/src/pose3.cairo` -> `pose3`."""
    return path.split("/src/", 1)[1].removesuffix(".cairo")


def inner_modules(lines: list[str]) -> list[tuple[int, int, str]]:
    """One-based inclusive ranges of the inline `mod x { .. }` blocks of a file."""
    ranges = []
    for index, line in enumerate(lines):
        match = MOD_RE.match(line)
        if not match:
            continue
        depth = 0
        for end in range(index, len(lines)):
            depth += lines[end].count("{") - lines[end].count("}")
            if depth == 0:
                ranges.append((index + 1, end + 1, match.group(2)))
                break
    return ranges


def qualify(owner: str, line: int, mods: list[tuple[int, int, str]]) -> str:
    """Append the inline module (`camera::rh::proj` -> `camera::rh::proj::opengl`)."""
    for start, end, name in mods:
        if start <= line <= end:
            return f"{owner}::{name}"
    return owner


def requirements(items: list[deviations.Item]) -> list[Requirement]:
    result = []
    mods_cache: dict[str, list] = {}
    for item in items:
        text = item.panics
        if not text or text.startswith("* Never"):
            continue
        if item.path not in mods_cache:
            mods_cache[item.path] = inner_modules((ROOT / item.path).read_text().splitlines())
        owner = qualify(item.owner, item.line, mods_cache[item.path])
        bullets: list[str] = []
        for line in text.splitlines():
            if line.startswith("* "):
                bullets.append(line[2:])
            elif bullets:
                bullets[-1] += " " + line.strip()
        found = False
        for bullet in bullets:
            groups = [tuple(MESSAGE_RE.findall(m.group(0)))
                      for m in ALTERNATIVES_RE.finditer(bullet)]
            rest = ALTERNATIVES_RE.sub("", bullet)
            groups += [(m,) for m in MESSAGE_RE.findall(rest)]
            for group in groups:
                found = True
                result.append(Requirement(owner, item.name, group, item.path, item.line))
        if not found and any(REFERENCE_RE.search(b) for b in bullets):
            result.append(Requirement(owner, item.name, (INHERITED,), item.path, item.line))
    return result


def public_items(paths: list[Path]) -> dict[str, list[tuple[str, str, str]]]:
    """Source module -> [(name, owner, item)] over every public fn / impl, documented or not.

    `name` is what a test name or a marker may use: the item itself, or the operator of an
    operator impl (`add` for `Vec3Add`); `item` is the name of the inventory.
    """
    result: dict[str, list[tuple[str, str, str]]] = defaultdict(list)
    for path in paths:
        relative = path.relative_to(ROOT).as_posix()
        module = source_module(relative)
        lines = path.read_text().splitlines()
        traits = deviations.trait_ranges(lines)
        mods = inner_modules(lines)
        fallback = deviations.module_owner(path)
        entries = result[module]
        documented = {(item.name, item.line): item.owner for item in deviations.parse_file(path)}
        for number, line in enumerate(lines, 1):
            impl = PUB_IMPL_RE.match(line)
            fn = PUB_FN_RE.match(line)
            name = impl.group(1) if impl else fn.group(1) if fn else None
            if name is None:
                continue
            in_trait = any(s <= number <= e for s, e, _ in traits)
            if fn and not (line.lstrip().startswith("pub ") or in_trait):
                continue
            owner = documented.get((name, number)) or deviations.owner_at(number, traits, fallback)
            owner = qualify(owner, number, mods)
            entries.append((name, owner, name))
            if impl and impl.group(2) in OPERATORS:
                entries.append((OPERATORS[impl.group(2)], owner, name))
    return result


def panic_tests() -> list[PanicTest]:
    result = []
    for package in PACKAGES:
        for path in sorted((ROOT / "packages" / package / "tests").glob("*.cairo")):
            lines = path.read_text().splitlines()
            mods = inner_modules(lines)
            relative = path.relative_to(ROOT).as_posix()
            for index, line in enumerate(lines):
                match = PANIC_RE.search(line)
                if not match or line.lstrip().startswith("//"):
                    continue
                name = None
                for after in lines[index + 1:index + 6]:
                    fn = FN_RE.match(after)
                    if fn:
                        name = fn.group(1)
                        break
                if name is None:
                    raise ValueError(f"{relative}:{index + 1}: should_panic without a fn")
                marker = None
                back = index - 1
                while back >= 0 and (lines[back].strip().startswith(("//", "#["))):
                    found = MARKER_RE.search(lines[back])
                    if found:
                        marker = found.group(1)
                    back -= 1
                scope = [m for s, e, m in mods if s <= index + 1 <= e]
                qualified = "::".join(scope + [name])
                test = PanicTest(relative, index + 1, qualified, match.group(1))
                test.how = marker or MARKERS.get((relative, qualified), "")
                result.append(test)
    return result


def attribute(tests: list[PanicTest], names: dict[str, list[tuple[str, str, str]]],
              documented: dict[tuple[str, str], set[str]]) -> list[str]:
    """Set `target` on every test that a marker or its name attributes; return marker errors."""
    lookup: dict[tuple[str, str], set[tuple[str, str]]] = defaultdict(set)
    for entries in names.values():
        for name, owner, item in entries:
            lookup[(owner, name)].add((owner, item))
    errors = []
    for test in tests:
        stem = Path(test.path).stem
        module = stem.split("_", 1)[1]
        if test.how:
            owner, _, name = test.how.rpartition("::")
            targets = lookup.get((owner, name), set())
            if len(targets) != 1:
                errors.append(f"{test.path}:{test.line}: {test.name}: the marker names "
                              f"{'no' if not targets else 'an ambiguous'} item: `{test.how}`")
                test.how = "unknown"
                continue
            test.target, test.how = next(iter(targets)), "marker"
            continue
        base = test.name.rpartition("::")[2]
        prefix = "test_" if stem.startswith("test_") else f"golden_{module}_"
        if not base.startswith(prefix):
            continue
        rest = base[len(prefix):]
        candidates = {(len(name), owner, item)
                      for source in TEST_MODULES.get(module, (module,))
                      for name, owner, item in names.get(source, [])
                      if rest == name or rest.startswith(name + "_")}
        # Prefer the items whose doc lists the message, then the longest name.
        listed = {c for c in candidates if test.message in documented.get(c[1:], ())}
        pool = listed or candidates
        if pool:
            longest = max(c[0] for c in pool)
            best = {c[1:] for c in pool if c[0] == longest}
            if len(best) == 1:
                test.target, test.how = next(iter(best)), "name"
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true",
                        help="exit 1 on a missing test, an unattributed test or a stale allowlist")
    args = parser.parse_args()
    try:
        items = deviations.inventory()
        tests = panic_tests()
    except ValueError as error:
        print(error, file=sys.stderr)
        return 1
    reqs = requirements(items)
    names = public_items(deviations.source_paths())
    documented: dict[tuple[str, str], set[str]] = defaultdict(set)
    for req in reqs:
        documented[(req.owner, req.item)].update(req.messages)
    errors = attribute(tests, names, documented)

    by_item: dict[tuple[str, str], list[PanicTest]] = defaultdict(list)
    for test in tests:
        if test.target:
            by_item[test.target].append(test)
    used_allow = set()
    for req in reqs:
        for test in by_item.get((req.owner, req.item), []):
            if req.messages[0] == INHERITED or test.message in req.messages:
                req.tests.append(f"{test.path}::{test.name}")
        if not req.tests:
            for message in req.messages:
                key = (req.owner, req.item, message)
                if key in ALLOWED_MISSING:
                    req.allowed = ALLOWED_MISSING[key]
                    used_allow.add(key)

    missing = [r for r in reqs if not r.tests and not r.allowed]
    unattributed = [t for t in tests if not t.target and t.how != "unknown"
                    and (t.path, t.name) not in ALLOWED_UNATTRIBUTED]
    stale = [k for k in ALLOWED_MISSING if k not in used_allow]
    stale += [k for k in ALLOWED_UNATTRIBUTED
              if not any((t.path, t.name) == k and not t.target for t in tests)]
    stale += [k for k in MARKERS if not any((t.path, t.name) == k for t in tests)]
    stale += [k for k in ALLOWED_UNDOCUMENTED if not any(
        (t.path, t.name) == k and t.target and t.message not in documented.get(t.target, ())
        for t in tests)]
    # A test attributed to a documented item whose doc does not list its message.
    undocumented = [t for t in tests if t.target and t.target in documented
                    and INHERITED not in documented[t.target]
                    and t.message not in documented[t.target]
                    and (t.path, t.name) not in ALLOWED_UNDOCUMENTED]

    if not args.check:
        for package in PACKAGES:
            rows = [r for r in reqs if package_of(r.path) == package]
            print(f"## {package}\n")
            print("| Item | Message | Test |")
            print("|---|---|---|")
            for r in rows:
                cell = "<br>".join(f"`{t}`" for t in r.tests) or (
                    f"allowlisted: {r.allowed}" if r.allowed else "**MISSING**")
                print(f"| `{r.owner}::{r.item}` | {r.label} | {cell} |")
            print()
        print("## Unattributed panic tests\n")
        for t in unattributed:
            print(f"- `{t.path}:{t.line}` `{t.name}` ({t.message})")
        print("\n## Panic tests whose message is not in the item's doc\n")
        for t in undocumented:
            print(f"- `{t.path}:{t.line}` `{t.name}` -> `{'::'.join(t.target)}` ({t.message})")
        print()

    covered = sum(1 for r in reqs if r.tests)
    print(f"requirements: {len(reqs)} (covered {covered}, allowlisted "
          f"{sum(1 for r in reqs if r.allowed)}, missing {len(missing)}); panic tests: "
          f"{len(tests)} (by marker {sum(t.how == 'marker' for t in tests)}, by name "
          f"{sum(t.how == 'name' for t in tests)}, unattributed {len(unattributed)}, "
          f"undocumented message {len(undocumented)})")
    if not args.check:
        return 0
    for r in missing:
        errors.append(f"{r.path}:{r.line}: `{r.owner}::{r.item}` {r.label}: no should_panic test")
    for t in unattributed:
        errors.append(f"{t.path}:{t.line}: `{t.name}` ({t.message}) is not attributed to an item: "
                      "add `// panics: <Owner>::<item>` above it")
    for t in undocumented:
        errors.append(f"{t.path}:{t.line}: `{t.name}` panics with '{t.message}', which the doc of "
                      f"`{'::'.join(t.target)}` does not list: fix the marker or escalate the doc")
    for k in stale:
        errors.append(f"stale allowlist entry: {k}")
    for error in errors:
        print(error, file=sys.stderr)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
