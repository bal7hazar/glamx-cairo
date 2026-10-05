# glamx 0.5.0: release record

Requested, published and released; the request is kept below.

| Package | Version | Commit | SHA-256 | Built on |
|---|---|---|---|---|
| glamx | 0.5.0 | 43aecc58edd5373b74d04ca2365b125c8f48179c | 32eada7ec22d63834f1275738d9b9d383f23c8e1fae51f1436469b75e99f4585 | srv1792539 (VPS), clean clone detached at 43aecc5, 2026-10-05T08:38Z, verification passed (25 files), peak RSS 1 150 652 KiB under the 8 GiB cap |

glamx 0.5.0 depends on fixed 0.5.0 and glam_core 0.5.0, both published (`facade_check`, which is not published, is on glam 0.5.0). The release commit is the merge of PR #10. It is a MINOR release: glamx's `Rot3` (= `glam_core::Quat`) inherits glam_core 0.5.0's change of `to_axis_angle` / `to_scaled_axis` for a vector part of length in [2^-16, 2^-8); every other result is unchanged (CHANGELOG [0.5.0]).

The archive embeds the building checkout's HEAD (`VCS.json`): only a checkout detached at exactly 43aecc5 reproduces the SHA-256.

## Published

- glamx 0.5.0 was published on 2026-10-05 by the orchestrator, by hand (`scarb publish -p glamx`, under `prlimit --as=8589934592`), from a clean clone detached at 43aecc58edd5373b74d04ca2365b125c8f48179c, after the archive's SHA-256 matched the go. Go: the project manager (slingfall).
- Registry read-back: version 0.5.0, cksum sha256:32eada7ec22d63834f1275738d9b9d383f23c8e1fae51f1436469b75e99f4585 (equal to the request), depending on glam_core ^0.5.0 and fixed ^0.5.0.
- Tag: v0.5.0 (annotated, on 43aecc5). Release: https://github.com/bal7hazar/glamx-cairo/releases/tag/v0.5.0.
