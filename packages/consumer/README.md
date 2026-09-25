# consumer

Unpublished. Starknet contract fixtures that link `glamx` and its dependencies into deployable
classes, so that the compiled class size of a realistic consumer is tracked against the network
limits:

| contract | content |
|---|---|
| `Particles2d` | a 2D integrator step on `Vec2` / `Rot2` / `Pose2` (semi-implicit Euler, circle-circle contact, particles in storage) |
| `Rigid3d` | what one rapier-style 3D step touches (`Vec3`, `Quat`, `Mat3`, `Pose3`, `SdpMatrix3` world inertia, bodies in storage) |
| `KitchenSink` | everything above plus scalar operations, `Mat4` inverse, `slerp`, Euler conversions, a camera projection and `SymmetricEigen3` |

Every input comes from calldata or storage (nothing is constant-folded). The shared simulation
logic is in `src/sim.cairo`.

Run from the repository root:

```sh
scripts/bytecode_size.py            # size table (release build)
scripts/bytecode_size.py check      # compare with gas/bytecode.size (part of scripts/check.sh and CI)
scripts/bytecode_size.py snapshot   # rewrite gas/bytecode.size
scripts/bytecode_size.py attribution --strategy default --strategy avoid   # CASM felts per call site
```

Keep the compiler's default `inlining-strategy` in a contract that uses these packages: smaller
numeric thresholds reduce class size at the cost of call overhead on hot library functions.
