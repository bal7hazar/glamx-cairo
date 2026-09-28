# facade_check

Unpublished fixture (`publish = false`). It depends on the **registry** `glam` facade and on
`glamx` by path (which itself depends on `glam_core`, not on the facade), and builds every value
through the facade paths (`glam::vec3::Vec3`, `glam::quat::Quat`, `glam::mat4::Mat4`, ...) before
passing it to `glamx` and reading the result back into facade-typed bindings.

The facade re-exports the `glam_core` modules, so the two paths name one type. If they ever
stopped doing so (a facade that defines its own types, or a `glam` / `glam_core` version skew),
these tests would fail to compile: that is the proof that a consumer of `glam` can use `glamx`.

```sh
snforge test -p facade_check
```
