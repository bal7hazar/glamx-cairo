//! Eigen-decomposition of symmetric 3x3 matrices: port of glamx `eigen3.rs` @ 0.3.1 and of the
//! `Mat3` part of its `matrix_ext.rs` (`symmetric_eigen`, `symmetric_eigenvalues`, `swap_cols`).
//!
//! Setup-time numerics (principal inertia and frame of compound / mesh mass properties, OBB
//! fitting, convex-hull seeding in parry): never called by a simulation step, so accuracy and
//! robustness come before gas here.
//!
//! #### Algorithm
//!
//! glamx uses Eberly's closed form (trigonometric roots of the characteristic cubic, eigenvectors
//! by cross products). In Q32.32 that form cannot resolve a repeated eigenvalue better than
//! `sqrt(1 ULP)`: the `acos` argument saturates, and the measured error reaches `6.8e4 ULP * |A|`
//! on matrices with two close eigenvalues, with eigenvalues returned out of order (308 of the
//! 7 513 matrices of the study). It is kept as `benches::alt::eigen3::symmetric_eigen_closed_form`.
//! The library ships a **cyclic Jacobi** iteration instead, the choice rapier made for its
//! degenerate soft-body matrices:
//!
//! 1. **Scaling.** The 6 entries are multiplied by the power of two that brings the largest
//!    magnitude into `[2^22, 2^26)` (an unrolled binary search, no loop, no bitwise operation).
//!    Scaling up is exact, so a tiny matrix (a thin rod inertia of `1e-3`) is diagonalised with
//!    54 to 58 significant bits instead of 22, and nothing overflows for large entries.
//! 2. **Rotations.** At most 18 rotations (6 cyclic sweeps) in the planes `(1,2)`, `(2,3)`,
//!    `(3,1)` in turn, stopped as soon as the 3 off-diagonal entries are zero (measured: 12
//!    rotations at most on every corpus of the study, 8.5 to 9.6 on average on generic ones). A
//!    plane whose pivot is already zero is skipped. After each plane the matrix and the eigenvector
//!    columns are relabelled `(1, 2, 3) -> (3, 1, 2)`, so the rotation code always acts on `(1,
//!    2)`. This is a cyclic, even permutation that keeps each column with its diagonal entry. Each
//!    loop iteration (the unit the gas meter charges) therefore performs exactly one rotation.
//!    Each rotation is computed without trigonometry:
//!    `t = 2 a_pq / (d + sgn(d) sqrt(d^2 + 4 a_pq^2))` with `d = a_qq - a_pp`,
//!    `c = 1 / sqrt(1 + t^2)`, `s = t c`; the diagonal is updated as `a_pp - t a_pq`,
//!    `a_qq + t a_pq` (the trace is preserved exactly). These rescales round to nearest instead of
//!    flooring: a one-sided error accumulates over ~12 rotations (measured residual 14 -> 7 ULP).
//! 3. **Polish.** The accumulated eigenvectors are orthonormal to about `2^-29` only (`c` and `s`
//!    have 32 fractional bits): one Gram-Schmidt pass (`normalize`, project, `normalize`,
//!    `cross`, `normalize`) brings them back to the resolution of the scalar.
//! 4. **Refinement.** Each eigenvalue is the Rayleigh quotient `v^T A v / v^T v` of its polished
//!    eigenvector on the exact scaled input (exact Q96.96 triple products, one rescale; the
//!    quotient is the first-order `r - (v^T v - 1) r`, exact to `2^-62`). A Rayleigh quotient is
//!    second-order in the eigenvector error, which removes the `2^-31` relative drift of step 2.
//! 5. Scaling back (rounded to nearest), ascending sort, and a sign flip of the third eigenvector
//!    when the sort was an odd permutation, so that the eigenvectors are always right-handed.
//!
//! Without any rotation (a diagonal input) steps 3 and 4 are identities: the eigenvectors are the
//! axes and the Rayleigh quotients the diagonal. They are then skipped, inside a `while` that runs
//! at most once: a loop body is charged only when it runs, whereas an `if` is always charged its
//! costliest branch.
//!
//! [`SymmetricEigen3Trait::eigenvalues`] skips step 3: a Rayleigh quotient only needs a nearly
//! unit vector (it is second order in the eigenvector error, and `r - (v^T v - 1) r` absorbs the
//! `2^-29` norm error), so the accumulated eigenvectors are used as they are. Its eigenvalues
//! differ from those of `new` by 1 ULP at most (measured; 64 ULP, i.e. 1 ULP of the output
//! resolution, for inputs scaled down), with the same worst-case error.
//!
//! #### Measured accuracy
//!
//! `scripts/gen_eigen3.py study` mirrors this module and the closed form bit for bit and compares
//! them with a 60-digit reference over random SPD matrices of condition number up to `1e6`,
//! indefinite, diagonal, rank-deficient matrices, two equal / nearly equal / three equal
//! eigenvalues, rod and plate inertia tensors, tiny (`|raw| <= 40`) and huge (`2^25..2^30`)
//! entries: 7 513 matrices (`study`) plus 37 513 of a second seed (`study --n 5000 --seed
//! 0xbeef`). Worst cases over both, in ULPs (`2^-32`) per unit of `|A| = max(1, max |a_ij|)`:
//!
//! | | eigenvalues | `|A v - lambda v|` | `|V^T V - I|` (absolute) | `|V diag V^T - A|` |
//! |---|---|---|---|---|
//! | Jacobi (this module) | 0.64 | 9.6 | 3.5 | 9.4 |
//! | closed form (`alt`, first study only) | 68 477 | 54 234 | 22.5 | 46 988 |
//!
//! Below `2^26` the eigenvalues of a diagonal matrix are exact and its eigenvectors are exactly
//! the axes. The worst eigenvalue error is `0.64 ULP * |A|` (`0.79` while `Fixed / Fixed`
//! truncated, up to 0.2); well separated eigenvalues are correctly rounded most of the time.
//!
//! #### Supported range
//!
//! Every symmetric matrix whose entries are above `Fixed::MIN` and whose eigenvalues fit the
//! scalar range: the routine has no other panic (the study includes entries up to `2^30.4`).
//! Eigenvalues are bounded by `3 max |a_ij|`, so `|a_ij| < 2^29` never panics. At `2^26` and
//! above the input is scaled **down** by `2^6` first: its 6 low bits are dropped and the
//! eigenvalues are multiples of 64 ULP (a relative `2^-52`).

use fixed::fixed::{EPSILON, Fixed, FixedTrait, HALF, ONE, ZERO};
use fixed::wide::{
    RecipTrait, W1, WideAdd, WideMul, WideNarrow, WideSqrt, WideSub, dot3, norm2, wide_from,
    wide_mul,
};
use glam_core::mat3::{Mat3, Mat3Trait};
use glam_core::vec3::{Vec3, Vec3Trait};
use crate::sdp::SdpMatrix3;

/// The eigen-decomposition of a symmetric 3x3 matrix: `A = V diag(eigenvalues) V^T`.
///
/// `eigenvalues` are in ascending order (`x <= y <= z`, as in glamx) and the columns of
/// `eigenvectors` are the matching unit eigenvectors. The sign of an eigenvector is arbitrary,
/// and so is the basis of the eigenspace of a repeated eigenvalue: compare decompositions through
/// `A v = lambda v` and the reconstruction, never component-wise. `eigenvectors` is always
/// right-handed (determinant `+1`).
///
/// Mirrors `glamx::SymmetricEigen3` (and `SymmetricEigen3A`, `DSymmetricEigen3`).
/// #### Deviations
/// * The eigenvectors differ from the ones of glamx by their sign, and by the basis of a
///   repeated eigenspace (the algorithm differs, see the module documentation).
/// * `Default` (not derived upstream) is the decomposition of the zero matrix: zero eigenvalues,
///   identity eigenvectors.
#[derive(Copy, Drop, Serde, PartialEq, Debug, Default, Hash)]
pub struct SymmetricEigen3 {
    /// The eigenvalues of the symmetric 3x3 matrix, in ascending order.
    pub eigenvalues: Vec3,
    /// The three unit eigenvectors of the symmetric 3x3 matrix (as columns).
    pub eigenvectors: Mat3,
}

pub trait SymmetricEigen3Trait {
    /// Computes the eigen-decomposition of the given symmetric 3x3 matrix.
    ///
    /// The matrix is assumed symmetric: only its diagonal and its **upper triangle**
    /// (`y_axis.x`, `z_axis.x`, `z_axis.y`) are read, the lower triangle is ignored. The
    /// eigenvalues are returned in ascending order; [`SymmetricEigen3Trait::reverse`] reverses
    /// it.
    ///
    /// Mirrors `glamx::SymmetricEigen3::new`.
    /// #### Panics
    /// * `'Fixed: overflow'` if an eigenvalue does not fit the scalar range (impossible when every
    ///   `|entry| < 2^29`) or if an entry is `Fixed::MIN`.
    /// #### Deviations
    /// * Cyclic Jacobi rotations instead of the closed form of glamx: see the module
    ///   documentation for the algorithm and the measured accuracy.
    /// * glamx reads the full matrix for the eigenvectors; a non-symmetric input gives
    ///   "incorrect results" there, and here the decomposition of its symmetrised upper triangle.
    /// * The eigenvectors are always right-handed; glamx guarantees it too (its third
    ///   eigenvector is a cross product) but does not document it.
    fn new(mat: Mat3) -> SymmetricEigen3;
    /// Computes the eigen-decomposition of a symmetric matrix stored as its 6 unique entries:
    /// the natural input of this module, identical to `new(m.into_matrix())`.
    ///
    /// Mirrors nothing in glamx (parry converts its `SdpMatrix3` to a `Mat3` first).
    /// #### Panics
    /// * As [`SymmetricEigen3Trait::new`].
    /// #### Deviations
    /// * Addition of this port.
    fn from_sdp(m: SdpMatrix3) -> SymmetricEigen3;
    /// Reverses the order of the eigenvalues and of their eigenvectors (descending order).
    ///
    /// Mirrors `glamx::SymmetricEigen3::reverse`.
    /// #### Panics
    /// * Never.
    /// #### Deviations
    /// * None: as upstream, swapping the first and the third column flips the handedness of
    ///   `eigenvectors` (determinant `-1`).
    fn reverse(self: SymmetricEigen3) -> SymmetricEigen3;
    /// Computes the eigenvalues of a symmetric 3x3 matrix, in ascending order (upper triangle
    /// read, as [`SymmetricEigen3Trait::new`]).
    ///
    /// Mirrors `glamx::SymmetricEigen3::eigenvalues`.
    /// #### Panics
    /// * As [`SymmetricEigen3Trait::new`].
    /// #### Deviations
    /// * Runs the rotations and the Rayleigh refinement of the full decomposition but not the
    ///   polish of the eigenvectors (module documentation): its eigenvalues have the same
    ///   worst-case error as `new(m).eigenvalues` and differ from them by at most 1 ULP (64 ULP
    ///   when `max |a_ij| >= 2^26`, where the output resolution is 64 ULP), measured over the
    ///   45 026 matrices of the study.
    /// * `eigenvector1`, `eigenvector2` and `eigenvector3`, the public building blocks of the
    ///   closed form of glamx, have no counterpart in a Jacobi iteration and no consumer in
    ///   parry / rapier: not ported.
    fn eigenvalues(mat: Mat3) -> Vec3;
}

pub impl SymmetricEigen3Impl of SymmetricEigen3Trait {
    fn new(mat: Mat3) -> SymmetricEigen3 {
        decompose(
            Sym {
                a11: mat.x_axis.x,
                a12: mat.y_axis.x,
                a13: mat.z_axis.x,
                a22: mat.y_axis.y,
                a23: mat.z_axis.y,
                a33: mat.z_axis.z,
            },
        )
    }
    fn from_sdp(m: SdpMatrix3) -> SymmetricEigen3 {
        decompose(Sym { a11: m.m11, a12: m.m12, a13: m.m13, a22: m.m22, a23: m.m23, a33: m.m33 })
    }
    #[inline(always)]
    fn reverse(self: SymmetricEigen3) -> SymmetricEigen3 {
        SymmetricEigen3 {
            eigenvalues: Vec3 {
                x: self.eigenvalues.z, y: self.eigenvalues.y, z: self.eigenvalues.x,
            },
            eigenvectors: Mat3 {
                x_axis: self.eigenvectors.z_axis,
                y_axis: self.eigenvectors.y_axis,
                z_axis: self.eigenvectors.x_axis,
            },
        }
    }
    fn eigenvalues(mat: Mat3) -> Vec3 {
        decompose_eigenvalues(
            Sym {
                a11: mat.x_axis.x,
                a12: mat.y_axis.x,
                a13: mat.z_axis.x,
                a22: mat.y_axis.y,
                a23: mat.z_axis.y,
                a33: mat.z_axis.z,
            },
        )
    }
}

/// The `Mat3` part of the `MatExt` extension trait of glamx.
pub trait Mat3ExtTrait {
    /// Swaps the columns `a` and `b` of this matrix.
    ///
    /// Mirrors `glamx::MatExt::swap_cols` for `glam::Mat3`.
    /// #### Panics
    /// * `'Mat3: index out of bounds'` if `a` or `b` is greater than 2.
    /// #### Deviations
    /// * None.
    fn swap_cols(ref self: Mat3, a: usize, b: usize);
    /// Computes the symmetric eigen-decomposition ([`SymmetricEigen3Trait::new`]). If `self` is
    /// not symmetric, only its upper triangle is taken into account.
    ///
    /// Implementation notes:
    /// * As [`SymmetricEigen3Trait::new`].
    ///
    /// Mirrors `glamx::MatExt::symmetric_eigen` for `glam::Mat3`.
    /// #### Panics
    /// * As [`SymmetricEigen3Trait::new`].
    /// #### Deviations
    /// * None.
    fn symmetric_eigen(self: Mat3) -> SymmetricEigen3;
    /// Computes the eigenvalues of a symmetric matrix, in ascending order
    /// ([`SymmetricEigen3Trait::eigenvalues`]).
    ///
    /// Implementation notes:
    /// * As [`SymmetricEigen3Trait::eigenvalues`].
    ///
    /// Mirrors `glamx::MatExt::symmetric_eigenvalues` for `glam::Mat3`.
    /// #### Panics
    /// * As [`SymmetricEigen3Trait::new`].
    /// #### Deviations
    /// * None.
    fn symmetric_eigenvalues(self: Mat3) -> Vec3;
}

pub impl Mat3ExtImpl of Mat3ExtTrait {
    fn swap_cols(ref self: Mat3, a: usize, b: usize) {
        let ca = self.col(a);
        let cb = self.col(b);
        set_col(ref self, a, cb);
        set_col(ref self, b, ca);
    }
    #[inline(always)]
    fn symmetric_eigen(self: Mat3) -> SymmetricEigen3 {
        SymmetricEigen3Trait::new(self)
    }
    #[inline(always)]
    fn symmetric_eigenvalues(self: Mat3) -> Vec3 {
        SymmetricEigen3Trait::eigenvalues(self)
    }
}

#[inline(always)]
fn set_col(ref m: Mat3, index: usize, v: Vec3) {
    match index {
        0 => m.x_axis = v,
        1 => m.y_axis = v,
        2 => m.z_axis = v,
        _ => core::panic_with_felt252('Mat3: index out of bounds'),
    }
}

/// The largest number of rotations, 6 cyclic sweeps of the 3 planes. Measured: every matrix of
/// the study has its 3 off-diagonal entries at zero after 12 rotations (4 sweeps).
const ROTATIONS: u8 = 18;

/// The 6 unique entries of a symmetric matrix (row, column indices from 1).
#[derive(Copy, Drop)]
struct Sym {
    a11: Fixed,
    a12: Fixed,
    a13: Fixed,
    a22: Fixed,
    a23: Fixed,
    a33: Fixed,
}

/// The state of the iteration: the matrix being diagonalised and the accumulated rotations (the
/// columns `v1`, `v2`, `v3`).
#[derive(Copy, Drop)]
struct Jacobi {
    a: Sym,
    v1: Vec3,
    v2: Vec3,
    v3: Vec3,
}

/// What one rotation in the plane `(p, q)` changes (`a_pq` becomes zero); `r` is the third index.
#[derive(Copy, Drop)]
struct Rotated {
    app: Fixed,
    aqq: Fixed,
    arp: Fixed,
    arq: Fixed,
    vp: Vec3,
    vq: Vec3,
}

/// A power-of-two scale, as the raw constants consumed by [`scale_up`] and [`scale_down`].
#[derive(Copy, Drop)]
struct Scale {
    up: Fixed,
    down: Fixed,
    half: Fixed,
}

#[inline(always)]
fn scale(up: i64, down: i64, half: i64) -> Scale {
    Scale { up: Fixed { raw: up }, down: Fixed { raw: down }, half: Fixed { raw: half } }
}

// GENERATED-BEGIN eigen3
/// The power-of-two scale `2^e` that brings a largest raw magnitude `m > 0` into
/// `[2^54, 2^58)`: `e` is even, from `56` down to `0` in steps of 4, then `-6` for
/// `m >= 2^58` (scaled into `[2^52, 2^57]`). An unrolled binary search on constant
/// thresholds (4 comparisons, no loop, no bitwise operation). `up = 2^(32 + e/2)`,
/// `down = 2^(32 - e/2)`, `half = 2^(e - 1)` (`0` if `e <= 0`), as raw values.
#[allow(collapsible_if_else)]
fn scale_of(m: i64) -> Scale {
    if m < 0x40000000 {
        if m < 0x4000 {
            if m < 0x40 {
                if m < 0x4 {
                    scale(0x1000000000000000, 0x10, 0x80000000000000)
                } else {
                    scale(0x400000000000000, 0x40, 0x8000000000000)
                }
            } else {
                if m < 0x400 {
                    scale(0x100000000000000, 0x100, 0x800000000000)
                } else {
                    scale(0x40000000000000, 0x400, 0x80000000000)
                }
            }
        } else {
            if m < 0x400000 {
                if m < 0x40000 {
                    scale(0x10000000000000, 0x1000, 0x8000000000)
                } else {
                    scale(0x4000000000000, 0x4000, 0x800000000)
                }
            } else {
                if m < 0x4000000 {
                    scale(0x1000000000000, 0x10000, 0x80000000)
                } else {
                    scale(0x400000000000, 0x40000, 0x8000000)
                }
            }
        }
    } else {
        if m < 0x400000000000 {
            if m < 0x4000000000 {
                if m < 0x400000000 {
                    scale(0x100000000000, 0x100000, 0x800000)
                } else {
                    scale(0x40000000000, 0x400000, 0x80000)
                }
            } else {
                if m < 0x40000000000 {
                    scale(0x10000000000, 0x1000000, 0x8000)
                } else {
                    scale(0x4000000000, 0x4000000, 0x800)
                }
            }
        } else {
            if m < 0x40000000000000 {
                if m < 0x4000000000000 {
                    scale(0x1000000000, 0x10000000, 0x80)
                } else {
                    scale(0x400000000, 0x40000000, 0x8)
                }
            } else {
                if m < 0x400000000000000 {
                    scale(0x100000000, 0x100000000, 0x0)
                } else {
                    scale(0x20000000, 0x800000000, 0x0)
                }
            }
        }
    }
}
// GENERATED-END eigen3

/// `x * 2^e`: `x * up * up / 2^64`, exact for `e >= 0` (floor otherwise).
#[inline(always)]
fn scale_up(x: Fixed, s: Scale) -> Fixed {
    wide_mul(x, s.up).mul(s.up).narrow()
}

/// `y / 2^e` rounded to nearest (ties toward +infinity): `(y + half) * down * down / 2^64`.
#[inline(always)]
fn scale_down(y: Fixed, s: Scale) -> Fixed {
    wide_mul(y + s.half, s.down).mul(s.down).narrow()
}

/// Half a ULP at the Q64.64 scale: added to an accumulator, it turns the floor of `narrow` into
/// a rounding to nearest (ties toward +infinity).
#[inline(always)]
fn half_ulp() -> W1 {
    wide_mul(EPSILON, HALF)
}

/// `a * b - c * d`, rounded to nearest.
#[inline(always)]
fn mul_sub_round(a: Fixed, b: Fixed, c: Fixed, d: Fixed) -> Fixed {
    wide_mul(a, b).sub(wide_mul(c, d)).add(half_ulp()).narrow()
}

/// `a * b + c * d`, rounded to nearest.
#[inline(always)]
fn dot2_round(a: Fixed, b: Fixed, c: Fixed, d: Fixed) -> Fixed {
    wide_mul(a, b).add(wide_mul(c, d)).add(half_ulp()).narrow()
}

/// The Jacobi rotation that zeroes `apq` (not zero), applied to the rows / columns `p` and `q`
/// of the matrix and to the columns `vp`, `vq` of the accumulated rotation.
fn rotate(
    app: Fixed, aqq: Fixed, apq: Fixed, arp: Fixed, arq: Fixed, vp: Vec3, vq: Vec3,
) -> Rotated {
    // t = tan(phi), the smaller root of t^2 + 2 t cot(2 phi) - 1 = 0: |t| <= 1.
    let d = aqq - app;
    let two_apq = apq + apq;
    let r = norm2(d, two_apq);
    let den = if d.raw < 0 {
        d - r
    } else {
        d + r
    };
    let t = two_apq / den;
    // c = 1 / sqrt(1 + t^2), s = t c: one square root, one shared reciprocal.
    let inv = RecipTrait::new(wide_mul(t, t).add(wide_from(ONE)).sqrt());
    let c = inv.mul(ONE);
    let s = inv.mul(t);
    let x = wide_mul(t, apq).add(half_ulp()).narrow();
    Rotated {
        app: app - x,
        aqq: aqq + x,
        arp: mul_sub_round(c, arp, s, arq),
        arq: dot2_round(s, arp, c, arq),
        vp: Vec3 {
            x: mul_sub_round(c, vp.x, s, vq.x),
            y: mul_sub_round(c, vp.y, s, vq.y),
            z: mul_sub_round(c, vp.z, s, vq.z),
        },
        vq: Vec3 {
            x: dot2_round(s, vp.x, c, vq.x),
            y: dot2_round(s, vp.y, c, vq.y),
            z: dot2_round(s, vp.z, c, vq.z),
        },
    }
}

/// The rotation in the plane `(1, 2)`, followed by the relabelling `(1, 2, 3) -> (3, 1, 2)` that
/// brings the next plane of the cyclic order `(1,2)`, `(2,3)`, `(3,1)` to `(1, 2)`.
#[inline(always)]
fn rotate_relabel(st: Jacobi) -> Jacobi {
    let a = st.a;
    let r = rotate(a.a11, a.a22, a.a12, a.a13, a.a23, st.v1, st.v2);
    Jacobi {
        a: Sym { a11: r.aqq, a12: r.arq, a13: ZERO, a22: a.a33, a23: r.arp, a33: r.app },
        v1: r.vq,
        v2: st.v3,
        v3: r.vp,
    }
}

/// The relabelling `(1, 2, 3) -> (3, 1, 2)` alone: the plane `(1, 2)` is skipped.
#[inline(always)]
fn relabel(st: Jacobi) -> Jacobi {
    let a = st.a;
    Jacobi {
        a: Sym { a11: a.a22, a12: a.a23, a13: a.a12, a22: a.a33, a23: a.a13, a33: a.a11 },
        v1: st.v2,
        v2: st.v3,
        v3: st.v1,
    }
}

/// The cyclic Jacobi iteration on the scaled matrix `a`: the planes `(1,2)`, `(2,3)`, `(3,1)`
/// in turn, at most [`ROTATIONS`] rotations, until the 3 off-diagonal entries are zero. The
/// matrix and the eigenvector columns are relabelled after each plane so that the rotation
/// always acts on `(1, 2)`: the columns stay paired with their diagonal entries, which is all
/// the rest of the algorithm reads (the relabelling is a cyclic, even permutation). A plane
/// whose pivot is already zero is skipped inside the iteration that performs the next rotation,
/// so that every iteration of the loop, which is what the gas meter charges, rotates.
/// Returns the final state and whether any rotation happened.
fn iterate(a: Sym) -> (Jacobi, bool) {
    let mut st = Jacobi { a, v1: Vec3Trait::X, v2: Vec3Trait::Y, v3: Vec3Trait::Z };
    let mut n = ROTATIONS;
    while n != 0 && (st.a.a12.raw != 0 || st.a.a13.raw != 0 || st.a.a23.raw != 0) {
        if st.a.a12.raw == 0 {
            st = relabel(st);
            if st.a.a12.raw == 0 {
                st = relabel(st);
            }
        }
        st = rotate_relabel(st);
        n -= 1;
    }
    (st, n != ROTATIONS)
}

/// The Rayleigh quotient `v^T A v / v^T v` of a nearly unit vector: the 9 exact triple products
/// are rescaled once, and the division by `v^T v = 1 + e` is the first-order `r - e r`.
fn rayleigh(a: Sym, v: Vec3) -> Fixed {
    let xx = wide_mul(v.x, v.x);
    let yy = wide_mul(v.y, v.y);
    let zz = wide_mul(v.z, v.z);
    let xy = wide_mul(v.x, v.y);
    let xz = wide_mul(v.x, v.z);
    let yz = wide_mul(v.y, v.z);
    let r = xx
        .mul(a.a11)
        .add(yy.mul(a.a22))
        .add(zz.mul(a.a33))
        .add(xy.add(xy).mul(a.a12))
        .add(xz.add(xz).mul(a.a13))
        .add(yz.add(yz).mul(a.a23))
        .narrow();
    let e = xx.add(yy).add(zz).sub(wide_from(ONE));
    r - e.mul(r).narrow()
}

/// The largest magnitude of the 6 entries.
#[inline(always)]
fn max_abs(a: Sym) -> Fixed {
    let m = a.a11.abs().max(a.a12.abs()).max(a.a13.abs());
    m.max(a.a22.abs()).max(a.a23.abs()).max(a.a33.abs())
}

/// The matrix scaled by `s` (module documentation, step 1).
#[inline(always)]
fn scale_sym(a: Sym, s: Scale) -> Sym {
    Sym {
        a11: scale_up(a.a11, s),
        a12: scale_up(a.a12, s),
        a13: scale_up(a.a13, s),
        a22: scale_up(a.a22, s),
        a23: scale_up(a.a23, s),
        a33: scale_up(a.a33, s),
    }
}

/// The whole decomposition (module documentation, steps 1 to 5).
fn decompose(a: Sym) -> SymmetricEigen3 {
    let m = max_abs(a);
    if m.raw == 0 {
        return SymmetricEigen3 { eigenvalues: Vec3Trait::ZERO, eigenvectors: Mat3Trait::IDENTITY };
    }
    let scale = scale_of(m.raw);
    let a = scale_sym(a, scale);
    let (st, rotated) = iterate(a);
    // Without any rotation the eigenvectors are the axes and the eigenvalues the scaled
    // diagonal, which the polish and the Rayleigh quotients would return unchanged. The `while`
    // runs at most once: a loop body is charged when it runs, an `if` branch always.
    let (mut v1, mut v2, mut v3) = (st.v1, st.v2, st.v3);
    let (mut l0, mut l1, mut l2) = (st.a.a11, st.a.a22, st.a.a33);
    let mut pending = rotated;
    while pending {
        // Gram-Schmidt polish: the accumulated rotations are orthonormal to ~2^-29 only.
        v1 = st.v1.normalize();
        let k = dot3(v1.x, st.v2.x, v1.y, st.v2.y, v1.z, st.v2.z);
        v2 =
            Vec3 {
                x: wide_from(st.v2.x).sub(wide_mul(k, v1.x)).narrow(),
                y: wide_from(st.v2.y).sub(wide_mul(k, v1.y)).narrow(),
                z: wide_from(st.v2.z).sub(wide_mul(k, v1.z)).narrow(),
            }
            .normalize();
        v3 = v1.cross(v2).normalize();
        // Eigenvalues: Rayleigh quotients on the exact scaled input.
        l0 = rayleigh(a, v1);
        l1 = rayleigh(a, v2);
        l2 = rayleigh(a, v3);
        pending = false;
    }
    // Back to the input scale.
    let mut l0 = scale_down(l0, scale);
    let mut l1 = scale_down(l1, scale);
    let mut l2 = scale_down(l2, scale);
    let mut c0 = v1;
    let mut c1 = v2;
    let mut c2 = v3;
    // Ascending 3-element sorting network; an odd permutation flips the handedness.
    let mut odd = false;
    if l0 > l1 {
        let (l, c) = (l0, c0);
        l0 = l1;
        c0 = c1;
        l1 = l;
        c1 = c;
        odd = !odd;
    }
    if l1 > l2 {
        let (l, c) = (l1, c1);
        l1 = l2;
        c1 = c2;
        l2 = l;
        c2 = c;
        odd = !odd;
    }
    if l0 > l1 {
        let (l, c) = (l0, c0);
        l0 = l1;
        c0 = c1;
        l1 = l;
        c1 = c;
        odd = !odd;
    }
    if odd {
        c2 = -c2;
    }
    SymmetricEigen3 {
        eigenvalues: Vec3 { x: l0, y: l1, z: l2 },
        eigenvectors: Mat3 { x_axis: c0, y_axis: c1, z_axis: c2 },
    }
}

/// The eigenvalues only: the Rayleigh quotients of the accumulated eigenvectors, without the
/// polish (a Rayleigh quotient only needs a nearly unit vector, its error is second order in
/// the eigenvector error), then the ascending sort.
fn decompose_eigenvalues(a: Sym) -> Vec3 {
    let m = max_abs(a);
    if m.raw == 0 {
        return Vec3Trait::ZERO;
    }
    let scale = scale_of(m.raw);
    let a = scale_sym(a, scale);
    let (st, rotated) = iterate(a);
    let (mut l0, mut l1, mut l2) = (st.a.a11, st.a.a22, st.a.a33);
    let mut pending = rotated;
    while pending {
        l0 = rayleigh(a, st.v1);
        l1 = rayleigh(a, st.v2);
        l2 = rayleigh(a, st.v3);
        pending = false;
    }
    let mut l0 = scale_down(l0, scale);
    let mut l1 = scale_down(l1, scale);
    let mut l2 = scale_down(l2, scale);
    if l0 > l1 {
        let l = l0;
        l0 = l1;
        l1 = l;
    }
    if l1 > l2 {
        let l = l1;
        l1 = l2;
        l2 = l;
    }
    if l0 > l1 {
        let l = l0;
        l0 = l1;
        l1 = l;
    }
    Vec3 { x: l0, y: l1, z: l2 }
}
