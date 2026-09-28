//! Alternative implementations benchmarked against `glamx::rot2`. The library ships the fused
//! formulations; these literal glamx expressions preserve the gas comparison across compilers.

use fixed::fixed::Fixed;
use fixed::wide::{WideAdd, WideNarrow, wide_mul};
use glam_core::vec2::Vec2;
use glamx::rot2::Rot2;

/// Literal complex multiplication with four separately rescaled products: 9,320 gas versus
/// 4,680 for the fused library implementation.
#[inline(always)]
pub fn mul_unfused(lhs: Rot2, rhs: Rot2) -> Rot2 {
    Rot2 { re: lhs.re * rhs.re - lhs.im * rhs.im, im: lhs.re * rhs.im + lhs.im * rhs.re }
}

/// Literal vector transform with four separately rescaled products: 9,320 gas versus 4,680 for
/// the fused library implementation.
#[inline(always)]
pub fn mul_vec2_unfused(lhs: Rot2, rhs: Vec2) -> Vec2 {
    Vec2 { x: lhs.re * rhs.x - lhs.im * rhs.y, y: lhs.im * rhs.x + lhs.re * rhs.y }
}

/// Unnormalized blend in the two-product form `a * (1 - s) + b * s`, one fused Q64.64 sum per
/// component. Its exact value equals `a + (b - a) * s`, so it is bit-identical to the library's
/// `Fixed::lerp` form (except that `1 - s` panics for `s` near `Fixed::MIN`): 5,520 gas versus
/// 4,880 for the library.
#[inline(always)]
pub fn lerp_two_product(lhs: Rot2, rhs: Rot2, s: Fixed) -> Rot2 {
    let t = Fixed { raw: 0x100000000 } - s;
    Rot2 {
        re: wide_mul(lhs.re, t).add(wide_mul(rhs.re, s)).narrow(),
        im: wide_mul(lhs.im, t).add(wide_mul(rhs.im, s)).narrow(),
    }
}
