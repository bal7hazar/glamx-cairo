//! Gas benchmarks of `glamx::pose3` and of the composed glamx forms kept in
//! `benches::alt::pose3` (the `alt_*` benches, bit-identical results).
//!
//! Every input goes through `bb` (otherwise the computation is constant-folded away) and every
//! result through `sink`. Branching functions are measured on one input per branch (`new__zero`,
//! `lerp__near`, `abs_diff_eq__true` / `__false`).
//!
//! Reference points of `gas/quat.snap`: `mul_quat` 10 040, `mul_vec3` 9 360, `lerp` 24 120,
//! `slerp` 122 180, `from_scaled_axis` 48 820, `from_mat4` 19 770; `gas/vec3.snap`: `add` / `sub`
//! 3 440.

use benches::alt::pose3 as alt;
use benches::harness::{bb, sink};
use fixed::fixed::Fixed;
use glam_core::mat4::Mat4;
use glam_core::quat::Quat;
use glam_core::vec3::Vec3;
use glam_core::vec4::Vec4;
use glamx::pose3::{Pose3, Pose3Trait, Rot3Pose3Trait};

/// A unit quaternion: 0.7 rad around the normalized (1, 2, 3).
const QA: Quat = Quat {
    x: Fixed { raw: 393604950 },
    y: Fixed { raw: 787209899 },
    z: Fixed { raw: 1180814849 },
    w: Fixed { raw: 4034575081 },
};
/// A unit quaternion: 2.4 rad around the normalized (-2, 1, 4).
const QB: Quat = Quat {
    x: Fixed { raw: -1747086207 },
    y: Fixed { raw: 873543103 },
    z: Fixed { raw: 3494172412 },
    w: Fixed { raw: 1556314705 },
};
/// `(1.5, -1.75, 1.375)`.
const T1: Vec3 = Vec3 {
    x: Fixed { raw: 0x180000000 }, y: Fixed { raw: -0x1c0000000 }, z: Fixed { raw: 0x160000000 },
};
/// `(-3.25, 2.5, 0.75)`.
const T2: Vec3 = Vec3 {
    x: Fixed { raw: -0x340000000 }, y: Fixed { raw: 0x280000000 }, z: Fixed { raw: 0xc0000000 },
};
/// `0.7 * normalize(1, 2, 3)`: the scaled axis of `QA`.
const AXISANGLE: Vec3 = Vec3 {
    x: Fixed { raw: 803514805 }, y: Fixed { raw: 1607029610 }, z: Fixed { raw: 2410544416 },
};
const ZERO: Vec3 = Vec3 { x: Fixed { raw: 0 }, y: Fixed { raw: 0 }, z: Fixed { raw: 0 } };
const PA: Pose3 = Pose3 { rotation: QA, translation: T1 };
const PB: Pose3 = Pose3 { rotation: QB, translation: T2 };
/// `PA` with its rotation 4.7e-5 rad further around the same axis: the nlerp branch of `slerp`.
const PA_NEARBY: Pose3 = Pose3 {
    rotation: Quat {
        x: Fixed { raw: 393630055 },
        y: Fixed { raw: 787260111 },
        z: Fixed { raw: 1180890166 },
        w: Fixed { raw: 4034540790 },
    },
    translation: T1,
};
/// The homogeneous matrix of `PA` (rounded to nearest: rigid to within 1 ULP).
const MA: Mat4 = Mat4 {
    x_axis: Vec4 {
        x: Fixed { raw: 3357114691 },
        y: Fixed { raw: 2362735513 },
        z: Fixed { raw: -1262539473 },
        w: Fixed { raw: 0 },
    },
    y_axis: Vec4 {
        x: Fixed { raw: -2074165480 },
        y: Fixed { raw: 3573542215 },
        z: Fixed { raw: 1172338548 },
        w: Fixed { raw: 0 },
    },
    z_axis: Vec4 {
        x: Fixed { raw: 1695394521 },
        y: Fixed { raw: -306628451 },
        z: Fixed { raw: 3934254756 },
        w: Fixed { raw: 0 },
    },
    w_axis: Vec4 {
        x: Fixed { raw: 0x180000000 },
        y: Fixed { raw: -0x1c0000000 },
        z: Fixed { raw: 0x160000000 },
        w: Fixed { raw: 0x100000000 },
    },
};
/// `1 / 3`.
const K_THIRD: Fixed = Fixed { raw: 1431655765 };
/// `2^-15`: `PA` and `PA_NEARBY` are within it, `PA` and `PB` are not.
const K_EPS: Fixed = Fixed { raw: 0x20000 };

#[test]
fn new__base() {
    let _t = bb(T1);
    let _aa = bb(AXISANGLE);
    let r = bb(PA);
    sink(r);
}

#[test]
fn new__op() {
    let t = bb(T1);
    let aa = bb(AXISANGLE);
    let _r = bb(PA);
    sink(Pose3Trait::new(t, aa));
}

#[test]
fn new__zero__base() {
    let _t = bb(T1);
    let _aa = bb(ZERO);
    let r = bb(PA);
    sink(r);
}

#[test]
fn new__zero__op() {
    let t = bb(T1);
    let aa = bb(ZERO);
    let _r = bb(PA);
    sink(Pose3Trait::new(t, aa));
}

#[test]
fn rotation__base() {
    let _aa = bb(AXISANGLE);
    let r = bb(PA);
    sink(r);
}

#[test]
fn rotation__op() {
    let aa = bb(AXISANGLE);
    let _r = bb(PA);
    sink(Pose3Trait::rotation(aa));
}

#[test]
fn prepend_translation__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(PA);
    sink(r);
}

#[test]
fn prepend_translation__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(PA);
    sink(a.prepend_translation(v));
}

#[test]
fn append_translation__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(PA);
    sink(r);
}

#[test]
fn append_translation__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(PA);
    sink(a.append_translation(v));
}

#[test]
fn inverse__base() {
    let _a = bb(PA);
    let r = bb(PA);
    sink(r);
}

#[test]
fn inverse__op() {
    let a = bb(PA);
    let _r = bb(PA);
    sink(a.inverse());
}

#[test]
fn alt_inverse_composed__base() {
    let _a = bb(PA);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_inverse_composed__op() {
    let a = bb(PA);
    let _r = bb(PA);
    sink(alt::inverse_composed(a));
}

#[test]
fn inv_mul__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn inv_mul__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(a.inv_mul(b));
}

#[test]
fn alt_inv_mul_composed__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_inv_mul_composed__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(alt::inv_mul_composed(a, b));
}

#[test]
fn alt_inv_mul_via_inverse__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_inv_mul_via_inverse__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(alt::inv_mul_via_inverse(a, b));
}

#[test]
fn mul__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn mul__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(a * b);
}

#[test]
fn alt_mul_composed__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn alt_mul_composed__op() {
    let a = bb(PA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(alt::mul_composed(a, b));
}

#[test]
fn mul_rot3__base() {
    let _a = bb(PA);
    let _q = bb(QB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn mul_rot3__op() {
    let a = bb(PA);
    let q = bb(QB);
    let _r = bb(PA);
    sink(a.mul_rot3(q));
}

#[test]
fn mul_pose3__base() {
    let _q = bb(QA);
    let _b = bb(PB);
    let r = bb(PA);
    sink(r);
}

#[test]
fn mul_pose3__op() {
    let q = bb(QA);
    let b = bb(PB);
    let _r = bb(PA);
    sink(q.mul_pose3(b));
}

#[test]
fn transform_point__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn transform_point__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(a.transform_point(v));
}

#[test]
fn alt_transform_point_composed__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn alt_transform_point_composed__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(alt::transform_point_composed(a, v));
}

#[test]
fn mul_vec3__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn mul_vec3__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(a.mul_vec3(v));
}

#[test]
fn transform_vector__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn transform_vector__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(a.transform_vector(v));
}

#[test]
fn inverse_transform_point__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn inverse_transform_point__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(a.inverse_transform_point(v));
}

#[test]
fn alt_inverse_transform_point_composed__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn alt_inverse_transform_point_composed__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(alt::inverse_transform_point_composed(a, v));
}

#[test]
fn inverse_transform_vector__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn inverse_transform_vector__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(a.inverse_transform_vector(v));
}

#[test]
fn alt_inverse_transform_vector_composed__base() {
    let _a = bb(PA);
    let _v = bb(T2);
    let r = bb(T1);
    sink(r);
}

#[test]
fn alt_inverse_transform_vector_composed__op() {
    let a = bb(PA);
    let v = bb(T2);
    let _r = bb(T1);
    sink(alt::inverse_transform_vector_composed(a, v));
}

#[test]
fn lerp__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let _k = bb(K_THIRD);
    let r = bb(PA);
    sink(r);
}

#[test]
fn lerp__op() {
    let a = bb(PA);
    let b = bb(PB);
    let k = bb(K_THIRD);
    let _r = bb(PA);
    sink(a.lerp(b, k));
}

#[test]
fn lerp__near__base() {
    let _a = bb(PA);
    let _b = bb(PA_NEARBY);
    let _k = bb(K_THIRD);
    let r = bb(PA);
    sink(r);
}

#[test]
fn lerp__near__op() {
    let a = bb(PA);
    let b = bb(PA_NEARBY);
    let k = bb(K_THIRD);
    let _r = bb(PA);
    sink(a.lerp(b, k));
}

#[test]
fn nlerp__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let _k = bb(K_THIRD);
    let r = bb(PA);
    sink(r);
}

#[test]
fn nlerp__op() {
    let a = bb(PA);
    let b = bb(PB);
    let k = bb(K_THIRD);
    let _r = bb(PA);
    sink(a.nlerp(b, k));
}

#[test]
fn to_mat4__base() {
    let _a = bb(PA);
    let r = bb(MA);
    sink(r);
}

#[test]
fn to_mat4__op() {
    let a = bb(PA);
    let _r = bb(MA);
    sink(a.to_mat4());
}

#[test]
fn from_mat4__base() {
    let _m = bb(MA);
    let r = bb(PA);
    sink(r);
}

#[test]
fn from_mat4__op() {
    let m = bb(MA);
    let _r = bb(PA);
    sink(Pose3Trait::from_mat4(m));
}

#[test]
fn abs_diff_eq__true__base() {
    let _a = bb(PA);
    let _b = bb(PA_NEARBY);
    let _k = bb(K_EPS);
    let r = bb(true);
    sink(r);
}

#[test]
fn abs_diff_eq__true__op() {
    let a = bb(PA);
    let b = bb(PA_NEARBY);
    let k = bb(K_EPS);
    let _r = bb(true);
    sink(a.abs_diff_eq(b, k));
}

#[test]
fn abs_diff_eq__false__base() {
    let _a = bb(PA);
    let _b = bb(PB);
    let _k = bb(K_EPS);
    let r = bb(true);
    sink(r);
}

#[test]
fn abs_diff_eq__false__op() {
    let a = bb(PA);
    let b = bb(PB);
    let k = bb(K_EPS);
    let _r = bb(true);
    sink(a.abs_diff_eq(b, k));
}
