//! The value types understood by the generator and their Cairo layout.
//!
//! Every value is flattened to a list of `i64` leaves (Fixed -> raw, integers -> value,
//! bool -> 0/1) in glam's column-major order (`to_cols_array`, `to_array`).

use std::collections::BTreeMap;
use std::fmt;

/// Scalar leaf kinds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Leaf {
    Fixed,
    I64,
    I32,
    U32,
    Bool,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Ty {
    Fixed,
    I64,
    I32,
    U32,
    Bool,
    Vec2,
    Vec3,
    Vec4,
    Quat,
    Mat2,
    Mat3,
    Mat4,
    Affine2,
    Affine3,
    /// `glamx::Rot2` (`re`, `im`). A spec sets its non-glam import path.
    Rot2,
    /// `glamx::Pose2` (`rotation: Rot2`, `translation: Vec2`). No default import path under
    /// `glam_core::`: a spec sets it with `[types.Pose2] path = "glamx::pose2::Pose2"`.
    Pose2,
    /// `glamx::Pose3` (`rotation: Quat`, `translation: Vec3`). No default import path under
    /// `glam_core::`: a spec sets it with `[types.Pose3] path = "glamx::pose3::Pose3"`.
    Pose3,
    BVec2,
    BVec3,
    BVec4,
    IVec2,
    IVec3,
    IVec4,
    UVec2,
    UVec3,
    UVec4,
    Tuple(Vec<Ty>),
}

/// Cairo layout of a struct type: its named fields.
#[derive(Clone, Debug)]
pub struct Layout {
    pub fields: Vec<(String, Ty)>,
}

/// Optional per-spec override of a struct layout (`[types.<Name>]`).
#[derive(Clone, Debug, Default, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayoutOverride {
    /// Import path, e.g. `glam_core::mat3::Mat3`.
    pub path: Option<String>,
    /// Emitted Cairo type name when it differs from the generator slot, e.g. `Rot2` for the
    /// two-Fixed `Vec2` layout.
    pub cairo: Option<String>,
    /// Field names, same order and types as the default layout.
    pub fields: Option<Vec<String>>,
}

impl Ty {
    pub fn parse(s: &str) -> Result<Ty, String> {
        let s = s.trim();
        if let Some(inner) = s.strip_prefix('(').and_then(|r| r.strip_suffix(')')) {
            let elems: Result<Vec<Ty>, String> = inner
                .split(',')
                .filter(|p| !p.trim().is_empty())
                .map(Ty::parse)
                .collect();
            let elems = elems?;
            if elems.len() < 2 {
                return Err(format!("tuple type `{s}` needs at least two elements"));
            }
            if elems.iter().any(|t| matches!(t, Ty::Tuple(_))) {
                return Err(format!("nested tuple type `{s}` is not supported"));
            }
            return Ok(Ty::Tuple(elems));
        }
        Ok(match s {
            "Fixed" => Ty::Fixed,
            "i64" => Ty::I64,
            "i32" => Ty::I32,
            "u32" => Ty::U32,
            "bool" => Ty::Bool,
            "Vec2" => Ty::Vec2,
            "Vec3" => Ty::Vec3,
            "Vec4" => Ty::Vec4,
            "Quat" => Ty::Quat,
            "Mat2" => Ty::Mat2,
            "Mat3" => Ty::Mat3,
            "Mat4" => Ty::Mat4,
            "Affine2" => Ty::Affine2,
            "Affine3" => Ty::Affine3,
            "Rot2" => Ty::Rot2,
            "Pose2" => Ty::Pose2,
            "Pose3" => Ty::Pose3,
            "BVec2" => Ty::BVec2,
            "BVec3" => Ty::BVec3,
            "BVec4" => Ty::BVec4,
            "IVec2" => Ty::IVec2,
            "IVec3" => Ty::IVec3,
            "IVec4" => Ty::IVec4,
            "UVec2" => Ty::UVec2,
            "UVec3" => Ty::UVec3,
            "UVec4" => Ty::UVec4,
            other => return Err(format!("unknown type `{other}`")),
        })
    }

    /// The Cairo type name (also the name used in specs).
    pub fn cairo(&self) -> String {
        match self {
            Ty::I64 => "i64".into(),
            Ty::I32 => "i32".into(),
            Ty::U32 => "u32".into(),
            Ty::Bool => "bool".into(),
            Ty::Tuple(elems) => {
                let inner: Vec<String> = elems.iter().map(Ty::cairo).collect();
                format!("({})", inner.join(", "))
            }
            other => format!("{other:?}"),
        }
    }

    /// Suffix of the generated `next_*` / `check_*` helpers.
    pub fn snake(&self) -> String {
        self.cairo().to_lowercase()
    }

    pub fn leaf(&self) -> Option<Leaf> {
        Some(match self {
            Ty::Fixed => Leaf::Fixed,
            Ty::I64 => Leaf::I64,
            Ty::I32 => Leaf::I32,
            Ty::U32 => Leaf::U32,
            Ty::Bool => Leaf::Bool,
            _ => return None,
        })
    }

    /// Default struct fields (glam-rs names). `None` for leaves and tuples.
    fn default_fields(&self) -> Option<Vec<(&'static str, Ty)>> {
        let xyzw = ["x", "y", "z", "w"];
        let axes = ["x_axis", "y_axis", "z_axis", "w_axis"];
        let comps = |n: usize, t: Ty| xyzw[..n].iter().map(|f| (*f, t.clone())).collect();
        let cols = |n: usize, t: Ty| axes[..n].iter().map(|f| (*f, t.clone())).collect();
        Some(match self {
            Ty::Vec2 => comps(2, Ty::Fixed),
            Ty::Vec3 => comps(3, Ty::Fixed),
            Ty::Vec4 | Ty::Quat => comps(4, Ty::Fixed),
            Ty::Mat2 => cols(2, Ty::Vec2),
            Ty::Mat3 => cols(3, Ty::Vec3),
            Ty::Mat4 => cols(4, Ty::Vec4),
            Ty::Affine2 => vec![("matrix2", Ty::Mat2), ("translation", Ty::Vec2)],
            Ty::Affine3 => vec![("matrix3", Ty::Mat3), ("translation", Ty::Vec3)],
            Ty::Rot2 => vec![("re", Ty::Fixed), ("im", Ty::Fixed)],
            Ty::Pose2 => vec![("rotation", Ty::Rot2), ("translation", Ty::Vec2)],
            Ty::Pose3 => vec![("rotation", Ty::Quat), ("translation", Ty::Vec3)],
            Ty::BVec2 => comps(2, Ty::Bool),
            Ty::BVec3 => comps(3, Ty::Bool),
            Ty::BVec4 => comps(4, Ty::Bool),
            Ty::IVec2 => comps(2, Ty::I32),
            Ty::IVec3 => comps(3, Ty::I32),
            Ty::IVec4 => comps(4, Ty::I32),
            Ty::UVec2 => comps(2, Ty::U32),
            Ty::UVec3 => comps(3, Ty::U32),
            Ty::UVec4 => comps(4, Ty::U32),
            _ => return None,
        })
    }

    /// Default import path of a struct type; `None` for primitives and tuples.
    fn default_path(&self) -> Option<String> {
        match self {
            Ty::Fixed => Some("fixed::Fixed".into()),
            Ty::I64 | Ty::I32 | Ty::U32 | Ty::Bool | Ty::Tuple(_) => None,
            other => {
                let name = other.cairo();
                Some(format!("glam_core::{}::{}", name.to_lowercase(), name))
            }
        }
    }

    /// The flattened leaves of the type, in order.
    pub fn leaves(&self) -> Vec<Leaf> {
        if let Some(leaf) = self.leaf() {
            return vec![leaf];
        }
        if let Ty::Tuple(elems) = self {
            return elems.iter().flat_map(Ty::leaves).collect();
        }
        let fields = self.default_fields().expect("struct type");
        fields.iter().flat_map(|(_, t)| t.leaves()).collect()
    }

    pub fn has_fixed_leaf(&self) -> bool {
        self.leaves().contains(&Leaf::Fixed)
    }
}

impl fmt::Display for Ty {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.cairo())
    }
}

/// Resolved layouts: defaults merged with the overrides of one spec.
pub struct Layouts {
    overrides: BTreeMap<Ty, LayoutOverride>,
}

impl Layouts {
    pub fn new(raw: &BTreeMap<String, LayoutOverride>) -> Result<Layouts, String> {
        let mut overrides = BTreeMap::new();
        for (name, o) in raw {
            let ty = Ty::parse(name)?;
            let Some(default) = ty.default_fields() else {
                if ty == Ty::Fixed && o.fields.is_none() {
                    overrides.insert(ty, o.clone());
                    continue;
                }
                return Err(format!(
                    "[types.{name}]: only struct types can be overridden"
                ));
            };
            if let Some(fields) = &o.fields {
                if fields.len() != default.len() {
                    return Err(format!(
                        "[types.{name}]: expected {} field names, got {}",
                        default.len(),
                        fields.len()
                    ));
                }
            }
            overrides.insert(ty, o.clone());
        }
        Ok(Layouts { overrides })
    }

    /// Import path of the type, if it needs one.
    pub fn path(&self, ty: &Ty) -> Option<String> {
        self.overrides
            .get(ty)
            .and_then(|o| o.path.clone())
            .or_else(|| ty.default_path())
    }

    /// Cairo type spelling, with per-spec overrides applied recursively inside tuples.
    pub fn cairo(&self, ty: &Ty) -> String {
        if let Ty::Tuple(elems) = ty {
            let inner: Vec<String> = elems.iter().map(|t| self.cairo(t)).collect();
            return format!("({})", inner.join(", "));
        }
        self.overrides
            .get(ty)
            .and_then(|o| o.cairo.clone())
            .unwrap_or_else(|| ty.cairo())
    }

    /// Struct layout; `None` for leaves (including `Fixed`) and tuples.
    pub fn layout(&self, ty: &Ty) -> Option<Layout> {
        let default = ty.default_fields()?;
        let names = self.overrides.get(ty).and_then(|o| o.fields.clone());
        let fields = default
            .into_iter()
            .enumerate()
            .map(|(i, (name, t))| {
                let name = names
                    .as_ref()
                    .map_or_else(|| name.to_owned(), |n| n[i].clone());
                (name, t)
            })
            .collect();
        Some(Layout { fields })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_and_flatten() {
        assert_eq!(Ty::parse("Vec3").unwrap(), Ty::Vec3);
        assert_eq!(
            Ty::parse("(Vec3, Fixed)").unwrap(),
            Ty::Tuple(vec![Ty::Vec3, Ty::Fixed])
        );
        assert_eq!(Ty::parse("(Vec3, Fixed)").unwrap().leaves().len(), 4);
        assert_eq!(Ty::Affine3.leaves().len(), 12);
        assert_eq!(Ty::Rot2.leaves().len(), 2);
        assert_eq!(Ty::Pose2.leaves().len(), 4);
        assert_eq!(Ty::Pose3.leaves().len(), 7);
        assert_eq!(Ty::Mat4.leaves().len(), 16);
        assert!(Ty::parse("((Fixed, Fixed), Fixed)").is_err());
        assert!(Ty::parse("Vec5").is_err());
        assert!(!Ty::BVec3.has_fixed_leaf());
    }
}
