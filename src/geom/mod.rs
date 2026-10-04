//! Exact-geometry resolution and tessellation (ISO 10303-42 resources).
//!
//! [`Geo`] resolves STEP geometric entities — points, directions,
//! placements, curves ([`curve`]), surfaces ([`surface`]) — and meshes
//! the topological solids and shells ([`brep`]) and the AP242
//! tessellated items ([`tessellated`]) through the neutral kernel of
//! `oxideav-ifc` ([`oxideav_ifc::kernel`]). All lengths stay in the
//! representation's own length unit; plane angles are converted to
//! radians with the context's angle unit.

pub mod brep;
pub mod curve;
pub mod surface;
pub mod tessellated;

use std::collections::HashMap;
use std::rc::Rc;

use oxideav_ifc::kernel::Surface;
use oxideav_ifc::{GeometryError, StepFile, Transform, Value};

use crate::schema::{number, Entity};
use crate::units::ContextUnits;

pub use curve::Curve;

/// Geometry result alias.
pub type GResult<T> = Result<T, GeometryError>;

/// Tessellation density controls.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tolerance {
    /// Largest distance between a mesh edge and the exact curve /
    /// surface, as a fraction of the representation's bounding-box
    /// diagonal (used when [`Tolerance::absolute`] is `None`).
    pub relative: f64,
    /// Absolute chordal tolerance in metres (overrides `relative`).
    pub absolute: Option<f64>,
    /// Largest turning angle of a curve per mesh edge, radians.
    pub max_angle: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self {
            relative: 1e-3,
            absolute: None,
            max_angle: 2.0 * core::f64::consts::PI / 32.0,
        }
    }
}

/// Hard caps on what one model may make the tessellator produce.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GeometryLimits {
    /// Total triangles across the whole model.
    pub max_triangles: usize,
    /// Samples one curve run may produce.
    pub max_curve_samples: usize,
}

impl Default for GeometryLimits {
    fn default() -> Self {
        Self {
            max_triangles: 40_000_000,
            max_curve_samples: 100_000,
        }
    }
}

/// Geometry resolution context for one representation.
#[derive(Debug)]
pub struct Geo<'a> {
    /// The parsed file.
    pub step: &'a StepFile,
    /// Radians per model plane-angle unit.
    pub angle: f64,
    /// Chordal tolerance in model length units.
    pub tol: f64,
    /// Largest turning angle per mesh edge (radians).
    pub max_angle: f64,
    /// Caps.
    pub limits: GeometryLimits,
    /// Extent of the geometry being meshed (bounding-box diagonal, model
    /// units) — a size hint for surfaces whose curvature varies.
    pub size: f64,
    curves: HashMap<u64, Rc<Curve>>,
    surfaces: HashMap<u64, Rc<Surface>>,
    /// Resolved surfaces before density refinement.
    raw_surfaces: HashMap<u64, Rc<Surface>>,
    /// Surfaces whose resolution failed (do not retry).
    bad_surfaces: HashMap<u64, GeometryError>,
}

impl<'a> Geo<'a> {
    /// A context for geometry in `units`, meshed to `tol` model units.
    pub fn new(
        step: &'a StepFile,
        units: &ContextUnits,
        tol: f64,
        max_angle: f64,
        limits: GeometryLimits,
    ) -> Self {
        Self {
            step,
            angle: units.angle_radians,
            tol: if tol > 0.0 && tol.is_finite() {
                tol
            } else {
                1e-3
            },
            max_angle: if max_angle > 0.0 && max_angle.is_finite() {
                max_angle.min(core::f64::consts::FRAC_PI_2)
            } else {
                Tolerance::default().max_angle
            },
            limits,
            size: 0.0,
            curves: HashMap::new(),
            surfaces: HashMap::new(),
            raw_surfaces: HashMap::new(),
            bad_surfaces: HashMap::new(),
        }
    }

    /// The entity view of `id`.
    pub fn entity(&self, id: u64) -> GResult<Entity<'a>> {
        Entity::get(self.step, id).ok_or(GeometryError::MissingInstance(id))
    }

    /// `CARTESIAN_POINT` coordinates (a 2-D point gets z = 0).
    pub fn point(&self, id: u64) -> GResult<[f64; 3]> {
        let e = self.entity(id)?;
        if !e.is_a("CARTESIAN_POINT") {
            return Err(GeometryError::Unsupported(format!(
                "{} as a point",
                e.inst.keyword
            )));
        }
        coords3(e.list("coordinates").ok_or(GeometryError::BadCoordinate)?)
    }

    /// The point of a reference value.
    pub fn point_ref(&self, v: Option<&Value>) -> GResult<[f64; 3]> {
        self.point(
            v.and_then(Value::as_reference)
                .ok_or(GeometryError::BadCoordinates)?,
        )
    }

    /// `DIRECTION` ratios (not normalised; 2-D gets z = 0).
    pub fn direction(&self, id: u64) -> GResult<[f64; 3]> {
        let e = self.entity(id)?;
        if !e.is_a("DIRECTION") {
            return Err(GeometryError::BadCoordinates);
        }
        coords3(
            e.list("direction_ratios")
                .ok_or(GeometryError::BadCoordinate)?,
        )
    }

    /// An optional direction attribute (`$` → `None`).
    pub fn opt_direction(&self, v: Option<&Value>) -> GResult<Option<[f64; 3]>> {
        match v {
            None | Some(Value::Unset) => Ok(None),
            Some(v) => self
                .direction(v.as_reference().ok_or(GeometryError::BadCoordinates)?)
                .map(Some),
        }
    }

    /// `VECTOR(orientation, magnitude)` as a 3-D vector.
    pub fn vector(&self, id: u64) -> GResult<[f64; 3]> {
        let e = self.entity(id)?;
        if e.is_a("DIRECTION") {
            return self.direction(id);
        }
        if !e.is_a("VECTOR") {
            return Err(GeometryError::BadCoordinates);
        }
        let d = self.direction(
            e.reference("orientation")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let m = e.number("magnitude").ok_or(GeometryError::BadCoordinate)?;
        let d = normalise(d).ok_or(GeometryError::BadCoordinates)?;
        Ok(scale(d, m))
    }

    /// A placement (`AXIS2_PLACEMENT_3D` / `_2D` / `AXIS1_PLACEMENT`) or
    /// a `CARTESIAN_TRANSFORMATION_OPERATOR` as a frame transform
    /// (local → representation coordinates).
    pub fn placement(&self, id: u64) -> GResult<Transform> {
        let e = self.entity(id)?;
        if e.is_a("AXIS2_PLACEMENT_3D") {
            let o = self.point_ref(e.attr("location"))?;
            let axis = self.opt_direction(e.attr("axis"))?;
            let refd = self.opt_direction(e.attr("ref_direction"))?;
            let [x, y, z] = build_axes(axis, refd);
            return Ok(Transform {
                cols: [x, y, z],
                translation: o,
            });
        }
        if e.is_a("AXIS2_PLACEMENT_2D") {
            let o = self.point_ref(e.attr("location"))?;
            let refd = self.opt_direction(e.attr("ref_direction"))?;
            let x = refd
                .and_then(|d| normalise([d[0], d[1], 0.0]))
                .unwrap_or([1.0, 0.0, 0.0]);
            return Ok(Transform {
                cols: [x, [-x[1], x[0], 0.0], [0.0, 0.0, 1.0]],
                translation: o,
            });
        }
        if e.is_a("AXIS1_PLACEMENT") {
            let o = self.point_ref(e.attr("location"))?;
            let axis = self.opt_direction(e.attr("axis"))?;
            let [x, y, z] = build_axes(axis, None);
            return Ok(Transform {
                cols: [x, y, z],
                translation: o,
            });
        }
        if e.is_a("CARTESIAN_TRANSFORMATION_OPERATOR") {
            return self.transformation_operator(e);
        }
        Err(GeometryError::Unsupported(format!(
            "{} as a placement",
            e.inst.keyword
        )))
    }

    /// `AXIS1_PLACEMENT` → (origin, unit axis).
    pub fn axis1(&self, id: u64) -> GResult<([f64; 3], [f64; 3])> {
        let e = self.entity(id)?;
        let o = self.point_ref(e.attr("location"))?;
        let axis = self
            .opt_direction(e.attr("axis"))?
            .and_then(normalise)
            .unwrap_or([0.0, 0.0, 1.0]);
        Ok((o, axis))
    }

    /// ISO 10303-42 `cartesian_transformation_operator(_3d)`: the
    /// `base_axis` derivation of `axis1` / `axis2` / `axis3`, scaled by
    /// `scale`, translated to `local_origin`.
    fn transformation_operator(&self, e: Entity<'a>) -> GResult<Transform> {
        let o = self.point_ref(e.attr("local_origin"))?;
        let a1 = self.opt_direction(e.attr("axis1"))?;
        let a2 = self.opt_direction(e.attr("axis2"))?;
        let a3 = self.opt_direction(e.attr("axis3"))?;
        let s = match e.attr("scale") {
            Some(v) => number(v).unwrap_or(1.0),
            None => 1.0,
        };
        if !(s > 0.0 && s.is_finite()) {
            return Err(GeometryError::BadCoordinate);
        }
        let [x, y, z] = base_axis_3d(a1, a2, a3);
        Ok(Transform {
            cols: [scale(x, s), scale(y, s), scale(z, s)],
            translation: o,
        })
    }

    /// The (cached) curve `id`.
    pub fn curve(&mut self, id: u64) -> GResult<Rc<Curve>> {
        if let Some(c) = self.curves.get(&id) {
            return Ok(c.clone());
        }
        let c = Rc::new(curve::resolve(self, id, 0)?);
        self.curves.insert(id, c.clone());
        Ok(c)
    }

    /// The (cached) face surface `id`, refined to the context's
    /// tolerance. `extent` bounds the size of the face(s) meshed on it:
    /// it matters for surfaces whose curvature varies across the face (a
    /// cone's section radius), which are refined per call instead of
    /// cached.
    pub fn surface(&mut self, id: u64, extent: f64) -> GResult<Rc<Surface>> {
        if let Some(s) = self.surfaces.get(&id) {
            return Ok(s.clone());
        }
        if let Some(e) = self.bad_surfaces.get(&id) {
            return Err(e.clone());
        }
        let raw = match self.raw_surfaces.get(&id) {
            Some(r) => r.clone(),
            None => match surface::resolve(self, id, 0) {
                Ok(s) => {
                    let r = Rc::new(s);
                    self.raw_surfaces.insert(id, r.clone());
                    r
                }
                Err(e) => {
                    self.bad_surfaces.insert(id, e.clone());
                    return Err(e);
                }
            },
        };
        if surface::extent_dependent(self, id) {
            let hint = if extent > 0.0 && extent.is_finite() {
                extent
            } else {
                self.size
            };
            return Ok(Rc::new((*raw).clone().with_chordal_tolerance(
                self.tol,
                self.max_angle,
                hint,
            )));
        }
        let s = Rc::new(
            (*raw)
                .clone()
                .with_chordal_tolerance(self.tol, self.max_angle, self.size),
        );
        self.surfaces.insert(id, s.clone());
        Ok(s)
    }
}

/// Coordinates of a 1–3 element numeric list (missing components 0).
pub fn coords3(list: &[Value]) -> GResult<[f64; 3]> {
    if list.is_empty() || list.len() > 3 {
        return Err(GeometryError::BadCoordinate);
    }
    let mut out = [0.0; 3];
    for (i, v) in list.iter().enumerate() {
        out[i] = number(v).ok_or(GeometryError::BadCoordinate)?;
    }
    Ok(out)
}

/// Unit vector, `None` for a zero / non-finite input.
pub fn normalise(v: [f64; 3]) -> Option<[f64; 3]> {
    let m = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    (m > 0.0 && m.is_finite()).then(|| [v[0] / m, v[1] / m, v[2] / m])
}

/// Dot product.
pub fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Cross product.
pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `a − b`.
pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `a + b`.
pub fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// `s · a`.
pub fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// Euclidean distance.
pub fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = sub(a, b);
    dot(d, d).sqrt()
}

/// ISO 10303-42 `first_proj_axis` / `build_axes`: the orthonormal
/// placement axes from an optional `axis` (z) and `ref_direction` (x).
pub fn build_axes(axis: Option<[f64; 3]>, ref_dir: Option<[f64; 3]>) -> [[f64; 3]; 3] {
    let z = axis.and_then(normalise).unwrap_or([0.0, 0.0, 1.0]);
    let x = first_proj_axis(z, ref_dir);
    let y = normalise(cross(z, x)).unwrap_or([0.0, 1.0, 0.0]);
    [x, y, z]
}

/// The x axis: `arg` (or a default not parallel to `z`) projected
/// perpendicular to `z`.
fn first_proj_axis(z: [f64; 3], arg: Option<[f64; 3]>) -> [f64; 3] {
    // ISO: v = [1,0,0] unless z is exactly [1,0,0]; a degenerate
    // projection (z = −x) falls back to the y axis.
    let default = if z == [1.0, 0.0, 0.0] {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let project = |v: [f64; 3]| normalise(sub(v, scale(z, dot(v, z))));
    arg.and_then(normalise)
        .and_then(project)
        .or_else(|| project(default))
        .or_else(|| project([0.0, 1.0, 0.0]))
        .unwrap_or([1.0, 0.0, 0.0])
}

/// ISO 10303-42 `base_axis` for three dimensions: `axis3` (default z)
/// as the third axis, `axis1` projected perpendicular to it as the
/// first, the second completing a right-handed set (an `axis2` only
/// breaks the tie when `axis1` is absent).
pub fn base_axis_3d(
    a1: Option<[f64; 3]>,
    a2: Option<[f64; 3]>,
    a3: Option<[f64; 3]>,
) -> [[f64; 3]; 3] {
    let z = a3.and_then(normalise).unwrap_or([0.0, 0.0, 1.0]);
    let x = match a1 {
        Some(a) => first_proj_axis(z, Some(a)),
        None => match a2.and_then(normalise) {
            // x ⟂ to both a2 and z.
            Some(y) => normalise(cross(y, z)).unwrap_or_else(|| first_proj_axis(z, None)),
            None => first_proj_axis(z, None),
        },
    };
    let y = normalise(cross(z, x)).unwrap_or([0.0, 1.0, 0.0]);
    [x, y, z]
}

/// The inverse of an affine transform (`None` when singular).
pub fn invert(t: &Transform) -> Option<Transform> {
    let [cx, cy, cz] = t.cols;
    let det = dot(cx, cross(cy, cz));
    if det.abs() <= 1e-300 || !det.is_finite() {
        return None;
    }
    // Rows of the inverse are (cy×cz, cz×cx, cx×cy) / det.
    let r0 = scale(cross(cy, cz), 1.0 / det);
    let r1 = scale(cross(cz, cx), 1.0 / det);
    let r2 = scale(cross(cx, cy), 1.0 / det);
    let cols = [
        [r0[0], r1[0], r2[0]],
        [r0[1], r1[1], r2[1]],
        [r0[2], r1[2], r2[2]],
    ];
    let tr = t.translation;
    let translation = [-dot(r0, tr), -dot(r1, tr), -dot(r2, tr)];
    Some(Transform { cols, translation })
}

/// A transform with its translation (and only its translation) scaled —
/// re-expresses a rigid placement in another length unit.
pub fn rescale_translation(t: &Transform, s: f64) -> Transform {
    Transform {
        cols: t.cols,
        translation: scale(t.translation, s),
    }
}

/// Map a direction (no translation).
pub fn apply_dir(t: &Transform, v: [f64; 3]) -> [f64; 3] {
    let [cx, cy, cz] = t.cols;
    [
        cx[0] * v[0] + cy[0] * v[1] + cz[0] * v[2],
        cx[1] * v[0] + cy[1] * v[1] + cz[1] * v[2],
        cx[2] * v[0] + cy[2] * v[1] + cz[2] * v[2],
    ]
}

/// A STEP-worded description of a kernel geometry error (the kernel's
/// own messages name the IFC entities it was first written for).
pub fn describe(e: &GeometryError) -> String {
    match e {
        GeometryError::MissingInstance(id) => format!("reference to missing instance #{id}"),
        GeometryError::Unsupported(what) => format!("unsupported: {what}"),
        GeometryError::BadCoordinates => {
            "malformed geometry or topology (the region could not be meshed)".into()
        }
        GeometryError::IndexOutOfRange => "index out of range or degenerate loop".into(),
        GeometryError::BadCoordinate => "malformed coordinate or number".into(),
        GeometryError::BadProfile => "malformed curve / surface definition".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse_round_trips() {
        let t = Transform {
            cols: [[0.0, 1.0, 0.0], [-2.0, 0.0, 0.0], [0.0, 0.0, 0.5]],
            translation: [1.0, 2.0, 3.0],
        };
        let i = invert(&t).unwrap();
        let p = [0.3, -4.0, 7.0];
        let q = i.apply(t.apply(p));
        assert!((0..3).all(|k| (p[k] - q[k]).abs() < 1e-12));
        assert!(invert(&Transform {
            cols: [[0.0; 3]; 3],
            translation: [0.0; 3]
        })
        .is_none());
    }

    #[test]
    fn axes_follow_iso_10303_42() {
        let [x, y, z] = build_axes(Some([0.0, 0.0, 2.0]), Some([1.0, 1.0, 0.0]));
        let s = core::f64::consts::FRAC_1_SQRT_2;
        assert!((x[0] - s).abs() < 1e-12 && (x[1] - s).abs() < 1e-12);
        assert!((y[0] + s).abs() < 1e-12 && (y[1] - s).abs() < 1e-12);
        assert_eq!(z, [0.0, 0.0, 1.0]);
        // Defaults when the axis is x.
        let [x, _, _] = build_axes(Some([1.0, 0.0, 0.0]), None);
        assert!((x[1] - 1.0).abs() < 1e-12);
    }
}
