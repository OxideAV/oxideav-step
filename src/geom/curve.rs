//! Curves (ISO 10303-42 `curve` subtypes): evaluation, point inversion
//! and tolerance-driven sampling.
//!
//! Parameterisations (frame `C, x, y` of the conic's `position`):
//!
//! * `line`: `λ(t) = pnt + t·dir` (`dir` a vector — its magnitude
//!   scales `t`);
//! * `circle`: `C + R(cos t·x + sin t·y)`; `ellipse`:
//!   `C + a cos t·x + b sin t·y` (t in radians internally; parameter
//!   values in the file are in the plane-angle unit);
//! * `hyperbola`: `C + a cosh t·x + b sinh t·y`; `parabola`:
//!   `C + f t²·x + 2 f t·y`;
//! * B-splines through the kernel (de Boor); `polyline`: `t` the
//!   fractional vertex index;
//! * a `pcurve`'s 2-D curve evaluated on its basis surface.
//!
//! Composite, trimmed-as-standalone and offset curves are sampled into
//! a polyline at the context tolerance on construction.

use std::rc::Rc;

use oxideav_ifc::kernel::{BSplineCurve, Surface};
use oxideav_ifc::{GeometryError, Transform, Value};

use super::{add, coords3, cross, dist, dot, normalise, scale, sub, GResult, Geo};
use crate::schema::{logical, number};

/// Recursion bound for curve definitions referring to other curves.
const MAX_CURVE_DEPTH: usize = 24;

/// The conic subtypes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Conic {
    /// `circle(radius)`.
    Circle(f64),
    /// `ellipse(semi_axis_1, semi_axis_2)`.
    Ellipse(f64, f64),
    /// `hyperbola(semi_axis, semi_imag_axis)`.
    Hyperbola(f64, f64),
    /// `parabola(focal_dist)`.
    Parabola(f64),
}

/// A resolved, evaluable curve.
#[derive(Debug, Clone)]
pub enum Curve {
    /// Unbounded line.
    Line {
        /// A point on the line (`t = 0`).
        origin: [f64; 3],
        /// The parameter-scaled direction.
        dir: [f64; 3],
    },
    /// A conic in its placement frame.
    Conic {
        /// Orthonormal frame (origin = centre / vertex).
        frame: Transform,
        /// Subtype and size.
        kind: Conic,
    },
    /// A (rational) B-spline.
    BSpline {
        /// The evaluator.
        curve: BSplineCurve,
        /// Whether the end points coincide (a closed loop).
        closed: bool,
    },
    /// A polyline (`t` the fractional vertex index).
    Polyline(Vec<[f64; 3]>),
    /// A 2-D curve in a surface's parameter space.
    OnSurface {
        /// The parameter-space curve (z ignored).
        uv: Box<Curve>,
        /// The basis surface.
        surface: Rc<Surface>,
    },
}

/// Turn a 2-D placement into one usable for 2-D conics in parameter
/// space (its own frame is already 2-D).
fn frame_point(frame: &Transform, x: f64, y: f64) -> [f64; 3] {
    frame.apply([x, y, 0.0])
}

impl Curve {
    /// The point at `t`.
    pub fn eval(&self, t: f64) -> [f64; 3] {
        match self {
            Self::Line { origin, dir } => add(*origin, scale(*dir, t)),
            Self::Conic { frame, kind } => match *kind {
                Conic::Circle(r) => frame_point(frame, r * t.cos(), r * t.sin()),
                Conic::Ellipse(a, b) => frame_point(frame, a * t.cos(), b * t.sin()),
                Conic::Hyperbola(a, b) => frame_point(frame, a * t.cosh(), b * t.sinh()),
                Conic::Parabola(f) => frame_point(frame, f * t * t, 2.0 * f * t),
            },
            Self::BSpline { curve, .. } => curve.point_at(t),
            Self::Polyline(pts) => polyline_point(pts, t),
            Self::OnSurface { uv, surface } => {
                let p = uv.eval(t);
                surface.point_at([p[0], p[1]])
            }
        }
    }

    /// The parameter period of a closed curve.
    pub fn period(&self) -> Option<f64> {
        match self {
            Self::Conic {
                kind: Conic::Circle(_) | Conic::Ellipse(..),
                ..
            } => Some(2.0 * core::f64::consts::PI),
            Self::BSpline { curve, closed } if *closed => {
                let (a, b) = curve.domain();
                Some(b - a)
            }
            Self::Polyline(pts) if pts.len() > 2 && pts.first() == pts.last() => {
                Some((pts.len() - 1) as f64)
            }
            Self::OnSurface { uv, .. } => uv.period(),
            _ => None,
        }
    }

    /// The bounded parameter domain, if any.
    pub fn domain(&self) -> Option<(f64, f64)> {
        match self {
            Self::Conic {
                kind: Conic::Circle(_) | Conic::Ellipse(..),
                ..
            } => Some((0.0, 2.0 * core::f64::consts::PI)),
            Self::BSpline { curve, .. } => Some(curve.domain()),
            Self::Polyline(pts) => Some((0.0, pts.len().saturating_sub(1) as f64)),
            Self::OnSurface { uv, .. } => uv.domain(),
            _ => None,
        }
    }

    /// Whether parameter values given in the file for this curve are
    /// plane angles (converted by the context's angle unit).
    pub fn angular(&self) -> bool {
        matches!(
            self,
            Self::Conic {
                kind: Conic::Circle(_) | Conic::Ellipse(..),
                ..
            }
        )
    }

    /// The parameter of (the curve point nearest to) `p`.
    pub fn param_of(&self, p: [f64; 3]) -> f64 {
        match self {
            Self::Line { origin, dir } => {
                let d2 = dot(*dir, *dir);
                if d2 > 0.0 {
                    dot(sub(p, *origin), *dir) / d2
                } else {
                    0.0
                }
            }
            Self::Conic { frame, kind } => {
                let l = to_local(frame, p);
                match *kind {
                    Conic::Circle(_) => l[1].atan2(l[0]),
                    Conic::Ellipse(a, b) => (l[1] / b).atan2(l[0] / a),
                    Conic::Hyperbola(_, b) => (l[1] / b).asinh(),
                    Conic::Parabola(f) => l[1] / (2.0 * f),
                }
            }
            Self::Polyline(pts) => nearest_on_polyline(pts, p),
            Self::BSpline { curve, .. } => {
                let breaks = curve.breaks();
                let mut seeds: Vec<f64> = Vec::new();
                for w in breaks.windows(2) {
                    for k in 0..8 {
                        seeds.push(w[0] + (w[1] - w[0]) * k as f64 / 8.0);
                    }
                }
                if let Some(&l) = breaks.last() {
                    seeds.push(l);
                }
                nearest_param(self, p, &seeds, curve.domain())
            }
            Self::OnSurface { uv, .. } => {
                let (a, b) = uv.domain().unwrap_or((0.0, 1.0));
                let seeds: Vec<f64> = (0..=64).map(|k| a + (b - a) * k as f64 / 64.0).collect();
                nearest_param(self, p, &seeds, (a, b))
            }
        }
    }

    /// The parameters of a sampling run from `t0` to `t1` (either
    /// order), both ends included: every chord stays within `tol` of
    /// the curve and turns by at most `max_angle`, capped at
    /// `max_samples`.
    pub fn sample(
        &self,
        t0: f64,
        t1: f64,
        tol: f64,
        max_angle: f64,
        max_samples: usize,
    ) -> Vec<f64> {
        if !(t0.is_finite() && t1.is_finite()) || t0 == t1 {
            return vec![t0, t1];
        }
        let span = t1 - t0;
        match self {
            Self::Line { .. } => vec![t0, t1],
            Self::Conic {
                kind: Conic::Circle(r),
                ..
            } => uniform(
                t0,
                t1,
                angular_count(span.abs(), *r, tol, max_angle),
                max_samples,
            ),
            Self::Conic {
                kind: Conic::Ellipse(a, b),
                ..
            } => uniform(
                t0,
                t1,
                angular_count(span.abs(), a.abs().max(b.abs()), tol, max_angle),
                max_samples,
            ),
            Self::Polyline(_) => {
                let (lo, hi) = if t0 < t1 { (t0, t1) } else { (t1, t0) };
                let mut out = vec![lo];
                let mut k = lo.floor() + 1.0;
                while k < hi && out.len() < max_samples {
                    out.push(k);
                    k += 1.0;
                }
                out.push(hi);
                if t0 > t1 {
                    out.reverse();
                }
                out
            }
            Self::BSpline { curve, .. } => {
                let (lo, hi) = if t0 < t1 { (t0, t1) } else { (t1, t0) };
                let mut breaks: Vec<f64> = vec![lo];
                for k in curve.breaks() {
                    if k > lo && k < hi {
                        breaks.push(k);
                    }
                }
                breaks.push(hi);
                let min_div = if curve.degree() == 1 { 1 } else { 2 };
                let mut out = adaptive(self, &breaks, min_div, tol, max_angle, max_samples);
                if t0 > t1 {
                    out.reverse();
                }
                out
            }
            _ => {
                let (lo, hi) = if t0 < t1 { (t0, t1) } else { (t1, t0) };
                let mut out = adaptive(self, &[lo, hi], 8, tol, max_angle, max_samples);
                if t0 > t1 {
                    out.reverse();
                }
                out
            }
        }
    }

    /// Points of a sampling run (see [`Curve::sample`]).
    pub fn sample_points(
        &self,
        t0: f64,
        t1: f64,
        tol: f64,
        max_angle: f64,
        max_samples: usize,
    ) -> Vec<[f64; 3]> {
        self.sample(t0, t1, tol, max_angle, max_samples)
            .into_iter()
            .map(|t| self.eval(t))
            .collect()
    }
}

/// Segment count for an arc of `sweep` radians on a circle of `r`.
fn angular_count(sweep: f64, r: f64, tol: f64, max_angle: f64) -> usize {
    let r = r.abs();
    let step = if r > tol && tol > 0.0 {
        (2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos()).min(max_angle)
    } else {
        max_angle
    }
    .max(2.0 * core::f64::consts::PI / 1024.0);
    ((sweep / step).ceil() as usize).max(1)
}

fn uniform(t0: f64, t1: f64, n: usize, cap: usize) -> Vec<f64> {
    let n = n.clamp(1, cap.max(2) - 1);
    (0..=n)
        .map(|k| t0 + (t1 - t0) * k as f64 / n as f64)
        .collect()
}

/// Adaptive subdivision of each `[breaks[i], breaks[i+1]]` (first into
/// `min_div` equal parts) until the chord deviation (probed at the
/// quarter points) is within `tol` and the turn per sample within
/// `max_angle`.
fn adaptive(
    c: &Curve,
    breaks: &[f64],
    min_div: usize,
    tol: f64,
    max_angle: f64,
    cap: usize,
) -> Vec<f64> {
    let mut out: Vec<f64> = Vec::new();
    let cos_max = max_angle.cos();
    for w in breaks.windows(2) {
        let (a, b) = (w[0], w[1]);
        if b <= a {
            continue;
        }
        let n = min_div.max(1);
        let mut stack: Vec<(f64, f64, usize)> = (0..n)
            .rev()
            .map(|k| {
                (
                    a + (b - a) * k as f64 / n as f64,
                    a + (b - a) * (k + 1) as f64 / n as f64,
                    0,
                )
            })
            .collect();
        while let Some((s, e, depth)) = stack.pop() {
            if out.len() >= cap {
                break;
            }
            let ps = c.eval(s);
            let pe = c.eval(e);
            let needs_split = depth < 16 && {
                let mut bad = false;
                for f in [0.25, 0.5, 0.75] {
                    let t = s + (e - s) * f;
                    let pt = c.eval(t);
                    let chord = add(ps, scale(sub(pe, ps), f));
                    if dist(pt, chord) > tol {
                        bad = true;
                        break;
                    }
                }
                if !bad {
                    // Turning angle at the midpoint.
                    let pm = c.eval(0.5 * (s + e));
                    if let (Some(d1), Some(d2)) = (normalise(sub(pm, ps)), normalise(sub(pe, pm))) {
                        bad = dot(d1, d2) < cos_max;
                    }
                }
                bad
            };
            if needs_split {
                let m = 0.5 * (s + e);
                stack.push((m, e, depth + 1));
                stack.push((s, m, depth + 1));
            } else {
                out.push(s);
            }
        }
    }
    out.push(*breaks.last().unwrap_or(&0.0));
    out.dedup();
    out
}

/// Parameter on `pts` (fractional vertex index) nearest to `p`.
fn nearest_on_polyline(pts: &[[f64; 3]], p: [f64; 3]) -> f64 {
    let mut best = (f64::INFINITY, 0.0);
    if pts.len() == 1 {
        return 0.0;
    }
    for (i, w) in pts.windows(2).enumerate() {
        let d = sub(w[1], w[0]);
        let l2 = dot(d, d);
        let f = if l2 > 0.0 {
            (dot(sub(p, w[0]), d) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = add(w[0], scale(d, f));
        let dd = dist(p, q);
        if dd < best.0 {
            best = (dd, i as f64 + f);
        }
    }
    best.1
}

fn polyline_point(pts: &[[f64; 3]], t: f64) -> [f64; 3] {
    match pts.len() {
        0 => [0.0; 3],
        1 => pts[0],
        n => {
            let t = if t.is_finite() {
                t.clamp(0.0, (n - 1) as f64)
            } else {
                0.0
            };
            let i = (t.floor() as usize).min(n - 2);
            let f = t - i as f64;
            add(pts[i], scale(sub(pts[i + 1], pts[i]), f))
        }
    }
}

/// Nearest parameter by seed scan + golden-section refinement in the
/// bracketing interval.
fn nearest_param(c: &Curve, p: [f64; 3], seeds: &[f64], domain: (f64, f64)) -> f64 {
    if seeds.is_empty() {
        return domain.0;
    }
    let mut best = 0usize;
    let mut bd = f64::INFINITY;
    for (i, &t) in seeds.iter().enumerate() {
        let d = dist(c.eval(t), p);
        if d < bd {
            bd = d;
            best = i;
        }
    }
    let lo = if best > 0 { seeds[best - 1] } else { seeds[0] };
    let hi = if best + 1 < seeds.len() {
        seeds[best + 1]
    } else {
        seeds[best]
    };
    let (mut a, mut b) = (lo, hi);
    let g = 0.618_033_988_749_894_9;
    let f = |t: f64| dist(c.eval(t), p);
    let mut x1 = b - g * (b - a);
    let mut x2 = a + g * (b - a);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..60 {
        if (b - a).abs() <= 1e-13 * (1.0 + a.abs().max(b.abs())) {
            break;
        }
        if f1 < f2 {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - g * (b - a);
            f1 = f(x1);
        } else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + g * (b - a);
            f2 = f(x2);
        }
    }
    let t = 0.5 * (a + b);
    // Keep the seed if refinement went nowhere better.
    if f(t) <= bd {
        t
    } else {
        seeds[best]
    }
}

fn to_local(frame: &Transform, p: [f64; 3]) -> [f64; 3] {
    let d = sub(p, frame.translation);
    [
        dot(d, frame.cols[0]),
        dot(d, frame.cols[1]),
        dot(d, frame.cols[2]),
    ]
}

/// Resolve curve `id` (the [`Geo::curve`] cache calls this).
pub(super) fn resolve(geo: &mut Geo<'_>, id: u64, depth: usize) -> GResult<Curve> {
    if depth > MAX_CURVE_DEPTH {
        return Err(GeometryError::Unsupported("curve nesting too deep".into()));
    }
    let e = geo.entity(id)?;
    if e.is_a("LINE") {
        let origin = geo.point_ref(e.attr("pnt"))?;
        let dir = geo.vector(e.reference("dir").ok_or(GeometryError::BadCoordinates)?)?;
        return Ok(Curve::Line { origin, dir });
    }
    if e.is_a("CONIC") {
        let frame = geo.placement(
            e.reference("position")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let pos = |name: &str| -> GResult<f64> {
            let v = e.number(name).ok_or(GeometryError::BadProfile)?;
            if v > 0.0 {
                Ok(v)
            } else {
                Err(GeometryError::BadProfile)
            }
        };
        let kind = if e.is_a("CIRCLE") {
            Conic::Circle(pos("radius")?)
        } else if e.is_a("ELLIPSE") {
            Conic::Ellipse(pos("semi_axis_1")?, pos("semi_axis_2")?)
        } else if e.is_a("HYPERBOLA") {
            Conic::Hyperbola(pos("semi_axis")?, pos("semi_imag_axis")?)
        } else if e.is_a("PARABOLA") {
            let f = e.number("focal_dist").ok_or(GeometryError::BadProfile)?;
            if f == 0.0 {
                return Err(GeometryError::BadProfile);
            }
            Conic::Parabola(f)
        } else {
            return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
        };
        return Ok(Curve::Conic { frame, kind });
    }
    if e.is_a("POLYLINE") {
        let pts = e.list("points").ok_or(GeometryError::BadCoordinates)?;
        if pts.len() < 2 || pts.len() > geo.limits.max_curve_samples {
            return Err(GeometryError::BadProfile);
        }
        let mut out = Vec::with_capacity(pts.len());
        for p in pts {
            out.push(geo.point_ref(Some(p))?);
        }
        return Ok(Curve::Polyline(out));
    }
    if e.is_a("B_SPLINE_CURVE") {
        return bspline(geo, e);
    }
    if e.is_a("SURFACE_CURVE") {
        // The 3-D curve, unless it is itself a p-curve; else the first
        // p-curve of the associated geometry.
        if let Some(c3) = e.reference("curve_3d") {
            match geo.curve(c3) {
                Ok(c) => return Ok((*c).clone()),
                Err(err) => {
                    if let Some(assoc) = e.list("associated_geometry") {
                        for a in assoc {
                            if let Some(aid) = a.as_reference() {
                                if let Ok(c) = resolve(geo, aid, depth + 1) {
                                    return Ok(c);
                                }
                            }
                        }
                    }
                    return Err(err);
                }
            }
        }
        return Err(GeometryError::BadCoordinates);
    }
    if e.is_a("PCURVE") {
        let sid = e
            .reference("basis_surface")
            .ok_or(GeometryError::BadCoordinates)?;
        let surface = geo.surface(sid, 0.0)?;
        let rep = geo.entity(
            e.reference("reference_to_curve")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let item = rep
            .list("items")
            .and_then(|l| l.iter().find_map(Value::as_reference))
            .ok_or(GeometryError::BadCoordinates)?;
        let uv = resolve(geo, item, depth + 1)?;
        return Ok(Curve::OnSurface {
            uv: Box::new(uv),
            surface,
        });
    }
    if e.is_a("TRIMMED_CURVE") {
        let (basis, t0, t1) = trimmed(geo, e, depth)?;
        return Ok(Curve::Polyline(basis.sample_points(
            t0,
            t1,
            geo.tol,
            geo.max_angle,
            geo.limits.max_curve_samples,
        )));
    }
    if e.is_a("COMPOSITE_CURVE") {
        let segs = e.list("segments").ok_or(GeometryError::BadCoordinates)?;
        let mut pts: Vec<[f64; 3]> = Vec::new();
        for s in segs {
            let se = geo.entity(s.as_reference().ok_or(GeometryError::BadCoordinates)?)?;
            let same = se.logical("same_sense").unwrap_or(true);
            let pid = se
                .reference("parent_curve")
                .ok_or(GeometryError::BadCoordinates)?;
            let pc = resolve(geo, pid, depth + 1)?;
            let (a, b) = bounded_range(&pc, pid)?;
            let mut run =
                pc.sample_points(a, b, geo.tol, geo.max_angle, geo.limits.max_curve_samples);
            if !same {
                run.reverse();
            }
            // Chain head-to-tail (tolerate a segment authored backwards).
            if let (Some(&last), Some(&first), Some(&end)) = (pts.last(), run.first(), run.last()) {
                if dist(last, end) < dist(last, first) {
                    run.reverse();
                }
            }
            for p in run {
                if pts.last() != Some(&p) {
                    pts.push(p);
                }
            }
            if pts.len() > geo.limits.max_curve_samples {
                return Err(GeometryError::BadProfile);
            }
        }
        if pts.len() < 2 {
            return Err(GeometryError::BadProfile);
        }
        return Ok(Curve::Polyline(pts));
    }
    if e.is_a("OFFSET_CURVE_3D") {
        let bid = e
            .reference("basis_curve")
            .ok_or(GeometryError::BadCoordinates)?;
        let basis = resolve(geo, bid, depth + 1)?;
        let d = e.number("distance").ok_or(GeometryError::BadCoordinate)?;
        let r = geo
            .opt_direction(e.attr("ref_direction"))?
            .and_then(normalise)
            .ok_or(GeometryError::BadCoordinates)?;
        let (a, b) = bounded_range(&basis, bid)?;
        let ts = basis.sample(a, b, geo.tol, geo.max_angle, geo.limits.max_curve_samples);
        let mut pts = Vec::with_capacity(ts.len());
        for &t in &ts {
            let h = 1e-6 * (1.0 + t.abs());
            let tan = normalise(sub(basis.eval(t + h), basis.eval(t - h)))
                .ok_or(GeometryError::BadCoordinates)?;
            // ISO 10303-42: offset along tangent × ref_direction.
            let off = normalise(cross(tan, r)).ok_or(GeometryError::BadCoordinates)?;
            pts.push(add(basis.eval(t), scale(off, d)));
        }
        return Ok(Curve::Polyline(pts));
    }
    Err(GeometryError::Unsupported(e.inst.keyword.clone()))
}

/// The parameter range of a bounded curve used as a segment / basis:
/// its domain (a trimmed curve resolves to a polyline already).
fn bounded_range(c: &Curve, id: u64) -> GResult<(f64, f64)> {
    c.domain().ok_or_else(|| {
        GeometryError::Unsupported(format!("unbounded curve #{id} as a composite segment"))
    })
}

/// Resolve a `trimmed_curve` into its basis and the (directed)
/// parameter range `trim_1 → trim_2` (reversed when `sense_agreement`
/// is FALSE). Trim selects prefer the parameter value
/// (`master_representation` aside — both are given in practice).
pub fn trimmed(
    geo: &mut Geo<'_>,
    e: crate::schema::Entity<'_>,
    depth: usize,
) -> GResult<(Curve, f64, f64)> {
    let bid = e
        .reference("basis_curve")
        .ok_or(GeometryError::BadCoordinates)?;
    let basis = resolve(geo, bid, depth + 1)?;
    let sense = e.attr("sense_agreement").and_then(logical).unwrap_or(true);
    let t1 = trim_param(geo, &basis, e.list("trim_1"))?;
    let t2 = trim_param(geo, &basis, e.list("trim_2"))?;
    let (a, mut b) = (t1, t2);
    if let Some(p) = basis.period() {
        // Positive sweep in the sense direction.
        if sense {
            b = a + (b - a).rem_euclid(p);
            if b - a <= 1e-12 * p {
                b = a + p;
            }
        } else {
            b = a - (a - b).rem_euclid(p);
            if a - b <= 1e-12 * p {
                b = a - p;
            }
        }
    }
    // A bounded basis runs trim_1 → trim_2 as given (the sense flag
    // only matters for the direction around a closed basis).
    Ok((basis, a, b))
}

fn trim_param(geo: &mut Geo<'_>, basis: &Curve, sel: Option<&[Value]>) -> GResult<f64> {
    let sel = sel.ok_or(GeometryError::BadCoordinates)?;
    let mut point: Option<[f64; 3]> = None;
    for v in sel {
        match v {
            Value::Typed { keyword, args } if keyword == "PARAMETER_VALUE" => {
                let t = args
                    .first()
                    .and_then(number)
                    .ok_or(GeometryError::BadCoordinate)?;
                return Ok(if basis.angular() { t * geo.angle } else { t });
            }
            Value::Real(_) | Value::Integer(_) => {
                let t = number(v).ok_or(GeometryError::BadCoordinate)?;
                return Ok(if basis.angular() { t * geo.angle } else { t });
            }
            Value::Reference(id) if point.is_none() => {
                point = geo.point(*id).ok();
            }
            _ => {}
        }
    }
    let p = point.ok_or(GeometryError::BadCoordinates)?;
    Ok(basis.param_of(p))
}

/// Resolve a `b_spline_curve` (with knots, uniform, quasi-uniform,
/// Bézier; rational via the complex-instance `weights_data`).
fn bspline(geo: &Geo<'_>, e: crate::schema::Entity<'_>) -> GResult<Curve> {
    let degree = e.number("degree").ok_or(GeometryError::BadProfile)?;
    if !(1.0..=32.0).contains(&degree) || degree.fract() != 0.0 {
        return Err(GeometryError::BadProfile);
    }
    let degree = degree as usize;
    let cps = e
        .list("control_points_list")
        .ok_or(GeometryError::BadProfile)?;
    if cps.len() < 2 || cps.len() > geo.limits.max_curve_samples {
        return Err(GeometryError::BadProfile);
    }
    let mut control = Vec::with_capacity(cps.len());
    for p in cps {
        control.push(geo.point_ref(Some(p))?);
    }
    let weights: Option<Vec<f64>> = if e.is_a("RATIONAL_B_SPLINE_CURVE") {
        let w = e.list("weights_data").ok_or(GeometryError::BadProfile)?;
        Some(
            w.iter()
                .map(|v| number(v).ok_or(GeometryError::BadProfile))
                .collect::<GResult<Vec<f64>>>()?,
        )
    } else {
        None
    };
    let n = control.len();
    let curve = if e.is_a("B_SPLINE_CURVE_WITH_KNOTS") {
        let mults = int_list(e.list("knot_multiplicities"))?;
        let knots = real_list(e.list("knots"))?;
        BSplineCurve::new(degree, &control, weights.as_deref(), &knots, &mults)?
    } else if e.is_a("BEZIER_CURVE") {
        BSplineCurve::bezier(degree, &control, weights.as_deref())?
    } else if e.is_a("UNIFORM_CURVE") {
        // Knots −degree … n, all simple (ISO 10303-42 uniform_curve).
        let count = n + degree + 1;
        let knots: Vec<f64> = (0..count).map(|k| k as f64 - degree as f64).collect();
        let mults = vec![1usize; count];
        BSplineCurve::new(degree, &control, weights.as_deref(), &knots, &mults)?
    } else if e.is_a("QUASI_UNIFORM_CURVE") {
        // Knots 0 … n − degree, end multiplicities degree + 1.
        let spans = n.checked_sub(degree).ok_or(GeometryError::BadProfile)?;
        let knots: Vec<f64> = (0..=spans).map(|k| k as f64).collect();
        let mut mults = vec![1usize; spans + 1];
        mults[0] = degree + 1;
        mults[spans] = degree + 1;
        BSplineCurve::new(degree, &control, weights.as_deref(), &knots, &mults)?
    } else {
        return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
    };
    let (a, b) = curve.domain();
    let closed = dist(curve.point_at(a), curve.point_at(b))
        <= 1e-9
            * (1.0
                + control
                    .iter()
                    .map(|p| dist(*p, control[0]))
                    .fold(0.0, f64::max));
    Ok(Curve::BSpline { curve, closed })
}

/// A list of integers (multiplicities).
pub fn int_list(v: Option<&[Value]>) -> GResult<Vec<usize>> {
    let v = v.ok_or(GeometryError::BadProfile)?;
    v.iter()
        .map(|x| match number(x) {
            Some(n) if n >= 0.0 && n.fract() == 0.0 && n < 1e6 => Ok(n as usize),
            _ => Err(GeometryError::BadProfile),
        })
        .collect()
}

/// A list of reals.
pub fn real_list(v: Option<&[Value]>) -> GResult<Vec<f64>> {
    let v = v.ok_or(GeometryError::BadProfile)?;
    v.iter()
        .map(|x| number(x).ok_or(GeometryError::BadProfile))
        .collect()
}

/// A raw coordinate list value (`(x, y, z)`) as a point.
pub fn point_of_list(v: &Value) -> GResult<[f64; 3]> {
    coords3(v.as_list().ok_or(GeometryError::BadCoordinate)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circle_sampling_respects_tolerance() {
        let c = Curve::Conic {
            frame: Transform::IDENTITY,
            kind: Conic::Circle(10.0),
        };
        let ts = c.sample(0.0, core::f64::consts::PI, 0.01, 1.0, 10_000);
        for w in ts.windows(2) {
            let sag = 10.0 * (1.0 - ((w[1] - w[0]) / 2.0).cos());
            assert!(sag <= 0.01 + 1e-12);
        }
        assert_eq!(ts[0], 0.0);
        assert_eq!(*ts.last().unwrap(), core::f64::consts::PI);
        let p = c.eval(1.0);
        assert!((c.param_of(p) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn bspline_param_inversion_and_adaptive_sampling() {
        let curve = BSplineCurve::new(
            3,
            &[
                [0.0, 0.0, 0.0],
                [1.0, 2.0, 0.0],
                [3.0, -2.0, 0.0],
                [4.0, 0.0, 0.0],
            ],
            None,
            &[0.0, 1.0],
            &[4, 4],
        )
        .unwrap();
        let c = Curve::BSpline {
            curve,
            closed: false,
        };
        for &t in &[0.0, 0.2, 0.5, 0.9, 1.0] {
            let p = c.eval(t);
            assert!((c.param_of(p) - t).abs() < 1e-7, "{t}");
        }
        let ts = c.sample(0.0, 1.0, 1e-3, 0.3, 10_000);
        assert!(ts.len() > 8);
        for w in ts.windows(2) {
            let (a, b) = (c.eval(w[0]), c.eval(w[1]));
            let m = c.eval(0.5 * (w[0] + w[1]));
            let mid = scale(add(a, b), 0.5);
            assert!(dist(m, mid) <= 1e-3 + 1e-9);
        }
        // Reverse runs mirror.
        let rs = c.sample(1.0, 0.0, 1e-3, 0.3, 10_000);
        assert_eq!(rs.first(), Some(&1.0));
        assert_eq!(rs.last(), Some(&0.0));
    }

    #[test]
    fn polyline_params() {
        let c = Curve::Polyline(vec![[0.0; 3], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]);
        assert_eq!(c.sample(0.5, 2.0, 0.1, 0.1, 100), vec![0.5, 1.0, 2.0]);
        assert!((c.param_of([1.0, 0.5, 0.0]) - 1.5).abs() < 1e-12);
        assert_eq!(c.eval(1.5), [1.0, 0.5, 0.0]);
    }
}
