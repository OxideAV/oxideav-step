//! Face surfaces (ISO 10303-42 `surface` subtypes) resolved onto the
//! kernel's [`Surface`] parameterisations.
//!
//! * elementary: `plane`, `cylindrical_surface`, `conical_surface`
//!   (`semi_angle` in the context's angle unit), `spherical_surface`,
//!   `toroidal_surface`, `degenerate_toroidal_surface` (a spindle torus:
//!   the face loops pick the apple or the lemon);
//! * `b_spline_surface` (with knots / uniform / quasi-uniform / Bézier,
//!   rational through the complex-instance `weights_data`);
//! * swept: `surface_of_linear_extrusion`, `surface_of_revolution` —
//!   analytic when the swept curve makes them elementary (a line or a
//!   circle in the right position), else over the curve sampled at the
//!   context tolerance;
//! * `offset_surface` — the equivalent elementary surface for
//!   elementary bases, the kernel's general offset otherwise;
//! * a `rectangular_trimmed_surface` / `curve_bounded_surface` used as a
//!   face geometry stands for its basis (the face bounds trim it).

use oxideav_ifc::kernel::{BSplineSurface, Surface};
use oxideav_ifc::{GeometryError, Transform};

use super::curve::{int_list, real_list, Conic, Curve};
use super::{add, apply_dir, cross, dot, normalise, scale, sub, GResult, Geo};
use crate::schema::Entity;

const MAX_SURFACE_DEPTH: usize = 16;

/// Resolve surface `id`.
pub(super) fn resolve(geo: &mut Geo<'_>, id: u64, depth: usize) -> GResult<Surface> {
    if depth > MAX_SURFACE_DEPTH {
        return Err(GeometryError::Unsupported(
            "surface nesting too deep".into(),
        ));
    }
    let e = geo.entity(id)?;
    if e.is_a("ELEMENTARY_SURFACE") {
        let frame = geo.placement(
            e.reference("position")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let num = |n: &str| e.number(n).ok_or(GeometryError::BadProfile);
        if e.is_a("PLANE") {
            return Ok(Surface::plane(frame));
        }
        if e.is_a("CYLINDRICAL_SURFACE") {
            return Surface::cylinder(frame, num("radius")?);
        }
        if e.is_a("CONICAL_SURFACE") {
            return Surface::cone(frame, num("radius")?, num("semi_angle")? * geo.angle);
        }
        if e.is_a("SPHERICAL_SURFACE") {
            return Surface::sphere(frame, num("radius")?);
        }
        if e.is_a("TOROIDAL_SURFACE") {
            let (major, minor) = (num("major_radius")?, num("minor_radius")?);
            if minor < major {
                return Surface::torus(frame, major, minor);
            }
            return Surface::degenerate_torus(frame, major, minor);
        }
        return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
    }
    if e.is_a("B_SPLINE_SURFACE") {
        return bspline_surface(geo, e);
    }
    if e.is_a("SURFACE_OF_LINEAR_EXTRUSION") {
        let cid = e
            .reference("swept_curve")
            .ok_or(GeometryError::BadCoordinates)?;
        let dir = geo.vector(
            e.reference("extrusion_axis")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let dir = normalise(dir).ok_or(GeometryError::BadCoordinates)?;
        let curve = geo.curve(cid)?;
        return extrusion(geo, &curve, dir);
    }
    if e.is_a("SURFACE_OF_REVOLUTION") {
        let cid = e
            .reference("swept_curve")
            .ok_or(GeometryError::BadCoordinates)?;
        let (o, a) = geo.axis1(
            e.reference("axis_position")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let curve = geo.curve(cid)?;
        return revolution(geo, &curve, o, a);
    }
    if e.is_a("OFFSET_SURFACE") {
        let bid = e
            .reference("basis_surface")
            .ok_or(GeometryError::BadCoordinates)?;
        let d = e.number("distance").ok_or(GeometryError::BadCoordinate)?;
        return offset(geo, bid, d, depth);
    }
    if e.is_a("RECTANGULAR_TRIMMED_SURFACE") || e.is_a("CURVE_BOUNDED_SURFACE") {
        let bid = e
            .reference("basis_surface")
            .ok_or(GeometryError::BadCoordinates)?;
        return resolve(geo, bid, depth + 1);
    }
    Err(GeometryError::Unsupported(e.inst.keyword.clone()))
}

/// `offset_surface(basis, d)`: elementary bases stay elementary (the
/// offset of a plane / cylinder / sphere / torus / cone along its
/// `∂S/∂u × ∂S/∂v` normal is the same kind with a shifted origin or
/// radius); any other basis uses the kernel's general offset.
fn offset(geo: &mut Geo<'_>, bid: u64, d: f64, depth: usize) -> GResult<Surface> {
    let b = geo.entity(bid)?;
    if b.is_a("ELEMENTARY_SURFACE") {
        let frame = geo.placement(
            b.reference("position")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let num = |n: &str| b.number(n).ok_or(GeometryError::BadProfile);
        if b.is_a("PLANE") {
            let z = frame.cols[2];
            return Ok(Surface::plane(Transform {
                cols: frame.cols,
                translation: add(frame.translation, scale(z, d)),
            }));
        }
        if b.is_a("CYLINDRICAL_SURFACE") {
            return Surface::cylinder(frame, num("radius")? + d);
        }
        if b.is_a("SPHERICAL_SURFACE") {
            return Surface::sphere(frame, num("radius")? + d);
        }
        if b.is_a("TOROIDAL_SURFACE") {
            let (major, minor) = (num("major_radius")?, num("minor_radius")?);
            return Surface::torus(frame, major, minor + d);
        }
        if b.is_a("CONICAL_SURFACE") {
            // Points move by d·n, n = (cos u, sin u, −tan α)/√(1+tan²α):
            // the same cone with radius R + d·√(1 + tan²α) at the frame
            // origin (re-parameterised along the axis).
            let alpha = num("semi_angle")? * geo.angle;
            let t = alpha.tan();
            return Surface::cone(frame, num("radius")? + d * (1.0 + t * t).sqrt(), alpha);
        }
    }
    let base = resolve(geo, bid, depth + 1)?;
    Surface::offset(base, d)
}

/// The swept-curve run of a swept surface (bounded curves over their
/// domain; a full period for closed conics).
fn swept_points(geo: &Geo<'_>, c: &Curve) -> GResult<(Vec<[f64; 3]>, bool)> {
    let (a, b) = c
        .domain()
        .ok_or_else(|| GeometryError::Unsupported("unbounded swept curve".into()))?;
    let pts = c.sample_points(a, b, geo.tol, geo.max_angle, geo.limits.max_curve_samples);
    let closed = c.period().is_some();
    Ok((pts, closed))
}

fn extrusion(geo: &Geo<'_>, c: &Curve, dir: [f64; 3]) -> GResult<Surface> {
    match c {
        Curve::Line { origin, dir: ldir } => {
            // A plane through the line containing the extrusion axis.
            let x = normalise(*ldir).ok_or(GeometryError::BadCoordinates)?;
            let z = normalise(cross(x, dir)).ok_or(GeometryError::BadProfile)?;
            let y = cross(z, x);
            Ok(Surface::plane(Transform {
                cols: [x, y, z],
                translation: *origin,
            }))
        }
        Curve::Conic {
            frame,
            kind: Conic::Circle(r),
        } if dot(frame.cols[2], dir).abs() > 1.0 - 1e-12 => {
            // A right circular cylinder.
            Surface::cylinder(*frame, *r)
        }
        _ => {
            let (pts, closed) = swept_points(geo, c)?;
            Surface::extrusion(&pts, closed, dir)
        }
    }
}

fn revolution(geo: &Geo<'_>, c: &Curve, o: [f64; 3], a: [f64; 3]) -> GResult<Surface> {
    let frame_on_axis = |origin: [f64; 3]| -> Transform {
        let [x, y, z] = super::build_axes(Some(a), None);
        Transform {
            cols: [x, y, z],
            translation: origin,
        }
    };
    match c {
        Curve::Line { origin, dir } => {
            let d = normalise(*dir).ok_or(GeometryError::BadCoordinates)?;
            let q = sub(*origin, o);
            let h0 = dot(q, a);
            let radial = sub(q, scale(a, h0));
            let r0 = dot(radial, radial).sqrt();
            let da = dot(d, a);
            let d_perp = sub(d, scale(a, da));
            let skew = dot(cross(radial, d_perp), a).abs();
            if skew > 1e-9 * (1.0 + r0) {
                // A line skew to the axis sweeps a hyperboloid: sample it.
                return Err(GeometryError::Unsupported(
                    "surface of revolution of a skew line".into(),
                ));
            }
            if da.abs() > 1.0 - 1e-12 {
                // Parallel: a cylinder of radius r0.
                return Surface::cylinder(frame_on_axis(o), r0);
            }
            if da.abs() < 1e-12 {
                // Perpendicular: the plane through the line, normal = axis.
                return Ok(Surface::plane(frame_on_axis(add(o, scale(a, h0)))));
            }
            // A cone: apex where the line meets the axis.
            let dr = dot(d_perp, d_perp).sqrt()
                * if dot(d_perp, radial) >= 0.0 {
                    1.0
                } else {
                    -1.0
                };
            // Along the line, radius r(s) = r0 + s·dr, height h(s) = h0 + s·da.
            let s_apex = -r0 / dr;
            let apex_h = h0 + s_apex * da;
            let semi = (dr / da).abs().atan();
            // Cone axis oriented so the radius grows with v.
            let grows_up = (dr / da) > 0.0;
            let axis = if grows_up { a } else { scale(a, -1.0) };
            let [x, y, z] = super::build_axes(Some(axis), None);
            let frame = Transform {
                cols: [x, y, z],
                translation: add(o, scale(a, apex_h)),
            };
            Surface::cone(frame, 0.0, semi)
        }
        Curve::Conic {
            frame,
            kind: Conic::Circle(r),
        } => {
            // A circle whose plane contains the axis: sphere (centre on
            // the axis) or torus.
            let n = frame.cols[2];
            let c0 = sub(frame.translation, o);
            let in_plane = dot(n, a).abs() < 1e-9 && dot(c0, n).abs() < 1e-9 * (1.0 + *r);
            if in_plane {
                let h = dot(c0, a);
                let radial = sub(c0, scale(a, h));
                let big_r = dot(radial, radial).sqrt();
                let centre = add(o, scale(a, h));
                if big_r <= 1e-9 * (1.0 + *r) {
                    return Surface::sphere(frame_on_axis(centre), *r);
                }
                if *r < big_r {
                    return Surface::torus(frame_on_axis(centre), big_r, *r);
                }
            }
            let (pts, _) = swept_points(geo, c)?;
            Surface::revolution(&pts, o, a)
        }
        _ => {
            let (pts, _) = swept_points(geo, c)?;
            Surface::revolution(&pts, o, a)
        }
    }
}

/// Resolve a `b_spline_surface` family member.
fn bspline_surface(geo: &Geo<'_>, e: Entity<'_>) -> GResult<Surface> {
    let deg = |n: &str| -> GResult<usize> {
        let d = e.number(n).ok_or(GeometryError::BadProfile)?;
        if (1.0..=32.0).contains(&d) && d.fract() == 0.0 {
            Ok(d as usize)
        } else {
            Err(GeometryError::BadProfile)
        }
    };
    let (ud, vd) = (deg("u_degree")?, deg("v_degree")?);
    let rows = e
        .list("control_points_list")
        .ok_or(GeometryError::BadProfile)?;
    let mut control: Vec<Vec<[f64; 3]>> = Vec::with_capacity(rows.len());
    let mut total = 0usize;
    for r in rows {
        let cols = r.as_list().ok_or(GeometryError::BadProfile)?;
        total += cols.len();
        if total > 1_000_000 {
            return Err(GeometryError::BadProfile);
        }
        let mut row = Vec::with_capacity(cols.len());
        for p in cols {
            row.push(geo.point_ref(Some(p))?);
        }
        control.push(row);
    }
    let (nu, nv) = (control.len(), control.first().map_or(0, Vec::len));
    let weights: Option<Vec<Vec<f64>>> = if e.is_a("RATIONAL_B_SPLINE_SURFACE") {
        let w = e.list("weights_data").ok_or(GeometryError::BadProfile)?;
        let mut out = Vec::with_capacity(w.len());
        for r in w {
            out.push(real_list(r.as_list())?);
        }
        Some(out)
    } else {
        None
    };
    let closed = |n: &str| e.logical(n);
    let (uk, um, vk, vm) = if e.is_a("B_SPLINE_SURFACE_WITH_KNOTS") {
        (
            real_list(e.list("u_knots"))?,
            int_list(e.list("u_multiplicities"))?,
            real_list(e.list("v_knots"))?,
            int_list(e.list("v_multiplicities"))?,
        )
    } else if e.is_a("BEZIER_SURFACE") {
        let (uk, um) = bezier_knots(ud, nu)?;
        let (vk, vm) = bezier_knots(vd, nv)?;
        (uk, um, vk, vm)
    } else if e.is_a("UNIFORM_SURFACE") {
        let (uk, um) = uniform_knots(ud, nu);
        let (vk, vm) = uniform_knots(vd, nv);
        (uk, um, vk, vm)
    } else if e.is_a("QUASI_UNIFORM_SURFACE") {
        let (uk, um) = quasi_uniform_knots(ud, nu)?;
        let (vk, vm) = quasi_uniform_knots(vd, nv)?;
        (uk, um, vk, vm)
    } else {
        return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
    };
    let s = BSplineSurface::new(
        ud,
        vd,
        &control,
        weights.as_deref(),
        (&uk, &um),
        (&vk, &vm),
        closed("u_closed"),
        closed("v_closed"),
    )?;
    // A closed flag is only honoured when the patch really closes (the
    // kernel treats a closed direction as periodic).
    let (u0, u1) = s.u_domain();
    let (v0, v1) = s.v_domain();
    let really = |a: [f64; 3], b: [f64; 3]| super::dist(a, b) <= geo.tol.max(1e-9);
    let u_ok = closed("u_closed") != Some(true)
        || (0..=4).all(|k| {
            let v = v0 + (v1 - v0) * k as f64 / 4.0;
            really(s.point_at(u0, v), s.point_at(u1, v))
        });
    let v_ok = closed("v_closed") != Some(true)
        || (0..=4).all(|k| {
            let u = u0 + (u1 - u0) * k as f64 / 4.0;
            really(s.point_at(u, v0), s.point_at(u, v1))
        });
    if u_ok && v_ok {
        return Ok(Surface::bspline(s));
    }
    let s = BSplineSurface::new(
        ud,
        vd,
        &control,
        weights.as_deref(),
        (&uk, &um),
        (&vk, &vm),
        if u_ok {
            closed("u_closed")
        } else {
            Some(false)
        },
        if v_ok {
            closed("v_closed")
        } else {
            Some(false)
        },
    )?;
    Ok(Surface::bspline(s))
}

fn bezier_knots(degree: usize, n: usize) -> GResult<(Vec<f64>, Vec<usize>)> {
    if n < degree + 1 || (n - 1) % degree != 0 {
        return Err(GeometryError::BadProfile);
    }
    let spans = (n - 1) / degree;
    let knots = (0..=spans).map(|k| k as f64).collect();
    let mut mults = vec![degree; spans + 1];
    mults[0] = degree + 1;
    mults[spans] = degree + 1;
    Ok((knots, mults))
}

fn uniform_knots(degree: usize, n: usize) -> (Vec<f64>, Vec<usize>) {
    let count = n + degree + 1;
    (
        (0..count).map(|k| k as f64 - degree as f64).collect(),
        vec![1; count],
    )
}

fn quasi_uniform_knots(degree: usize, n: usize) -> GResult<(Vec<f64>, Vec<usize>)> {
    let spans = n.checked_sub(degree).ok_or(GeometryError::BadProfile)?;
    let knots = (0..=spans).map(|k| k as f64).collect();
    let mut mults = vec![1; spans + 1];
    mults[0] = degree + 1;
    mults[spans] = degree + 1;
    Ok((knots, mults))
}

/// The unit normal of a plane face geometry (a `plane`, or a trimmed /
/// curve-bounded surface on one): the plane's z axis.
pub fn plane_normal(geo: &Geo<'_>, id: u64) -> Option<[f64; 3]> {
    let mut e = geo.entity(id).ok()?;
    for _ in 0..MAX_SURFACE_DEPTH {
        if e.is_a("PLANE") {
            let frame = geo.placement(e.reference("position")?).ok()?;
            return normalise(apply_dir(&frame, [0.0, 0.0, 1.0]));
        }
        if e.is_a("RECTANGULAR_TRIMMED_SURFACE") || e.is_a("CURVE_BOUNDED_SURFACE") {
            e = geo.entity(e.reference("basis_surface")?).ok()?;
            continue;
        }
        return None;
    }
    None
}

/// True when face geometry `id` is planar (a `plane`, or a trimmed /
/// curve-bounded surface on one): its faces are triangulated directly.
pub fn is_plane(geo: &Geo<'_>, id: u64) -> bool {
    plane_normal(geo, id).is_some()
}

/// True when the refinement density of surface `id` depends on the
/// extent of the face meshed on it (cones, and offsets of them).
pub fn extent_dependent(geo: &Geo<'_>, id: u64) -> bool {
    let mut cur = id;
    for _ in 0..MAX_SURFACE_DEPTH {
        let Ok(e) = geo.entity(cur) else { return false };
        if e.is_a("CONICAL_SURFACE") {
            return true;
        }
        if e.is_a("OFFSET_SURFACE")
            || e.is_a("RECTANGULAR_TRIMMED_SURFACE")
            || e.is_a("CURVE_BOUNDED_SURFACE")
        {
            match e.reference("basis_surface") {
                Some(b) => cur = b,
                None => return false,
            }
            continue;
        }
        return false;
    }
    false
}
