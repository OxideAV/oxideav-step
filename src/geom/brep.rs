//! Boundary-representation tessellation (ISO 10303-42 topology).
//!
//! Solids (`manifold_solid_brep`, `brep_with_voids`, `faceted_brep`),
//! surface models (`shell_based_surface_model`,
//! `face_based_surface_model`), shells (closed / open / oriented /
//! connected face sets) and faces (`advanced_face`, `face_surface`,
//! `oriented_face`, poly-loop `face`) are meshed into one shared
//! [`FaceMesher`]:
//!
//! * every `vertex_point` is one mesh vertex;
//! * every `edge_curve` is sampled **once** (between its vertices, in
//!   its own direction, at the context tolerance) and the run is reused
//!   — reversed per `oriented_edge.orientation` — by both faces that
//!   share it, so solids come out watertight;
//! * a face's loops (`face_bound.orientation` applied) are triangulated
//!   directly on a plane, or inverted into the face surface's `(u, v)`
//!   domain and trimmed there (the kernel's trimmed-face tessellator);
//!   `same_sense` relates the face normal to the surface normal, and a
//!   face reached through a reversed `oriented_face` / `oriented_*_shell`
//!   has its triangles flipped afterwards;
//! * a closed solid shell is re-oriented outward by its signed volume,
//!   voids inward.
//!
//! A face that cannot be meshed is skipped with a warning; the rest of
//! the shell is kept.

use std::collections::HashMap;

use oxideav_ifc::kernel::FaceMesher;
use oxideav_ifc::{GeometryError, TriMesh};

use super::curve::{trimmed, Curve};
use super::{cross, dist, dot, sub, surface, GResult, Geo};

/// Nesting bound for oriented-edge / subedge / oriented-face chains.
const MAX_TOPO_DEPTH: usize = 16;

/// The meshed result of one or more B-rep items.
#[derive(Debug, Clone, Default)]
pub struct BrepMesh {
    /// The triangle mesh (model units).
    pub mesh: TriMesh,
    /// The `face` instance id each triangle belongs to.
    pub face_of_triangle: Vec<u64>,
}

/// Meshes topological items into one shared vertex pool.
#[derive(Debug)]
pub struct BrepMesher<'g, 'a> {
    geo: &'g mut Geo<'a>,
    m: FaceMesher,
    vertex: HashMap<u64, u32>,
    point: HashMap<u64, u32>,
    runs: HashMap<u64, Vec<u32>>,
    faces: Vec<u64>,
    /// Non-fatal problems (skipped faces / loops).
    pub warnings: Vec<String>,
}

impl<'g, 'a> BrepMesher<'g, 'a> {
    /// A mesher over `geo`.
    pub fn new(geo: &'g mut Geo<'a>) -> Self {
        Self {
            geo,
            m: FaceMesher::new(),
            vertex: HashMap::new(),
            point: HashMap::new(),
            runs: HashMap::new(),
            faces: Vec::new(),
            warnings: Vec::new(),
        }
    }

    /// Triangles produced so far.
    pub fn triangle_count(&self) -> usize {
        self.m.triangle_count()
    }

    /// Finish into a mesh with per-triangle face ids.
    pub fn finish(self) -> BrepMesh {
        let (mesh, tags) = self.m.finish_tagged();
        let face_of_triangle = tags
            .into_iter()
            .map(|t| self.faces.get(t as usize).copied().unwrap_or(0))
            .collect();
        BrepMesh {
            mesh,
            face_of_triangle,
        }
    }

    fn budget(&self) -> GResult<()> {
        if self.m.triangle_count() > self.geo.limits.max_triangles {
            return Err(GeometryError::Unsupported(
                "tessellation triangle budget exhausted".into(),
            ));
        }
        Ok(())
    }

    /// Mesh one topological representation item (solid, surface model,
    /// shell or face).
    pub fn item(&mut self, id: u64) -> GResult<()> {
        let e = self.geo.entity(id)?;
        if e.is_a("MANIFOLD_SOLID_BREP") {
            let start = self.m.triangle_count();
            let outer = e.reference("outer").ok_or(GeometryError::BadCoordinates)?;
            self.shell(outer, false, 0)?;
            self.orient_closed(start, true);
            if e.is_a("BREP_WITH_VOIDS") {
                if let Some(voids) = e.list("voids") {
                    for v in voids {
                        let Some(vid) = v.as_reference() else {
                            continue;
                        };
                        let s = self.m.triangle_count();
                        self.shell(vid, false, 0)?;
                        self.orient_closed(s, false);
                    }
                }
            }
            return Ok(());
        }
        if e.is_a("SHELL_BASED_SURFACE_MODEL") {
            for s in e.list("sbsm_boundary").unwrap_or(&[]) {
                if let Some(sid) = s.as_reference() {
                    self.shell(sid, false, 0)?;
                }
            }
            return Ok(());
        }
        if e.is_a("FACE_BASED_SURFACE_MODEL") {
            for s in e.list("fbsm_faces").unwrap_or(&[]) {
                if let Some(sid) = s.as_reference() {
                    self.shell(sid, false, 0)?;
                }
            }
            return Ok(());
        }
        if e.is_a("CONNECTED_FACE_SET") {
            let start = self.m.triangle_count();
            self.shell(id, false, 0)?;
            if e.is_a("CLOSED_SHELL") {
                self.orient_closed(start, true);
            }
            return Ok(());
        }
        if e.is_a("FACE") {
            return self.face(id, false, 0);
        }
        Err(GeometryError::Unsupported(e.inst.keyword.clone()))
    }

    /// Re-orient the triangles from `start` on so their signed volume
    /// is positive (`outward`) or negative (a void).
    fn orient_closed(&mut self, start: usize, outward: bool) {
        let v = self.m.signed_volume_from(start);
        if (v < 0.0 && outward) || (v > 0.0 && !outward) {
            self.m.reverse_from(start);
        }
    }

    /// Mesh a shell (connected face set, possibly oriented).
    fn shell(&mut self, id: u64, flip: bool, depth: usize) -> GResult<()> {
        if depth > MAX_TOPO_DEPTH {
            return Err(GeometryError::BadCoordinates);
        }
        let e = self.geo.entity(id)?;
        if e.is_a("ORIENTED_CLOSED_SHELL") || e.is_a("ORIENTED_OPEN_SHELL") {
            let el = e
                .reference("closed_shell_element")
                .or_else(|| e.reference("open_shell_element"))
                .ok_or(GeometryError::BadCoordinates)?;
            let same = e.logical("orientation").unwrap_or(true);
            return self.shell(el, flip ^ !same, depth + 1);
        }
        if !e.is_a("CONNECTED_FACE_SET") {
            return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
        }
        let faces = e.list("cfs_faces").ok_or(GeometryError::BadCoordinates)?;
        for f in faces {
            let Some(fid) = f.as_reference() else {
                continue;
            };
            self.budget()?;
            if let Err(err) = self.face(fid, flip, 0) {
                self.warnings.push(format!("face #{fid} skipped: {err}"));
            }
        }
        Ok(())
    }

    /// Mesh one face.
    fn face(&mut self, id: u64, flip: bool, depth: usize) -> GResult<()> {
        if depth > MAX_TOPO_DEPTH {
            return Err(GeometryError::BadCoordinates);
        }
        let e = self.geo.entity(id)?;
        if e.is_a("ORIENTED_FACE") {
            let el = e
                .reference("face_element")
                .ok_or(GeometryError::BadCoordinates)?;
            let same = e.logical("orientation").unwrap_or(true);
            return self.face(el, flip ^ !same, depth + 1);
        }
        if !e.is_a("FACE") {
            return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
        }
        let surface_id = if e.is_a("FACE_SURFACE") {
            Some(
                e.reference("face_geometry")
                    .ok_or(GeometryError::BadCoordinates)?,
            )
        } else {
            None
        };
        let same_sense = e.logical("same_sense").unwrap_or(true);
        let bounds = e.list("bounds").ok_or(GeometryError::BadCoordinates)?;
        let mut loops: Vec<(Vec<u32>, bool)> = Vec::with_capacity(bounds.len());
        for b in bounds {
            let Some(bid) = b.as_reference() else {
                continue;
            };
            let be = self.geo.entity(bid)?;
            let lid = be.reference("bound").ok_or(GeometryError::BadCoordinates)?;
            let ori = be.logical("orientation").unwrap_or(true);
            let outer = be.is_a("FACE_OUTER_BOUND");
            match self.loop_ids(lid) {
                Ok(Some(mut ids)) => {
                    if !ori {
                        ids.reverse();
                    }
                    loops.push((ids, outer));
                }
                Ok(None) => {}
                Err(err) => {
                    self.warnings
                        .push(format!("face #{id}: loop #{lid} skipped: {err}"));
                }
            }
        }
        if loops.is_empty() {
            return Ok(());
        }
        let planar = match surface_id {
            None => true,
            Some(sid) => surface::is_plane(self.geo, sid),
        };
        let normal = surface_id.and_then(|sid| surface::plane_normal(self.geo, sid));
        // Outer loop first. On a plane it is the loop enclosing the
        // largest area (writers do mislabel a hole as the
        // FACE_OUTER_BOUND); on a curved surface the declared outer
        // bound, else the first.
        let outer_idx = if planar {
            let mut best = (0usize, -1.0f64);
            for (i, (l, _)) in loops.iter().enumerate() {
                let a = loop_area(&self.m, l, normal);
                if a > best.1 {
                    best = (i, a);
                }
            }
            best.0
        } else {
            loops.iter().position(|(_, o)| *o).unwrap_or(0)
        };
        let outer = loops.remove(outer_idx).0;
        let holes: Vec<Vec<u32>> = loops.into_iter().map(|(l, _)| l).collect();

        let tag = self.faces.len() as u32;
        self.faces.push(id);
        self.m.set_tag(tag);
        let start = self.m.triangle_count();
        let result = if planar {
            self.m.add_planar_face(&outer, &holes).map(|()| {
                // Orient against the plane normal (× same_sense); a
                // surface-less (poly-loop) face keeps its loop winding.
                if let Some(n) = normal {
                    let want = if same_sense { n } else { super::scale(n, -1.0) };
                    let got = normal_sum(&self.m, start);
                    if dot(got, want) < 0.0 {
                        self.m.reverse_from(start);
                    }
                }
            })
        } else {
            let sid = surface_id.unwrap_or(0);
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for &v in outer.iter().chain(holes.iter().flatten()) {
                if let Some(p) = self.m.position(v) {
                    for k in 0..3 {
                        lo[k] = lo[k].min(p[k]);
                        hi[k] = hi[k].max(p[k]);
                    }
                }
            }
            let extent = if lo[0] <= hi[0] { dist(lo, hi) } else { 0.0 };
            match self.geo.surface(sid, extent) {
                Ok(s) => {
                    let mut all = Vec::with_capacity(holes.len() + 1);
                    all.push(outer);
                    all.extend(holes);
                    self.m.add_surface_face(&s, sid, &all, same_sense)
                }
                Err(err) => Err(err),
            }
        };
        match result {
            Ok(()) => {
                if flip {
                    self.m.reverse_from(start);
                }
                Ok(())
            }
            Err(err) => {
                self.m.truncate(start);
                Err(err)
            }
        }
    }

    /// The vertex ids of a loop (`None` for a vertex loop, which bounds
    /// no area).
    fn loop_ids(&mut self, id: u64) -> GResult<Option<Vec<u32>>> {
        let e = self.geo.entity(id)?;
        if e.is_a("VERTEX_LOOP") {
            return Ok(None);
        }
        let mut out: Vec<u32> = Vec::new();
        if e.is_a("POLY_LOOP") {
            for p in e.list("polygon").ok_or(GeometryError::BadCoordinates)? {
                let pid = p.as_reference().ok_or(GeometryError::BadCoordinates)?;
                let v = match self.point.get(&pid) {
                    Some(&v) => v,
                    None => {
                        let pt = self.geo.point(pid)?;
                        let v = self.m.add_vertex(pt);
                        self.point.insert(pid, v);
                        v
                    }
                };
                out.push(v);
            }
        } else if e.is_a("EDGE_LOOP") {
            for ed in e.list("edge_list").ok_or(GeometryError::BadCoordinates)? {
                let eid = ed.as_reference().ok_or(GeometryError::BadCoordinates)?;
                let mut run = self.edge_run(eid, 0)?;
                // Chain head-to-tail; tolerate an edge listed backwards.
                if let Some(&last) = out.last() {
                    if run.first() != Some(&last) && run.last() == Some(&last) {
                        run.reverse();
                    }
                }
                let skip = usize::from(out.last().is_some() && run.first() == out.last());
                out.extend(run.into_iter().skip(skip));
                if out.len() > self.geo.limits.max_curve_samples * 4 {
                    return Err(GeometryError::BadProfile);
                }
            }
        } else {
            return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
        }
        out.dedup();
        while out.len() > 1 && out.first() == out.last() {
            out.pop();
        }
        if out.len() < 3 {
            return Err(GeometryError::IndexOutOfRange);
        }
        Ok(Some(out))
    }

    fn vertex(&mut self, id: u64) -> GResult<u32> {
        if let Some(&v) = self.vertex.get(&id) {
            return Ok(v);
        }
        let e = self.geo.entity(id)?;
        if !e.is_a("VERTEX_POINT") {
            return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
        }
        let p = self.geo.point_ref(e.attr("vertex_geometry"))?;
        let v = self.m.add_vertex(p);
        self.vertex.insert(id, v);
        Ok(v)
    }

    /// The sampled vertex run of an edge in its own direction (start
    /// vertex first, end vertex last), cached per edge.
    fn edge_run(&mut self, id: u64, depth: usize) -> GResult<Vec<u32>> {
        if let Some(r) = self.runs.get(&id) {
            return Ok(r.clone());
        }
        if depth > MAX_TOPO_DEPTH {
            return Err(GeometryError::BadCoordinates);
        }
        let e = self.geo.entity(id)?;
        if e.is_a("ORIENTED_EDGE") {
            let el = e
                .reference("edge_element")
                .ok_or(GeometryError::BadCoordinates)?;
            let mut run = self.edge_run(el, depth + 1)?;
            if e.logical("orientation") == Some(false) {
                run.reverse();
            }
            return Ok(run);
        }
        if !e.is_a("EDGE") {
            return Err(GeometryError::Unsupported(e.inst.keyword.clone()));
        }
        let vs = self.vertex(
            e.reference("edge_start")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let ve = self.vertex(
            e.reference("edge_end")
                .ok_or(GeometryError::BadCoordinates)?,
        )?;
        let mut run = vec![vs];
        if e.is_a("EDGE_CURVE") {
            let gid = e
                .reference("edge_geometry")
                .ok_or(GeometryError::BadCoordinates)?;
            let same = e.logical("same_sense").unwrap_or(true);
            let (curve, sense) = self.edge_curve(gid, same, 0)?;
            let ps = self.m.position(vs).unwrap_or_default();
            let pe = self.m.position(ve).unwrap_or_default();
            let closed = vs == ve || dist(ps, pe) <= self.geo.tol * 1e-3;
            for p in edge_interior(self.geo, &curve, ps, pe, sense, closed) {
                run.push(self.m.add_vertex(p));
            }
        } else if e.is_a("SUBEDGE") {
            let parent = e
                .reference("parent_edge")
                .ok_or(GeometryError::BadCoordinates)?;
            let prun = self.edge_run(parent, depth + 1)?;
            let pos: Vec<[f64; 3]> = prun
                .iter()
                .map(|&v| self.m.position(v).unwrap_or_default())
                .collect();
            let near = |p: [f64; 3]| {
                pos.iter()
                    .enumerate()
                    .min_by(|a, b| {
                        dist(*a.1, p)
                            .partial_cmp(&dist(*b.1, p))
                            .unwrap_or(core::cmp::Ordering::Equal)
                    })
                    .map_or(0, |(i, _)| i)
            };
            let a = near(self.m.position(vs).unwrap_or_default());
            let b = near(self.m.position(ve).unwrap_or_default());
            if a < b {
                run.extend(prun[a + 1..b].iter().copied());
            } else if b < a {
                run.extend(prun[b + 1..a].iter().rev().copied());
            }
        }
        run.push(ve);
        self.runs.insert(id, run.clone());
        Ok(run)
    }

    /// The curve an edge runs along and whether the edge follows the
    /// curve's parameter direction.
    fn edge_curve(&mut self, gid: u64, same: bool, depth: usize) -> GResult<(Curve, bool)> {
        if depth > MAX_TOPO_DEPTH {
            return Err(GeometryError::BadCoordinates);
        }
        let g = self.geo.entity(gid)?;
        if g.is_a("TRIMMED_CURVE") {
            let (basis, _, _) = trimmed(self.geo, g, 0)?;
            let agree = g
                .attr("sense_agreement")
                .and_then(crate::schema::logical)
                .unwrap_or(true);
            return Ok((basis, same == agree));
        }
        if g.is_a("SURFACE_CURVE") {
            if let Some(c3) = g.reference("curve_3d") {
                let c3e = self.geo.entity(c3)?;
                if !c3e.is_a("PCURVE") {
                    if let Ok(r) = self.edge_curve(c3, same, depth + 1) {
                        return Ok(r);
                    }
                }
            }
        }
        let c = self.geo.curve(gid)?;
        Ok(((*c).clone(), same))
    }
}

/// Interior sample points of an edge from `ps` to `pe` along `curve`
/// (`sense`: in the curve's parameter direction).
fn edge_interior(
    geo: &Geo<'_>,
    curve: &Curve,
    ps: [f64; 3],
    pe: [f64; 3],
    sense: bool,
    closed: bool,
) -> Vec<[f64; 3]> {
    if matches!(curve, Curve::Line { .. }) {
        return Vec::new();
    }
    let ts = curve.param_of(ps);
    let mut te = curve.param_of(pe);
    if let Some(p) = curve.period() {
        let eps = 1e-9 * p;
        if sense {
            te = ts + (te - ts).rem_euclid(p);
            if te - ts <= eps || (closed && te - ts >= p - eps) {
                te = if closed { ts + p } else { ts };
            }
        } else {
            te = ts - (ts - te).rem_euclid(p);
            if ts - te <= eps || (closed && ts - te >= p - eps) {
                te = if closed { ts - p } else { ts };
            }
        }
    } else if closed {
        // A closed edge on an open curve: the whole domain.
        if let Some((a, b)) = curve.domain() {
            if sense {
                return interior_points(geo, curve, a, b);
            }
            return interior_points(geo, curve, b, a);
        }
    }
    interior_points(geo, curve, ts, te)
}

fn interior_points(geo: &Geo<'_>, curve: &Curve, t0: f64, t1: f64) -> Vec<[f64; 3]> {
    let ts = curve.sample(t0, t1, geo.tol, geo.max_angle, geo.limits.max_curve_samples);
    if ts.len() <= 2 {
        return Vec::new();
    }
    ts[1..ts.len() - 1].iter().map(|&t| curve.eval(t)).collect()
}

/// Sum of the (area-weighted) normals of the triangles from `start` on.
fn normal_sum(m: &FaceMesher, start: usize) -> [f64; 3] {
    let pos = m.positions();
    let mut n = [0.0; 3];
    for t in m.triangles().iter().skip(start) {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let c = cross(sub(b, a), sub(c, a));
        n = super::add(n, c);
    }
    n
}

/// Projected area of a loop (onto `normal` when given, else its Newell
/// normal's magnitude).
fn loop_area(m: &FaceMesher, ids: &[u32], normal: Option<[f64; 3]>) -> f64 {
    let pos = m.positions();
    let mut n = [0.0; 3];
    for i in 0..ids.len() {
        let a = pos.get(ids[i] as usize).copied().unwrap_or_default();
        let b = pos
            .get(ids[(i + 1) % ids.len()] as usize)
            .copied()
            .unwrap_or_default();
        n = super::add(n, cross(a, b));
    }
    match normal {
        Some(d) => dot(n, d).abs() * 0.5,
        None => dot(n, n).sqrt() * 0.5,
    }
}

/// True when a representation item is a topological / B-rep item this
/// module meshes.
pub fn is_brep_item(geo: &Geo<'_>, id: u64) -> bool {
    geo.entity(id).is_ok_and(|e| {
        e.is_a("MANIFOLD_SOLID_BREP")
            || e.is_a("SHELL_BASED_SURFACE_MODEL")
            || e.is_a("FACE_BASED_SURFACE_MODEL")
            || e.is_a("CONNECTED_FACE_SET")
            || e.is_a("FACE")
    })
}
