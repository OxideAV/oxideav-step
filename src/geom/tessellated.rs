//! AP242 tessellated geometry (ISO 10303-42 tessellated items, CAx-IF
//! "3D Tessellated Geometry" recommended practice).
//!
//! * `coordinates_list(name, npoints, position_coords)` — the shared
//!   point table;
//! * `triangulated_face(name, coordinates, pnmax, normals,
//!   geometric_link, pnindex, triangles)` /
//!   `triangulated_surface_set(name, coordinates, pnmax, normals,
//!   pnindex, triangles)` — one-based triangle indices into `pnindex`
//!   (or straight into the coordinates when `pnindex` is empty);
//! * `complex_triangulated_face` / `complex_triangulated_surface_set` —
//!   the same with `triangle_strips` (alternating winding) and
//!   `triangle_fans` (shared first vertex);
//! * `tessellated_solid(name, items, geometric_link)` /
//!   `tessellated_shell(name, items, topological_link)` — face
//!   collections; faces sharing a coordinate list share vertices, so a
//!   closed tessellated solid stays watertight.

use std::collections::HashMap;

use oxideav_ifc::{GeometryError, TriMesh, Value};

use super::brep::BrepMesh;
use super::{coords3, GResult, Geo};
use crate::schema::number;

/// Accumulates tessellated items into one mesh.
#[derive(Debug, Default)]
pub struct TessMesher {
    positions: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
    faces: Vec<u64>,
    vertex: HashMap<(u64, usize), u32>,
    coords: HashMap<u64, Vec<[f64; 3]>>,
}

const MAX_TESS_DEPTH: usize = 8;

impl TessMesher {
    /// An empty mesher.
    pub fn new() -> Self {
        Self::default()
    }

    /// Finish into a mesh with per-triangle (face / set) item ids.
    pub fn finish(self) -> BrepMesh {
        BrepMesh {
            mesh: TriMesh {
                positions: self.positions,
                triangles: self.triangles,
            },
            face_of_triangle: self.faces,
        }
    }

    /// Mesh one tessellated item.
    pub fn item(&mut self, geo: &Geo<'_>, id: u64) -> GResult<()> {
        self.item_depth(geo, id, 0)
    }

    fn item_depth(&mut self, geo: &Geo<'_>, id: u64, depth: usize) -> GResult<()> {
        if depth > MAX_TESS_DEPTH {
            return Err(GeometryError::BadCoordinates);
        }
        let e = geo.entity(id)?;
        if e.is_a("TESSELLATED_SOLID") || e.is_a("TESSELLATED_SHELL") {
            for it in e.list("items").ok_or(GeometryError::BadCoordinates)? {
                if let Some(fid) = it.as_reference() {
                    self.item_depth(geo, fid, depth + 1)?;
                }
            }
            return Ok(());
        }
        if e.is_a("TESSELLATED_FACE") || e.is_a("TESSELLATED_SURFACE_SET") {
            let cid = e
                .reference("coordinates")
                .ok_or(GeometryError::BadCoordinates)?;
            let n = self.coordinates(geo, cid)?;
            let pn: Vec<usize> = match e.list("pnindex") {
                Some(l) => l
                    .iter()
                    .map(|v| index(v).ok_or(GeometryError::IndexOutOfRange))
                    .collect::<GResult<_>>()?,
                None => Vec::new(),
            };
            // Resolve one-based index → coordinate index.
            let resolve = |i: usize| -> Option<usize> {
                let j = if pn.is_empty() {
                    i
                } else {
                    *pn.get(i.checked_sub(1)?)?
                };
                let j = j.checked_sub(1)?;
                (j < n).then_some(j)
            };
            let mut tris: Vec<[usize; 3]> = Vec::new();
            if let Some(list) = e.list("triangles") {
                for t in list {
                    let t = t.as_list().ok_or(GeometryError::BadCoordinates)?;
                    if t.len() != 3 {
                        return Err(GeometryError::IndexOutOfRange);
                    }
                    let mut tri = [0usize; 3];
                    for (k, v) in t.iter().enumerate() {
                        tri[k] = index(v)
                            .and_then(resolve)
                            .ok_or(GeometryError::IndexOutOfRange)?;
                    }
                    tris.push(tri);
                }
            }
            if let Some(strips) = e.list("triangle_strips") {
                for s in strips {
                    let s: Vec<usize> = idx_list(s, &resolve)?;
                    for i in 2..s.len() {
                        if i % 2 == 0 {
                            tris.push([s[i - 2], s[i - 1], s[i]]);
                        } else {
                            tris.push([s[i - 1], s[i - 2], s[i]]);
                        }
                    }
                }
            }
            if let Some(fans) = e.list("triangle_fans") {
                for f in fans {
                    let f: Vec<usize> = idx_list(f, &resolve)?;
                    for i in 2..f.len() {
                        tris.push([f[0], f[i - 1], f[i]]);
                    }
                }
            }
            if self.triangles.len() + tris.len() > geo.limits.max_triangles {
                return Err(GeometryError::Unsupported(
                    "tessellation triangle budget exhausted".into(),
                ));
            }
            for t in tris {
                if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                    continue;
                }
                let a = self.vertex(cid, t[0]);
                let b = self.vertex(cid, t[1]);
                let c = self.vertex(cid, t[2]);
                self.triangles.push([a, b, c]);
                self.faces.push(id);
            }
            return Ok(());
        }
        Err(GeometryError::Unsupported(e.inst.keyword.clone()))
    }

    fn coordinates(&mut self, geo: &Geo<'_>, cid: u64) -> GResult<usize> {
        if let Some(c) = self.coords.get(&cid) {
            return Ok(c.len());
        }
        let e = geo.entity(cid)?;
        if !e.is_a("COORDINATES_LIST") {
            return Err(GeometryError::BadCoordinates);
        }
        let rows = e
            .list("position_coords")
            .ok_or(GeometryError::BadCoordinates)?;
        let mut pts = Vec::with_capacity(rows.len());
        for r in rows {
            pts.push(coords3(r.as_list().ok_or(GeometryError::BadCoordinate)?)?);
        }
        let n = pts.len();
        self.coords.insert(cid, pts);
        Ok(n)
    }

    fn vertex(&mut self, cid: u64, j: usize) -> u32 {
        if let Some(&v) = self.vertex.get(&(cid, j)) {
            return v;
        }
        let p = self
            .coords
            .get(&cid)
            .and_then(|c| c.get(j))
            .copied()
            .unwrap_or_default();
        let v = self.positions.len() as u32;
        self.positions.push(p);
        self.vertex.insert((cid, j), v);
        v
    }
}

fn index(v: &Value) -> Option<usize> {
    let n = number(v)?;
    (n >= 1.0 && n.fract() == 0.0 && n < 1e12).then_some(n as usize)
}

fn idx_list(v: &Value, resolve: &dyn Fn(usize) -> Option<usize>) -> GResult<Vec<usize>> {
    v.as_list()
        .ok_or(GeometryError::BadCoordinates)?
        .iter()
        .map(|x| {
            index(x)
                .and_then(resolve)
                .ok_or(GeometryError::IndexOutOfRange)
        })
        .collect()
}

/// True for a tessellated item this module meshes.
pub fn is_tessellated_item(geo: &Geo<'_>, id: u64) -> bool {
    geo.entity(id).is_ok_and(|e| {
        e.is_a("TESSELLATED_SOLID")
            || e.is_a("TESSELLATED_SHELL")
            || e.is_a("TESSELLATED_FACE")
            || e.is_a("TESSELLATED_SURFACE_SET")
    })
}
