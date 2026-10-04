//! Hand-built STEP fixtures (`tests/fixtures/*.stp`): exact B-rep
//! solids (planar, cylindrical, conical, B-spline faces), per-face
//! colours, an assembly with NAUO / CDSR and mapped-item occurrences in
//! mixed units, and AP242 tessellated geometry.

use std::collections::HashMap;
use std::path::PathBuf;

use oxideav_step::{read_step, ApSchema, Occurrence, StepModel, Transform, TriMesh};

fn load(name: &str) -> StepModel {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name);
    let bytes = std::fs::read(&p).unwrap();
    read_step(&bytes).unwrap_or_else(|e| panic!("{name}: {e}"))
}

/// Number of edges not matched by an opposite half-edge, after welding
/// coincident positions (0 = closed, consistently oriented).
fn open_edges(m: &TriMesh) -> usize {
    let mut weld: HashMap<[i64; 3], u32> = HashMap::new();
    let ids: Vec<u32> = m
        .positions
        .iter()
        .map(|p| {
            let k = [
                (p[0] * 1e7).round() as i64,
                (p[1] * 1e7).round() as i64,
                (p[2] * 1e7).round() as i64,
            ];
            let n = weld.len() as u32;
            *weld.entry(k).or_insert(n)
        })
        .collect();
    let mut count: HashMap<(u32, u32), i32> = HashMap::new();
    for t in &m.triangles {
        for i in 0..3 {
            let (a, b) = (ids[t[i] as usize], ids[t[(i + 1) % 3] as usize]);
            if a == b {
                continue;
            }
            *count.entry((a, b)).or_default() += 1;
            *count.entry((b, a)).or_default() -= 1;
        }
    }
    count.values().filter(|&&c| c != 0).count() / 2
}

fn single_shape(m: &StepModel) -> &oxideav_step::Shape {
    let shapes: Vec<_> = m.parts.iter().flat_map(|p| &p.shapes).collect();
    assert_eq!(shapes.len(), 1, "{:?}", m.warnings);
    shapes[0]
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1e-12)
}

#[test]
fn cube() {
    let m = load("cube.stp");
    assert_eq!(m.schema, ApSchema::Ap214);
    assert_eq!(m.length_unit_metres, 0.001);
    assert_eq!(m.roots.len(), 1);
    assert_eq!(m.parts[m.roots[0].part].name.as_deref(), Some("cube"));
    let s = single_shape(&m);
    assert_eq!(s.mesh.triangles.len(), 12);
    assert_eq!(open_edges(&s.mesh), 0);
    assert!(close(s.mesh.signed_volume(), 1000.0, 1e-12));
    assert_eq!(s.colour, Some([0.8, 0.2, 0.2, 1.0]));
    assert!(m.warnings.is_empty(), "{:?}", m.warnings);
}

#[test]
fn cylinder_with_seam() {
    let m = load("cylinder.stp");
    assert_eq!(m.schema, ApSchema::Ap203);
    let s = single_shape(&m);
    assert_eq!(open_edges(&s.mesh), 0);
    let exact = core::f64::consts::PI * 25.0 * 10.0;
    assert!(
        close(s.mesh.signed_volume(), exact, 0.01),
        "{}",
        s.mesh.signed_volume()
    );
    assert!(m.warnings.is_empty(), "{:?}", m.warnings);
}

#[test]
fn plate_with_hole_and_face_colours() {
    let m = load("plate_with_hole.stp");
    let s = single_shape(&m);
    assert_eq!(open_edges(&s.mesh), 0);
    let exact = 20.0 * 20.0 * 4.0 - core::f64::consts::PI * 9.0 * 4.0;
    assert!(
        close(s.mesh.signed_volume(), exact, 0.01),
        "{}",
        s.mesh.signed_volume()
    );
    assert_eq!(s.colour, Some([0.6, 0.6, 0.6, 1.0]));
    assert_eq!(s.triangle_colours.len(), s.mesh.triangles.len());
    let blue = s
        .triangle_colours
        .iter()
        .filter(|c| **c == Some([0.1, 0.3, 0.9, 1.0]))
        .count();
    let yellow = s
        .triangle_colours
        .iter()
        .filter(|c| **c == Some([1.0, 0.8, 0.0, 1.0]))
        .count();
    assert!(blue > 0 && yellow > 0, "{blue} {yellow}");
    assert!(m.warnings.is_empty(), "{:?}", m.warnings);
}

#[test]
fn bspline_top_face() {
    let m = load("bspline_box.stp");
    assert_eq!(m.schema, ApSchema::Ap242);
    let s = single_shape(&m);
    assert_eq!(open_edges(&s.mesh), 0);
    // Box 10×10×2 plus the bulge: the bicubic Bézier with interior
    // control heights +3 adds 3·(4/9)² · 100 · … — check it lies between
    // the flat box and the control-net hull.
    let v = s.mesh.signed_volume();
    assert!(v > 200.0 && v < 200.0 + 300.0 * 4.0 / 9.0, "{v}");
    // The exact bulge volume: ∫∫ 3·B(u)B(v) with B = 3u(1−u) (sum of the
    // two interior Bernstein cubics) = 3 · (1/2)² · 100 = 75.
    assert!(close(v, 275.0, 0.01), "{v}");
    let top = s.mesh.positions.iter().map(|p| p[2]).fold(0.0, f64::max);
    // Peak 2 + 3·(3/4)² = 3.6875 at the centre.
    assert!((top - 3.6875).abs() < 0.05, "{top}");
    assert!(m.warnings.is_empty(), "{:?}", m.warnings);
}

#[test]
fn cone_with_apex_vertex_loop() {
    let m = load("cone.stp");
    let s = single_shape(&m);
    assert_eq!(open_edges(&s.mesh), 0);
    let exact = core::f64::consts::PI * 25.0 * 8.0 / 3.0;
    assert!(
        close(s.mesh.signed_volume(), exact, 0.02),
        "{}",
        s.mesh.signed_volume()
    );
    let top = s.mesh.positions.iter().map(|p| p[2]).fold(0.0, f64::max);
    assert!((top - 8.0).abs() < 1e-6, "{top}");
    assert!(m.warnings.is_empty(), "{:?}", m.warnings);
}

fn world_points(
    m: &StepModel,
    occ: &Occurrence,
    parent: &Transform,
    out: &mut Vec<(String, Vec<[f64; 3]>)>,
) {
    let t = parent.compose(&occ.transform);
    let part = &m.parts[occ.part];
    for s in &part.shapes {
        out.push((
            part.name.clone().unwrap_or_default(),
            s.mesh.positions.iter().map(|&p| t.apply(p)).collect(),
        ));
    }
    for c in &part.children {
        world_points(m, c, &t, out);
    }
}

type Bounds = ([f64; 3], [f64; 3]);

fn bbox(pts: &[[f64; 3]]) -> Bounds {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in pts {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    (lo, hi)
}

#[test]
fn assembly_occurrences_and_units() {
    let m = load("assembly.stp");
    assert_eq!(m.roots.len(), 1, "{:?}", m.roots);
    let root = &m.parts[m.roots[0].part];
    assert_eq!(root.name.as_deref(), Some("assembly"));
    assert_eq!(root.children.len(), 3, "{:?}", root.children);
    // The cube part is meshed once and instanced twice.
    let cube_parts: Vec<_> = m
        .parts
        .iter()
        .filter(|p| p.name.as_deref() == Some("cube"))
        .collect();
    assert_eq!(cube_parts.len(), 1);
    let mut placed = Vec::new();
    world_points(&m, &m.roots[0], &Transform::IDENTITY, &mut placed);
    assert_eq!(placed.len(), 3);
    let mut boxes: Vec<(String, Bounds)> =
        placed.iter().map(|(n, p)| (n.clone(), bbox(p))).collect();
    boxes.sort_by(|a, b| a.1 .0.partial_cmp(&b.1 .0).unwrap());
    let near = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6);
    // Cube 1 at the origin.
    assert!(
        boxes
            .iter()
            .any(|(_, b)| near(b.0, [0.0, 0.0, 0.0]) && near(b.1, [10.0, 10.0, 10.0])),
        "{boxes:?}"
    );
    // Cube 2 rotated 90° about z and moved to x = 30: x ∈ [20, 30].
    assert!(
        boxes
            .iter()
            .any(|(_, b)| near(b.0, [20.0, 0.0, 0.0]) && near(b.1, [30.0, 10.0, 10.0])),
        "{boxes:?}"
    );
    // The inch block (1 in = 25.4 mm) mapped to y = 30 mm.
    assert!(
        boxes
            .iter()
            .any(|(_, b)| near(b.0, [0.0, 30.0, 0.0]) && near(b.1, [25.4, 55.4, 25.4])),
        "{boxes:?}"
    );
    assert!(m.warnings.is_empty(), "{:?}", m.warnings);
}

#[test]
fn tessellated_geometry() {
    let m = load("tessellated.stp");
    let shapes: Vec<_> = m.parts.iter().flat_map(|p| &p.shapes).collect();
    assert_eq!(shapes.len(), 2);
    let solid = shapes
        .iter()
        .find(|s| s.name.as_deref() == Some("tess cube"))
        .unwrap();
    assert_eq!(solid.mesh.triangles.len(), 12);
    assert_eq!(open_edges(&solid.mesh), 0);
    assert!(close(solid.mesh.signed_volume(), 1000.0, 1e-12));
    assert_eq!(solid.colour, Some([0.3, 0.3, 0.3, 1.0]));
    let red = solid
        .triangle_colours
        .iter()
        .filter(|c| **c == Some([0.9, 0.1, 0.1, 1.0]))
        .count();
    assert_eq!(red, 2);
    let set = shapes
        .iter()
        .find(|s| s.name.as_deref() == Some("strip and fan"))
        .unwrap();
    // Strip of 6 → 4 triangles, fan of 3 → 1.
    assert_eq!(set.mesh.triangles.len(), 5);
    // Strip winding alternates so every triangle faces +z.
    for t in &set.mesh.triangles {
        let [a, b, c] = t.map(|i| set.mesh.positions[i as usize]);
        let n = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        assert!(n > 0.0, "{t:?}");
    }
}
