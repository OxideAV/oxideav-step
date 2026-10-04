//! STEP writer (`registry` feature): a [`Scene3D`] as an AP242 file of
//! tessellated geometry (CAx-IF "3D Tessellated Geometry" practice).
//!
//! * every mesh instantiated by a node becomes a product
//!   (`PRODUCT` / `PRODUCT_DEFINITION` / `SHAPE_REPRESENTATION`) whose
//!   `TESSELLATED_SHAPE_REPRESENTATION` holds one
//!   `TRIANGULATED_SURFACE_SET` (over its own `COORDINATES_LIST`) per
//!   triangle-topology primitive;
//! * a root product (`scene`) places every mesh-bearing node through a
//!   `NEXT_ASSEMBLY_USAGE_OCCURRENCE` + `CONTEXT_DEPENDENT_SHAPE_
//!   REPRESENTATION` + `ITEM_DEFINED_TRANSFORMATION` at the node's world
//!   transform — instancing is kept; a node whose world matrix is not a
//!   rigid motion has its geometry baked into a part of its own;
//! * primitive materials become `STYLED_ITEM` colours (fill-area colour,
//!   plus `SURFACE_STYLE_TRANSPARENT` when the base colour is
//!   translucent) collected in a
//!   `MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION`;
//! * `Scene3D::unit` becomes the length unit (SI with prefix, or a
//!   conversion-based inch / foot / yard).

use std::collections::HashMap;
use std::fmt::Write as _;

use oxideav_mesh3d::{Mesh3DEncoder, Scene3D, Topology, Unit};

/// AP242 tessellated-geometry writer for the OxideAV 3D-format registry.
#[derive(Debug, Clone, Default)]
pub struct StepEncoder;

impl StepEncoder {
    /// A writer.
    pub fn new() -> Self {
        Self
    }
}

impl Mesh3DEncoder for StepEncoder {
    fn encode(&mut self, scene: &Scene3D) -> oxideav_mesh3d::Result<Vec<u8>> {
        Ok(encode_scene(scene).into_bytes())
    }
}

/// A Part 21 REAL literal (always with a decimal point).
fn real(x: f64) -> String {
    if !x.is_finite() {
        return "0.".into();
    }
    let s = format!("{x:?}");
    match s.split_once('e') {
        Some((m, e)) => {
            let m = if m.contains('.') {
                m.to_string()
            } else {
                format!("{m}.")
            };
            format!("{m}E{e}")
        }
        None => {
            if s.contains('.') {
                s
            } else {
                format!("{s}.")
            }
        }
    }
}

/// A Part 21 string literal: apostrophes and backslashes doubled,
/// non-ASCII as `\X2\…\X0\` UTF-16 hex.
fn string(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\'' => out.push_str("''"),
            '\\' => out.push_str("\\\\"),
            ' '..='~' => out.push(c),
            _ => {
                out.push_str("\\X2\\");
                let mut buf = [0u16; 2];
                for u in c.encode_utf16(&mut buf) {
                    let _ = write!(out, "{u:04X}");
                }
                out.push_str("\\X0\\");
            }
        }
    }
    out.push('\'');
    out
}

struct Writer {
    out: String,
    next: u64,
}

impl Writer {
    fn add(&mut self, record: impl AsRef<str>) -> String {
        let id = self.next;
        self.next += 1;
        let _ = writeln!(self.out, "#{id}={};", record.as_ref());
        format!("#{id}")
    }

    fn point(&mut self, p: [f64; 3]) -> String {
        self.add(format!(
            "CARTESIAN_POINT('',({},{},{}))",
            real(p[0]),
            real(p[1]),
            real(p[2])
        ))
    }

    fn direction(&mut self, d: [f64; 3]) -> String {
        self.add(format!(
            "DIRECTION('',({},{},{}))",
            real(d[0]),
            real(d[1]),
            real(d[2])
        ))
    }

    fn placement(&mut self, o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> String {
        let (p, zd, xd) = (self.point(o), self.direction(z), self.direction(x));
        self.add(format!("AXIS2_PLACEMENT_3D('',{p},{zd},{xd})"))
    }
}

/// The rigid decomposition (origin, z axis, x axis) of a row-major
/// column-vector matrix, if it is a rotation + translation.
fn rigid(m: &[[f32; 4]; 4]) -> Option<([f64; 3], [f64; 3], [f64; 3])> {
    let col = |j: usize| [m[0][j] as f64, m[1][j] as f64, m[2][j] as f64];
    let (x, y, z) = (col(0), col(1), col(2));
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let unit = |a: [f64; 3]| (dot(a, a) - 1.0).abs() < 1e-4;
    let cross = [
        x[1] * y[2] - x[2] * y[1],
        x[2] * y[0] - x[0] * y[2],
        x[0] * y[1] - x[1] * y[0],
    ];
    let right_handed = dot(cross, z) > 0.999;
    if unit(x) && unit(y) && unit(z) && dot(x, y).abs() < 1e-4 && right_handed {
        let o = [m[0][3] as f64, m[1][3] as f64, m[2][3] as f64];
        Some((o, z, x))
    } else {
        None
    }
}

fn apply(m: &[[f32; 4]; 4], p: [f32; 3]) -> [f64; 3] {
    let mut out = [0.0f64; 3];
    for (r, o) in out.iter_mut().enumerate() {
        *o = m[r][0] as f64 * p[0] as f64
            + m[r][1] as f64 * p[1] as f64
            + m[r][2] as f64 * p[2] as f64
            + m[r][3] as f64;
    }
    out
}

const IDENTITY: [[f32; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// Serialise `scene` as an AP242 tessellated-geometry exchange
/// structure.
pub fn encode_scene(scene: &Scene3D) -> String {
    let mut w = Writer {
        out: String::new(),
        next: 1,
    };
    // Units + context.
    let length = match scene.unit {
        Unit::Metres => w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) )"),
        Unit::Centimetres => w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.) )"),
        Unit::Millimetres => w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )"),
        u @ (Unit::Inches | Unit::Feet | Unit::Yards) => {
            let (name, mm) = match u {
                Unit::Inches => ("INCH", 25.4),
                Unit::Feet => ("FOOT", 304.8),
                _ => ("YARD", 914.4),
            };
            let base = w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
            let dims = w.add("DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.)");
            let f = w.add(format!(
                "LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),{base})",
                real(mm)
            ));
            w.add(format!(
                "( CONVERSION_BASED_UNIT('{name}',{f}) LENGTH_UNIT() NAMED_UNIT({dims}) )"
            ))
        }
    };
    let angle = w.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
    let solid = w.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
    let ctx = w.add(format!(
        "( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT(({length},{angle},{solid})) REPRESENTATION_CONTEXT('','3D') )"
    ));
    let app = w.add("APPLICATION_CONTEXT('managed model based 3d engineering')");
    w.add(format!(
        "APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering',2014,{app})"
    ));
    let pc = w.add(format!("PRODUCT_CONTEXT('',{app},'mechanical')"));
    let pdc = w.add(format!(
        "PRODUCT_DEFINITION_CONTEXT('part definition',{app},'design')"
    ));
    let product = |w: &mut Writer, name: &str| -> (String, String, String) {
        let n = string(name);
        let prod = w.add(format!("PRODUCT({n},{n},'',({pc}))"));
        let pdf = w.add(format!("PRODUCT_DEFINITION_FORMATION('','',{prod})"));
        let pd = w.add(format!("PRODUCT_DEFINITION('design','',{pdf},{pdc})"));
        let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',{pd})"));
        let origin = w.placement([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
        let sr = w.add(format!("SHAPE_REPRESENTATION({n},({origin}),{ctx})"));
        w.add(format!("SHAPE_DEFINITION_REPRESENTATION({pds},{sr})"));
        (pd, sr, origin)
    };
    let mut styled: Vec<String> = Vec::new();
    // One part per (mesh, baked transform).
    let world = scene.world_node_transforms();
    let mut parts: HashMap<(u32, Option<usize>), (String, String, String)> = HashMap::new();
    let mut placements: Vec<(usize, (String, String, String), [[f32; 4]; 4])> = Vec::new();
    for (ni, node) in scene.nodes.iter().enumerate() {
        let (Some(mesh_id), Some(Some(m))) = (node.mesh, world.get(ni)) else {
            continue;
        };
        let Some(mesh) = scene.meshes.get(mesh_id.0 as usize) else {
            continue;
        };
        let rigid_motion = rigid(m).is_some();
        let key = (mesh_id.0, (!rigid_motion).then_some(ni));
        if !parts.contains_key(&key) {
            let name = mesh
                .name
                .clone()
                .or_else(|| node.name.clone())
                .unwrap_or_else(|| format!("mesh {}", mesh_id.0));
            let (pd, sr, origin) = product(&mut w, &name);
            let bake = if rigid_motion { IDENTITY } else { *m };
            let mut items = Vec::new();
            for (pi, prim) in mesh.primitives.iter().enumerate() {
                if !matches!(
                    prim.topology,
                    Topology::Triangles | Topology::TriangleStrip | Topology::TriangleFan
                ) {
                    continue;
                }
                let tris = prim.triangle_indices();
                if tris.is_empty() || prim.positions.is_empty() {
                    continue;
                }
                let mut coords = String::new();
                for (k, p) in prim.positions.iter().enumerate() {
                    let q = apply(&bake, *p);
                    if k > 0 {
                        coords.push(',');
                    }
                    let _ = write!(coords, "({},{},{})", real(q[0]), real(q[1]), real(q[2]));
                }
                let cl = w.add(format!(
                    "COORDINATES_LIST('',{},({coords}))",
                    prim.positions.len()
                ));
                let mut t = String::new();
                for (k, tri) in tris.iter().enumerate() {
                    if k > 0 {
                        t.push(',');
                    }
                    let _ = write!(t, "({},{},{})", tri[0] + 1, tri[1] + 1, tri[2] + 1);
                }
                let label = string(&format!("{name} {pi}"));
                let set = w.add(format!(
                    "TRIANGULATED_SURFACE_SET({label},{cl},{},(),(),({t}))",
                    prim.positions.len()
                ));
                if let Some(mat) = prim
                    .material
                    .and_then(|id| scene.materials.get(id.0 as usize))
                {
                    let c = mat.base_color;
                    let rgb = w.add(format!(
                        "COLOUR_RGB('',{},{},{})",
                        real(c[0] as f64),
                        real(c[1] as f64),
                        real(c[2] as f64)
                    ));
                    let fac = w.add(format!("FILL_AREA_STYLE_COLOUR('',{rgb})"));
                    let fas = w.add(format!("FILL_AREA_STYLE('',({fac}))"));
                    let sfa = w.add(format!("SURFACE_STYLE_FILL_AREA({fas})"));
                    let mut side = vec![sfa];
                    if c[3] < 1.0 {
                        let tr = w.add(format!(
                            "SURFACE_STYLE_TRANSPARENT({})",
                            real(1.0 - c[3] as f64)
                        ));
                        side.push(w.add(format!(
                            "SURFACE_STYLE_RENDERING_WITH_PROPERTIES(.NORMAL_SHADING.,{rgb},({tr}))"
                        )));
                    }
                    let sss = w.add(format!("SURFACE_SIDE_STYLE('',({}))", side.join(",")));
                    let ssu = w.add(format!("SURFACE_STYLE_USAGE(.BOTH.,{sss})"));
                    let psa = w.add(format!("PRESENTATION_STYLE_ASSIGNMENT(({ssu}))"));
                    styled.push(w.add(format!("STYLED_ITEM('color',({psa}),{set})")));
                }
                items.push(set);
            }
            if !items.is_empty() {
                let tsr = w.add(format!(
                    "TESSELLATED_SHAPE_REPRESENTATION({},({}),{ctx})",
                    string(&name),
                    items.join(",")
                ));
                w.add(format!(
                    "SHAPE_REPRESENTATION_RELATIONSHIP('','',{sr},{tsr})"
                ));
            }
            parts.insert(key, (pd, sr, origin));
        }
        let part = parts[&key].clone();
        let m = if rigid_motion { *m } else { IDENTITY };
        placements.push((ni, part, m));
    }
    // Root assembly.
    let (root_pd, root_sr, root_origin) = product(&mut w, "scene");
    let mut root_items = vec![root_origin];
    for (k, (ni, (pd, sr, origin), m)) in placements.iter().enumerate() {
        let (o, z, x) = rigid(m).unwrap_or(([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]));
        let place = w.placement(o, z, x);
        root_items.push(place.clone());
        let name = scene.nodes[*ni]
            .name
            .clone()
            .unwrap_or_else(|| format!("occurrence {}", k + 1));
        let nauo = w.add(format!(
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE('{}',{},'',{root_pd},{pd},$)",
            k + 1,
            string(&name)
        ));
        let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',{nauo})"));
        let idt = w.add(format!(
            "ITEM_DEFINED_TRANSFORMATION('','',{origin},{place})"
        ));
        let rr = w.add(format!(
            "( REPRESENTATION_RELATIONSHIP('','',{sr},{root_sr}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({idt}) SHAPE_REPRESENTATION_RELATIONSHIP() )"
        ));
        w.add(format!(
            "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION({rr},{pds})"
        ));
    }
    // The root's placements are its representation items: rewrite the
    // root SHAPE_REPRESENTATION record with them.
    let root_line = format!("{root_sr}=SHAPE_REPRESENTATION('scene',(");
    if let Some(start) = w.out.find(&root_line) {
        let items_start = start + root_line.len();
        if let Some(end_rel) = w.out[items_start..].find(')') {
            w.out
                .replace_range(items_start..items_start + end_rel, &root_items.join(","));
        }
    }
    if !styled.is_empty() {
        w.add(format!(
            "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',({}),{ctx})",
            styled.join(",")
        ));
    }
    let mut file = String::from("ISO-10303-21;\nHEADER;\n");
    file.push_str("FILE_DESCRIPTION(('oxideav-step tessellated export'),'2;1');\n");
    file.push_str("FILE_NAME('','',(''),(''),'oxideav-step','oxideav-step','');\n");
    file.push_str(
        "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }'));\n",
    );
    file.push_str("ENDSEC;\nDATA;\n");
    file.push_str(&w.out);
    file.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    file
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn literals() {
        assert_eq!(real(1.0), "1.0");
        assert_eq!(real(1e-7), "1.E-7");
        assert_eq!(real(-2.5e20), "-2.5E20");
        assert_eq!(real(f64::NAN), "0.");
        assert_eq!(string("it's"), "'it''s'");
        assert_eq!(string("é"), "'\\X2\\00E9\\X0\\'");
    }
}
