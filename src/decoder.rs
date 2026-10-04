//! Framework integration (`registry` feature): [`StepDecoder`] (a
//! `Mesh3DDecoder`), the [`StepModel`] → [`Scene3D`] mapping, and the
//! [`Mesh3DRegistry`] registration helper.
//!
//! * every [`Part`](crate::Part) with geometry becomes one [`Mesh`],
//!   shared by all the nodes that instance it (assembly occurrences);
//!   each shape becomes one `Triangles` primitive per colour (face-level
//!   colours split a shape), with a deduplicated [`Material`] per RGBA
//!   (`AlphaMode::Blend` when transparent), with vertices split per face
//!   and area-weighted normals smooth within a face (hard face edges);
//! * occurrences become nodes carrying the child → parent matrix, named
//!   after the occurrence / product; the product id and definition land
//!   in node extras (`step:product_id`, `step:product_definition`);
//! * `Scene3D::unit` is the model length unit when mesh3d has it
//!   (mm / cm / m / in / ft / yd), else coordinates are converted to
//!   metres; STEP models are Z-up (`up_axis = PosZ`, `front_axis =
//!   NegY` — the front view looks along +Y).

use std::collections::HashMap;

use oxideav_core::Error as CoreError;
use oxideav_mesh3d::{
    AlphaMode, Axis, Indices, Material, MaterialId, Mesh, Mesh3DDecoder, Mesh3DRegistry, MeshId,
    Node, NodeId, Primitive, Scene3D, Topology, Transform as NodeTransform, Unit,
};
use serde_json::Value as Json;

use crate::error::Error;
use crate::model::{read_step_with, ReadOptions, StepModel};
use crate::style::Rgba;

/// Largest number of scene nodes an assembly may expand into (shared
/// sub-assemblies multiply).
const MAX_NODES: usize = 2_000_000;

/// STEP decoder front-end for the OxideAV 3D-format registry.
#[derive(Debug, Clone, Default)]
pub struct StepDecoder {
    options: ReadOptions,
}

impl StepDecoder {
    /// Decoder with default [`ReadOptions`].
    pub fn new() -> Self {
        Self::default()
    }

    /// Decoder with caller-supplied options (limits, tessellation
    /// tolerance).
    pub fn with_options(options: ReadOptions) -> Self {
        Self { options }
    }
}

impl Mesh3DDecoder for StepDecoder {
    fn decode(&mut self, bytes: &[u8]) -> oxideav_mesh3d::Result<Scene3D> {
        if !oxideav_ifc::probe_step(bytes) {
            return Err(CoreError::InvalidData(
                "not an ISO 10303-21 exchange structure (missing `ISO-10303-21;` magic)".into(),
            ));
        }
        let model = read_step_with(bytes, &self.options).map_err(|e| match e {
            Error::LimitExceeded(m) => CoreError::ResourceExhausted(m),
            Error::NoGeometry(m) => CoreError::Unsupported(m),
            Error::Parse(p) => CoreError::InvalidData(p.to_string()),
        })?;
        Ok(scene_from_model(&model))
    }
}

/// The mesh3d unit for a model unit, if representable.
fn unit_of(metres: f64) -> Option<Unit> {
    let table = [
        (1.0, Unit::Metres),
        (0.01, Unit::Centimetres),
        (0.001, Unit::Millimetres),
        (0.0254, Unit::Inches),
        (0.3048, Unit::Feet),
        (0.9144, Unit::Yards),
    ];
    table
        .iter()
        .find(|(m, _)| (metres - m).abs() <= 1e-9 * m)
        .map(|&(_, u)| u)
}

/// Map a [`StepModel`] onto a [`Scene3D`].
pub fn scene_from_model(model: &StepModel) -> Scene3D {
    let mut scene = Scene3D::new();
    scene.up_axis = Axis::PosZ;
    scene.front_axis = Axis::NegY;
    let (unit, factor) = match unit_of(model.length_unit_metres) {
        Some(u) => (u, 1.0),
        None => (Unit::Metres, model.length_unit_metres),
    };
    scene.unit = unit;
    scene.extras.insert(
        "step:schema".into(),
        Json::String(model.schema.label().to_string()),
    );
    if !model.header.file_name.originating_system.is_empty() {
        scene.extras.insert(
            "step:originating_system".into(),
            Json::String(model.header.file_name.originating_system.clone()),
        );
    }
    scene.extras.insert(
        "step:length_unit_metres".into(),
        serde_json::Number::from_f64(model.length_unit_metres).map_or(Json::Null, Json::Number),
    );

    let mut b = SceneBuilder {
        model,
        factor,
        scene,
        materials: HashMap::new(),
        base: Vec::with_capacity(model.parts.len()),
        variants: HashMap::new(),
        count: 0,
    };
    for i in 0..model.parts.len() {
        let id = b.build_mesh(i, &[]);
        b.base.push(id);
    }
    // Expand the occurrence DAG into nodes (cycle-safe, bounded).
    let mut parts_path: Vec<usize> = Vec::new();
    let mut occ_path: Vec<[u64; 2]> = Vec::new();
    let mut roots = Vec::new();
    for occ in &model.roots {
        if let Some(id) = b.expand(occ, &mut parts_path, &mut occ_path) {
            roots.push(id);
        }
    }
    let mut scene = b.scene;
    for r in roots {
        scene.add_root(r);
    }
    scene
}

fn matrix(t: &oxideav_ifc::Transform, factor: f64) -> [[f32; 4]; 4] {
    let c = t.cols;
    let tr = t.translation;
    [
        [
            c[0][0] as f32,
            c[1][0] as f32,
            c[2][0] as f32,
            (tr[0] * factor) as f32,
        ],
        [
            c[0][1] as f32,
            c[1][1] as f32,
            c[2][1] as f32,
            (tr[1] * factor) as f32,
        ],
        [
            c[0][2] as f32,
            c[1][2] as f32,
            c[2][2] as f32,
            (tr[2] * factor) as f32,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// A colour override for one item: `(item id, colour)`.
type Override = (u64, Rgba);

/// A mesh variant key: the part and its overrides (colours as bits).
type VariantKey = (usize, Vec<(u64, [u32; 4])>);

struct SceneBuilder<'m> {
    model: &'m StepModel,
    factor: f64,
    scene: Scene3D,
    materials: HashMap<[u32; 4], MaterialId>,
    /// Each part's mesh with its own colours.
    base: Vec<Option<MeshId>>,
    /// Per-occurrence colour variants: (part, overrides) → mesh.
    variants: HashMap<VariantKey, Option<MeshId>>,
    count: usize,
}

fn colour_key(c: Rgba) -> [u32; 4] {
    [
        c[0].to_bits(),
        c[1].to_bits(),
        c[2].to_bits(),
        c[3].to_bits(),
    ]
}

impl SceneBuilder<'_> {
    fn material(&mut self, c: Rgba) -> MaterialId {
        let scene = &mut self.scene;
        *self.materials.entry(colour_key(c)).or_insert_with(|| {
            let mut m = Material::new().with_base_color(c);
            m.metallic = 0.0;
            m.roughness = 0.5;
            if c[3] < 1.0 {
                m.alpha_mode = AlphaMode::Blend;
            }
            scene.add_material(m)
        })
    }

    /// The mesh of part `i` with `overrides` applied (an override on a
    /// shape item recolours the whole item, one on a face that face).
    fn build_mesh(&mut self, i: usize, overrides: &[Override]) -> Option<MeshId> {
        let part = self.model.parts.get(i)?;
        if part.shapes.is_empty() {
            return None;
        }
        let mut mesh = Mesh::new(part.name.clone());
        for shape in &part.shapes {
            for (colour, mut prim) in split_by_colour(shape, self.factor, overrides) {
                if let Some(c) = colour {
                    prim.material = Some(self.material(c));
                }
                if let Some(n) = &shape.name {
                    prim.extras
                        .insert("step:name".into(), Json::String(n.clone()));
                }
                prim.extras
                    .insert("step:item".into(), Json::Number(shape.item.into()));
                if !shape.layers.is_empty() {
                    prim.extras.insert(
                        "step:layers".into(),
                        Json::Array(shape.layers.iter().cloned().map(Json::String).collect()),
                    );
                }
                mesh.primitives.push(prim);
            }
        }
        Some(self.scene.add_mesh(mesh))
    }

    /// The overrides that apply to part `i` at occurrence path `path`
    /// (per level: the placing relationship / mapped item and the usage
    /// id; a style context may name either).
    fn overrides_for(&self, i: usize, path: &[[u64; 2]]) -> Vec<Override> {
        let Some(part) = self.model.parts.get(i) else {
            return Vec::new();
        };
        let mut out: Vec<Override> = Vec::new();
        for oc in &self.model.occurrence_colours {
            let same_path = oc.path.len() == path.len()
                && path
                    .iter()
                    .all(|level| oc.path.iter().any(|p| level.contains(p)));
            if !same_path {
                continue;
            }
            let applies = part
                .shapes
                .iter()
                .any(|s| s.item == oc.item || s.triangle_faces.contains(&oc.item));
            if applies {
                out.push((oc.item, oc.colour));
            }
        }
        out
    }

    fn mesh_for(&mut self, i: usize, path: &[[u64; 2]]) -> Option<MeshId> {
        let overrides = self.overrides_for(i, path);
        if overrides.is_empty() {
            return self.base.get(i).copied().flatten();
        }
        let key = (
            i,
            overrides
                .iter()
                .map(|&(item, c)| (item, colour_key(c)))
                .collect::<Vec<_>>(),
        );
        if let Some(m) = self.variants.get(&key) {
            return *m;
        }
        let m = self.build_mesh(i, &overrides);
        self.variants.insert(key, m);
        m
    }

    fn expand(
        &mut self,
        occ: &crate::model::Occurrence,
        parts_path: &mut Vec<usize>,
        occ_path: &mut Vec<[u64; 2]>,
    ) -> Option<NodeId> {
        if parts_path.contains(&occ.part) || self.count >= MAX_NODES || parts_path.len() > 256 {
            return None;
        }
        let model = self.model;
        let part = model.parts.get(occ.part)?;
        self.count += 1;
        let mut node = Node::new();
        node.name = occ.name.clone().or_else(|| part.name.clone());
        if occ.transform != oxideav_ifc::Transform::IDENTITY {
            node.transform = NodeTransform::Matrix(matrix(&occ.transform, self.factor));
        }
        let level = occ.placed_by.is_some() || occ.id.is_some();
        if level {
            let p = occ.placed_by.unwrap_or(u64::MAX);
            occ_path.push([p, occ.id.unwrap_or(p)]);
        }
        node.mesh = self.mesh_for(occ.part, occ_path);
        if let Some(pid) = &part.product_id {
            node.extras
                .insert("step:product_id".into(), Json::String(pid.clone()));
        }
        if let Some(d) = part.definition {
            node.extras
                .insert("step:product_definition".into(), Json::Number(d.into()));
        }
        if let Some(id) = occ.id {
            node.extras
                .insert("step:occurrence".into(), Json::Number(id.into()));
        }
        parts_path.push(occ.part);
        let mut children = Vec::new();
        for c in &part.children {
            if let Some(id) = self.expand(c, parts_path, occ_path) {
                children.push(id);
            }
        }
        parts_path.pop();
        if level {
            occ_path.pop();
        }
        node.children = children;
        Some(self.scene.add_node(node))
    }
}

/// One primitive per colour group of a shape.
fn split_by_colour(
    shape: &crate::model::Shape,
    factor: f64,
    overrides: &[Override],
) -> Vec<(Option<Rgba>, Primitive)> {
    let mesh = &shape.mesh;
    let whole = overrides
        .iter()
        .find(|(item, _)| *item == shape.item)
        .map(|&(_, c)| c);
    let colour_at = |i: usize| -> Option<Rgba> {
        let face = shape.triangle_faces.get(i).copied();
        if let Some(c) = overrides
            .iter()
            .find(|(item, _)| Some(*item) == face)
            .map(|&(_, c)| c)
        {
            return Some(c);
        }
        if whole.is_some() {
            return whole;
        }
        shape
            .triangle_colours
            .get(i)
            .copied()
            .flatten()
            .or(shape.colour)
    };
    let mut groups: Vec<(Option<Rgba>, Vec<usize>)> = Vec::new();
    for i in 0..mesh.triangles.len() {
        let c = colour_at(i);
        match groups.iter_mut().find(|(gc, _)| *gc == c) {
            Some((_, v)) => v.push(i),
            None => groups.push((c, vec![i])),
        }
    }
    let mut out = Vec::with_capacity(groups.len());
    for (c, tris) in groups {
        // Vertices split per (position, face): normals are smooth within
        // a face and break across face edges (CAD shading).
        let mut remap: HashMap<(u32, u64), u32> = HashMap::new();
        let mut positions: Vec<[f32; 3]> = Vec::new();
        let mut normals: Vec<[f64; 3]> = Vec::new();
        let mut indices: Vec<u32> = Vec::with_capacity(tris.len() * 3);
        for &t in &tris {
            let face = shape.triangle_faces.get(t).copied().unwrap_or(0);
            let tri = mesh.triangles[t];
            let p = tri.map(|v| mesh.positions.get(v as usize).copied().unwrap_or_default());
            let e1 = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
            let e2 = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
            let n = [
                e1[1] * e2[2] - e1[2] * e2[1],
                e1[2] * e2[0] - e1[0] * e2[2],
                e1[0] * e2[1] - e1[1] * e2[0],
            ];
            for (k, &v) in tri.iter().enumerate() {
                let id = *remap.entry((v, face)).or_insert_with(|| {
                    let q = p[k];
                    positions.push([
                        (q[0] * factor) as f32,
                        (q[1] * factor) as f32,
                        (q[2] * factor) as f32,
                    ]);
                    normals.push([0.0; 3]);
                    (positions.len() - 1) as u32
                });
                let acc = &mut normals[id as usize];
                for j in 0..3 {
                    acc[j] += n[j];
                }
                indices.push(id);
            }
        }
        let normals: Vec<[f32; 3]> = normals
            .into_iter()
            .map(|n| {
                let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                if l > 0.0 && l.is_finite() {
                    [(n[0] / l) as f32, (n[1] / l) as f32, (n[2] / l) as f32]
                } else {
                    [0.0, 0.0, 1.0]
                }
            })
            .collect();
        let mut prim = Primitive::new(Topology::Triangles);
        prim.indices = Some(if positions.len() <= u16::MAX as usize + 1 {
            Indices::U16(indices.iter().map(|&i| i as u16).collect())
        } else {
            Indices::U32(indices)
        });
        prim.positions = positions;
        prim.normals = Some(normals);
        out.push((c, prim));
    }
    out
}

/// Fresh decoder with default options.
pub fn make_decoder() -> StepDecoder {
    StepDecoder::new()
}

/// Register the STEP decoder and the AP242 tessellated writer into a
/// [`Mesh3DRegistry`] under format id `"step"` with the `.step` /
/// `.stp` / `.p21` extensions.
pub fn register_mesh3d(registry: &mut Mesh3DRegistry) {
    registry.register_decoder(
        "step",
        &["step", "stp", "p21"],
        Box::new(|| Box::new(StepDecoder::new())),
    );
    registry.register_encoder(
        "step",
        &["step", "stp", "p21"],
        Box::new(|| Box::new(crate::encoder::StepEncoder::new())),
    );
}
