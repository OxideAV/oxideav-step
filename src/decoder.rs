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

    let mut materials: HashMap<[u32; 4], MaterialId> = HashMap::new();
    let mut mesh_of_part: Vec<Option<MeshId>> = Vec::with_capacity(model.parts.len());
    for part in &model.parts {
        if part.shapes.is_empty() {
            mesh_of_part.push(None);
            continue;
        }
        let mut mesh = Mesh::new(part.name.clone());
        for shape in &part.shapes {
            for (colour, prim) in split_by_colour(shape, factor) {
                let mut prim = prim;
                if let Some(c) = colour {
                    let key = [
                        c[0].to_bits(),
                        c[1].to_bits(),
                        c[2].to_bits(),
                        c[3].to_bits(),
                    ];
                    let id = *materials.entry(key).or_insert_with(|| {
                        let mut m = Material::new().with_base_color(c);
                        m.metallic = 0.0;
                        m.roughness = 0.5;
                        if c[3] < 1.0 {
                            m.alpha_mode = AlphaMode::Blend;
                        }
                        scene.add_material(m)
                    });
                    prim.material = Some(id);
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
        mesh_of_part.push(Some(scene.add_mesh(mesh)));
    }

    // Expand the occurrence DAG into nodes (cycle-safe, bounded).
    let mut stack_path: Vec<usize> = Vec::new();
    let mut count = 0usize;
    for occ in &model.roots {
        if let Some(id) = expand(
            model,
            &mesh_of_part,
            occ,
            factor,
            &mut scene,
            &mut stack_path,
            &mut count,
        ) {
            scene.add_root(id);
        }
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

fn expand(
    model: &StepModel,
    meshes: &[Option<MeshId>],
    occ: &crate::model::Occurrence,
    factor: f64,
    scene: &mut Scene3D,
    path: &mut Vec<usize>,
    count: &mut usize,
) -> Option<NodeId> {
    if path.contains(&occ.part) || *count >= MAX_NODES || path.len() > 256 {
        return None;
    }
    let part = model.parts.get(occ.part)?;
    *count += 1;
    let mut node = Node::new();
    node.name = occ.name.clone().or_else(|| part.name.clone());
    if occ.transform != oxideav_ifc::Transform::IDENTITY {
        node.transform = NodeTransform::Matrix(matrix(&occ.transform, factor));
    }
    node.mesh = meshes.get(occ.part).copied().flatten();
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
    path.push(occ.part);
    let mut children = Vec::new();
    for c in &part.children {
        if let Some(id) = expand(model, meshes, c, factor, scene, path, count) {
            children.push(id);
        }
    }
    path.pop();
    node.children = children;
    Some(scene.add_node(node))
}

/// One primitive per colour group of a shape.
fn split_by_colour(shape: &crate::model::Shape, factor: f64) -> Vec<(Option<Rgba>, Primitive)> {
    let mesh = &shape.mesh;
    let colour_at = |i: usize| -> Option<Rgba> {
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
