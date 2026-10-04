# oxideav-step

Pure-Rust STEP CAD reader — ISO 10303-21 exchange structures carrying
**AP242** (`AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF`), **AP214**
(`AUTOMOTIVE_DESIGN`) and **AP203** (`CONFIG_CONTROL_DESIGN`, AP203e2) —
whose exact boundary-representation geometry is tessellated into
triangle meshes and mapped onto an `oxideav_mesh3d::Scene3D`.

It reuses [`oxideav-ifc`](https://github.com/OxideAV/oxideav-ifc)'s
framework-free core: the ISO 10303-21 physical-file parser (complex /
external-mapping entity instances included) and the neutral ISO
10303-42 geometry kernel (`oxideav_ifc::kernel`: B-splines, elementary
/ swept / offset surfaces, trimmed-face tessellation in the surface
parameter domain).

## Status

| Area | Supported |
| --- | --- |
| Physical file | ISO 10303-21 parser of `oxideav-ifc` (complex instances, user-defined keywords, strings wrapped across lines, DoS caps) |
| Schemas | `FILE_SCHEMA` detection → AP203 / AP203e2 / AP214 / AP242 (object-identifier suffixes ignored); EXPRESS typing of the consumed entity slice by name under both mappings, cross-checked against the AP242 / AP214 / AP203 long forms |
| Units | `SI_UNIT` with prefixes, `CONVERSION_BASED_UNIT` (inch, foot, degree, …) for length and plane angle, per representation context; uncertainty |
| Product structure | `PRODUCT` / `PRODUCT_DEFINITION` / `SHAPE_DEFINITION_REPRESENTATION`, `SHAPE_REPRESENTATION_RELATIONSHIP` groups, `NEXT_ASSEMBLY_USAGE_OCCURRENCE` + `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` + `ITEM_DEFINED_TRANSFORMATION` / cartesian transformation operators, `MAPPED_ITEM`; parts meshed once and instanced; mixed-unit assemblies harmonised to the root unit |
| Solids & shells | `MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS`, `FACETED_BREP`, `SHELL_BASED_SURFACE_MODEL`, `FACE_BASED_SURFACE_MODEL`, closed / open / oriented shells, `ADVANCED_FACE`, `FACE_SURFACE`, `ORIENTED_FACE`, poly-loop faces |
| Surfaces | `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `TOROIDAL_SURFACE`, `DEGENERATE_TOROIDAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS` (+ uniform / quasi-uniform / Bézier, rational), `SURFACE_OF_LINEAR_EXTRUSION`, `SURFACE_OF_REVOLUTION`, `OFFSET_SURFACE`, trimmed / curve-bounded bases |
| Curves | `LINE`, `CIRCLE`, `ELLIPSE`, `HYPERBOLA`, `PARABOLA`, `POLYLINE`, `B_SPLINE_CURVE_WITH_KNOTS` (+ uniform / quasi-uniform / Bézier, rational), `TRIMMED_CURVE`, `COMPOSITE_CURVE`, `SURFACE_CURVE` / `SEAM_CURVE` / `INTERSECTION_CURVE`, `PCURVE`, `OFFSET_CURVE_3D` |
| Tessellation | every edge sampled once and shared by both faces (watertight solids); faces trimmed in the surface `(u, v)` domain (seams, poles, cone apices); chordal-tolerance + angular density (`Tolerance`) |
| AP242 tessellated | `TESSELLATED_SHAPE_REPRESENTATION`, `TESSELLATED_SOLID` / `TESSELLATED_SHELL`, `TRIANGULATED_FACE`, `COMPLEX_TRIANGULATED_FACE`, `TRIANGULATED_SURFACE_SET`, `COMPLEX_TRIANGULATED_SURFACE_SET`, `COORDINATES_LIST` |
| Presentation | `STYLED_ITEM` / `OVER_RIDING_STYLED_ITEM` → `PRESENTATION_STYLE_ASSIGNMENT` → `SURFACE_STYLE_USAGE` → `SURFACE_SIDE_STYLE` → fill-area colour or `SURFACE_STYLE_RENDERING(_WITH_PROPERTIES)` + `SURFACE_STYLE_TRANSPARENT`; `COLOUR_RGB`, `DRAUGHTING_PRE_DEFINED_COLOUR`; solid / shell / face / representation targets; `INVISIBILITY`; layer names |
| Scene3D | one mesh per part (shared by occurrences), one primitive per colour, vertices split per face with smooth-within-face normals, deduplicated materials, occurrence nodes with matrices, `unit` (mm / cm / m / in / ft / yd, else metres), Z-up |

| Writer | `StepEncoder` / `encode_scene` (`registry`): a `Scene3D` as AP242 tessellated geometry — `TRIANGULATED_SURFACE_SET` per primitive, one product per mesh instanced through an assembly (`NEXT_ASSEMBLY_USAGE_OCCURRENCE` + `ITEM_DEFINED_TRANSFORMATION`; non-rigid node transforms baked), colours / transparency, length unit |

Not yet: PMI (semantic / graphical annotations), wireframe-only
geometry (`GEOMETRIC_CURVE_SET`), CSG / swept solid primitives,
context-dependent (per-occurrence) over-riding styles, exact-B-rep
export.

## Usage

```rust,no_run
// std-only model
let bytes = std::fs::read("part.stp")?;
let model = oxideav_step::read_step(&bytes)?;
for part in &model.parts {
    println!("{:?}: {} shapes", part.name, part.shapes.len());
}

// Scene3D through the registry (default `registry` feature)
let mut reg = oxideav_mesh3d::Mesh3DRegistry::new();
oxideav_step::register_mesh3d(&mut reg);
let scene = reg.decoder_for_extension("stp").unwrap().decode(&bytes)?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

Tessellation density: `ReadOptions::tolerance` — `relative` chordal
tolerance (fraction of each representation's bounding-box diagonal,
default `1e-3`), optional `absolute` tolerance in metres, and
`max_angle` (radians per mesh edge, default `2π/32`).

Build with `default-features = false` for the std-only reader
(`read_step` / `StepModel`, no `oxideav-core` / `oxideav-mesh3d`).

## Clean-room note

Implemented from the ISO 10303 public EXPRESS schemas (AP242 / AP214 /
AP203 long forms and the integrated resources), the CAx-IF / MBx-IF
recommended practices, and the published geometry literature (de Boor's
algorithm, Piegl & Tiller "The NURBS Book"). The reference material is
staged with provenance in the OxideAV docs repository
(`docs/3d/step/`). No CAD-kernel or STEP-toolkit source code
(OpenCASCADE, FreeCAD, STEPcode, ifcopenshell, …) was consulted.

## License

MIT — see `LICENSE`.
