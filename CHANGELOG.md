# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- Crate scaffold.
- STEP reader over `oxideav-ifc`'s ISO 10303-21 parser: `FILE_SCHEMA`
  detection (AP203 / AP203e2 / AP214 / AP242), EXPRESS typing of the
  consumed AP entity slice (attributes by name under internal and
  external mapping; cross-checked against the staged AP242 / AP214 /
  AP203 long-form schemas), SI / conversion-based length and angle
  units per representation context.
- Product structure → `StepModel`: products, shape-representation
  groups, assembly occurrences through `NEXT_ASSEMBLY_USAGE_OCCURRENCE`
  + `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` (component side decided
  from the data) and `MAPPED_ITEM`, mixed-unit harmonisation.
- B-rep tessellation through the shared `oxideav_ifc::kernel`: solids,
  shells, faces (planar, cylindrical, conical, spherical, toroidal,
  B-spline, extrusion, revolution, offset), curves (lines, conics,
  B-splines, polylines, trimmed / composite / surface / p-curves),
  edges sampled once for watertight solids, chordal tolerance density.
- AP242 tessellated geometry (triangulated / complex triangulated
  faces and surface sets, tessellated solids / shells).
- Presentation: surface colours (RGB / pre-defined), transparency,
  per-face colours, invisibility, layers.
- `registry` feature: `StepDecoder` (`Mesh3DDecoder`),
  `scene_from_model`, `make_decoder`, `register_mesh3d` (`.step` /
  `.stp` / `.p21`): instanced part meshes, per-colour primitives,
  occurrence nodes, units, Z-up.
- `fuzz/` cargo-fuzz target `read_step` (whole reader on hostile bytes,
  fixtures as seeds) and the daily Fuzz workflow.
- Fixtures: surface of revolution of a B-spline profile, sphere from
  hemispheres, elliptic linear extrusion, degree-unit trimmed arcs, NURBS
  circle edges on an offset surface, `BREP_WITH_VOIDS`, faceted brep +
  open shell, nested assembly (parent-first relationship, cartesian-
  operator mapped item) — all watertight with exact volumes.
- `StepEncoder` / `encode_scene`: AP242 tessellated writer (registered
  under `"step"` for `.step` / `.stp` / `.p21`), round-trip tested.
- Shell-level colours, per-face vertex split + normals in the scene,
  `DEGENERATE_TOROIDAL_SURFACE`, STEP-worded warnings.
