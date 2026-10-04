//! # oxideav-step
//!
//! Pure-Rust STEP reader: ISO 10303-21 exchange structures carrying the
//! AP242 (`AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF`), AP214
//! (`AUTOMOTIVE_DESIGN`) and AP203 (`CONFIG_CONTROL_DESIGN`, AP203e2)
//! schemas, with the exact boundary-representation geometry tessellated
//! into triangle meshes.
//!
//! ```no_run
//! let bytes = std::fs::read("part.stp").unwrap();
//! let model = oxideav_step::read_step(&bytes).unwrap();
//! println!("{} parts, {} triangles", model.parts.len(), model.triangle_count());
//! ```
//!
//! ## Layers
//!
//! * **Physical file** — `oxideav-ifc`'s clean-room ISO 10303-21 parser
//!   (complex / external-mapping instances included).
//! * [`schema`] — EXPRESS typing: the AP entity slice this reader uses,
//!   transcribed from the AP242 long form (cross-checked against the
//!   AP242 / AP214 / AP203 schemas), resolving attributes by name under
//!   both the internal and the external mapping; [`ApSchema`] detection.
//! * [`units`] — SI / conversion-based length and plane-angle units of a
//!   representation context.
//! * [`geom`] — ISO 10303-42 curves, surfaces and topology meshed
//!   through the neutral kernel `oxideav_ifc::kernel` (trimmed-face
//!   tessellation in the surface parameter domain, shared edge sampling
//!   for watertight solids, chordal-tolerance density), and AP242
//!   tessellated geometry.
//! * [`style`] — colours / transparency / invisibility / layers.
//! * [`model`] — products, shape representations, assembly occurrences
//!   (`next_assembly_usage_occurrence` + context-dependent shape
//!   representations, `mapped_item`s), unit harmonisation →
//!   [`StepModel`].
//!
//! With the default `registry` feature, [`StepDecoder`] maps the model
//! onto an `oxideav_mesh3d::Scene3D` and [`register_mesh3d`] plugs it
//! into the OxideAV 3D-format registry (`.step` / `.stp` / `.p21`);
//! [`StepEncoder`] writes a scene back as AP242 tessellated geometry
//! (parts instanced through an assembly, colours, units).
//! Build with `default-features = false` for the std-only reader.
//!
//! Clean-room: implemented from the ISO 10303 public schemas, the
//! CAx-IF recommended practices and the published geometry literature;
//! no CAD-kernel or STEP-toolkit source was consulted.

#![forbid(unsafe_code)]
#![deny(missing_docs)]

pub mod error;
pub mod geom;
pub mod model;
pub mod schema;
pub mod style;
pub mod units;

#[cfg(feature = "registry")]
pub mod decoder;
#[cfg(feature = "registry")]
pub mod encoder;

pub use error::{Error, Result};
pub use geom::{GeometryLimits, Tolerance};
pub use model::{
    model_from_file, read_step, read_step_with, Occurrence, Part, ReadOptions, Shape, StepModel,
};
pub use oxideav_ifc::{Header, StepFile, StepLimits, Transform, TriMesh};
pub use schema::ApSchema;
pub use style::Rgba;

#[cfg(feature = "registry")]
pub use decoder::{make_decoder, register_mesh3d, scene_from_model, StepDecoder};
#[cfg(feature = "registry")]
pub use encoder::{encode_scene, StepEncoder};
