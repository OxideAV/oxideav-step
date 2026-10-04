//! EXPRESS typing for the STEP application-protocol schemas.
//!
//! The physical file carries attributes by **position**; the EXPRESS
//! schema names them. An entity's serialised attribute list is its
//! supertype graph's explicit attributes, parent-first, supertypes in
//! declaration order, each entity contributing once (ISO 10303-21
//! §12.2.4 internal mapping), followed by its own. In the external
//! mapping (complex instance `#id = (A(…) B(…));`) every entity of the
//! instance's supertype graph appears as its own partial record carrying
//! only the attributes that entity declares (§12.2.5).
//!
//! [`ENTITIES`] transcribes the slice of the AP242 / AP214 / AP203 long-
//! form schemas this reader consumes — product structure, representations
//! with their contexts and units, ISO 10303-42 geometry and topology,
//! presentation, and AP242 tessellated geometry — as `(entity,
//! supertypes, explicit attributes)`. The three APs share these resources (AP203 and AP214
//! are AP242's ancestors and their declarations of these entities agree
//! on attribute order). [`Entity`] is a borrowing view of one instance
//! that resolves attributes **by name** under either mapping.

use std::collections::HashMap;
use std::sync::OnceLock;

use oxideav_ifc::{ParsedInstance, StepFile, Value};

/// The STEP application protocol a file declares in `FILE_SCHEMA`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ApSchema {
    /// AP203 edition 1 — `CONFIG_CONTROL_DESIGN`.
    Ap203,
    /// AP203 edition 2 —
    /// `AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF`.
    Ap203e2,
    /// AP214 — `AUTOMOTIVE_DESIGN` (or the older `AUTOMOTIVE_DESIGN_CC2`).
    Ap214,
    /// AP242 — `AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF`.
    Ap242,
    /// Any other schema identifier (kept upper-cased, object-identifier
    /// suffix stripped). The reader still tries the shared resources.
    Other(String),
}

impl ApSchema {
    /// Classify one `FILE_SCHEMA` entry. The optional ASN.1 object
    /// identifier suffix (`AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }`)
    /// is ignored.
    pub fn from_identifier(id: &str) -> Self {
        let base = id
            .split('{')
            .next()
            .unwrap_or("")
            .trim()
            .to_ascii_uppercase();
        match base.as_str() {
            "CONFIG_CONTROL_DESIGN" => Self::Ap203,
            "AUTOMOTIVE_DESIGN" | "AUTOMOTIVE_DESIGN_CC2" => Self::Ap214,
            s if s.starts_with("AP203_CONFIGURATION_CONTROLLED_3D_DESIGN") => Self::Ap203e2,
            s if s.starts_with("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING") => Self::Ap242,
            _ => Self::Other(base),
        }
    }

    /// The protocol of a parsed file: the first recognised `FILE_SCHEMA`
    /// entry, else the first entry as [`ApSchema::Other`].
    pub fn of_file(step: &StepFile) -> Self {
        let mut first: Option<Self> = None;
        for id in &step.header.file_schema {
            let s = Self::from_identifier(id);
            if !matches!(s, Self::Other(_)) {
                return s;
            }
            first.get_or_insert(s);
        }
        first.unwrap_or(Self::Other(String::new()))
    }

    /// A short label (`"AP242"`, `"AP214"`, …).
    pub fn label(&self) -> &str {
        match self {
            Self::Ap203 => "AP203",
            Self::Ap203e2 => "AP203e2",
            Self::Ap214 => "AP214",
            Self::Ap242 => "AP242",
            Self::Other(s) => s,
        }
    }
}

/// One EXPRESS entity declaration: name, direct supertypes (declaration
/// order) and explicit attributes (declaration order). Names are upper
/// case, as they appear in the physical file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntityDef {
    /// Entity name (`"CARTESIAN_POINT"`).
    pub name: &'static str,
    /// Direct supertypes, in `SUBTYPE OF (…)` order.
    pub supertypes: &'static [&'static str],
    /// Explicit attributes this entity declares, lower case.
    pub attrs: &'static [&'static str],
}

macro_rules! ent {
    ($name:literal, [$($sup:literal),*], [$($attr:literal),*]) => {
        EntityDef { name: $name, supertypes: &[$($sup),*], attrs: &[$($attr),*] }
    };
}

/// The transcribed entity slice (AP242 MIM long form; the AP203 / AP214
/// declarations of these entities serialise identically).
pub static ENTITIES: &[EntityDef] = &[
    // --- representation / context (ISO 10303-43) --------------------
    ent!("REPRESENTATION_ITEM", [], ["name"]),
    ent!("GEOMETRIC_REPRESENTATION_ITEM", ["REPRESENTATION_ITEM"], []),
    ent!(
        "TOPOLOGICAL_REPRESENTATION_ITEM",
        ["REPRESENTATION_ITEM"],
        []
    ),
    ent!("FOUNDED_ITEM", [], []),
    ent!("REPRESENTATION", [], ["name", "items", "context_of_items"]),
    ent!("SHAPE_REPRESENTATION", ["REPRESENTATION"], []),
    ent!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "FACETED_BREP_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "EDGE_BASED_WIREFRAME_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "SHELL_BASED_WIREFRAME_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "TESSELLATED_SHAPE_REPRESENTATION",
        ["SHAPE_REPRESENTATION"],
        []
    ),
    ent!(
        "TESSELLATED_SHAPE_REPRESENTATION_WITH_ACCURACY_PARAMETERS",
        ["TESSELLATED_SHAPE_REPRESENTATION"],
        ["tessellation_accuracy_parameters"]
    ),
    ent!("PRESENTATION_REPRESENTATION", ["REPRESENTATION"], []),
    ent!(
        "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION",
        ["REPRESENTATION"],
        []
    ),
    ent!("DRAUGHTING_MODEL", ["REPRESENTATION"], []),
    ent!("DEFINITIONAL_REPRESENTATION", ["REPRESENTATION"], []),
    ent!(
        "REPRESENTATION_CONTEXT",
        [],
        ["context_identifier", "context_type"]
    ),
    ent!(
        "GEOMETRIC_REPRESENTATION_CONTEXT",
        ["REPRESENTATION_CONTEXT"],
        ["coordinate_space_dimension"]
    ),
    ent!(
        "GLOBAL_UNIT_ASSIGNED_CONTEXT",
        ["REPRESENTATION_CONTEXT"],
        ["units"]
    ),
    ent!(
        "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT",
        ["REPRESENTATION_CONTEXT"],
        ["uncertainty"]
    ),
    ent!(
        "REPRESENTATION_RELATIONSHIP",
        [],
        ["name", "description", "rep_1", "rep_2"]
    ),
    ent!(
        "SHAPE_REPRESENTATION_RELATIONSHIP",
        ["REPRESENTATION_RELATIONSHIP"],
        []
    ),
    ent!(
        "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
        ["REPRESENTATION_RELATIONSHIP"],
        ["transformation_operator"]
    ),
    ent!(
        "ITEM_DEFINED_TRANSFORMATION",
        [],
        [
            "name",
            "description",
            "transform_item_1",
            "transform_item_2"
        ]
    ),
    ent!(
        "FUNCTIONALLY_DEFINED_TRANSFORMATION",
        [],
        ["name", "description"]
    ),
    ent!(
        "MAPPED_ITEM",
        ["REPRESENTATION_ITEM"],
        ["mapping_source", "mapping_target"]
    ),
    ent!(
        "REPRESENTATION_MAP",
        [],
        ["mapping_origin", "mapped_representation"]
    ),
    // --- product structure (ISO 10303-41 / -44) ----------------------
    ent!(
        "PRODUCT",
        [],
        ["id", "name", "description", "frame_of_reference"]
    ),
    ent!(
        "PRODUCT_DEFINITION_FORMATION",
        [],
        ["id", "description", "of_product"]
    ),
    ent!(
        "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
        ["PRODUCT_DEFINITION_FORMATION"],
        ["make_or_buy"]
    ),
    ent!(
        "PRODUCT_DEFINITION",
        [],
        ["id", "description", "formation", "frame_of_reference"]
    ),
    ent!(
        "PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS",
        ["PRODUCT_DEFINITION"],
        ["documentation_ids"]
    ),
    ent!(
        "PROPERTY_DEFINITION",
        [],
        ["name", "description", "definition"]
    ),
    ent!("PRODUCT_DEFINITION_SHAPE", ["PROPERTY_DEFINITION"], []),
    ent!(
        "PROPERTY_DEFINITION_REPRESENTATION",
        [],
        ["definition", "used_representation"]
    ),
    ent!(
        "SHAPE_DEFINITION_REPRESENTATION",
        ["PROPERTY_DEFINITION_REPRESENTATION"],
        []
    ),
    ent!(
        "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",
        [],
        ["representation_relation", "represented_product_relation"]
    ),
    ent!(
        "PRODUCT_DEFINITION_RELATIONSHIP",
        [],
        [
            "id",
            "name",
            "description",
            "relating_product_definition",
            "related_product_definition"
        ]
    ),
    ent!(
        "PRODUCT_DEFINITION_USAGE",
        ["PRODUCT_DEFINITION_RELATIONSHIP"],
        []
    ),
    ent!(
        "ASSEMBLY_COMPONENT_USAGE",
        ["PRODUCT_DEFINITION_USAGE"],
        ["reference_designator"]
    ),
    ent!(
        "NEXT_ASSEMBLY_USAGE_OCCURRENCE",
        ["ASSEMBLY_COMPONENT_USAGE"],
        []
    ),
    // --- units / measures (ISO 10303-41) ------------------------------
    ent!("NAMED_UNIT", [], ["dimensions"]),
    ent!("SI_UNIT", ["NAMED_UNIT"], ["prefix", "name"]),
    ent!(
        "CONVERSION_BASED_UNIT",
        ["NAMED_UNIT"],
        ["name", "conversion_factor"]
    ),
    ent!("LENGTH_UNIT", ["NAMED_UNIT"], []),
    ent!("PLANE_ANGLE_UNIT", ["NAMED_UNIT"], []),
    ent!("SOLID_ANGLE_UNIT", ["NAMED_UNIT"], []),
    ent!(
        "MEASURE_WITH_UNIT",
        [],
        ["value_component", "unit_component"]
    ),
    ent!("LENGTH_MEASURE_WITH_UNIT", ["MEASURE_WITH_UNIT"], []),
    ent!("PLANE_ANGLE_MEASURE_WITH_UNIT", ["MEASURE_WITH_UNIT"], []),
    ent!(
        "UNCERTAINTY_MEASURE_WITH_UNIT",
        ["MEASURE_WITH_UNIT"],
        ["name", "description"]
    ),
    ent!(
        "DIMENSIONAL_EXPONENTS",
        [],
        [
            "length_exponent",
            "mass_exponent",
            "time_exponent",
            "electric_current_exponent",
            "thermodynamic_temperature_exponent",
            "amount_of_substance_exponent",
            "luminous_intensity_exponent"
        ]
    ),
    // --- geometry (ISO 10303-42) -------------------------------------
    ent!("POINT", ["GEOMETRIC_REPRESENTATION_ITEM"], []),
    ent!("CARTESIAN_POINT", ["POINT"], ["coordinates"]),
    ent!(
        "DIRECTION",
        ["GEOMETRIC_REPRESENTATION_ITEM"],
        ["direction_ratios"]
    ),
    ent!(
        "VECTOR",
        ["GEOMETRIC_REPRESENTATION_ITEM"],
        ["orientation", "magnitude"]
    ),
    ent!("PLACEMENT", ["GEOMETRIC_REPRESENTATION_ITEM"], ["location"]),
    ent!("AXIS1_PLACEMENT", ["PLACEMENT"], ["axis"]),
    ent!("AXIS2_PLACEMENT_2D", ["PLACEMENT"], ["ref_direction"]),
    ent!(
        "AXIS2_PLACEMENT_3D",
        ["PLACEMENT"],
        ["axis", "ref_direction"]
    ),
    ent!(
        "CARTESIAN_TRANSFORMATION_OPERATOR",
        [
            "GEOMETRIC_REPRESENTATION_ITEM",
            "FUNCTIONALLY_DEFINED_TRANSFORMATION"
        ],
        ["axis1", "axis2", "local_origin", "scale"]
    ),
    ent!(
        "CARTESIAN_TRANSFORMATION_OPERATOR_3D",
        ["CARTESIAN_TRANSFORMATION_OPERATOR"],
        ["axis3"]
    ),
    ent!(
        "CARTESIAN_TRANSFORMATION_OPERATOR_2D",
        ["CARTESIAN_TRANSFORMATION_OPERATOR"],
        []
    ),
    ent!("CURVE", ["GEOMETRIC_REPRESENTATION_ITEM"], []),
    ent!("LINE", ["CURVE"], ["pnt", "dir"]),
    ent!("CONIC", ["CURVE"], ["position"]),
    ent!("CIRCLE", ["CONIC"], ["radius"]),
    ent!("ELLIPSE", ["CONIC"], ["semi_axis_1", "semi_axis_2"]),
    ent!("HYPERBOLA", ["CONIC"], ["semi_axis", "semi_imag_axis"]),
    ent!("PARABOLA", ["CONIC"], ["focal_dist"]),
    ent!("BOUNDED_CURVE", ["CURVE"], []),
    ent!("POLYLINE", ["BOUNDED_CURVE"], ["points"]),
    ent!(
        "B_SPLINE_CURVE",
        ["BOUNDED_CURVE"],
        [
            "degree",
            "control_points_list",
            "curve_form",
            "closed_curve",
            "self_intersect"
        ]
    ),
    ent!(
        "B_SPLINE_CURVE_WITH_KNOTS",
        ["B_SPLINE_CURVE"],
        ["knot_multiplicities", "knots", "knot_spec"]
    ),
    ent!("UNIFORM_CURVE", ["B_SPLINE_CURVE"], []),
    ent!("QUASI_UNIFORM_CURVE", ["B_SPLINE_CURVE"], []),
    ent!("BEZIER_CURVE", ["B_SPLINE_CURVE"], []),
    ent!(
        "RATIONAL_B_SPLINE_CURVE",
        ["B_SPLINE_CURVE"],
        ["weights_data"]
    ),
    ent!(
        "TRIMMED_CURVE",
        ["BOUNDED_CURVE"],
        [
            "basis_curve",
            "trim_1",
            "trim_2",
            "sense_agreement",
            "master_representation"
        ]
    ),
    ent!(
        "COMPOSITE_CURVE",
        ["BOUNDED_CURVE"],
        ["segments", "self_intersect"]
    ),
    ent!("COMPOSITE_CURVE_ON_SURFACE", ["COMPOSITE_CURVE"], []),
    ent!("BOUNDARY_CURVE", ["COMPOSITE_CURVE_ON_SURFACE"], []),
    ent!("OUTER_BOUNDARY_CURVE", ["BOUNDARY_CURVE"], []),
    ent!(
        "COMPOSITE_CURVE_SEGMENT",
        ["FOUNDED_ITEM"],
        ["transition", "same_sense", "parent_curve"]
    ),
    ent!(
        "REPARAMETRISED_COMPOSITE_CURVE_SEGMENT",
        ["COMPOSITE_CURVE_SEGMENT"],
        ["param_length"]
    ),
    ent!("PCURVE", ["CURVE"], ["basis_surface", "reference_to_curve"]),
    ent!(
        "SURFACE_CURVE",
        ["CURVE"],
        ["curve_3d", "associated_geometry", "master_representation"]
    ),
    ent!("SEAM_CURVE", ["SURFACE_CURVE"], []),
    ent!("INTERSECTION_CURVE", ["SURFACE_CURVE"], []),
    ent!(
        "BOUNDED_SURFACE_CURVE",
        ["SURFACE_CURVE", "BOUNDED_CURVE"],
        []
    ),
    ent!(
        "OFFSET_CURVE_3D",
        ["CURVE"],
        ["basis_curve", "distance", "self_intersect", "ref_direction"]
    ),
    ent!(
        "OFFSET_CURVE_2D",
        ["CURVE"],
        ["basis_curve", "distance", "self_intersect"]
    ),
    ent!("SURFACE", ["GEOMETRIC_REPRESENTATION_ITEM"], []),
    ent!("ELEMENTARY_SURFACE", ["SURFACE"], ["position"]),
    ent!("PLANE", ["ELEMENTARY_SURFACE"], []),
    ent!("CYLINDRICAL_SURFACE", ["ELEMENTARY_SURFACE"], ["radius"]),
    ent!(
        "CONICAL_SURFACE",
        ["ELEMENTARY_SURFACE"],
        ["radius", "semi_angle"]
    ),
    ent!("SPHERICAL_SURFACE", ["ELEMENTARY_SURFACE"], ["radius"]),
    ent!(
        "TOROIDAL_SURFACE",
        ["ELEMENTARY_SURFACE"],
        ["major_radius", "minor_radius"]
    ),
    ent!(
        "DEGENERATE_TOROIDAL_SURFACE",
        ["TOROIDAL_SURFACE"],
        ["select_outer"]
    ),
    ent!("SWEPT_SURFACE", ["SURFACE"], ["swept_curve"]),
    ent!(
        "SURFACE_OF_LINEAR_EXTRUSION",
        ["SWEPT_SURFACE"],
        ["extrusion_axis"]
    ),
    ent!(
        "SURFACE_OF_REVOLUTION",
        ["SWEPT_SURFACE"],
        ["axis_position"]
    ),
    ent!(
        "OFFSET_SURFACE",
        ["SURFACE"],
        ["basis_surface", "distance", "self_intersect"]
    ),
    ent!("BOUNDED_SURFACE", ["SURFACE"], []),
    ent!(
        "B_SPLINE_SURFACE",
        ["BOUNDED_SURFACE"],
        [
            "u_degree",
            "v_degree",
            "control_points_list",
            "surface_form",
            "u_closed",
            "v_closed",
            "self_intersect"
        ]
    ),
    ent!(
        "B_SPLINE_SURFACE_WITH_KNOTS",
        ["B_SPLINE_SURFACE"],
        [
            "u_multiplicities",
            "v_multiplicities",
            "u_knots",
            "v_knots",
            "knot_spec"
        ]
    ),
    ent!("UNIFORM_SURFACE", ["B_SPLINE_SURFACE"], []),
    ent!("QUASI_UNIFORM_SURFACE", ["B_SPLINE_SURFACE"], []),
    ent!("BEZIER_SURFACE", ["B_SPLINE_SURFACE"], []),
    ent!(
        "RATIONAL_B_SPLINE_SURFACE",
        ["B_SPLINE_SURFACE"],
        ["weights_data"]
    ),
    ent!(
        "RECTANGULAR_TRIMMED_SURFACE",
        ["BOUNDED_SURFACE"],
        ["basis_surface", "u1", "u2", "v1", "v2", "usense", "vsense"]
    ),
    ent!(
        "CURVE_BOUNDED_SURFACE",
        ["BOUNDED_SURFACE"],
        ["basis_surface", "boundaries", "implicit_outer"]
    ),
    // --- topology (ISO 10303-42) -------------------------------------
    ent!("VERTEX", ["TOPOLOGICAL_REPRESENTATION_ITEM"], []),
    ent!(
        "VERTEX_POINT",
        ["VERTEX", "GEOMETRIC_REPRESENTATION_ITEM"],
        ["vertex_geometry"]
    ),
    ent!(
        "EDGE",
        ["TOPOLOGICAL_REPRESENTATION_ITEM"],
        ["edge_start", "edge_end"]
    ),
    ent!(
        "EDGE_CURVE",
        ["EDGE", "GEOMETRIC_REPRESENTATION_ITEM"],
        ["edge_geometry", "same_sense"]
    ),
    ent!("ORIENTED_EDGE", ["EDGE"], ["edge_element", "orientation"]),
    ent!("SUBEDGE", ["EDGE"], ["parent_edge"]),
    ent!("PATH", ["TOPOLOGICAL_REPRESENTATION_ITEM"], ["edge_list"]),
    ent!("LOOP", ["TOPOLOGICAL_REPRESENTATION_ITEM"], []),
    ent!("EDGE_LOOP", ["LOOP", "PATH"], []),
    ent!(
        "POLY_LOOP",
        ["LOOP", "GEOMETRIC_REPRESENTATION_ITEM"],
        ["polygon"]
    ),
    ent!("VERTEX_LOOP", ["LOOP"], ["loop_vertex"]),
    ent!(
        "FACE_BOUND",
        ["TOPOLOGICAL_REPRESENTATION_ITEM"],
        ["bound", "orientation"]
    ),
    ent!("FACE_OUTER_BOUND", ["FACE_BOUND"], []),
    ent!("FACE", ["TOPOLOGICAL_REPRESENTATION_ITEM"], ["bounds"]),
    ent!(
        "FACE_SURFACE",
        ["FACE", "GEOMETRIC_REPRESENTATION_ITEM"],
        ["face_geometry", "same_sense"]
    ),
    ent!("ADVANCED_FACE", ["FACE_SURFACE"], []),
    ent!("ORIENTED_FACE", ["FACE"], ["face_element", "orientation"]),
    ent!(
        "CONNECTED_FACE_SET",
        ["TOPOLOGICAL_REPRESENTATION_ITEM"],
        ["cfs_faces"]
    ),
    ent!("OPEN_SHELL", ["CONNECTED_FACE_SET"], []),
    ent!("CLOSED_SHELL", ["CONNECTED_FACE_SET"], []),
    ent!(
        "ORIENTED_CLOSED_SHELL",
        ["CLOSED_SHELL"],
        ["closed_shell_element", "orientation"]
    ),
    ent!(
        "ORIENTED_OPEN_SHELL",
        ["OPEN_SHELL"],
        ["open_shell_element", "orientation"]
    ),
    ent!("SOLID_MODEL", ["GEOMETRIC_REPRESENTATION_ITEM"], []),
    ent!("MANIFOLD_SOLID_BREP", ["SOLID_MODEL"], ["outer"]),
    ent!("BREP_WITH_VOIDS", ["MANIFOLD_SOLID_BREP"], ["voids"]),
    ent!("FACETED_BREP", ["MANIFOLD_SOLID_BREP"], []),
    ent!(
        "SHELL_BASED_SURFACE_MODEL",
        ["GEOMETRIC_REPRESENTATION_ITEM"],
        ["sbsm_boundary"]
    ),
    ent!(
        "FACE_BASED_SURFACE_MODEL",
        ["GEOMETRIC_REPRESENTATION_ITEM"],
        ["fbsm_faces"]
    ),
    ent!(
        "GEOMETRIC_SET",
        ["GEOMETRIC_REPRESENTATION_ITEM"],
        ["elements"]
    ),
    ent!("GEOMETRIC_CURVE_SET", ["GEOMETRIC_SET"], []),
    // --- AP242 tessellated geometry (ISO 10303-42 ed. 4+) -------------
    ent!("TESSELLATED_ITEM", ["GEOMETRIC_REPRESENTATION_ITEM"], []),
    ent!(
        "COORDINATES_LIST",
        ["TESSELLATED_ITEM"],
        ["npoints", "position_coords"]
    ),
    ent!(
        "TESSELLATED_SOLID",
        ["TESSELLATED_ITEM"],
        ["items", "geometric_link"]
    ),
    ent!(
        "TESSELLATED_SHELL",
        ["TESSELLATED_ITEM"],
        ["items", "topological_link"]
    ),
    ent!("TESSELLATED_STRUCTURED_ITEM", ["TESSELLATED_ITEM"], []),
    ent!(
        "TESSELLATED_FACE",
        ["TESSELLATED_STRUCTURED_ITEM"],
        ["coordinates", "pnmax", "normals", "geometric_link"]
    ),
    ent!(
        "TRIANGULATED_FACE",
        ["TESSELLATED_FACE"],
        ["pnindex", "triangles"]
    ),
    ent!(
        "COMPLEX_TRIANGULATED_FACE",
        ["TESSELLATED_FACE"],
        ["pnindex", "triangle_strips", "triangle_fans"]
    ),
    ent!(
        "TESSELLATED_SURFACE_SET",
        ["TESSELLATED_ITEM"],
        ["coordinates", "pnmax", "normals"]
    ),
    ent!(
        "TRIANGULATED_SURFACE_SET",
        ["TESSELLATED_SURFACE_SET"],
        ["pnindex", "triangles"]
    ),
    ent!(
        "COMPLEX_TRIANGULATED_SURFACE_SET",
        ["TESSELLATED_SURFACE_SET"],
        ["pnindex", "triangle_strips", "triangle_fans"]
    ),
    ent!(
        "TESSELLATED_GEOMETRIC_SET",
        ["TESSELLATED_ITEM"],
        ["children"]
    ),
    // --- presentation (ISO 10303-46) ---------------------------------
    ent!("STYLED_ITEM", ["REPRESENTATION_ITEM"], ["styles", "item"]),
    ent!(
        "OVER_RIDING_STYLED_ITEM",
        ["STYLED_ITEM"],
        ["over_ridden_style"]
    ),
    ent!(
        "CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM",
        ["OVER_RIDING_STYLED_ITEM"],
        ["style_context"]
    ),
    ent!(
        "PRESENTATION_STYLE_ASSIGNMENT",
        ["FOUNDED_ITEM"],
        ["styles"]
    ),
    ent!(
        "PRESENTATION_STYLE_BY_CONTEXT",
        ["PRESENTATION_STYLE_ASSIGNMENT"],
        ["style_context"]
    ),
    ent!("SURFACE_STYLE_USAGE", ["FOUNDED_ITEM"], ["side", "style"]),
    ent!("SURFACE_SIDE_STYLE", ["FOUNDED_ITEM"], ["name", "styles"]),
    ent!("SURFACE_STYLE_FILL_AREA", ["FOUNDED_ITEM"], ["fill_area"]),
    ent!("FILL_AREA_STYLE", ["FOUNDED_ITEM"], ["name", "fill_styles"]),
    ent!("FILL_AREA_STYLE_COLOUR", [], ["name", "fill_colour"]),
    ent!(
        "SURFACE_STYLE_RENDERING",
        [],
        ["rendering_method", "surface_colour"]
    ),
    ent!(
        "SURFACE_STYLE_RENDERING_WITH_PROPERTIES",
        ["SURFACE_STYLE_RENDERING"],
        ["properties"]
    ),
    ent!("SURFACE_STYLE_TRANSPARENT", [], ["transparency"]),
    ent!("COLOUR", [], []),
    ent!("COLOUR_SPECIFICATION", ["COLOUR"], ["name"]),
    ent!(
        "COLOUR_RGB",
        ["COLOUR_SPECIFICATION"],
        ["red", "green", "blue"]
    ),
    ent!("PRE_DEFINED_ITEM", [], ["name"]),
    ent!("PRE_DEFINED_COLOUR", ["PRE_DEFINED_ITEM", "COLOUR"], []),
    ent!("DRAUGHTING_PRE_DEFINED_COLOUR", ["PRE_DEFINED_COLOUR"], []),
    ent!(
        "PRESENTATION_LAYER_ASSIGNMENT",
        [],
        ["name", "description", "assigned_items"]
    ),
    ent!("INVISIBILITY", [], ["invisible_items"]),
];

/// Look up a transcribed entity by (upper-case) name.
pub fn entity_def(name: &str) -> Option<&'static EntityDef> {
    index().defs.get(name).copied()
}

struct Index {
    defs: HashMap<&'static str, &'static EntityDef>,
    /// Inheritance-resolved attribute order per entity:
    /// `(declaring entity, attribute)`.
    orders: HashMap<&'static str, Vec<(&'static str, &'static str)>>,
    /// Every supertype (transitively, the entity itself included).
    ancestors: HashMap<&'static str, Vec<&'static str>>,
}

fn index() -> &'static Index {
    static INDEX: OnceLock<Index> = OnceLock::new();
    INDEX.get_or_init(|| {
        let defs: HashMap<&'static str, &'static EntityDef> =
            ENTITIES.iter().map(|d| (d.name, d)).collect();
        let mut orders = HashMap::new();
        let mut ancestors = HashMap::new();
        for d in ENTITIES {
            let mut seen: Vec<&'static str> = Vec::new();
            collect_chain(&defs, d.name, &mut seen, 0);
            let mut order = Vec::new();
            for &e in &seen {
                if let Some(def) = defs.get(e) {
                    for &a in def.attrs {
                        order.push((def.name, a));
                    }
                }
            }
            orders.insert(d.name, order);
            ancestors.insert(d.name, seen);
        }
        Index {
            defs,
            orders,
            ancestors,
        }
    })
}

/// Parent-first post-order walk of the supertype graph: every entity
/// once, its supertypes (in declaration order) before itself.
fn collect_chain(
    defs: &HashMap<&'static str, &'static EntityDef>,
    name: &'static str,
    out: &mut Vec<&'static str>,
    depth: usize,
) {
    if depth > 32 || out.contains(&name) {
        return;
    }
    if let Some(def) = defs.get(name) {
        for &s in def.supertypes {
            collect_chain(defs, s, out, depth + 1);
        }
        out.push(def.name);
    }
}

/// The inheritance-resolved serialisation order of `entity`'s explicit
/// attributes as `(declaring entity, attribute name)` pairs.
pub fn attribute_order(entity: &str) -> Option<&'static [(&'static str, &'static str)]> {
    index().orders.get(entity).map(Vec::as_slice)
}

/// True when `entity` is `ancestor` or (transitively) a subtype of it.
pub fn is_subtype_of(entity: &str, ancestor: &str) -> bool {
    index()
        .ancestors
        .get(entity)
        .is_some_and(|a| a.contains(&ancestor))
}

/// A borrowing, schema-aware view of one instance.
#[derive(Debug, Clone, Copy)]
pub struct Entity<'a> {
    /// The instance.
    pub inst: &'a ParsedInstance,
}

impl<'a> Entity<'a> {
    /// View instance `id` of `step`.
    pub fn get(step: &'a StepFile, id: u64) -> Option<Self> {
        step.get(id).map(|inst| Self { inst })
    }

    /// The instance id.
    pub fn id(&self) -> u64 {
        self.inst.id
    }

    /// True when the instance is an `entity` (by its own type or a
    /// partial record of a complex instance, or through the supertype
    /// graph of either).
    pub fn is_a(&self, entity: &str) -> bool {
        self.inst
            .keywords()
            .any(|k| k == entity || is_subtype_of(k, entity))
    }

    /// The leaf keyword for a simple instance; for a complex one the
    /// first partial keyword that is a subtype of `family` (if any).
    pub fn type_in(&self, family: &str) -> Option<&'a str> {
        self.inst
            .keywords()
            .find(|k| *k == family || is_subtype_of(k, family))
    }

    /// Attribute `name` (lower case) resolved by the schema: for a
    /// simple instance by its inheritance-resolved position, for a
    /// complex instance from the partial record of the entity that
    /// declares it. `None` when the attribute is unknown or absent.
    pub fn attr(&self, name: &str) -> Option<&'a Value> {
        let inst = self.inst;
        if inst.parts.is_empty() {
            let order = attribute_order(&inst.keyword)?;
            let idx = order.iter().position(|(_, a)| *a == name)?;
            return inst.args.get(idx);
        }
        for part in &inst.parts {
            if let Some(def) = entity_def(&part.keyword) {
                if let Some(i) = def.attrs.iter().position(|a| *a == name) {
                    return part.args.get(i);
                }
            }
        }
        None
    }

    /// Attribute `name` as declared by entity `declaring` specifically
    /// (disambiguates inherited homonyms such as the two `name`
    /// attributes of a cartesian transformation operator).
    pub fn attr_of(&self, declaring: &str, name: &str) -> Option<&'a Value> {
        let inst = self.inst;
        if inst.parts.is_empty() {
            let order = attribute_order(&inst.keyword)?;
            let idx = order
                .iter()
                .position(|(e, a)| *e == declaring && *a == name)?;
            return inst.args.get(idx);
        }
        let part = inst.parts.iter().find(|p| p.keyword == declaring)?;
        let def = entity_def(declaring)?;
        let i = def.attrs.iter().position(|a| *a == name)?;
        part.args.get(i)
    }

    /// Referenced instance id of attribute `name`.
    pub fn reference(&self, name: &str) -> Option<u64> {
        self.attr(name).and_then(Value::as_reference)
    }

    /// Numeric value of attribute `name` (a typed measure such as
    /// `LENGTH_MEASURE(2.5)` unwrapped).
    pub fn number(&self, name: &str) -> Option<f64> {
        self.attr(name).and_then(number)
    }

    /// String value of attribute `name`.
    pub fn string(&self, name: &str) -> Option<&'a str> {
        match self.attr(name)? {
            Value::String(s) => Some(s),
            Value::Typed { args, .. } => args.first().and_then(Value::as_str),
            _ => None,
        }
    }

    /// Logical / boolean attribute: `Some(true)` for `.T.`, `Some(false)`
    /// for `.F.`, `None` for `.U.` / absent.
    pub fn logical(&self, name: &str) -> Option<bool> {
        logical(self.attr(name)?)
    }

    /// Aggregate attribute items.
    pub fn list(&self, name: &str) -> Option<&'a [Value]> {
        self.attr(name).and_then(Value::as_list)
    }
}

/// Numeric payload of a value, unwrapping one typed-measure wrapper.
pub fn number(v: &Value) -> Option<f64> {
    let x = match v {
        Value::Typed { args, .. } => args.first().and_then(Value::as_number)?,
        other => other.as_number()?,
    };
    x.is_finite().then_some(x)
}

/// `.T.` / `.F.` → `Some(bool)`; anything else `None`.
pub fn logical(v: &Value) -> Option<bool> {
    match v.as_enum()? {
        "T" | "TRUE" => Some(true),
        "F" | "FALSE" => Some(false),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_ifc::parse_step;

    fn file(data: &str) -> StepFile {
        let text = format!(
            "ISO-10303-21;HEADER;FILE_DESCRIPTION((''),'2;1');\
             FILE_NAME('','',(''),(''),'','','');\
             FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));ENDSEC;\
             DATA;{data}ENDSEC;END-ISO-10303-21;"
        );
        parse_step(text.as_bytes()).unwrap()
    }

    #[test]
    fn schema_detection_strips_object_identifiers() {
        assert_eq!(
            ApSchema::from_identifier("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }"),
            ApSchema::Ap214
        );
        assert_eq!(
            ApSchema::from_identifier("config_control_design"),
            ApSchema::Ap203
        );
        assert_eq!(
            ApSchema::from_identifier(
                "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }"
            ),
            ApSchema::Ap242
        );
        assert_eq!(
            ApSchema::from_identifier(
                "AP203_CONFIGURATION_CONTROLLED_3D_DESIGN_OF_MECHANICAL_PARTS_AND_ASSEMBLIES_MIM_LF"
            ),
            ApSchema::Ap203e2
        );
        assert!(matches!(ApSchema::from_identifier("IFC4"), ApSchema::Other(s) if s == "IFC4"));
        assert_eq!(ApSchema::of_file(&file("")), ApSchema::Ap214);
    }

    #[test]
    fn inheritance_resolved_orders() {
        let order: Vec<&str> = attribute_order("ADVANCED_FACE")
            .unwrap()
            .iter()
            .map(|(_, a)| *a)
            .collect();
        assert_eq!(order, ["name", "bounds", "face_geometry", "same_sense"]);
        let order: Vec<&str> = attribute_order("EDGE_LOOP")
            .unwrap()
            .iter()
            .map(|(_, a)| *a)
            .collect();
        assert_eq!(order, ["name", "edge_list"]);
        let order: Vec<&str> = attribute_order("NEXT_ASSEMBLY_USAGE_OCCURRENCE")
            .unwrap()
            .iter()
            .map(|(_, a)| *a)
            .collect();
        assert_eq!(
            order,
            [
                "id",
                "name",
                "description",
                "relating_product_definition",
                "related_product_definition",
                "reference_designator"
            ]
        );
        // Homonymous attributes from two supertypes both serialise.
        let order = attribute_order("CARTESIAN_TRANSFORMATION_OPERATOR_3D").unwrap();
        assert_eq!(order.len(), 8);
        assert_eq!(order[0], ("REPRESENTATION_ITEM", "name"));
        assert_eq!(order[1], ("FUNCTIONALLY_DEFINED_TRANSFORMATION", "name"));
        assert!(is_subtype_of("ADVANCED_FACE", "FACE"));
        assert!(is_subtype_of("CIRCLE", "CURVE"));
        assert!(!is_subtype_of("CIRCLE", "SURFACE"));
    }

    #[test]
    fn attributes_by_name_under_both_mappings() {
        let f = file(
            "#1=CARTESIAN_POINT('p',(1.,2.,3.));\
             #2=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\
             #3=( BOUNDED_CURVE() B_SPLINE_CURVE(2,(#1,#1,#1),.UNSPECIFIED.,.F.,.F.) \
                  B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.) CURVE() \
                  GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,0.5,1.)) \
                  REPRESENTATION_ITEM('') );\
             #4=CIRCLE('',#5,LENGTH_MEASURE(2.5));",
        );
        let p = Entity::get(&f, 1).unwrap();
        assert_eq!(p.string("name"), Some("p"));
        assert_eq!(p.list("coordinates").unwrap().len(), 3);
        assert!(p.is_a("GEOMETRIC_REPRESENTATION_ITEM"));
        let u = Entity::get(&f, 2).unwrap();
        assert!(u.is_a("LENGTH_UNIT") && u.is_a("NAMED_UNIT"));
        assert_eq!(u.attr("prefix").and_then(Value::as_enum), Some("MILLI"));
        assert_eq!(u.attr("name").and_then(Value::as_enum), Some("METRE"));
        let c = Entity::get(&f, 3).unwrap();
        assert!(c.is_a("CURVE") && c.is_a("RATIONAL_B_SPLINE_CURVE"));
        assert_eq!(c.number("degree"), Some(2.0));
        assert_eq!(c.list("weights_data").unwrap().len(), 3);
        assert_eq!(c.logical("closed_curve"), Some(false));
        assert_eq!(c.type_in("B_SPLINE_CURVE"), Some("B_SPLINE_CURVE"));
        let k = Entity::get(&f, 4).unwrap();
        assert_eq!(k.number("radius"), Some(2.5));
        assert_eq!(k.reference("position"), Some(5));
    }

    #[test]
    fn every_supertype_is_transcribed() {
        for d in ENTITIES {
            for s in d.supertypes {
                assert!(entity_def(s).is_some(), "{} -> {s}", d.name);
            }
        }
    }
}
