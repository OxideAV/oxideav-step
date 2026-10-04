//! The product-structure walk: products, their shape representations,
//! assembly occurrences and the tessellated geometry, as a std-typed
//! [`StepModel`].
//!
//! Product structure (ISO 10303-41 / -44, CAx-IF assembly practice):
//!
//! ```text
//! PRODUCT ← PRODUCT_DEFINITION_FORMATION ← PRODUCT_DEFINITION (pd)
//!   pd ← PRODUCT_DEFINITION_SHAPE ← SHAPE_DEFINITION_REPRESENTATION → SHAPE_REPRESENTATION
//!   SHAPE_REPRESENTATION ⇄ SHAPE_REPRESENTATION_RELATIONSHIP ⇄ ADVANCED_BREP_SHAPE_REPRESENTATION …
//! NEXT_ASSEMBLY_USAGE_OCCURRENCE(relating = parent pd, related = child pd)
//!   ← PRODUCT_DEFINITION_SHAPE ← CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
//!     → (REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(rep_1, rep_2) → ITEM_DEFINED_TRANSFORMATION(item_1, item_2))
//! MAPPED_ITEM(REPRESENTATION_MAP(origin, child rep), target) inside a parent representation
//! ```
//!
//! Each product definition becomes a [`Part`] (meshed once, instanced
//! by every [`Occurrence`]); a representation reached only through a
//! `mapped_item` becomes a part of its own. Which side of a
//! representation relationship is the component is decided from the
//! data (the representation belonging to the child product), and the
//! occurrence transform maps the child's placement item onto the
//! parent's: `T = M(item in parent) · M(item in child)⁻¹`. Every
//! representation's coordinates are converted to the model length unit
//! (the root product's), so parts authored in different units assemble
//! consistently.

use std::collections::{BTreeSet, HashMap, HashSet};

use oxideav_ifc::{
    parse_step_with_limits, Header, StepFile, StepLimits, Transform, TriMesh, Value,
};

use crate::error::{Error, Result};
use crate::geom::brep::{is_brep_item, BrepMesher};
use crate::geom::tessellated::{is_tessellated_item, TessMesher};
use crate::geom::{invert, rescale_translation, Geo, GeometryLimits, Tolerance};
use crate::schema::{ApSchema, Entity};
use crate::style::{Rgba, Styles};
use crate::units::{context_units, ContextUnits};

/// Reader options.
#[derive(Debug, Clone, Default)]
pub struct ReadOptions {
    /// Physical-file parser caps.
    pub parse_limits: StepLimits,
    /// Tessellation caps.
    pub geometry_limits: GeometryLimits,
    /// Tessellation density.
    pub tolerance: Tolerance,
    /// Keep items marked invisible (`invisibility`) instead of
    /// dropping them.
    pub keep_invisible: bool,
}

/// A tessellated representation item.
#[derive(Debug, Clone)]
pub struct Shape {
    /// The representation item id (solid, shell, face set, …).
    pub item: u64,
    /// The item's `name`, when not empty.
    pub name: Option<String>,
    /// Triangles in model length units, in the part's coordinate frame.
    pub mesh: TriMesh,
    /// The item's surface colour, if styled.
    pub colour: Option<Rgba>,
    /// Per-triangle colour overrides (face-level styling); empty when
    /// no face of the item is styled individually.
    pub triangle_colours: Vec<Option<Rgba>>,
    /// Presentation layers the item is assigned to.
    pub layers: Vec<String>,
}

/// A placed use of a part inside another (or at the top level).
#[derive(Debug, Clone)]
pub struct Occurrence {
    /// Index into [`StepModel::parts`].
    pub part: usize,
    /// The occurrence instance (`next_assembly_usage_occurrence` or
    /// `mapped_item`) id, if any.
    pub id: Option<u64>,
    /// The occurrence name / reference designator, when given.
    pub name: Option<String>,
    /// Child frame → parent frame (model length units).
    pub transform: Transform,
}

/// A product definition (or a stand-alone representation) with its
/// geometry and sub-occurrences.
#[derive(Debug, Clone, Default)]
pub struct Part {
    /// Product name.
    pub name: Option<String>,
    /// Product id (part number).
    pub product_id: Option<String>,
    /// The `product_definition` id, when the part is a product.
    pub definition: Option<u64>,
    /// The main shape representation id.
    pub representation: Option<u64>,
    /// Tessellated geometry.
    pub shapes: Vec<Shape>,
    /// Sub-assembly occurrences.
    pub children: Vec<Occurrence>,
}

/// A read STEP model.
#[derive(Debug, Clone)]
pub struct StepModel {
    /// The application protocol the file declares.
    pub schema: ApSchema,
    /// The HEADER section.
    pub header: Header,
    /// Metres per model length unit (every part's coordinates use it).
    pub length_unit_metres: f64,
    /// Parts (meshed once each).
    pub parts: Vec<Part>,
    /// Top-level occurrences.
    pub roots: Vec<Occurrence>,
    /// Non-fatal problems met while reading.
    pub warnings: Vec<String>,
}

impl StepModel {
    /// Total triangles across all parts (each part counted once).
    pub fn triangle_count(&self) -> usize {
        self.parts
            .iter()
            .flat_map(|p| &p.shapes)
            .map(|s| s.mesh.triangles.len())
            .sum()
    }
}

/// Read a STEP file into a [`StepModel`] with default options.
pub fn read_step(bytes: &[u8]) -> Result<StepModel> {
    read_step_with(bytes, &ReadOptions::default())
}

/// Read a STEP file into a [`StepModel`].
pub fn read_step_with(bytes: &[u8], opts: &ReadOptions) -> Result<StepModel> {
    let step = parse_step_with_limits(bytes, &opts.parse_limits)?;
    model_from_file(&step, opts)
}

/// Build a [`StepModel`] from an already parsed file.
pub fn model_from_file(step: &StepFile, opts: &ReadOptions) -> Result<StepModel> {
    let mut b = Builder::new(step, opts);
    b.index();
    b.build()?;
    let roots = std::mem::take(&mut b.roots);
    if b.parts.iter().all(|p| p.shapes.is_empty()) {
        return Err(Error::NoGeometry(match b.unsupported.iter().next() {
            Some(kw) => format!("only unsupported representation items (e.g. `{kw}`)"),
            None => "no shape representation with geometry".into(),
        }));
    }
    let mut warnings = b.warnings;
    for kw in &b.unsupported {
        warnings.push(format!("unsupported representation item `{kw}` skipped"));
    }
    Ok(StepModel {
        schema: ApSchema::of_file(step),
        header: step.header.clone(),
        length_unit_metres: b.unit_metres,
        parts: b.parts,
        roots,
        warnings,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum PartKey {
    Definition(u64),
    Representation(u64),
}

struct Builder<'a> {
    step: &'a StepFile,
    opts: &'a ReadOptions,
    styles: Styles,
    /// pd → its shape representations (from SDRs).
    reps_of_pd: HashMap<u64, Vec<u64>>,
    /// representation → owning pd.
    pd_of_rep: HashMap<u64, u64>,
    /// Plain shape-representation relationships (no transformation).
    links: HashMap<u64, Vec<u64>>,
    /// parent pd → [(nauo id, child pd)].
    nauo: HashMap<u64, Vec<(u64, u64)>>,
    /// Child pds (used by some NAUO).
    children: HashSet<u64>,
    /// nauo id → representation relationship (with transformation).
    cdsr: HashMap<u64, u64>,
    unit_metres: f64,
    parts: Vec<Part>,
    index: HashMap<PartKey, usize>,
    roots: Vec<Occurrence>,
    warnings: Vec<String>,
    unsupported: BTreeSet<String>,
    triangles: usize,
}

const MAX_PARTS: usize = 1_000_000;

impl<'a> Builder<'a> {
    fn new(step: &'a StepFile, opts: &'a ReadOptions) -> Self {
        Self {
            step,
            opts,
            styles: Styles::collect(step),
            reps_of_pd: HashMap::new(),
            pd_of_rep: HashMap::new(),
            links: HashMap::new(),
            nauo: HashMap::new(),
            children: HashSet::new(),
            cdsr: HashMap::new(),
            unit_metres: 0.001,
            parts: Vec::new(),
            index: HashMap::new(),
            roots: Vec::new(),
            warnings: Vec::new(),
            unsupported: BTreeSet::new(),
            triangles: 0,
        }
    }

    fn entity(&self, id: u64) -> Option<Entity<'a>> {
        Entity::get(self.step, id)
    }

    fn index(&mut self) {
        let step = self.step;
        // PRODUCT_DEFINITION_SHAPE → its definition.
        let mut pds_def: HashMap<u64, u64> = HashMap::new();
        for inst in step.instances.values() {
            let e = Entity { inst };
            if e.is_a("PROPERTY_DEFINITION") {
                if let Some(d) = e.reference("definition") {
                    pds_def.insert(inst.id, d);
                }
            }
        }
        for inst in step.instances.values() {
            let e = Entity { inst };
            if e.is_a("SHAPE_DEFINITION_REPRESENTATION") {
                let (Some(pds), Some(rep)) = (
                    e.reference("definition"),
                    e.reference("used_representation"),
                ) else {
                    continue;
                };
                let Some(&def) = pds_def.get(&pds) else {
                    continue;
                };
                if self
                    .entity(def)
                    .is_some_and(|d| d.is_a("PRODUCT_DEFINITION"))
                {
                    self.reps_of_pd.entry(def).or_default().push(rep);
                    self.pd_of_rep.entry(rep).or_insert(def);
                }
            } else if e.is_a("REPRESENTATION_RELATIONSHIP") {
                if e.is_a("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
                    continue;
                }
                let (Some(a), Some(b)) = (e.reference("rep_1"), e.reference("rep_2")) else {
                    continue;
                };
                self.links.entry(a).or_default().push(b);
                self.links.entry(b).or_default().push(a);
            } else if e.is_a("PRODUCT_DEFINITION_USAGE") {
                let (Some(parent), Some(child)) = (
                    e.reference("relating_product_definition"),
                    e.reference("related_product_definition"),
                ) else {
                    continue;
                };
                self.nauo.entry(parent).or_default().push((inst.id, child));
                self.children.insert(child);
            } else if e.is_a("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") {
                let (Some(rel), Some(pds)) = (
                    e.reference("representation_relation"),
                    e.reference("represented_product_relation"),
                ) else {
                    continue;
                };
                if let Some(&usage) = pds_def.get(&pds) {
                    self.cdsr.insert(usage, rel);
                }
            }
        }
        for v in self.reps_of_pd.values_mut() {
            v.sort_unstable();
            v.dedup();
        }
    }

    /// The representations of a group: `start` plus everything reached
    /// through plain shape-representation relationships.
    fn rep_group(&self, start: &[u64]) -> Vec<u64> {
        let mut seen: Vec<u64> = Vec::new();
        let mut stack: Vec<u64> = start.to_vec();
        while let Some(r) = stack.pop() {
            if seen.contains(&r) || seen.len() > 4096 {
                continue;
            }
            seen.push(r);
            if let Some(ls) = self.links.get(&r) {
                for &n in ls {
                    // Do not wander into another product's representations.
                    let other = self
                        .pd_of_rep
                        .get(&n)
                        .zip(start.first().and_then(|s| self.pd_of_rep.get(s)))
                        .is_some_and(|(a, b)| a != b);
                    if !other {
                        stack.push(n);
                    }
                }
            }
        }
        seen.sort_unstable();
        seen
    }

    fn context_units_of(&self, rep: u64) -> ContextUnits {
        match self
            .entity(rep)
            .and_then(|e| e.reference("context_of_items"))
        {
            Some(ctx) => context_units(self.step, ctx),
            None => ContextUnits::default(),
        }
    }

    fn build(&mut self) -> Result<()> {
        // Root product definitions: not the child of any usage.
        let mut pds: Vec<u64> = self
            .step
            .instances
            .values()
            .filter(|i| Entity { inst: i }.is_a("PRODUCT_DEFINITION"))
            .map(|i| i.id)
            .collect();
        pds.sort_unstable();
        let roots: Vec<u64> = pds
            .iter()
            .copied()
            .filter(|p| !self.children.contains(p))
            .collect();
        // Model unit: the first root with a representation.
        let unit_rep = roots
            .iter()
            .chain(pds.iter())
            .find_map(|p| self.reps_of_pd.get(p).and_then(|r| r.first().copied()))
            .or_else(|| self.shape_representations().first().copied());
        if let Some(r) = unit_rep {
            self.unit_metres = self.context_units_of(r).length_metres;
        }
        for &p in &roots {
            let idx = self.part_for(PartKey::Definition(p), 0)?;
            if self.subtree_has_geometry(idx) {
                self.roots.push(Occurrence {
                    part: idx,
                    id: None,
                    name: None,
                    transform: Transform::IDENTITY,
                });
            }
        }
        if self.roots.is_empty() {
            // No product structure: every shape representation that is
            // not part of another's group stands alone.
            let reps = self.shape_representations();
            let mut covered: HashSet<u64> = HashSet::new();
            for r in reps {
                if covered.contains(&r) {
                    continue;
                }
                for g in self.rep_group(&[r]) {
                    covered.insert(g);
                }
                let idx = self.part_for(PartKey::Representation(r), 0)?;
                if self.subtree_has_geometry(idx) {
                    self.roots.push(Occurrence {
                        part: idx,
                        id: None,
                        name: None,
                        transform: Transform::IDENTITY,
                    });
                }
            }
        }
        Ok(())
    }

    fn shape_representations(&self) -> Vec<u64> {
        let mut out: Vec<u64> = self
            .step
            .instances
            .values()
            .filter(|i| {
                let e = Entity { inst: i };
                e.is_a("SHAPE_REPRESENTATION")
            })
            .map(|i| i.id)
            .collect();
        out.sort_unstable();
        out
    }

    fn subtree_has_geometry(&self, idx: usize) -> bool {
        let mut seen = HashSet::new();
        let mut stack = vec![idx];
        while let Some(i) = stack.pop() {
            if !seen.insert(i) {
                continue;
            }
            let p = &self.parts[i];
            if !p.shapes.is_empty() {
                return true;
            }
            stack.extend(p.children.iter().map(|c| c.part));
        }
        false
    }

    /// The part for `key`, created (and its geometry meshed) on first
    /// use. Inserted into the index before its children are resolved,
    /// so a cyclic structure terminates.
    fn part_for(&mut self, key: PartKey, depth: usize) -> Result<usize> {
        if let Some(&i) = self.index.get(&key) {
            return Ok(i);
        }
        if self.parts.len() >= MAX_PARTS || depth > 256 {
            return Err(Error::LimitExceeded("assembly structure too large".into()));
        }
        let idx = self.parts.len();
        self.parts.push(Part::default());
        self.index.insert(key, idx);
        let (reps, mut part) = match key {
            PartKey::Definition(pd) => {
                let reps = self.reps_of_pd.get(&pd).cloned().unwrap_or_default();
                let mut part = Part {
                    definition: Some(pd),
                    representation: reps.first().copied(),
                    ..Part::default()
                };
                if let Some((id, name)) = self.product_of(pd) {
                    part.product_id = id;
                    part.name = name;
                }
                (self.rep_group(&reps), part)
            }
            PartKey::Representation(rep) => {
                let part = Part {
                    representation: Some(rep),
                    name: self
                        .entity(rep)
                        .and_then(|e| e.string("name"))
                        .filter(|s| !s.is_empty())
                        .map(str::to_string),
                    ..Part::default()
                };
                (self.rep_group(&[rep]), part)
            }
        };
        let mut mapped: Vec<(u64, u64)> = Vec::new();
        for &r in &reps {
            let (shapes, maps) = self.mesh_representation(r);
            part.shapes.extend(shapes);
            mapped.extend(maps.into_iter().map(|m| (r, m)));
        }
        // Mapped items → occurrences of the mapped representation.
        let mut mapped_reps: HashSet<u64> = HashSet::new();
        for (parent_rep, item) in mapped {
            match self.mapped_occurrence(parent_rep, item, depth) {
                Ok(Some((occ, rep))) => {
                    mapped_reps.insert(rep);
                    part.children.push(occ);
                }
                Ok(None) => {}
                Err(e) => self.warnings.push(format!("mapped item #{item}: {e}")),
            }
        }
        // Assembly usages.
        if let PartKey::Definition(pd) = key {
            let usages = self.nauo.get(&pd).cloned().unwrap_or_default();
            for (usage, child) in usages {
                let child_reps =
                    self.rep_group(&self.reps_of_pd.get(&child).cloned().unwrap_or_default());
                let transform = match self.cdsr.get(&usage).copied() {
                    Some(rel) => match self.usage_transform(rel, &reps, &child_reps) {
                        Some(t) => t,
                        None => {
                            self.warnings.push(format!(
                                "assembly usage #{usage}: transformation not resolved, identity used"
                            ));
                            Transform::IDENTITY
                        }
                    },
                    None => {
                        // Placed by a mapped item of the parent instead.
                        if child_reps.iter().any(|r| mapped_reps.contains(r)) {
                            continue;
                        }
                        Transform::IDENTITY
                    }
                };
                let child_idx = self.part_for(PartKey::Definition(child), depth + 1)?;
                let e = self.entity(usage);
                let name = e
                    .and_then(|e| {
                        e.string("reference_designator")
                            .filter(|s| !s.is_empty())
                            .or_else(|| e.string("id").filter(|s| !s.is_empty()))
                            .or_else(|| e.string("name").filter(|s| !s.is_empty()))
                    })
                    .map(str::to_string);
                part.children.push(Occurrence {
                    part: child_idx,
                    id: Some(usage),
                    name,
                    transform,
                });
            }
        }
        self.parts[idx] = part;
        Ok(idx)
    }

    /// `(product id, product name)` of a product definition.
    fn product_of(&self, pd: u64) -> Option<(Option<String>, Option<String>)> {
        let pde = self.entity(pd)?;
        let pdf = self.entity(pde.reference("formation")?)?;
        let prod = self.entity(pdf.reference("of_product")?)?;
        let s = |n: &str| prod.string(n).filter(|s| !s.is_empty()).map(str::to_string);
        Some((s("id"), s("name").or_else(|| s("id"))))
    }

    /// The child-to-parent transform of an assembly usage from its
    /// representation relationship (model units).
    fn usage_transform(
        &self,
        rel: u64,
        parent_reps: &[u64],
        child_reps: &[u64],
    ) -> Option<Transform> {
        let e = self.entity(rel)?;
        let r1 = e.reference("rep_1")?;
        let r2 = e.reference("rep_2")?;
        let op = e.reference("transformation_operator")?;
        let ope = self.entity(op)?;
        // Which side is the child?
        let child_is_1 = if child_reps.contains(&r1) {
            true
        } else if child_reps.contains(&r2) {
            false
        } else {
            // Fall back on the parent side, else the CAx-IF convention
            // (rep_1 = component).
            !parent_reps.contains(&r1)
        };
        let f1 = self.context_units_of(r1).length_metres / self.unit_metres;
        let f2 = self.context_units_of(r2).length_metres / self.unit_metres;
        let geo = Geo::new(
            self.step,
            &ContextUnits::default(),
            1e-3,
            0.2,
            self.opts.geometry_limits,
        );
        // T maps rep_1 coordinates into rep_2 coordinates.
        let t12 = if ope.is_a("ITEM_DEFINED_TRANSFORMATION") {
            let m1 =
                rescale_translation(&geo.placement(ope.reference("transform_item_1")?).ok()?, f1);
            let m2 =
                rescale_translation(&geo.placement(ope.reference("transform_item_2")?).ok()?, f2);
            m2.compose(&invert(&m1)?)
        } else {
            // A functionally defined transformation (cartesian operator)
            // maps rep_1 into rep_2 directly.
            let t = geo.placement(op).ok()?;
            let t = rescale_translation(&t, f2);
            // Re-express the source side in its own units.
            Transform {
                cols: [
                    crate::geom::scale(t.cols[0], f2 / f1),
                    crate::geom::scale(t.cols[1], f2 / f1),
                    crate::geom::scale(t.cols[2], f2 / f1),
                ],
                translation: t.translation,
            }
        };
        if child_is_1 {
            Some(t12)
        } else {
            invert(&t12)
        }
    }

    /// The occurrence a `mapped_item` stands for.
    fn mapped_occurrence(
        &mut self,
        parent_rep: u64,
        item: u64,
        depth: usize,
    ) -> Result<Option<(Occurrence, u64)>> {
        let Some(e) = self.entity(item) else {
            return Ok(None);
        };
        let Some(map) = e.reference("mapping_source").and_then(|m| self.entity(m)) else {
            return Ok(None);
        };
        let (Some(origin), Some(child_rep)) = (
            map.reference("mapping_origin"),
            map.reference("mapped_representation"),
        ) else {
            return Ok(None);
        };
        let Some(target) = e.reference("mapping_target") else {
            return Ok(None);
        };
        let fp = self.context_units_of(parent_rep).length_metres / self.unit_metres;
        let fc = self.context_units_of(child_rep).length_metres / self.unit_metres;
        let geo = Geo::new(
            self.step,
            &ContextUnits::default(),
            1e-3,
            0.2,
            self.opts.geometry_limits,
        );
        let mo = geo
            .placement(origin)
            .map_err(|e| Error::NoGeometry(e.to_string()))?;
        let mt = geo
            .placement(target)
            .map_err(|e| Error::NoGeometry(e.to_string()))?;
        let mo = rescale_translation(&mo, fc);
        let mt = rescale_translation(&mt, fp);
        let Some(inv) = invert(&mo) else {
            return Ok(None);
        };
        let transform = mt.compose(&inv);
        let key = match self.pd_of_rep.get(&child_rep) {
            Some(&pd) => PartKey::Definition(pd),
            None => PartKey::Representation(child_rep),
        };
        let part = self.part_for(key, depth + 1)?;
        let name = e
            .string("name")
            .filter(|s| !s.is_empty())
            .map(str::to_string);
        Ok(Some((
            Occurrence {
                part,
                id: Some(item),
                name,
                transform,
            },
            child_rep,
        )))
    }

    /// Mesh the items of one representation (model units); returns the
    /// shapes and the mapped items found.
    fn mesh_representation(&mut self, rep: u64) -> (Vec<Shape>, Vec<u64>) {
        let mut shapes = Vec::new();
        let mut mapped = Vec::new();
        let Some(e) = self.entity(rep) else {
            return (shapes, mapped);
        };
        let items: Vec<u64> = e
            .list("items")
            .unwrap_or(&[])
            .iter()
            .filter_map(Value::as_reference)
            .collect();
        let units = self.context_units_of(rep);
        let factor = units.length_metres / self.unit_metres;
        let diag = bbox_diagonal(self.step, rep);
        let tol = match self.opts.tolerance.absolute {
            Some(t) if t > 0.0 => t / units.length_metres,
            _ => (diag * self.opts.tolerance.relative).max(units.uncertainty.unwrap_or(0.0)),
        };
        let tol = if tol > 0.0 && tol.is_finite() {
            tol
        } else {
            1e-3
        };
        let rep_colour = self.styles.colour_of.get(&rep).copied();
        let mut geo = Geo::new(
            self.step,
            &units,
            tol,
            self.opts.tolerance.max_angle,
            self.opts.geometry_limits,
        );
        geo.size = diag;
        for item in items {
            if !self.opts.keep_invisible && self.styles.hidden.contains(&item) {
                continue;
            }
            let Some(ie) = self.entity(item) else {
                continue;
            };
            if ie.is_a("MAPPED_ITEM") {
                mapped.push(item);
                continue;
            }
            let meshed = if is_brep_item(&geo, item) {
                let mut m = BrepMesher::new(&mut geo);
                let r = m.item(item);
                let warnings = std::mem::take(&mut m.warnings);
                self.warnings.extend(warnings);
                r.map(|()| m.finish())
            } else if is_tessellated_item(&geo, item) {
                let mut t = TessMesher::new();
                t.item(&geo, item).map(|()| t.finish())
            } else {
                if !ie.is_a("PLACEMENT") && !ie.is_a("STYLED_ITEM") {
                    self.unsupported.insert(ie.inst.keyword.clone());
                }
                continue;
            };
            let bm = match meshed {
                Ok(m) => m,
                Err(err) => {
                    self.warnings.push(format!("item #{item}: {err}"));
                    continue;
                }
            };
            if bm.mesh.triangles.is_empty() {
                continue;
            }
            self.triangles += bm.mesh.triangles.len();
            if self.triangles > self.opts.geometry_limits.max_triangles {
                self.warnings
                    .push("triangle budget exhausted; remaining geometry skipped".into());
                break;
            }
            let mut mesh = bm.mesh;
            if (factor - 1.0).abs() > 1e-15 {
                for p in &mut mesh.positions {
                    *p = crate::geom::scale(*p, factor);
                }
            }
            let colour = self.styles.colour_of.get(&item).copied().or(rep_colour);
            let mut triangle_colours = Vec::new();
            let face_styled = bm
                .face_of_triangle
                .iter()
                .any(|f| self.styles.colour_of.contains_key(f));
            if face_styled {
                triangle_colours = bm
                    .face_of_triangle
                    .iter()
                    .map(|f| self.styles.colour_of.get(f).copied())
                    .collect();
            }
            if !self.opts.keep_invisible {
                let hidden: Vec<bool> = bm
                    .face_of_triangle
                    .iter()
                    .map(|f| self.styles.hidden.contains(f))
                    .collect();
                if hidden.iter().any(|&h| h) {
                    let mut tris = Vec::with_capacity(mesh.triangles.len());
                    let mut cols = Vec::new();
                    for (i, t) in mesh.triangles.iter().enumerate() {
                        if !hidden[i] {
                            tris.push(*t);
                            if !triangle_colours.is_empty() {
                                cols.push(triangle_colours[i]);
                            }
                        }
                    }
                    mesh.triangles = tris;
                    triangle_colours = cols;
                }
            }
            shapes.push(Shape {
                item,
                name: ie
                    .string("name")
                    .filter(|s| !s.is_empty())
                    .map(str::to_string),
                mesh,
                colour,
                triangle_colours,
                layers: self.styles.layers.get(&item).cloned().unwrap_or_default(),
            });
        }
        (shapes, mapped)
    }
}

/// Bounding-box diagonal of every Cartesian point reachable from `id`.
fn bbox_diagonal(step: &StepFile, id: u64) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for r in step.reachable_from(id) {
        let Some(e) = Entity::get(step, r) else {
            continue;
        };
        if !e.is_a("CARTESIAN_POINT") {
            continue;
        }
        if let Some(Ok(p)) = e.list("coordinates").map(crate::geom::coords3) {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    if lo[0] > hi[0] {
        return 0.0;
    }
    crate::geom::dist(lo, hi)
}
