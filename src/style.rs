//! Presentation (ISO 10303-46; CAx-IF "Model Styling and Organization"
//! recommended practice): surface colours, transparency, invisibility
//! and layers attached to representation items.
//!
//! The colour chain is
//! `styled_item(name, styles, item)` → `presentation_style_assignment
//! (styles)` (or, in later AP242 editions, the style directly) →
//! `surface_style_usage(side, style)` → `surface_side_style(name,
//! styles)` → `surface_style_fill_area(fill_area)` → `fill_area_style
//! (name, fill_styles)` → `fill_area_style_colour(name, fill_colour)` →
//! `colour_rgb(name, r, g, b)` / `draughting_pre_defined_colour(name)`;
//! a `surface_style_rendering(_with_properties)` member can carry the
//! colour too, and its `surface_style_transparent(transparency)`
//! property the alpha (`1 − transparency`).

use std::collections::{HashMap, HashSet};

use oxideav_ifc::{StepFile, Value};

use crate::schema::{number, Entity};

/// An RGBA colour, components in `[0, 1]`.
pub type Rgba = [f32; 4];

/// The RGB of a `draughting_pre_defined_colour` name (CAx-IF table).
pub fn predefined_colour(name: &str) -> Option<[f32; 3]> {
    Some(match name.trim().to_ascii_lowercase().as_str() {
        "black" => [0.0, 0.0, 0.0],
        "white" => [1.0, 1.0, 1.0],
        "red" => [1.0, 0.0, 0.0],
        "green" => [0.0, 1.0, 0.0],
        "blue" => [0.0, 0.0, 1.0],
        "yellow" => [1.0, 1.0, 0.0],
        "cyan" => [0.0, 1.0, 1.0],
        "magenta" => [1.0, 0.0, 1.0],
        _ => return None,
    })
}

/// Styling resolved for a whole file.
#[derive(Debug, Clone, Default)]
pub struct Styles {
    /// Item id (solid, shell, face, mapped item, representation) →
    /// surface colour.
    pub colour_of: HashMap<u64, Rgba>,
    /// Items that are invisible (`invisibility` over their styled item
    /// or their layer).
    pub hidden: HashSet<u64>,
    /// Item id → names of the presentation layers it is assigned to.
    pub layers: HashMap<u64, Vec<String>>,
    /// Context-dependent over-riding colours: `(item, style context,
    /// colour)` — the colour applies to the item only in the assembly
    /// occurrence the context (a chain of representation relationships
    /// / mapped items) identifies.
    pub contextual: Vec<(u64, Vec<u64>, Rgba)>,
}

const MAX_STYLE_DEPTH: usize = 12;

impl Styles {
    /// Collect every `styled_item`, `invisibility` and
    /// `presentation_layer_assignment` of the file.
    pub fn collect(step: &StepFile) -> Self {
        let mut out = Self::default();
        let mut invisible_styled: HashSet<u64> = HashSet::new();
        let mut invisible_layers: HashSet<u64> = HashSet::new();
        for inst in step.instances.values() {
            let e = Entity { inst };
            if e.is_a("INVISIBILITY") {
                for v in e.list("invisible_items").unwrap_or(&[]) {
                    if let Some(id) = v.as_reference() {
                        invisible_styled.insert(id);
                        invisible_layers.insert(id);
                    }
                }
            }
        }
        // Context-dependent over-riding styles apply to one assembly
        // occurrence only; they are not mapped onto shared geometry.
        let mut overriding: Vec<(u64, Rgba)> = Vec::new();
        for inst in step.instances.values() {
            let e = Entity { inst };
            if e.is_a("PRESENTATION_LAYER_ASSIGNMENT") {
                let name = e.string("name").unwrap_or("").to_string();
                let hide = invisible_layers.contains(&inst.id);
                for v in e.list("assigned_items").unwrap_or(&[]) {
                    if let Some(id) = v.as_reference() {
                        out.layers.entry(id).or_default().push(name.clone());
                        if hide {
                            out.hidden.insert(id);
                        }
                    }
                }
                continue;
            }
            if e.is_a("CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM") {
                let (Some(item), Some(rgba)) = (e.reference("item"), styled_colour(step, e)) else {
                    continue;
                };
                let context: Vec<u64> = e
                    .list("style_context")
                    .unwrap_or(&[])
                    .iter()
                    .filter_map(Value::as_reference)
                    .collect();
                if !context.is_empty() {
                    out.contextual.push((item, context, rgba));
                }
                continue;
            }
            if !e.is_a("STYLED_ITEM") {
                continue;
            }
            let Some(item) = e.reference("item") else {
                continue;
            };
            if invisible_styled.contains(&inst.id) {
                out.hidden.insert(item);
            }
            let Some(rgba) = styled_colour(step, e) else {
                continue;
            };
            if e.is_a("OVER_RIDING_STYLED_ITEM") {
                overriding.push((item, rgba));
            } else {
                out.colour_of.entry(item).or_insert(rgba);
            }
        }
        for (item, rgba) in overriding {
            out.colour_of.insert(item, rgba);
        }
        out
    }
}

/// The surface colour carried by one styled item.
fn styled_colour(step: &StepFile, e: Entity<'_>) -> Option<Rgba> {
    let mut found: Option<[f32; 3]> = None;
    let mut alpha: f32 = 1.0;
    for s in e.list("styles").unwrap_or(&[]) {
        walk(step, s, 0, &mut found, &mut alpha);
    }
    found.map(|c| [c[0], c[1], c[2], alpha])
}

/// Depth-first over the style selects, picking up the first surface
/// colour and any transparency.
fn walk(step: &StepFile, v: &Value, depth: usize, found: &mut Option<[f32; 3]>, alpha: &mut f32) {
    if depth > MAX_STYLE_DEPTH {
        return;
    }
    let Some(e) = v.as_reference().and_then(|id| Entity::get(step, id)) else {
        if let Some(items) = v.as_list() {
            for it in items {
                walk(step, it, depth + 1, found, alpha);
            }
        }
        return;
    };
    let next = |name: &str| e.attr(name);
    if e.is_a("PRESENTATION_STYLE_ASSIGNMENT") || e.is_a("SURFACE_SIDE_STYLE") {
        for s in e.list("styles").unwrap_or(&[]) {
            walk(step, s, depth + 1, found, alpha);
        }
    } else if e.is_a("SURFACE_STYLE_USAGE") {
        if let Some(s) = next("style") {
            walk(step, s, depth + 1, found, alpha);
        }
    } else if e.is_a("SURFACE_STYLE_FILL_AREA") {
        if let Some(s) = next("fill_area") {
            walk(step, s, depth + 1, found, alpha);
        }
    } else if e.is_a("FILL_AREA_STYLE") {
        for s in e.list("fill_styles").unwrap_or(&[]) {
            walk(step, s, depth + 1, found, alpha);
        }
    } else if e.is_a("FILL_AREA_STYLE_COLOUR") {
        if found.is_none() {
            *found = next("fill_colour").and_then(|c| colour(step, c, depth + 1));
        }
    } else if e.is_a("SURFACE_STYLE_RENDERING") {
        if found.is_none() {
            *found = next("surface_colour").and_then(|c| colour(step, c, depth + 1));
        }
        for p in e.list("properties").unwrap_or(&[]) {
            if let Some(pe) = p.as_reference().and_then(|id| Entity::get(step, id)) {
                if pe.is_a("SURFACE_STYLE_TRANSPARENT") {
                    if let Some(t) = pe.number("transparency") {
                        *alpha = (1.0 - t as f32).clamp(0.0, 1.0);
                    }
                }
            }
        }
    }
}

/// Resolve a colour select.
fn colour(step: &StepFile, v: &Value, depth: usize) -> Option<[f32; 3]> {
    if depth > MAX_STYLE_DEPTH {
        return None;
    }
    let e = Entity::get(step, v.as_reference()?)?;
    if e.is_a("COLOUR_RGB") {
        let c = |n: &str| e.attr(n).and_then(number).map(|x| x.clamp(0.0, 1.0) as f32);
        return Some([c("red")?, c("green")?, c("blue")?]);
    }
    if e.is_a("DRAUGHTING_PRE_DEFINED_COLOUR") || e.is_a("PRE_DEFINED_COLOUR") {
        return predefined_colour(e.string("name")?);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_ifc::parse_step;

    #[test]
    fn colour_chains() {
        let text = "ISO-10303-21;HEADER;FILE_DESCRIPTION((''),'2;1');\
             FILE_NAME('','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));ENDSEC;DATA;\
             #1=COLOUR_RGB('',0.5,0.25,1.);\
             #2=FILL_AREA_STYLE_COLOUR('',#1);#3=FILL_AREA_STYLE('',(#2));\
             #4=SURFACE_STYLE_FILL_AREA(#3);#5=SURFACE_SIDE_STYLE('',(#4));\
             #6=SURFACE_STYLE_USAGE(.BOTH.,#5);#7=PRESENTATION_STYLE_ASSIGNMENT((#6));\
             #8=STYLED_ITEM('',(#7),#100);\
             #9=DRAUGHTING_PRE_DEFINED_COLOUR('red');\
             #10=SURFACE_STYLE_TRANSPARENT(0.25);\
             #11=SURFACE_STYLE_RENDERING_WITH_PROPERTIES(.NORMAL_SHADING.,#9,(#10));\
             #12=SURFACE_SIDE_STYLE('',(#11));#13=SURFACE_STYLE_USAGE(.BOTH.,#12);\
             #14=STYLED_ITEM('',(#13),#101);\
             #15=INVISIBILITY((#14));\
             #16=PRESENTATION_LAYER_ASSIGNMENT('L1','',(#100));\
             ENDSEC;END-ISO-10303-21;";
        let f = parse_step(text.as_bytes()).unwrap();
        let s = Styles::collect(&f);
        assert_eq!(s.colour_of[&100], [0.5, 0.25, 1.0, 1.0]);
        assert_eq!(s.colour_of[&101], [1.0, 0.0, 0.0, 0.75]);
        assert!(s.hidden.contains(&101));
        assert!(!s.hidden.contains(&100));
        assert_eq!(s.layers[&100], ["L1"]);
    }
}
