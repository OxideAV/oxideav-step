//! Units of a representation context (ISO 10303-41 measure schema).
//!
//! A shape representation's `context_of_items` is (usually as a complex
//! instance) a `GLOBAL_UNIT_ASSIGNED_CONTEXT` whose `units` set holds the
//! length, plane-angle and solid-angle units every measure in that
//! representation is expressed in:
//!
//! * `( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )` — an SI
//!   unit with an optional prefix;
//! * `( CONVERSION_BASED_UNIT('INCH',#m) LENGTH_UNIT() NAMED_UNIT(#d) )`
//!   with `#m = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#mm)` — a
//!   named multiple of another unit (resolved recursively);
//! * the plane-angle analogues (`.RADIAN.`, or `DEGREE` as a
//!   conversion-based unit over radians).

use oxideav_ifc::{StepFile, Value};

use crate::schema::{number, Entity};

/// Resolved units of one representation context.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContextUnits {
    /// Metres per model length unit (`0.001` for millimetres).
    pub length_metres: f64,
    /// Radians per model plane-angle unit (`π/180` for degrees).
    pub angle_radians: f64,
    /// The context's global length uncertainty, in model units, if
    /// declared (`GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT`).
    pub uncertainty: Option<f64>,
}

impl Default for ContextUnits {
    /// Millimetres and radians — the overwhelmingly common STEP default
    /// when a context names no units.
    fn default() -> Self {
        Self {
            length_metres: 0.001,
            angle_radians: 1.0,
            uncertainty: None,
        }
    }
}

/// The SI prefix multiplier (ISO 10303-41 `si_prefix`).
pub fn si_prefix(prefix: &str) -> Option<f64> {
    Some(match prefix {
        "EXA" => 1e18,
        "PETA" => 1e15,
        "TERA" => 1e12,
        "GIGA" => 1e9,
        "MEGA" => 1e6,
        "KILO" => 1e3,
        "HECTO" => 1e2,
        "DECA" => 1e1,
        "DECI" => 1e-1,
        "CENTI" => 1e-2,
        "MILLI" => 1e-3,
        "MICRO" => 1e-6,
        "NANO" => 1e-9,
        "PICO" => 1e-12,
        "FEMTO" => 1e-15,
        "ATTO" => 1e-18,
        _ => return None,
    })
}

/// What a unit measures.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnitKind {
    /// Length (base: metre).
    Length,
    /// Plane angle (base: radian).
    PlaneAngle,
}

/// The size of unit instance `id` in base SI units (metres / radians),
/// with its kind. Conversion-based units recurse through their
/// `conversion_factor` (bounded depth).
pub fn unit_value(step: &StepFile, id: u64) -> Option<(UnitKind, f64)> {
    unit_value_depth(step, id, 0)
}

fn unit_value_depth(step: &StepFile, id: u64, depth: usize) -> Option<(UnitKind, f64)> {
    if depth > 16 {
        return None;
    }
    let e = Entity::get(step, id)?;
    let kind = if e.is_a("LENGTH_UNIT") {
        UnitKind::Length
    } else if e.is_a("PLANE_ANGLE_UNIT") {
        UnitKind::PlaneAngle
    } else {
        // A bare SI unit still names its quantity.
        match e.attr_of("SI_UNIT", "name").and_then(Value::as_enum) {
            Some("METRE") => UnitKind::Length,
            Some("RADIAN") => UnitKind::PlaneAngle,
            _ => return None,
        }
    };
    if e.is_a("SI_UNIT") {
        let base = match e.attr_of("SI_UNIT", "name").and_then(Value::as_enum)? {
            "METRE" | "RADIAN" => 1.0,
            _ => return None,
        };
        let prefix = match e.attr_of("SI_UNIT", "prefix") {
            Some(Value::Enum(p)) => si_prefix(p)?,
            _ => 1.0,
        };
        return Some((kind, base * prefix));
    }
    if e.is_a("CONVERSION_BASED_UNIT") {
        let m = e.attr_of("CONVERSION_BASED_UNIT", "conversion_factor")?;
        let me = Entity::get(step, m.as_reference()?)?;
        let factor = me.attr("value_component").and_then(number)?;
        let (_, base) = unit_value_depth(step, me.reference("unit_component")?, depth + 1)?;
        let v = factor * base;
        return (v.is_finite() && v > 0.0).then_some((kind, v));
    }
    None
}

/// Resolve the units of representation context `ctx_id` (defaults for
/// anything not declared).
pub fn context_units(step: &StepFile, ctx_id: u64) -> ContextUnits {
    let mut out = ContextUnits::default();
    let Some(ctx) = Entity::get(step, ctx_id) else {
        return out;
    };
    if let Some(units) = ctx.list("units") {
        for u in units {
            match u.as_reference().and_then(|id| unit_value(step, id)) {
                Some((UnitKind::Length, v)) if v > 0.0 => out.length_metres = v,
                Some((UnitKind::PlaneAngle, v)) if v > 0.0 => out.angle_radians = v,
                _ => {}
            }
        }
    }
    if let Some(unc) = ctx.list("uncertainty") {
        for u in unc {
            let Some(ue) = u.as_reference().and_then(|id| Entity::get(step, id)) else {
                continue;
            };
            let Some(v) = ue.attr("value_component").and_then(number) else {
                continue;
            };
            let unit = ue
                .reference("unit_component")
                .and_then(|id| unit_value(step, id));
            if let Some((UnitKind::Length, m)) = unit {
                // Express in the context's length unit.
                let model = v * m / out.length_metres;
                if model.is_finite() && model > 0.0 {
                    out.uncertainty = Some(model);
                }
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_ifc::parse_step;

    fn file(data: &str) -> StepFile {
        let text = format!(
            "ISO-10303-21;HEADER;FILE_DESCRIPTION((''),'2;1');\
             FILE_NAME('','',(''),(''),'','','');FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));ENDSEC;\
             DATA;{data}ENDSEC;END-ISO-10303-21;"
        );
        parse_step(text.as_bytes()).unwrap()
    }

    #[test]
    fn si_and_conversion_based_units() {
        let f = file(
            "#1=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\
             #2=( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\
             #3=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\
             #4=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#1);\
             #5=( CONVERSION_BASED_UNIT('INCH',#4) LENGTH_UNIT() NAMED_UNIT(#3) );\
             #6=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925),#2);\
             #7=( CONVERSION_BASED_UNIT('DEGREE',#6) NAMED_UNIT(#3) PLANE_ANGLE_UNIT() );\
             #8=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#5,'distance_accuracy_value','');\
             #9=( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#8)) \
                  GLOBAL_UNIT_ASSIGNED_CONTEXT((#5,#7)) REPRESENTATION_CONTEXT('','') );",
        );
        assert_eq!(unit_value(&f, 1), Some((UnitKind::Length, 0.001)));
        assert_eq!(unit_value(&f, 2), Some((UnitKind::PlaneAngle, 1.0)));
        let (k, inch) = unit_value(&f, 5).unwrap();
        assert_eq!(k, UnitKind::Length);
        assert!((inch - 0.0254).abs() < 1e-12);
        let u = context_units(&f, 9);
        assert!((u.length_metres - 0.0254).abs() < 1e-12);
        assert!((u.angle_radians - core::f64::consts::PI / 180.0).abs() < 1e-9);
        assert!((u.uncertainty.unwrap() - 1e-5).abs() < 1e-15);
        // Missing context → defaults.
        assert_eq!(context_units(&f, 99), ContextUnits::default());
    }

    #[test]
    fn self_referential_conversion_is_bounded() {
        let f = file(
            "#1=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.),#2);\
             #2=( CONVERSION_BASED_UNIT('LOOP',#1) LENGTH_UNIT() NAMED_UNIT(*) );",
        );
        assert_eq!(unit_value(&f, 2), None);
    }
}
