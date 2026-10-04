//! The `registry` feature: decode through `Mesh3DDecoder` /
//! `Mesh3DRegistry` into a `Scene3D`.
#![cfg(feature = "registry")]

use std::path::PathBuf;

use oxideav_mesh3d::{Axis, Mesh3DRegistry, Transform, Unit};

fn bytes(name: &str) -> Vec<u8> {
    std::fs::read(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(name),
    )
    .unwrap()
}

#[test]
fn registry_decodes_an_assembly_with_instancing() {
    let mut reg = Mesh3DRegistry::new();
    oxideav_step::register_mesh3d(&mut reg);
    for ext in ["step", "STP", "p21"] {
        assert!(reg.decoder_for_extension(ext).is_some(), "{ext}");
    }
    let mut dec = reg.decoder_for_format("step").unwrap();
    let scene = dec.decode(&bytes("assembly.stp")).unwrap();
    assert_eq!(scene.unit, Unit::Millimetres);
    assert_eq!(scene.up_axis, Axis::PosZ);
    assert_eq!(scene.roots.len(), 1);
    let root = scene.node(scene.roots[0]).unwrap();
    assert_eq!(root.name.as_deref(), Some("assembly"));
    assert_eq!(root.children.len(), 3);
    // The two cube occurrences share one mesh.
    let meshes: Vec<_> = root
        .children
        .iter()
        .filter_map(|&c| scene.node(c).unwrap().mesh)
        .collect();
    assert_eq!(meshes.len(), 3);
    let shared = meshes
        .iter()
        .map(|m| meshes.iter().filter(|&n| n == m).count())
        .max();
    assert_eq!(shared, Some(2));
    // The rotated occurrence carries a matrix.
    assert!(root
        .children
        .iter()
        .any(|&c| matches!(scene.node(c).unwrap().transform, Transform::Matrix(_))));
    assert!(scene.validate().is_ok(), "{:?}", scene.validate());
    // World volume: two 1000 mm³ cubes + a 1 in³ block.
    let v = scene.world_volume();
    let exact = 2000.0 + 25.4f64.powi(3);
    assert!((v - exact).abs() / exact < 1e-4, "{v}");
}

#[test]
fn materials_from_styles() {
    let mut dec = oxideav_step::make_decoder();
    let scene =
        oxideav_mesh3d::Mesh3DDecoder::decode(&mut dec, &bytes("plate_with_hole.stp")).unwrap();
    // Grey body, blue top, yellow hole → three primitives / materials.
    assert_eq!(scene.materials.len(), 3);
    assert_eq!(scene.meshes.len(), 1);
    assert_eq!(scene.meshes[0].primitives.len(), 3);
}

#[test]
fn non_step_input_is_rejected() {
    let mut dec = oxideav_step::make_decoder();
    assert!(oxideav_mesh3d::Mesh3DDecoder::decode(&mut dec, b"solid x\nendsolid x\n").is_err());
}

#[test]
fn face_split_normals() {
    let mut dec = oxideav_step::make_decoder();
    let scene = oxideav_mesh3d::Mesh3DDecoder::decode(&mut dec, &bytes("cube.stp")).unwrap();
    let prim = &scene.meshes[0].primitives[0];
    // 6 faces × 4 corners, each with its face's axis-aligned normal.
    assert_eq!(prim.positions.len(), 24);
    let normals = prim.normals.as_ref().unwrap();
    for n in normals {
        let axis = n.iter().filter(|c| (c.abs() - 1.0).abs() < 1e-6).count();
        assert_eq!(axis, 1, "{n:?}");
    }
}

#[test]
fn ap242_tessellated_round_trip() {
    let mut reg = Mesh3DRegistry::new();
    oxideav_step::register_mesh3d(&mut reg);
    for name in ["assembly.stp", "plate_with_hole.stp", "revolved.stp"] {
        let mut dec = reg.decoder_for_format("step").unwrap();
        let scene = dec.decode(&bytes(name)).unwrap();
        let mut enc = reg.encoder_for_extension("stp").unwrap();
        let out = enc.encode(&scene).unwrap();
        let again = reg
            .decoder_for_format("step")
            .unwrap()
            .decode(&out)
            .unwrap();
        assert_eq!(again.unit, scene.unit, "{name}");
        assert_eq!(again.triangle_count(), scene.triangle_count(), "{name}");
        let (a, b) = (scene.world_volume(), again.world_volume());
        assert!((a - b).abs() <= 1e-5 * a.abs(), "{name}: {a} vs {b}");
        assert_eq!(again.materials.len(), scene.materials.len(), "{name}");
        // Instancing survives: as many meshes as before.
        assert_eq!(again.meshes.len(), scene.meshes.len(), "{name}");
        let model = oxideav_step::read_step(&out).unwrap();
        assert_eq!(model.schema, oxideav_step::ApSchema::Ap242);
        assert!(model.warnings.is_empty(), "{name}: {:?}", model.warnings);
    }
}

#[test]
fn context_dependent_colour_overrides_one_occurrence() {
    let text = String::from_utf8(bytes("assembly.stp")).unwrap();
    let id_of = |pred: &dyn Fn(&str) -> bool, nth: usize| -> String {
        let line = text.lines().filter(|l| pred(l)).nth(nth).unwrap();
        line.split('=').next().unwrap().to_string()
    };
    let solid = id_of(&|l| l.contains("=MANIFOLD_SOLID_BREP('cube'"), 0);
    let base = id_of(&|l| l.contains("=STYLED_ITEM("), 0);
    let rel2 = id_of(
        &|l| l.contains("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"),
        1,
    );
    let extra = format!(
        "#9001=COLOUR_RGB('',0.,0.,1.);\n#9002=FILL_AREA_STYLE_COLOUR('',#9001);\n\
         #9003=FILL_AREA_STYLE('',(#9002));\n#9004=SURFACE_STYLE_FILL_AREA(#9003);\n\
         #9005=SURFACE_SIDE_STYLE('',(#9004));\n#9006=SURFACE_STYLE_USAGE(.BOTH.,#9005);\n\
         #9007=PRESENTATION_STYLE_ASSIGNMENT((#9006));\n\
         #9008=CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM('',(#9007),{solid},{base},({rel2}));\nENDSEC;"
    );
    let text = text.replacen("ENDSEC;\nEND-ISO", &format!("{extra}\nEND-ISO"), 1);
    let model = oxideav_step::read_step(text.as_bytes()).unwrap();
    assert_eq!(model.occurrence_colours.len(), 1);
    let scene = oxideav_step::scene_from_model(&model);
    // Base cube mesh (green), a blue variant for the second occurrence,
    // and the inch block.
    assert_eq!(scene.meshes.len(), 3);
    let root = scene.node(scene.roots[0]).unwrap();
    let colours: Vec<[f32; 4]> = root
        .children
        .iter()
        .filter_map(|&c| scene.node(c).unwrap().mesh)
        .filter_map(|m| scene.meshes[m.0 as usize].primitives[0].material)
        .map(|m| scene.materials[m.0 as usize].base_color)
        .collect();
    assert!(colours.contains(&[0.2, 0.7, 0.2, 1.0]), "{colours:?}");
    assert!(colours.contains(&[0.0, 0.0, 1.0, 1.0]), "{colours:?}");
}
