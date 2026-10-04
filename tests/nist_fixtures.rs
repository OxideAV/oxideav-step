//! Read the public-domain NIST MBE PMI test models staged in the docs
//! repository (`docs/3d/step/fixtures/`, a sibling checkout in the
//! OxideAV workspace). Skipped when absent.

use std::collections::HashMap;
use std::path::PathBuf;

use oxideav_step::{read_step, StepModel, TriMesh};

fn fixture(name: &str) -> Option<Vec<u8>> {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/3d/step/fixtures")
        .join(name);
    std::fs::read(p).ok()
}

/// Boundary edges (used by exactly one triangle in one direction)
/// after welding coincident positions.
fn open_edges(m: &TriMesh) -> usize {
    let mut weld: HashMap<[i64; 3], u32> = HashMap::new();
    let ids: Vec<u32> = m
        .positions
        .iter()
        .map(|p| {
            let k = [
                (p[0] * 1e6).round() as i64,
                (p[1] * 1e6).round() as i64,
                (p[2] * 1e6).round() as i64,
            ];
            let n = weld.len() as u32;
            *weld.entry(k).or_insert(n)
        })
        .collect();
    let mut count: HashMap<(u32, u32), i32> = HashMap::new();
    for t in &m.triangles {
        for i in 0..3 {
            let (a, b) = (ids[t[i] as usize], ids[t[(i + 1) % 3] as usize]);
            if a == b {
                continue;
            }
            *count.entry((a, b)).or_default() += 1;
            *count.entry((b, a)).or_default() -= 1;
        }
    }
    count.values().filter(|&&c| c != 0).count() / 2
}

fn summary(name: &str, m: &StepModel) {
    let tris = m.triangle_count();
    let mut open = 0;
    let mut vol = 0.0;
    for p in &m.parts {
        for s in &p.shapes {
            open += open_edges(&s.mesh);
            vol += s.mesh.signed_volume();
        }
    }
    eprintln!(
        "{name}: {} {} parts, {} roots, {tris} triangles, {open} open edges, volume {vol:.1}, {} warnings",
        m.schema.label(),
        m.parts.len(),
        m.roots.len(),
        m.warnings.len()
    );
    for w in m.warnings.iter().take(10) {
        eprintln!("  warning: {w}");
    }
}

#[test]
fn nist_models_read() {
    for name in [
        "nist_ftc_11_asme1_rb.stp",
        "nist_ctc_01_asme1_rd.stp",
        "nist_ctc_01_asme1_ap203.stp",
        "nist_ctc_01_asme1_ap242-e1.stp",
        "nist_ctc_03_asme1_ap242-e2.stp",
        "nist_stc_06_asme1_ap242-e3.stp",
        "nist_ftc_08_asme1_ap242-e1-tg.stp",
    ] {
        let Some(bytes) = fixture(name) else {
            eprintln!("skipping {name}: not staged");
            continue;
        };
        let t = std::time::Instant::now();
        let m = read_step(&bytes).unwrap_or_else(|e| panic!("{name}: {e}"));
        summary(name, &m);
        eprintln!("  {:?}", t.elapsed());
        assert!(m.triangle_count() > 0, "{name}");
    }
}
