//! Cross-check the transcribed EXPRESS slice (`schema::ENTITIES`)
//! against the staged long-form schemas in the docs repository
//! (`docs/3d/step/schemas/*.exp`, a sibling checkout of this crate in
//! the OxideAV workspace). Skipped when the docs checkout is absent.

use std::collections::HashMap;
use std::path::PathBuf;

use oxideav_step::schema::ENTITIES;

struct Decl {
    supertypes: Vec<String>,
    attrs: Vec<String>,
}

/// Minimal EXPRESS entity scanner: `ENTITY name … SUBTYPE OF (a, b);`
/// then explicit attribute statements up to the first DERIVE / INVERSE
/// / UNIQUE / WHERE / END_ENTITY. Redeclarations (`SELF\x.y : …`) are
/// not new explicit attributes.
fn scan(text: &str) -> HashMap<String, Decl> {
    // Strip (* … *) comments and -- line comments.
    let mut clean = String::with_capacity(text.len());
    let b = text.as_bytes();
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'(' && b.get(i + 1) == Some(&b'*') {
            let mut j = i + 2;
            while j + 1 < b.len() && !(b[j] == b'*' && b[j + 1] == b')') {
                j += 1;
            }
            i = j + 2;
            clean.push(' ');
        } else if b[i] == b'-' && b.get(i + 1) == Some(&b'-') {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else {
            clean.push(b[i] as char);
            i += 1;
        }
    }
    let upper = clean.to_ascii_uppercase();
    let mut out = HashMap::new();
    let mut pos = 0;
    while let Some(k) = upper[pos..].find("ENTITY ") {
        let start = pos + k;
        pos = start + 7;
        // Must be a word boundary (not END_ENTITY).
        if start > 0 && !upper.as_bytes()[start - 1].is_ascii_whitespace() {
            continue;
        }
        let Some(end_rel) = upper[start..].find("END_ENTITY") else {
            break;
        };
        let body = &upper[start + 7..start + end_rel];
        let Some(semi) = body.find(';') else { continue };
        let header = &body[..semi];
        let name = header.split_whitespace().next().unwrap_or("").to_string();
        let supertypes = match header.find("SUBTYPE OF") {
            Some(s) => {
                let rest = &header[s + 10..];
                let open = rest.find('(').unwrap();
                let close = rest.find(')').unwrap();
                rest[open + 1..close]
                    .split(',')
                    .map(|x| x.trim().to_string())
                    .collect()
            }
            None => Vec::new(),
        };
        let mut attrs = Vec::new();
        for stmt in body[semi + 1..].split(';') {
            let s = stmt.trim();
            let first = s.split_whitespace().next().unwrap_or("");
            if matches!(first, "DERIVE" | "INVERSE" | "UNIQUE" | "WHERE") {
                break;
            }
            let Some(colon) = s.find(':') else { continue };
            let names = &s[..colon];
            if names.trim_start().starts_with("SELF\\") {
                continue;
            }
            for n in names.split(',') {
                let n = n.trim();
                if !n.is_empty() {
                    attrs.push(n.to_ascii_lowercase());
                }
            }
        }
        out.insert(name, Decl { supertypes, attrs });
        pos = start + end_rel;
    }
    out
}

fn schema(file: &str) -> Option<HashMap<String, Decl>> {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/3d/step/schemas")
        .join(file);
    let text = std::fs::read(&p).ok()?;
    Some(scan(&String::from_utf8_lossy(&text)))
}

fn check(file: &str, require_all: bool) {
    let Some(decls) = schema(file) else {
        eprintln!("skipping: {file} not staged");
        return;
    };
    let mut missing = Vec::new();
    let mut wrong = Vec::new();
    for d in ENTITIES {
        let Some(x) = decls.get(d.name) else {
            missing.push(d.name);
            continue;
        };
        let attrs: Vec<&str> = x.attrs.iter().map(String::as_str).collect();
        let sups: Vec<&str> = x.supertypes.iter().map(String::as_str).collect();
        if attrs != d.attrs || sups != d.supertypes {
            wrong.push(format!(
                "{}: schema {sups:?} {attrs:?} vs table {:?} {:?}",
                d.name, d.supertypes, d.attrs
            ));
        }
    }
    assert!(wrong.is_empty(), "{file}:\n{}", wrong.join("\n"));
    if require_all {
        assert!(missing.is_empty(), "{file}: not declared: {missing:?}");
    }
}

#[test]
fn ap242_declarations_match() {
    check("ap242-ed4-mim_lf.exp", true);
}

#[test]
fn ap214_declarations_match_where_declared() {
    check("ap214-ed3-AP214E3_2010.exp", false);
}

#[test]
fn ap203e2_declarations_match_where_declared() {
    check("ap203-ed2-part403ts_wg3n2635mim_lf.exp", false);
}

#[test]
fn ap203_declarations_match_where_declared() {
    check("ap203-ed1-config_control_design-aim_lf.exp", false);
}
