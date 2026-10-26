//! Parametric reconstruction of imported mechanical mesh bodies.

use serde_json::{Value, json};
use solidcraft_doc::FeatureKind;
use solidcraft_doc::expr::Kind;

use super::{
    CommandSpec,
    features::{add_feature, check_expr},
};
use crate::{
    Result, Sel, Session,
    params::{bad, expr, str_},
};

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("mesh.convert", "Convert Mesh", convert)
        .icon("import")
        .params("body?: mesh body name (default: the selected mesh body); tolerance?: length expression (default 0.05 mm); mesh?: percent 1..100 (default 100, share of triangles used to start a fit); cylinders?, cones?, tori?, spheres?, threads?, freeform?: bool (default true). Planes are always fitted. Freeform keeps leftover triangles as facets. Rebuilds a closed mesh as editable faces, with a parametric timeline feature; name?"),
];

/// The mesh body to convert: `body`, else the one selected body.
fn target(s: &Session, p: &Value, cmd: &str) -> Result<String> {
    if let Some(name) = str_(p, "body").filter(|b| !b.trim().is_empty()) {
        return Ok(name.to_owned());
    }
    let selected: Vec<&str> = s.selection.iter().filter_map(|x| if let Sel::Body { name } = x { Some(name.as_str()) } else { None }).collect();
    match selected.as_slice() {
        [name] => Ok((*name).to_owned()),
        [] => Err(bad(cmd, "select a mesh body")),
        _ => Err(bad(cmd, "select one mesh body")),
    }
}

fn convert(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "mesh.convert";
    let body = target(s, p, cmd)?;
    let tolerance = expr(p, "tolerance").unwrap_or_else(|| "0.05 mm".into());
    check_expr(s, &tolerance, Kind::Length, cmd, "tolerance")?;
    let mesh = crate::params::num(p, "mesh").unwrap_or(100.0);
    if !(1.0..=100.0).contains(&mesh) {
        return Err(bad(cmd, "mesh must be a percentage from 1 to 100"));
    }
    let flag = |key| crate::params::bool_(p, key).unwrap_or(true);
    convert_named(
        s,
        cmd,
        &body,
        tolerance.as_str(),
        str_(p, "name"),
        mesh,
        flag("cylinders"),
        flag("cones"),
        flag("tori"),
        flag("spheres"),
        flag("threads"),
        flag("freeform"),
    )
}

/// Convert one mesh body that already belongs to the active component.
pub(super) fn convert_named(
    s: &mut Session,
    cmd: &str,
    body: &str,
    tolerance: &str,
    name: Option<&str>,
    mesh: f64,
    cylinders: bool,
    cones: bool,
    tori: bool,
    spheres: bool,
    threads: bool,
    freeform: bool,
) -> Result<Value> {
    let state = s.model.state();
    let source = state.body(body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
    if !source.body.is_mesh() {
        return Err(bad(cmd, "select a mesh body"));
    }
    if s.doc.model_body_component(source) != s.active_component {
        return Err(bad(cmd, "activate the mesh body's component before converting it"));
    }
    let mut params = json!({"body": body, "tolerance": tolerance, "mesh": mesh, "cylinders": cylinders, "cones": cones, "tori": tori, "spheres": spheres, "threads": threads, "freeform": freeform});
    if let Some(name) = name.filter(|n| !n.trim().is_empty()) {
        params["name"] = json!(name);
    }
    add_feature(
        s,
        &params,
        FeatureKind::MeshConvert {
            body: body.into(),
            tolerance: tolerance.into(),
            mesh,
            cylinders,
            cones,
            tori,
            spheres,
            threads,
            freeform,
        },
    )
}

/// Convert every named mesh body: activate each body's component, add a MeshConvert feature, then
/// restore the caller's active component. Used by mesh import-as-solid.
pub(super) fn convert_bodies(s: &mut Session, cmd: &str, bodies: &[String], tolerance: &str) -> Result<Vec<String>> {
    check_expr(s, tolerance, Kind::Length, cmd, "tolerance")?;
    let previous = s.active_component;
    let mut converted = Vec::with_capacity(bodies.len());
    let result = (|| {
        for body in bodies {
            let state = s.model.state();
            let source = state.body(body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
            if !source.body.is_mesh() {
                return Err(bad(cmd, format!("body `{body}` is not a mesh")));
            }
            s.active_component = s.doc.model_body_component(source);
            convert_named(s, cmd, body, tolerance, None, 100.0, true, true, true, true, true, true)?;
            converted.push(body.clone());
        }
        Ok(converted)
    })();
    s.active_component = previous;
    result
}
