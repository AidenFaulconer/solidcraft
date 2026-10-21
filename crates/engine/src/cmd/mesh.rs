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
        .params("body?: mesh body name (default: the selected mesh body); tolerance?: length expression (default 0.05 mm): reconstruct a closed mesh as editable analytic faces (planes, cylinders, spheres; constant-section prisms as a fallback), with a parametric timeline feature; name?"),
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
    convert_named(s, cmd, &body, tolerance.as_str(), str_(p, "name"))
}

/// Convert one mesh body that already belongs to the active component.
pub(super) fn convert_named(s: &mut Session, cmd: &str, body: &str, tolerance: &str, name: Option<&str>) -> Result<Value> {
    let state = s.model.state();
    let source = state.body(body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
    if !source.body.is_mesh() {
        return Err(bad(cmd, "select a mesh body"));
    }
    if s.doc.model_body_component(source) != s.active_component {
        return Err(bad(cmd, "activate the mesh body's component before converting it"));
    }
    let mut params = json!({"body": body, "tolerance": tolerance});
    if let Some(name) = name.filter(|n| !n.trim().is_empty()) {
        params["name"] = json!(name);
    }
    add_feature(s, &params, FeatureKind::MeshConvert { body: body.into(), tolerance: tolerance.into() })
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
            convert_named(s, cmd, body, tolerance, None)?;
            converted.push(body.clone());
        }
        Ok(converted)
    })();
    s.active_component = previous;
    result
}
