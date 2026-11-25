//! Parametric reconstruction of imported mechanical mesh bodies.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use serde_json::{Value, json};
use solidcraft_doc::expr::Kind;
use solidcraft_doc::{Expr, FeatureKind, MeshData, MeshSolid};
use solidcraft_geom::Vec3;

use super::{
    CommandSpec,
    features::{add_feature, check_expr},
};
use crate::{
    Result, Sel, Session,
    params::{bad, bool_, expr, str_, string_list},
};

/// How many STL files one folder conversion will write.
const MAX_BATCH: usize = 200;
/// Matches the kernel conversion budget, so a huge file is refused before its vertices are built.
const MAX_BATCH_TRIANGLES: usize = 100_000;
const MAX_STL_BYTES: usize = 128 << 20;
const MAX_FOLDER_DEPTH: usize = 8;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("mesh.convert", "Convert Mesh", convert)
        .icon("import")
        .params("body?: mesh body name (default: the selected mesh body); tolerance?: length expression (default 0.05 mm); angle?: angle expression (default 15 deg: how far a triangle normal may turn while a planar face grows; 2 deg keeps fine faces such as louvers apart); mesh?: percent 1..100 (default 100, share of triangles used to start a fit); cylinders?, cones?, tori?, spheres?, threads?, freeform?: bool (default true). Planes are always fitted. Freeform keeps leftover triangles as facets. Rebuilds a closed mesh as editable faces, with a parametric timeline feature; name?"),
    CommandSpec::new("mesh.convert_files", "Convert STL", convert_files)
        .icon("import")
        .noundo()
        .params("path?: an .stl file or a folder of them; paths?: more files or folders; output?: a folder, or one .step path when a single STL is converted (default: <name>.step beside each STL); recursive?: bool (default false: only the STL files directly in each folder); tolerance?: length expression (default 0.05 mm; a smaller value, such as 0.02 mm, keeps fine detail); angle?: angle expression (default 15 deg; 2 deg for louvers); mesh?: percent 1..100 (default 100); cylinders?, cones?, tori?, spheres?, threads?, freeform?: bool (default true). Writes one STEP file per STL and does not change the open design → written, failed"),
    CommandSpec::new("mesh.import", "Import STL", import_stl)
        .icon("import")
        .params("path?: an .stl file or a folder of them; paths?: more files or folders; recursive?: bool (default false); origin?: [x, y, z] length expressions (default 0, added in the active component); tolerance?, angle?, mesh?, cylinders?, cones?, tori?, spheres?, threads?, freeform?: the same recognition settings as mesh.convert. Embeds the meshes and rebuilds each as a solid on the timeline; name?"),
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

struct Choices {
    tolerance: solidcraft_doc::Expr,
    angle: solidcraft_doc::Expr,
    mesh: f64,
    cylinders: bool,
    cones: bool,
    tori: bool,
    spheres: bool,
    threads: bool,
    freeform: bool,
}

fn choices(s: &Session, p: &Value, cmd: &str) -> Result<Choices> {
    let tolerance = expr(p, "tolerance").unwrap_or_else(|| "0.05 mm".into());
    check_expr(s, &tolerance, Kind::Length, cmd, "tolerance")?;
    let angle = expr(p, "angle").unwrap_or_else(|| "15 deg".into());
    check_expr(s, &angle, Kind::Angle, cmd, "angle")?;
    let mesh = crate::params::num(p, "mesh").unwrap_or(100.0);
    if !(1.0..=100.0).contains(&mesh) {
        return Err(bad(cmd, "mesh must be a percentage from 1 to 100"));
    }
    let flag = |key| bool_(p, key).unwrap_or(true);
    Ok(Choices {
        tolerance,
        angle,
        mesh,
        cylinders: flag("cylinders"),
        cones: flag("cones"),
        tori: flag("tori"),
        spheres: flag("spheres"),
        threads: flag("threads"),
        freeform: flag("freeform"),
    })
}

fn convert(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "mesh.convert";
    let body = target(s, p, cmd)?;
    let choices = choices(s, p, cmd)?;
    convert_named(s, cmd, &body, str_(p, "name"), &choices)
}

/// Convert one mesh body that already belongs to the active component.
fn convert_named(s: &mut Session, cmd: &str, body: &str, name: Option<&str>, choices: &Choices) -> Result<Value> {
    let state = s.model.state();
    let source = state.body(body).ok_or_else(|| bad(cmd, format!("no body `{body}`")))?;
    if !source.body.is_mesh() {
        return Err(bad(cmd, "select a mesh body"));
    }
    if s.doc.model_body_component(source) != s.active_component {
        return Err(bad(cmd, "activate the mesh body's component before converting it"));
    }
    let mut params = json!({"body": body, "tolerance": choices.tolerance.as_str(), "angle": choices.angle.as_str(), "mesh": choices.mesh, "cylinders": choices.cylinders, "cones": choices.cones, "tori": choices.tori, "spheres": choices.spheres, "threads": choices.threads, "freeform": choices.freeform});
    if let Some(name) = name.filter(|n| !n.trim().is_empty()) {
        params["name"] = json!(name);
    }
    add_feature(
        s,
        &params,
        FeatureKind::MeshConvert {
            body: body.into(),
            tolerance: choices.tolerance.clone(),
            angle: choices.angle.clone(),
            mesh: choices.mesh,
            cylinders: choices.cylinders,
            cones: choices.cones,
            tori: choices.tori,
            spheres: choices.spheres,
            threads: choices.threads,
            freeform: choices.freeform,
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
            let choices = Choices {
                tolerance: tolerance.into(),
                angle: "15 deg".into(),
                mesh: 100.0,
                cylinders: true,
                cones: true,
                tori: true,
                spheres: true,
                threads: true,
                freeform: true,
            };
            convert_named(s, cmd, body, None, &choices)?;
            converted.push(body.clone());
        }
        Ok(converted)
    })();
    s.active_component = previous;
    result
}

/// Write one STEP file for every STL in a file or folder. The open design is left as it is.
fn convert_files(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "mesh.convert_files";
    let choices = choices(s, p, cmd)?;
    let tolerance = s.doc.eval(&choices.tolerance, Kind::Length).map_err(|e| bad(cmd, format!("tolerance: {e}")))?;
    let angle = s.doc.eval(&choices.angle, Kind::Angle).map_err(|e| bad(cmd, format!("angle: {e}")))?;
    let options = solidcraft_kernel::ConvertOptions {
        tolerance,
        angle,
        mesh_percent: choices.mesh,
        cylinders: choices.cylinders,
        cones: choices.cones,
        tori: choices.tori,
        spheres: choices.spheres,
        threads: choices.threads,
        freeform: choices.freeform,
    };
    let stls = gather_stls(p, cmd)?;
    let output = str_(p, "output").filter(|text| !text.trim().is_empty());
    let alone = stls.len() == 1;
    let mut used = BTreeSet::new();
    let mut written = Vec::new();
    let mut failed = Vec::new();
    for stl in &stls {
        let dest = step_destination(stl, output, alone, cmd)?;
        let key = dest.to_string_lossy().to_string();
        if !used.insert(key.clone()) {
            return Err(bad(cmd, format!("two STL files would write {key}")));
        }
        match convert_one_stl(stl, &dest, &options) {
            Ok(bytes) => written.push(json!({"stl": stl.display().to_string(), "step": key, "bytes": bytes})),
            Err(error) => failed.push(json!({"stl": stl.display().to_string(), "error": error})),
        }
    }
    if written.is_empty() {
        let why = failed.first().and_then(|item| item.get("error")).and_then(Value::as_str).unwrap_or("no STEP file was written");
        return Err(bad(cmd, why));
    }
    Ok(json!({"written": written, "failed": failed}))
}

/// STL paths from `path` and `paths`, files and folders, capped and de-duplicated.
fn gather_stls(p: &Value, cmd: &str) -> Result<Vec<PathBuf>> {
    let recursive = bool_(p, "recursive").unwrap_or(false);
    let mut raw = string_list(p, "paths");
    if let Some(one) = str_(p, "path").filter(|text| !text.trim().is_empty()) {
        raw.push(one.to_string());
    }
    if raw.is_empty() {
        return Err(bad(cmd, "choose an STL file or a folder"));
    }
    if raw.len() > MAX_BATCH {
        return Err(bad(cmd, format!("at most {MAX_BATCH} paths")));
    }
    let mut stls = Vec::new();
    for item in &raw {
        if item.len() >= 4096 {
            return Err(bad(cmd, "path is too long"));
        }
        collect_stls(Path::new(item), recursive, &mut stls, 0, cmd)?;
    }
    stls.sort();
    stls.dedup();
    if stls.is_empty() {
        return Err(bad(cmd, "no STL files"));
    }
    if stls.len() > MAX_BATCH {
        return Err(bad(cmd, format!("at most {MAX_BATCH} STL files")));
    }
    Ok(stls)
}

/// Read the STL files into the open design as one timeline feature of solids.
fn import_stl(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "mesh.import";
    let choices = choices(s, p, cmd)?;
    let stls = gather_stls(p, cmd)?;
    let origin = origin_exprs(s, p, cmd)?;
    let mut meshes = Vec::with_capacity(stls.len());
    let mut files = Vec::with_capacity(stls.len());
    for stl in &stls {
        meshes.push(stl_mesh(stl, cmd)?);
        files.push(stl.display().to_string());
    }
    let file = files.first().cloned().unwrap_or_else(|| "STL".into());
    let names: Vec<String> = meshes.iter().map(|m| m.name.clone()).collect();
    let mut params = json!({
        "tolerance": choices.tolerance.as_str(),
        "angle": choices.angle.as_str(),
        "mesh": choices.mesh,
        "body_names": names,
    });
    if let Some(name) = str_(p, "name").filter(|n| !n.trim().is_empty()) {
        params["name"] = json!(name);
    }
    let solid = MeshSolid {
        files,
        tolerance: choices.tolerance,
        angle: choices.angle,
        mesh: choices.mesh,
        cylinders: choices.cylinders,
        cones: choices.cones,
        tori: choices.tori,
        spheres: choices.spheres,
        threads: choices.threads,
        freeform: choices.freeform,
        origin,
        scad: None,
    };
    add_feature(s, &params, FeatureKind::MeshImport { file, meshes, solid: Some(solid) })
}

fn origin_exprs(s: &Session, p: &Value, cmd: &str) -> Result<[Expr; 3]> {
    let mut text = ["0 mm".to_string(), "0 mm".to_string(), "0 mm".to_string()];
    if let Some(raw) = p.get("origin").and_then(Value::as_array) {
        for (slot, value) in text.iter_mut().zip(raw) {
            if let Some(n) = value.as_f64() {
                if !n.is_finite() {
                    return Err(bad(cmd, "origin must be finite"));
                }
                *slot = format!("{n} mm");
            } else if let Some(expr) = value.as_str().filter(|t| !t.trim().is_empty()) {
                *slot = expr.to_string();
            }
        }
    }
    for expr in &text {
        check_expr(s, expr, Kind::Length, cmd, "origin")?;
    }
    Ok([Expr::new(&text[0]), Expr::new(&text[1]), Expr::new(&text[2])])
}

fn stl_mesh(stl: &Path, cmd: &str) -> Result<MeshData> {
    let bytes = solidcraft_io::vfs::read(&stl.to_string_lossy(), MAX_STL_BYTES).map_err(|e| bad(cmd, e.to_string()))?;
    let tris = solidcraft_io::read_stl(&bytes).map_err(|e| bad(cmd, e.to_string()))?;
    if tris.is_empty() {
        return Err(bad(cmd, format!("{} has no triangles", stl.display())));
    }
    if tris.len() > MAX_BATCH_TRIANGLES {
        return Err(bad(cmd, format!("the mesh has {} triangles, more than the {MAX_BATCH_TRIANGLES} conversion can take", tris.len())));
    }
    let mut positions = Vec::with_capacity(tris.len().saturating_mul(9));
    let mut triangles = Vec::with_capacity(tris.len().saturating_mul(3));
    for tri in &tris {
        let base = u32::try_from(positions.len() / 3).map_err(|_| bad(cmd, "too many vertices"))?;
        for corner in tri {
            if !corner.iter().all(|coordinate| coordinate.is_finite()) {
                return Err(bad(cmd, "STL has a non-finite coordinate"));
            }
            positions.extend(corner.iter().map(|c| *c as f64));
        }
        triangles.extend([base, base + 1, base + 2]);
    }
    let name = stl
        .file_stem()
        .map(|n| n.to_string_lossy().into_owned())
        .filter(|n| !n.is_empty() && !n.contains(['/', '\\', ':']))
        .unwrap_or_else(|| "Mesh".into());
    let check_pos: Vec<Vec3> = positions.chunks(3).filter_map(|c| c.get(2).map(|_| Vec3::new(c[0], c[1], c[2]))).collect();
    let check_tri: Vec<[u32; 3]> = triangles.chunks(3).filter_map(|c| c.get(2).map(|_| [c[0], c[1], c[2]])).collect();
    solidcraft_kernel::mesh_body(&check_pos, &check_tri).map_err(|e| bad(cmd, e.to_string()))?;
    Ok(MeshData { name, positions, triangles, color: None })
}

fn is_stl(path: &Path) -> bool {
    path.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("stl"))
}

fn collect_stls(path: &Path, recursive: bool, out: &mut Vec<PathBuf>, depth: usize, cmd: &str) -> Result<()> {
    let text = path.to_string_lossy();
    solidcraft_io::vfs::check(&text).map_err(|e| bad(cmd, e.to_string()))?;
    if path.is_file() {
        if !is_stl(path) {
            return Err(bad(cmd, format!("{} is not an STL file", path.display())));
        }
        out.push(path.to_path_buf());
        return Ok(());
    }
    if !path.is_dir() {
        return Err(bad(cmd, format!("{} was not found", path.display())));
    }
    if depth > MAX_FOLDER_DEPTH {
        return Err(bad(cmd, "the folder is nested too deeply"));
    }
    let entries = std::fs::read_dir(path).map_err(|e| bad(cmd, format!("{}: {e}", path.display())))?;
    let mut names = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|e| bad(cmd, format!("{}: {e}", path.display())))?;
        names.push(entry.path());
        if names.len() > MAX_BATCH.saturating_mul(4) {
            return Err(bad(cmd, format!("at most {MAX_BATCH} STL files")));
        }
    }
    names.sort();
    for child in names {
        if child.is_file() && is_stl(&child) {
            out.push(child);
        } else if recursive && child.is_dir() {
            collect_stls(&child, true, out, depth + 1, cmd)?;
        }
        if out.len() > MAX_BATCH {
            return Err(bad(cmd, format!("at most {MAX_BATCH} STL files")));
        }
    }
    Ok(())
}

fn step_destination(stl: &Path, output: Option<&str>, alone: bool, cmd: &str) -> Result<PathBuf> {
    let Some(output) = output else {
        return Ok(stl.with_extension("step"));
    };
    if output.len() >= 4096 {
        return Err(bad(cmd, "output path is too long"));
    }
    let dest = Path::new(output);
    let as_file = dest.extension().is_some_and(|ext| {
        let ext = ext.to_string_lossy();
        ext.eq_ignore_ascii_case("step") || ext.eq_ignore_ascii_case("stp")
    });
    if as_file && alone {
        return Ok(dest.to_path_buf());
    }
    if as_file {
        return Err(bad(cmd, "output must be a folder when converting more than one STL"));
    }
    let stem = stl
        .file_stem()
        .map(|name| name.to_string_lossy().into_owned())
        .filter(|name| !name.is_empty() && !name.contains(['/', '\\', ':']))
        .ok_or_else(|| bad(cmd, "the STL file has no name"))?;
    Ok(dest.join(format!("{stem}.step")))
}

fn convert_one_stl(stl: &Path, dest: &Path, options: &solidcraft_kernel::ConvertOptions) -> std::result::Result<usize, String> {
    let bytes = solidcraft_io::vfs::read(&stl.to_string_lossy(), MAX_STL_BYTES).map_err(|e| e.to_string())?;
    let tris = solidcraft_io::read_stl(&bytes).map_err(|e| e.to_string())?;
    if tris.is_empty() {
        return Err("STL has no triangles".into());
    }
    if tris.len() > MAX_BATCH_TRIANGLES {
        return Err(format!("the mesh has {} triangles, more than the {MAX_BATCH_TRIANGLES} conversion can take", tris.len()));
    }
    let mut positions = Vec::with_capacity(tris.len().saturating_mul(3));
    let mut triangles = Vec::with_capacity(tris.len());
    for tri in &tris {
        let base = u32::try_from(positions.len()).map_err(|_| "too many vertices".to_string())?;
        for corner in tri {
            if !corner.iter().all(|coordinate| coordinate.is_finite()) {
                return Err("STL has a non-finite coordinate".into());
            }
            positions.push(Vec3::new(corner[0] as f64, corner[1] as f64, corner[2] as f64));
        }
        triangles.push([base, base + 1, base + 2]);
    }
    let body = solidcraft_kernel::mesh_body(&positions, &triangles).map_err(|e| e.to_string())?;
    let solid = solidcraft_kernel::convert_mesh_with(&body, options).map_err(|e| e.to_string())?;
    let step = solidcraft_kernel::step_export(&[&solid], "SolidCraft").map_err(|e| e.to_string())?;
    if let Some(parent) = dest.parent()
        && !parent.as_os_str().is_empty()
    {
        solidcraft_io::vfs::create_dir_all(&parent.to_string_lossy()).map_err(|e| e.to_string())?;
    }
    let path = dest.to_string_lossy();
    solidcraft_io::vfs::write(&path, step.as_bytes()).map_err(|e| format!("{path}: {e}"))?;
    Ok(step.len())
}
