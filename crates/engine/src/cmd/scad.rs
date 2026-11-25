//! OpenSCAD scripts as editable solids.
//!
//! A library is a folder of `.scad` files the user already has. `scad.list` and
//! `scad.parameters` read the Customizer assignments at the top of each file (a group
//! comment, then `name = value; // [min:max]`). `scad.insert` asks the installed OpenSCAD
//! program to write an STL, then rebuilds that mesh as a solid with the same face
//! recognition as `mesh.convert`, so faces highlight and sketch, extrude and fillet apply.
//! The feature keeps the script path and the values used. `scad.revise` runs OpenSCAD
//! again and replaces that same solid. OpenSCAD is not linked or shipped: it is a
//! separate program, the same way a slicer is.

use std::path::{Path, PathBuf};

use serde_json::{Map, Value, json};

use super::CommandSpec;
use crate::params::{bad, str_};
use crate::{EngineError, Result, Session};

/// Bytes read from one script while looking for parameters.
const MAX_SOURCE: usize = 1 << 20;
/// Scripts one library listing returns.
const MAX_FILES: usize = 400;
/// Directory entries looked at in one folder.
const MAX_SCAN: usize = 4_000;
/// Parameters taken from one script.
const MAX_PARAMS: usize = 128;
/// STL OpenSCAD may hand back.
const MAX_STL: usize = 64 << 20;
/// Matches the kernel's solid-reconstruction budget. A denser STL is refused before it
/// is stored as a mesh the rest of the model cannot edit.
const MAX_SCAD_TRIANGLES: u32 = 100_000;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("scad.list", "List OpenSCAD Scripts", list)
        .icon("library")
        .noundo()
        .doc_only()
        .params(
            "path: a folder of .scad files, or one .scad file (its folder is listed and that file is `selected`) → path, files: \
             [{name, path, parameters: [{name, kind, value, group, description?, hidden?, minimum?, maximum?, step?, options?}]}], \
             truncated?, openscad? (the program found on this machine, if any)",
        ),
    CommandSpec::new("scad.parameters", "OpenSCAD Parameters", parameters)
        .icon("inspect")
        .noundo()
        .doc_only()
        .params("path: one .scad file → name, path, parameters (the same parameter objects as scad.list)"),
    CommandSpec::new("scad.insert", "OpenSCAD Library", insert)
        .at("INSERT", "LIBRARY")
        .icon("import")
        .enabled(super::not_in_sketch)
        .params(
            "path: .scad file; parameters?: {name: value} overriding that file's Customizer values (names and kinds must match the file); \
             name?; openscad?: path of the OpenSCAD program (its file name must be openscad, openscad.exe or openscad.com; default: the one on PATH); \
             timeout?: seconds 1–180 (default 60); fn?, fa?, fs? (OpenSCAD's $fn, $fa, $fs). \
             Runs that program to write an STL, then rebuilds it as an editable solid in the active component (faces, edges, sketch, extrude, fillet). \
             The feature keeps the script and these values; editing it runs scad.revise.",
        ),
    CommandSpec::new("scad.revise", "Revise OpenSCAD", revise)
        .icon("import")
        .enabled(super::not_in_sketch)
        .params(
            "feature: the OpenSCAD import to rebuild; path, parameters?, name?, openscad?, timeout?, fn?, fa?, fs?: the same as scad.insert. \
             Runs OpenSCAD again and replaces that feature's solid. The component stays the one the import created.",
        ),
];

#[derive(Clone, Debug, PartialEq)]
struct Param {
    name: String,
    kind: &'static str,
    value: Value,
    group: String,
    description: String,
    hidden: bool,
    minimum: Option<f64>,
    maximum: Option<f64>,
    step: Option<f64>,
    options: Vec<Value>,
}

fn list(s: &mut Session, p: &Value) -> Result<Value> {
    let _ = s;
    #[cfg(target_arch = "wasm32")]
    {
        let _ = p;
        Err(EngineError::Other("scad.list: OpenSCAD libraries are a desktop feature".into()))
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        list_native(p)
    }
}

fn parameters(s: &mut Session, p: &Value) -> Result<Value> {
    let _ = s;
    let path = path_arg(p, "scad.parameters")?;
    solidcraft_io::vfs::check(path).map_err(|e| EngineError::Other(e.to_string()))?;
    if !is_scad(path) {
        return Err(bad("scad.parameters", "`path` must be a .scad file"));
    }
    let source = read_source(path).map_err(|e| bad("scad.parameters", e))?;
    let name = Path::new(path).file_name().and_then(|n| n.to_str()).unwrap_or(path);
    Ok(json!({"path": path, "name": name, "parameters": param_json(&parse_parameters(&source))}))
}

fn insert(s: &mut Session, p: &Value) -> Result<Value> {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (s, p);
        Err(EngineError::Other("scad.insert: OpenSCAD import is a desktop feature".into()))
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        insert_native(s, p)
    }
}

fn revise(s: &mut Session, p: &Value) -> Result<Value> {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (s, p);
        Err(EngineError::Other("scad.revise: OpenSCAD import is a desktop feature".into()))
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        revise_native(s, p)
    }
}

fn path_arg<'a>(p: &'a Value, cmd: &str) -> Result<&'a str> {
    let path = str_(p, "path").map(str::trim).filter(|x| !x.is_empty()).ok_or_else(|| bad(cmd, "`path` is required"))?;
    if path.len() > 4096 || path.chars().any(|c| c == '\0' || c.is_control()) {
        return Err(bad(cmd, "`path` must be at most 4096 characters with no line breaks"));
    }
    Ok(path)
}

fn is_scad(path: &str) -> bool {
    Path::new(path).extension().and_then(|e| e.to_str()).is_some_and(|e| e.eq_ignore_ascii_case("scad"))
}

/// The start of a script, as text. A longer file is cut at [`MAX_SOURCE`]: Customizer
/// assignments sit at the top, and OpenSCAD itself still compiles the whole file.
fn read_source(path: &str) -> std::result::Result<String, String> {
    solidcraft_io::vfs::check(path).map_err(|e| e.to_string())?;
    match solidcraft_io::vfs::read(path, MAX_SOURCE) {
        Ok(bytes) => Ok(String::from_utf8_lossy(&bytes).into_owned()),
        Err(e) => {
            let msg = e.to_string();
            if msg.contains("too large") { read_prefix(path) } else { Err(format!("{path}: {msg}")) }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn read_prefix(path: &str) -> std::result::Result<String, String> {
    use std::io::Read;
    let meta = std::fs::metadata(path).map_err(|e| format!("{path}: {e}"))?;
    if !meta.is_file() {
        return Err(format!("{path}: not a regular file"));
    }
    let mut file = std::fs::File::open(path).map_err(|e| format!("{path}: {e}"))?;
    let mut buf = vec![0u8; MAX_SOURCE];
    let n = file.read(&mut buf).map_err(|e| format!("{path}: {e}"))?;
    buf.truncate(n);
    Ok(String::from_utf8_lossy(&buf).into_owned())
}

#[cfg(target_arch = "wasm32")]
fn read_prefix(path: &str) -> std::result::Result<String, String> {
    Err(format!("{path}: file too large"))
}

fn param_json(params: &[Param]) -> Vec<Value> {
    params.iter().map(Param::to_json).collect()
}

#[cfg(not(target_arch = "wasm32"))]
fn list_native(p: &Value) -> Result<Value> {
    let path = path_arg(p, "scad.list")?;
    solidcraft_io::vfs::check(path).map_err(|e| EngineError::Other(e.to_string()))?;
    let given = Path::new(path);
    let meta = std::fs::metadata(given).map_err(|e| bad("scad.list", format!("{path}: {e}")))?;
    if meta.is_file() {
        if !is_scad(path) {
            return Err(bad("scad.list", "`path` must be a folder or a .scad file"));
        }
        let parent = given.parent().filter(|d| !d.as_os_str().is_empty()).unwrap_or_else(|| Path::new("."));
        let mut out = list_dir(parent)?;
        if let Some(name) = given.file_name().and_then(|n| n.to_str()) {
            out["selected"] = json!(name);
        }
        return Ok(out);
    }
    if meta.is_dir() {
        return list_dir(given);
    }
    Err(bad("scad.list", "`path` must be a folder or a .scad file"))
}

#[cfg(not(target_arch = "wasm32"))]
fn list_dir(dir: &Path) -> Result<Value> {
    let shown = dir.to_string_lossy();
    solidcraft_io::vfs::check(&shown).map_err(|e| EngineError::Other(e.to_string()))?;
    let entries = std::fs::read_dir(dir).map_err(|e| bad("scad.list", format!("{shown}: {e}")))?;
    let mut paths = Vec::new();
    let mut scanned = 0usize;
    let mut truncated = false;
    for entry in entries {
        scanned += 1;
        if scanned > MAX_SCAN {
            truncated = true;
            break;
        }
        let Ok(entry) = entry else { continue };
        let Ok(meta) = entry.metadata() else { continue };
        if !meta.is_file() {
            continue;
        }
        let Some(name) = entry.file_name().into_string().ok() else { continue };
        if name.starts_with('.') || !name.to_ascii_lowercase().ends_with(".scad") {
            continue;
        }
        if paths.len() >= MAX_FILES {
            truncated = true;
            break;
        }
        paths.push(entry.path());
    }
    paths.sort_by(|a, b| a.file_name().cmp(&b.file_name()));
    let mut files = Vec::with_capacity(paths.len());
    for path in paths {
        files.push(describe_file(&path));
    }
    let mut out = json!({"path": shown, "files": files});
    if truncated {
        out["truncated"] = json!(true);
    }
    if let Ok(program) = find_program(None) {
        out["openscad"] = json!(program.display().to_string());
    }
    Ok(out)
}

#[cfg(not(target_arch = "wasm32"))]
fn describe_file(path: &Path) -> Value {
    let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("").to_string();
    let shown = path.to_string_lossy().to_string();
    match read_source(&shown) {
        Ok(source) => json!({"name": name, "path": shown, "parameters": param_json(&parse_parameters(&source))}),
        Err(error) => json!({"name": name, "path": shown, "parameters": [], "error": error}),
    }
}

#[cfg(not(target_arch = "wasm32"))]
struct Built {
    bytes: Vec<u8>,
    program: PathBuf,
    overrides: Vec<(String, Value)>,
    quality: Vec<(String, Value)>,
    path: String,
}

#[cfg(not(target_arch = "wasm32"))]
fn build(p: &Value, cmd: &str) -> Result<Built> {
    let path = path_arg(p, cmd)?.to_string();
    solidcraft_io::vfs::check(&path).map_err(|e| EngineError::Other(e.to_string()))?;
    if !is_scad(&path) {
        return Err(bad(cmd, "`path` must be a .scad file"));
    }
    let source = read_source(&path).map_err(|e| bad(cmd, e))?;
    let known = parse_parameters(&source);
    let overrides = overrides(p, &known, cmd)?;
    let quality = quality(p, cmd)?;
    let timeout = timeout_secs(p, cmd)?;
    let explicit = str_(p, "openscad").map(str::trim).filter(|x| !x.is_empty());
    if let Some(program) = explicit && (program.len() > 4096 || program.chars().any(|c| c == '\0' || c.is_control())) {
        return Err(bad(cmd, "`openscad` must be a program path"));
    }
    let program = find_program(explicit).map_err(|e| bad(cmd, e))?;
    let mut assignments = Vec::new();
    for (name, value) in &quality {
        assignments.push(scad_assign(name, value).map_err(|e| bad(cmd, e))?);
    }
    for (name, value) in &overrides {
        assignments.push(scad_assign(name, value).map_err(|e| bad(cmd, e))?);
    }
    let bytes = render(&program, Path::new(&path), &assignments, timeout).map_err(EngineError::Other)?;
    let triangles = stl_triangle_count(&bytes);
    if triangles > MAX_SCAD_TRIANGLES {
        return Err(EngineError::Other(format!(
            "OpenSCAD produced {triangles} triangles, above the {MAX_SCAD_TRIANGLES} that can become an editable solid. Set a lower $fn or a larger $fa and try again."
        )));
    }
    Ok(Built { bytes, program, overrides, quality, path })
}

#[cfg(not(target_arch = "wasm32"))]
fn insert_native(s: &mut Session, p: &Value) -> Result<Value> {
    let built = build(p, "scad.insert")?;
    let recipe = recipe_of(&built.path, &built.program, &built.overrides, &built.quality);
    let mut out = import_stl(s, &built.bytes, str_(p, "name"), recipe)?;
    if let Value::Object(map) = &mut out {
        map.insert("path".into(), json!(built.path));
        map.insert("openscad".into(), json!(built.program.display().to_string()));
        map.insert("parameters".into(), Value::Object(built.overrides.into_iter().collect()));
    }
    Ok(out)
}

#[cfg(not(target_arch = "wasm32"))]
fn revise_native(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "scad.revise";
    let built = build(p, cmd)?;
    let id = p.get("feature").and_then(Value::as_u64).ok_or_else(|| bad(cmd, "`feature` is required"))?;
    let existing = s.doc.feature(id).ok_or_else(|| bad(cmd, "that feature is not in this design"))?;
    if !is_scad_kind(&existing.kind) {
        return Err(bad(cmd, "that feature is not an OpenSCAD import"));
    }
    let label = format!("{}.stl", built.path);
    let mf = solidcraft_io::mesh_import_feature(&built.bytes, &label).map_err(|e| EngineError::Other(e.to_string()))?;
    let recipe = recipe_of(&built.path, &built.program, &built.overrides, &built.quality);
    let kind = editable_solid(mf.kind, recipe)?;
    install(s, id, kind, mf.body_names, str_(p, "name"), cmd)?;
    let bodies: Vec<String> = s.model.state().bodies.iter().filter(|b| b.feature == id).map(|b| b.name.clone()).collect();
    let feature_name = s.doc.feature(id).map(|f| f.name.clone()).unwrap_or_default();
    Ok(json!({
        "feature": id,
        "name": feature_name,
        "bodies": bodies,
        "solid": true,
        "path": built.path,
        "openscad": built.program.display().to_string(),
        "parameters": Value::Object(built.overrides.into_iter().collect()),
    }))
}

#[cfg(not(target_arch = "wasm32"))]
fn is_scad_kind(kind: &solidcraft_doc::FeatureKind) -> bool {
    let solidcraft_doc::FeatureKind::MeshImport { solid: Some(solid), .. } = kind else { return false };
    solid.scad.is_some() || solid.files.iter().any(|path| path.to_ascii_lowercase().ends_with(".scad"))
}

#[cfg(not(target_arch = "wasm32"))]
fn recipe_of(path: &str, program: &Path, overrides: &[(String, Value)], quality: &[(String, Value)]) -> solidcraft_doc::ScadSource {
    let mut parameters = serde_json::Map::new();
    for (name, value) in overrides {
        parameters.insert(name.clone(), value.clone());
    }
    let number = |key: &str| quality.iter().find(|(name, _)| name == key).and_then(|(_, value)| value.as_f64());
    let segments = number("$fn").filter(|n| n.fract() == 0.0 && (0.0..=1024.0).contains(n)).map(|n| n as u64);
    solidcraft_doc::ScadSource {
        path: path.to_string(),
        parameters,
        openscad: Some(program.display().to_string()),
        segments,
        fa: number("$fa"),
        fs: number("$fs"),
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn import_stl(s: &mut Session, bytes: &[u8], name: Option<&str>, recipe: solidcraft_doc::ScadSource) -> Result<Value> {
    let label = format!("{}.stl", recipe.path);
    let mf = solidcraft_io::mesh_import_feature(bytes, &label).map_err(|e| EngineError::Other(e.to_string()))?;
    let feature_name = name.map(str::trim).filter(|n| !n.is_empty()).map(str::to_string).unwrap_or_else(|| mf.name.clone());
    let component = super::file::import::component_name(&recipe.path, name);
    let kind = editable_solid(mf.kind, recipe)?;
    let (id, placed) = super::file::import::add_in_component(s, kind, &feature_name, mf.body_names, &component, super::file::import::Tree::Flat)?;
    let bodies: Vec<String> = s.model.state().bodies.iter().filter(|b| b.feature == id).map(|b| b.name.clone()).collect();
    let created_bodies = bodies.clone();
    let feature_name = s.doc.feature(id).map(|f| f.name.clone()).unwrap_or_default();
    let mut out = json!({"feature": id, "name": feature_name, "bodies": bodies, "created_bodies": created_bodies, "warnings": mf.warnings, "solid": true});
    placed.report(&mut out);
    Ok(out)
}

/// Replace one OpenSCAD feature's solid. The component it already lives in stays.
#[cfg(not(target_arch = "wasm32"))]
fn install(s: &mut Session, id: u64, kind: solidcraft_doc::FeatureKind, body_names: Vec<String>, name: Option<&str>, cmd: &str) -> Result<()> {
    let component = {
        let feature = s.doc_mut().features.iter_mut().find(|f| f.id == id).ok_or_else(|| bad(cmd, "that feature is not in this design"))?;
        feature.kind = kind;
        feature.body_names = body_names;
        if let Some(name) = name.map(str::trim).filter(|n| !n.is_empty()) {
            feature.name = name.chars().take(128).collect();
        }
        feature.component
    };
    if let Some(name) = name.map(str::trim).filter(|n| !n.is_empty()) {
        let name: String = name.chars().take(128).collect();
        if let Some(component) = s.doc_mut().components.iter_mut().find(|c| c.id == component) {
            component.name = name;
        }
    }
    s.refresh();
    if let Some(error) = s.model.result(id).and_then(|r| r.error.clone()) {
        return Err(EngineError::Other(error));
    }
    Ok(())
}

/// The STL OpenSCAD wrote, rebuilt as analytic faces (planes, cylinders, spheres, and
/// leftover triangles as facets). That is the same body a STEP solid is: faces highlight
/// and the modelling commands apply. The script and the values used stay on the feature
/// so the timeline step can be opened again.
#[cfg(not(target_arch = "wasm32"))]
fn editable_solid(kind: solidcraft_doc::FeatureKind, recipe: solidcraft_doc::ScadSource) -> Result<solidcraft_doc::FeatureKind> {
    let solidcraft_doc::FeatureKind::MeshImport { file, meshes, .. } = kind else {
        return Err(EngineError::Other("OpenSCAD import did not produce a mesh".into()));
    };
    let triangles: usize = meshes.iter().map(|mesh| mesh.triangles.len() / 3).sum();
    if triangles > MAX_SCAD_TRIANGLES as usize {
        return Err(EngineError::Other(format!(
            "OpenSCAD produced {triangles} triangles, above the {MAX_SCAD_TRIANGLES} that can become an editable solid. Set a lower $fn or a larger $fa and try again."
        )));
    }
    let expr = |text: &str| solidcraft_doc::Expr::new(text);
    Ok(solidcraft_doc::FeatureKind::MeshImport {
        file,
        meshes,
        solid: Some(solidcraft_doc::MeshSolid {
            files: vec![recipe.path.clone()],
            tolerance: expr("0.05 mm"),
            angle: expr("15 deg"),
            mesh: 100.0,
            cylinders: true,
            cones: true,
            tori: true,
            spheres: true,
            threads: true,
            freeform: true,
            origin: [expr("0 mm"), expr("0 mm"), expr("0 mm")],
            scad: Some(recipe),
        }),
    })
}

fn overrides(p: &Value, known: &[Param], cmd: &str) -> Result<Vec<(String, Value)>> {
    let Some(raw) = p.get("parameters").filter(|v| !v.is_null()) else { return Ok(Vec::new()) };
    let obj = raw.as_object().ok_or_else(|| bad(cmd, "`parameters` must be an object of names and values"))?;
    if obj.len() > MAX_PARAMS {
        return Err(bad(cmd, format!("`parameters` has more than {MAX_PARAMS} values")));
    }
    let mut out = Vec::with_capacity(obj.len());
    for (name, value) in obj {
        let spec = known.iter().find(|p| p.name == *name).ok_or_else(|| bad(cmd, format!("`{name}` is not a parameter of this file")))?;
        out.push((name.clone(), spec.coerce(value).map_err(|e| bad(cmd, e))?));
    }
    Ok(out)
}

fn quality(p: &Value, cmd: &str) -> Result<Vec<(String, Value)>> {
    let mut out = Vec::new();
    for (key, scad, min, max) in [("$fn", "fn", 0.0, 1024.0), ("$fa", "fa", 0.01, 180.0), ("$fs", "fs", 0.01, 100.0)] {
        let Some(raw) = p.get(scad).filter(|v| !v.is_null()) else { continue };
        let n = raw
            .as_f64()
            .filter(|n| n.is_finite() && *n >= min && *n <= max)
            .ok_or_else(|| bad(cmd, format!("`{scad}` must be a number from {min} to {max}")))?;
        out.push((key.to_string(), json!(n)));
    }
    Ok(out)
}

fn timeout_secs(p: &Value, cmd: &str) -> Result<u64> {
    match p.get("timeout") {
        None | Some(Value::Null) => Ok(60),
        Some(v) => v.as_u64().filter(|n| (1..=180).contains(n)).ok_or_else(|| bad(cmd, "`timeout` must be a whole number of seconds from 1 to 180")),
    }
}

fn find_program(explicit: Option<&str>) -> std::result::Result<PathBuf, String> {
    if let Some(path) = explicit {
        if !program_name_ok(path) {
            return Err("`openscad` must name the OpenSCAD program (openscad, openscad.exe or openscad.com)".into());
        }
        let path = PathBuf::from(path);
        return path.is_file().then_some(path).ok_or_else(|| "OpenSCAD was not found at that path".into());
    }
    for name in ["openscad", "openscad.com", "openscad.exe", "OpenSCAD", "OpenSCAD.com", "OpenSCAD.exe"] {
        if let Some(found) = on_path(name) {
            return Ok(found);
        }
    }
    for candidate in installed_candidates() {
        if candidate.is_file() && program_name_ok(&candidate.to_string_lossy()) {
            return Ok(candidate);
        }
    }
    Err("OpenSCAD is not installed or not on PATH. Install OpenSCAD, or pass `openscad` as its program path. SolidCraft does not include OpenSCAD."
        .into())
}

fn program_name_ok(path: &str) -> bool {
    let Some(name) = Path::new(path).file_name().and_then(|n| n.to_str()) else { return false };
    matches!(name.to_ascii_lowercase().as_str(), "openscad" | "openscad.exe" | "openscad.com" | "openscad-nightly" | "openscad-nightly.exe")
}

#[cfg(not(target_arch = "wasm32"))]
fn on_path(name: &str) -> Option<PathBuf> {
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path).take(128).map(|dir| dir.join(name)).find(|candidate| candidate.is_file())
}

#[cfg(target_arch = "wasm32")]
fn on_path(_name: &str) -> Option<PathBuf> {
    None
}

#[cfg(windows)]
fn installed_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    for key in ["ProgramFiles", "ProgramFiles(x86)", "LOCALAPPDATA"] {
        let Some(root) = std::env::var_os(key) else { continue };
        let root = PathBuf::from(root);
        out.push(root.join("OpenSCAD").join("openscad.exe"));
        out.push(root.join("OpenSCAD").join("openscad.com"));
        out.push(root.join("OpenSCAD (Nightly)").join("openscad.exe"));
        if key == "LOCALAPPDATA" {
            out.push(root.join("Programs").join("OpenSCAD").join("openscad.exe"));
        }
    }
    out
}

#[cfg(not(windows))]
fn installed_candidates() -> Vec<PathBuf> {
    Vec::new()
}

#[cfg(not(target_arch = "wasm32"))]
fn render(program: &Path, scad: &Path, assignments: &[String], timeout: u64) -> std::result::Result<Vec<u8>, String> {
    let n = TEMP_SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("solidcraft-scad-{}-{n}", std::process::id()));
    std::fs::create_dir_all(&dir).map_err(|e| format!("could not create a temporary folder ({e})"))?;
    let _cleanup = DirGuard(dir.clone());
    let stl = dir.join("out.stl");
    let err_path = dir.join("openscad.txt");
    let err_file = std::fs::File::create(&err_path).map_err(|e| format!("could not record OpenSCAD's messages ({e})"))?;
    let mut cmd = std::process::Command::new(program);
    cmd.arg("-o").arg(&stl);
    for assignment in assignments {
        cmd.arg("-D").arg(assignment);
    }
    cmd.arg(scad);
    if let Some(parent) = scad.parent().filter(|p| !p.as_os_str().is_empty()) {
        cmd.current_dir(parent);
    }
    cmd.stdin(std::process::Stdio::null()).stdout(std::process::Stdio::null()).stderr(std::process::Stdio::from(err_file));
    let mut child = cmd.spawn().map_err(|e| format!("could not start OpenSCAD ({e})"))?;
    let limit = std::time::Duration::from_secs(timeout);
    let started = web_time::Instant::now();
    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break status,
            Ok(None) if started.elapsed() >= limit => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(format!("OpenSCAD did not finish within {timeout} seconds"));
            }
            Ok(None) => std::thread::sleep(std::time::Duration::from_millis(40)),
            Err(e) => return Err(format!("OpenSCAD stopped ({e})")),
        }
    };
    let stderr = read_capped(&err_path, 8 * 1024);
    if !status.success() {
        return Err(format!("OpenSCAD failed ({status}). {stderr}"));
    }
    let meta = std::fs::metadata(&stl).map_err(|_| format!("OpenSCAD wrote no STL. {stderr}"))?;
    if !meta.is_file() || meta.len() == 0 || meta.len() > MAX_STL as u64 {
        return Err(format!("OpenSCAD wrote no STL, or one larger than {} MB. {stderr}", MAX_STL >> 20));
    }
    let bytes = std::fs::read(&stl).map_err(|e| format!("could not read the STL ({e})"))?;
    if bytes.len() > MAX_STL {
        return Err(format!("OpenSCAD STL is larger than {} MB", MAX_STL >> 20));
    }
    Ok(bytes)
}

#[cfg(not(target_arch = "wasm32"))]
fn read_capped(path: &Path, max: usize) -> String {
    let Ok(bytes) = std::fs::read(path) else { return String::new() };
    let n = bytes.len().min(max);
    String::from_utf8_lossy(&bytes[..n]).trim().to_string()
}

/// Triangle count of a binary STL, or of an ASCII STL by its `facet normal` lines.
#[cfg(not(target_arch = "wasm32"))]
fn stl_triangle_count(bytes: &[u8]) -> u32 {
    if bytes.len() >= 84 {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]);
        let body = 84usize.saturating_add((n as usize).saturating_mul(50));
        if n > 0 && body == bytes.len() {
            return n;
        }
    }
    let needle = b"facet normal";
    let mut n = 0u32;
    let mut i = 0usize;
    while i + needle.len() <= bytes.len() {
        if bytes[i..i + needle.len()] == *needle {
            n = n.saturating_add(1);
            if n > MAX_SCAD_TRIANGLES {
                return n;
            }
            i += needle.len();
        } else {
            i += 1;
        }
    }
    n
}

#[cfg(not(target_arch = "wasm32"))]
struct DirGuard(PathBuf);

#[cfg(not(target_arch = "wasm32"))]
impl Drop for DirGuard {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[cfg(not(target_arch = "wasm32"))]
static TEMP_SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

fn scad_assign(name: &str, value: &Value) -> std::result::Result<String, String> {
    if !valid_name(name) {
        return Err(format!("`{name}` is not an OpenSCAD name"));
    }
    let literal = match value {
        Value::Bool(true) => "true".to_string(),
        Value::Bool(false) => "false".to_string(),
        Value::Number(n) => {
            let n = n.as_f64().filter(|n| n.is_finite() && n.abs() < 1e9).ok_or_else(|| format!("`{name}` is not a finite number"))?;
            format_number(n).ok_or_else(|| format!("`{name}` is not a finite number"))?
        }
        Value::String(s) => format!("\"{}\"", escape_scad(s)?),
        _ => return Err(format!("`{name}` must be a number, true or false, or text")),
    };
    let assignment = format!("{name}={literal}");
    if assignment.len() > 512 || assignment.chars().any(|c| c.is_control()) {
        return Err(format!("`{name}` is too long to pass to OpenSCAD"));
    }
    Ok(assignment)
}

fn format_number(n: f64) -> Option<String> {
    if !n.is_finite() || n.abs() >= 1e9 {
        return None;
    }
    if n.fract() == 0.0 && n.abs() < 1e15 {
        return Some(format!("{}", n as i64));
    }
    let text = format!("{n:.8}");
    Some(text.trim_end_matches('0').trim_end_matches('.').to_string())
}

fn escape_scad(text: &str) -> std::result::Result<String, String> {
    if text.chars().count() > 240 || text.chars().any(char::is_control) {
        return Err("text parameters must be at most 240 characters with no line breaks".into());
    }
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            _ => out.push(c),
        }
    }
    Ok(out)
}

fn valid_name(name: &str) -> bool {
    let mut chars = name.chars();
    let Some(first) = chars.next() else { return false };
    let rest = |c: char| c.is_ascii_alphanumeric() || c == '_';
    let ok = if first == '$' {
        matches!(chars.next(), Some(c) if c.is_ascii_alphabetic() || c == '_') && chars.all(rest)
    } else {
        (first.is_ascii_alphabetic() || first == '_') && chars.all(rest)
    };
    ok && (1..=64).contains(&name.len())
}

impl Param {
    fn to_json(&self) -> Value {
        let mut m = Map::new();
        m.insert("name".into(), json!(self.name));
        m.insert("kind".into(), json!(self.kind));
        m.insert("value".into(), self.value.clone());
        m.insert("group".into(), json!(self.group));
        if !self.description.is_empty() {
            m.insert("description".into(), json!(self.description));
        }
        if self.hidden {
            m.insert("hidden".into(), json!(true));
        }
        if let Some(n) = self.minimum {
            m.insert("minimum".into(), json!(n));
        }
        if let Some(n) = self.maximum {
            m.insert("maximum".into(), json!(n));
        }
        if let Some(n) = self.step {
            m.insert("step".into(), json!(n));
        }
        if !self.options.is_empty() {
            m.insert("options".into(), json!(self.options));
        }
        Value::Object(m)
    }

    fn coerce(&self, value: &Value) -> std::result::Result<Value, String> {
        let coerced = match self.kind {
            "bool" => Value::Bool(value.as_bool().ok_or_else(|| format!("`{}` must be true or false", self.name))?),
            "int" => {
                let n = value
                    .as_f64()
                    .filter(|n| n.is_finite() && n.fract() == 0.0 && n.abs() < 1e15)
                    .ok_or_else(|| format!("`{}` must be a whole number", self.name))?;
                json!(n as i64)
            }
            "number" => {
                let n =
                    value.as_f64().filter(|n| n.is_finite() && n.abs() < 1e9).ok_or_else(|| format!("`{}` must be a finite number", self.name))?;
                json!(n)
            }
            "string" => {
                let s = value.as_str().ok_or_else(|| format!("`{}` must be text", self.name))?;
                escape_scad(s)?;
                json!(s)
            }
            "enum" => {
                if !self.options.iter().any(|option| same_value(option, value)) {
                    return Err(format!("`{}` must be one of the script's choices", self.name));
                }
                value.clone()
            }
            _ => return Err(format!("`{}` has no supported kind", self.name)),
        };
        if let Some(n) = coerced.as_f64()
            && (self.minimum.is_some_and(|min| n < min) || self.maximum.is_some_and(|max| n > max))
        {
            return Err(format!("`{}` is outside the range written in the script", self.name));
        }
        Ok(coerced)
    }
}

fn same_value(want: &Value, got: &Value) -> bool {
    match (want, got) {
        (Value::Bool(a), Value::Bool(b)) => a == b,
        (Value::String(a), Value::String(b)) => a == b,
        (Value::Number(a), Value::Number(b)) => match (a.as_f64(), b.as_f64()) {
            (Some(a), Some(b)) => a.is_finite() && b.is_finite() && (a - b).abs() <= 1e-9 * a.abs().max(1.0),
            _ => false,
        },
        _ => false,
    }
}

/// Customizer parameters: top-level `name = value;` assignments, with an optional
/// `// [min:max]`, `// [min:step:max]` or `// [a, b]` comment. Assignments inside
/// `module` or `function` bodies are not parameters. `/* [Hidden] */` marks the
/// following ones hidden.
fn parse_parameters(source: &str) -> Vec<Param> {
    let mut out = Vec::new();
    let mut depth = 0i32;
    let mut block = false;
    let mut group = "Parameters".to_string();
    let mut hidden = false;
    for line in source.lines().take(20_000) {
        if out.len() >= MAX_PARAMS {
            break;
        }
        let started = depth;
        let in_block = block;
        if !in_block
            && started == 0
            && let Some(name) = group_marker(line)
        {
            hidden = name.eq_ignore_ascii_case("hidden");
            group = if hidden { "Hidden".to_string() } else { name.to_string() };
        }
        let (code, comment, next_block) = strip_line(line, in_block);
        block = next_block;
        if started == 0
            && !in_block
            && let Some(param) = assignment(&code, &comment, &group, hidden)
            && !out.iter().any(|p: &Param| p.name == param.name)
        {
            out.push(param);
        }
        depth = (started + brace_delta(&code)).max(0);
    }
    out
}

fn group_marker(line: &str) -> Option<&str> {
    let rest = line.trim().strip_prefix("/*")?.trim_start().strip_prefix('[')?;
    let (name, after) = rest.split_once(']')?;
    let name = name.trim();
    if name.is_empty() || name.chars().count() > 80 || name.chars().any(char::is_control) {
        return None;
    }
    after.trim_start().starts_with("*/").then_some(name)
}

fn strip_line(line: &str, mut block: bool) -> (String, String, bool) {
    let mut code = String::new();
    let mut comment = String::new();
    let mut chars = line.chars().peekable();
    let mut line_comment = false;
    while let Some(c) = chars.next() {
        if block {
            if c == '*' && chars.peek() == Some(&'/') {
                chars.next();
                block = false;
            }
            continue;
        }
        if line_comment {
            comment.push(c);
            continue;
        }
        if c == '/' && chars.peek() == Some(&'/') {
            chars.next();
            line_comment = true;
            continue;
        }
        if c == '/' && chars.peek() == Some(&'*') {
            chars.next();
            let mut closed = false;
            while let Some(d) = chars.next() {
                if d == '*' && chars.peek() == Some(&'/') {
                    chars.next();
                    closed = true;
                    break;
                }
            }
            if !closed {
                block = true;
            }
            continue;
        }
        if c == '"' || c == '\'' {
            code.push(c);
            while let Some(d) = chars.next() {
                code.push(d);
                if d == '\\' {
                    if let Some(e) = chars.next() {
                        code.push(e);
                    }
                    continue;
                }
                if d == c {
                    break;
                }
            }
            continue;
        }
        code.push(c);
    }
    (code, comment, block)
}

fn brace_delta(code: &str) -> i32 {
    let mut delta = 0i32;
    let mut chars = code.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '"' || c == '\'' {
            while let Some(d) = chars.next() {
                if d == '\\' {
                    chars.next();
                    continue;
                }
                if d == c {
                    break;
                }
            }
            continue;
        }
        match c {
            '{' => delta += 1,
            '}' => delta -= 1,
            _ => {}
        }
    }
    delta
}

fn assignment(code: &str, comment: &str, group: &str, hidden: bool) -> Option<Param> {
    let code = code.trim();
    if code.is_empty() {
        return None;
    }
    let head = code.split_whitespace().next().unwrap_or("").to_ascii_lowercase();
    if matches!(head.as_str(), "module" | "function" | "include" | "use" | "echo" | "assert") {
        return None;
    }
    let (name, value) = split_assign(code)?;
    if value.starts_with('[') || value.starts_with('{') {
        return None;
    }
    let (kind, parsed) = parse_literal(&value)?;
    let (description, ann) = annotation(comment);
    let mut param = Param {
        name,
        kind,
        value: parsed,
        group: group.to_string(),
        description,
        hidden,
        minimum: None,
        maximum: None,
        step: None,
        options: Vec::new(),
    };
    match ann {
        Some(Ann::Range { min, step, max }) if matches!(param.kind, "number" | "int") => {
            param.minimum = Some(min);
            param.maximum = Some(max);
            param.step = step;
            if step.is_some_and(|s| s.fract() != 0.0) || min.fract() != 0.0 || max.fract() != 0.0 {
                param.kind = "number";
            }
        }
        Some(Ann::Enum(options)) if !options.is_empty() => {
            param.kind = "enum";
            param.options = options;
        }
        _ => {}
    }
    Some(param)
}

fn split_assign(code: &str) -> Option<(String, String)> {
    let mut chars = code.chars().peekable();
    let mut name = String::new();
    if chars.peek() == Some(&'$') {
        name.push('$');
        chars.next();
    }
    let first = chars.peek().copied()?;
    if !(first.is_ascii_alphabetic() || first == '_') {
        return None;
    }
    while let Some(c) = chars.peek().copied() {
        if c.is_ascii_alphanumeric() || c == '_' {
            name.push(c);
            chars.next();
        } else {
            break;
        }
    }
    if !valid_name(&name) {
        return None;
    }
    while chars.peek().is_some_and(|c| c.is_whitespace()) {
        chars.next();
    }
    if chars.next() != Some('=') {
        return None;
    }
    let rest: String = chars.collect();
    let value = until_semicolon(&rest)?.trim().to_string();
    if value.is_empty() || value.len() > 400 {
        return None;
    }
    Some((name, value))
}

fn until_semicolon(text: &str) -> Option<&str> {
    let mut chars = text.char_indices();
    while let Some((i, c)) = chars.next() {
        if c == '"' || c == '\'' {
            while let Some((_, d)) = chars.next() {
                if d == '\\' {
                    chars.next();
                    continue;
                }
                if d == c {
                    break;
                }
            }
            continue;
        }
        if c == ';' {
            return Some(&text[..i]);
        }
    }
    None
}

fn parse_literal(raw: &str) -> Option<(&'static str, Value)> {
    let text = raw.trim();
    if text.eq_ignore_ascii_case("true") {
        return Some(("bool", json!(true)));
    }
    if text.eq_ignore_ascii_case("false") {
        return Some(("bool", json!(false)));
    }
    if let Some(s) = unquote(text) {
        return (s.chars().count() <= 240 && !s.chars().any(char::is_control)).then_some(("string", json!(s)));
    }
    let (n, integral) = eval_numeric(text)?;
    if integral { Some(("int", json!(n as i64))) } else { Some(("number", json!(n))) }
}

fn unquote(text: &str) -> Option<String> {
    let mut chars = text.chars();
    let quote = chars.next().filter(|c| *c == '"' || *c == '\'')?;
    let mut out = String::new();
    while let Some(c) = chars.next() {
        if c == '\\' {
            out.push(chars.next()?);
            continue;
        }
        if c == quote {
            return chars.next().is_none().then_some(out);
        }
        out.push(c);
    }
    None
}

enum Ann {
    Range { min: f64, step: Option<f64>, max: f64 },
    Enum(Vec<Value>),
}

fn annotation(comment: &str) -> (String, Option<Ann>) {
    let comment = comment.trim();
    let Some(start) = comment.find('[') else { return (clean_desc(comment), None) };
    let Some(end) = comment[start + 1..].find(']') else { return (clean_desc(comment), None) };
    let body = &comment[start + 1..start + 1 + end];
    let before = comment[..start].trim();
    let after = comment[start + 1 + end + 1..].trim();
    let description = if after.is_empty() {
        before.to_string()
    } else if before.is_empty() {
        after.to_string()
    } else {
        format!("{before} {after}")
    };
    (clean_desc(&description), parse_ann(body))
}

fn clean_desc(text: &str) -> String {
    let text = text.trim();
    if text.chars().any(char::is_control) {
        return String::new();
    }
    text.chars().take(160).collect()
}

fn parse_ann(body: &str) -> Option<Ann> {
    if body.contains(':') && !body.contains(',') {
        let parts: Vec<&str> = body.split(':').collect();
        let nums: Option<Vec<f64>> = parts.iter().map(|p| finite_token(p)).collect();
        return match nums?.as_slice() {
            [min, max] => Some(Ann::Range { min: *min, step: None, max: *max }),
            [min, step, max] => Some(Ann::Range { min: *min, step: Some(*step), max: *max }),
            _ => None,
        };
    }
    if body.contains(',') || !body.trim().is_empty() {
        let options: Vec<Value> = body
            .split(',')
            .filter_map(|part| {
                let part = part.trim();
                if part.is_empty() || part.chars().count() > 64 {
                    return None;
                }
                if let Some(s) = unquote(part) {
                    return Some(json!(s));
                }
                if let Some(n) = finite_token(part) {
                    return Some(if n.fract() == 0.0 { json!(n as i64) } else { json!(n) });
                }
                if part.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == ' ') {
                    return Some(json!(part));
                }
                None
            })
            .take(32)
            .collect();
        return (!options.is_empty()).then_some(Ann::Enum(options));
    }
    None
}

fn finite_token(text: &str) -> Option<f64> {
    let text = text.trim();
    if text.is_empty() || text.len() > 32 {
        return None;
    }
    text.parse::<f64>().ok().filter(|n| n.is_finite() && n.abs() < 1e9)
}

struct NumParser<'a> {
    s: &'a [u8],
    i: usize,
}

fn eval_numeric(text: &str) -> Option<(f64, bool)> {
    if text.is_empty()
        || text.len() > 64
        || !text.chars().all(|c| c.is_ascii_digit() || matches!(c, '+' | '-' | '*' | '/' | '(' | ')' | '.' | ' ' | '\t'))
    {
        return None;
    }
    let mut p = NumParser { s: text.as_bytes(), i: 0 };
    let v = p.expr(0)?;
    p.skip();
    if p.i != p.s.len() || !v.is_finite() || v.abs() >= 1e9 {
        return None;
    }
    Some((v, v.fract() == 0.0 && !text.contains('.')))
}

impl NumParser<'_> {
    fn skip(&mut self) {
        while self.s.get(self.i).is_some_and(u8::is_ascii_whitespace) {
            self.i += 1;
        }
    }
    fn expr(&mut self, depth: u8) -> Option<f64> {
        if depth > 32 {
            return None;
        }
        let mut v = self.mul(depth)?;
        loop {
            self.skip();
            match self.s.get(self.i) {
                Some(b'+') => {
                    self.i += 1;
                    v += self.mul(depth)?;
                }
                Some(b'-') => {
                    self.i += 1;
                    v -= self.mul(depth)?;
                }
                _ => return Some(v),
            }
        }
    }
    fn mul(&mut self, depth: u8) -> Option<f64> {
        let mut v = self.unary(depth)?;
        loop {
            self.skip();
            match self.s.get(self.i) {
                Some(b'*') => {
                    self.i += 1;
                    v *= self.unary(depth)?;
                }
                Some(b'/') => {
                    self.i += 1;
                    let d = self.unary(depth)?;
                    if d == 0.0 {
                        return None;
                    }
                    v /= d;
                }
                _ => return Some(v),
            }
        }
    }
    fn unary(&mut self, depth: u8) -> Option<f64> {
        self.skip();
        if self.s.get(self.i) == Some(&b'+') || self.s.get(self.i) == Some(&b'-') {
            let neg = self.s.get(self.i) == Some(&b'-');
            self.i += 1;
            let v = self.unary(depth)?;
            return Some(if neg { -v } else { v });
        }
        self.atom(depth)
    }
    fn atom(&mut self, depth: u8) -> Option<f64> {
        self.skip();
        if self.s.get(self.i) == Some(&b'(') {
            self.i += 1;
            let v = self.expr(depth + 1)?;
            self.skip();
            if self.s.get(self.i) != Some(&b')') {
                return None;
            }
            self.i += 1;
            return Some(v);
        }
        let start = self.i;
        while self.s.get(self.i).is_some_and(|c| c.is_ascii_digit() || *c == b'.') {
            self.i += 1;
        }
        if self.i == start {
            return None;
        }
        std::str::from_utf8(self.s.get(start..self.i)?).ok()?.parse().ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Session;
    use serde_json::json;

    const SAMPLE: &str = r#"
/* [Size] */
width = 10; // outer [1:100]
height = 20; // [1:0.5:40]
label = "box"; // name
shape = "round"; // [round, square]
on = true;
inner = 120 - 14;
hole = [1, 2, 3];
/* [Hidden] */
secret = 1;
module foo() {
    x = 2;
}
"#;

    #[test]
    fn customizer_assignments_keep_groups_ranges_and_skip_modules() {
        let params = parse_parameters(SAMPLE);
        let names: Vec<&str> = params.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["width", "height", "label", "shape", "on", "inner", "secret"]);
        assert_eq!(params[0].kind, "int");
        assert_eq!(params[0].value, json!(10));
        assert_eq!(params[0].minimum, Some(1.0));
        assert_eq!(params[0].maximum, Some(100.0));
        assert_eq!(params[0].group, "Size");
        assert_eq!(params[0].description, "outer");
        assert_eq!(params[1].kind, "number");
        assert_eq!(params[1].step, Some(0.5));
        assert_eq!(params[2].kind, "string");
        assert_eq!(params[2].value, json!("box"));
        assert_eq!(params[3].kind, "enum");
        assert_eq!(params[3].options, vec![json!("round"), json!("square")]);
        assert_eq!(params[4].kind, "bool");
        assert_eq!(params[5].value, json!(106));
        assert!(params[6].hidden);
        assert!(params.iter().all(|p| p.name != "x" && p.name != "hole"));
    }

    #[test]
    fn overrides_reject_unknown_names_and_values_outside_the_script() {
        let known = parse_parameters(SAMPLE);
        let ok = overrides(&json!({"parameters": {"width": 12, "shape": "square", "on": false}}), &known, "scad.insert").unwrap();
        assert_eq!(ok.len(), 3);
        assert!(scad_assign("width", &json!(12)).unwrap() == "width=12");
        assert_eq!(scad_assign("label", &json!("a\"b")).unwrap(), r#"label="a\"b""#);
        let err = overrides(&json!({"parameters": {"width": 1000}}), &known, "scad.insert").unwrap_err().to_string();
        assert!(err.contains("outside"), "{err}");
        let err = overrides(&json!({"parameters": {"system": "rm"}}), &known, "scad.insert").unwrap_err().to_string();
        assert!(err.contains("not a parameter"), "{err}");
        assert!(find_program(Some("cmd.exe")).unwrap_err().contains("OpenSCAD"));
        assert!(quality(&json!({"fn": 48}), "scad.insert").unwrap()[0].0 == "$fn");
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn list_reads_a_folder_and_a_missing_program_inserts_nothing() {
        let dir = std::env::temp_dir().join(format!("solidcraft-scad-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("bracket.scad"), b"width = 10; // [1:20]\n").unwrap();
        let listed = list_native(&json!({"path": dir.to_string_lossy()})).unwrap();
        assert_eq!(listed["files"][0]["name"], "bracket.scad");
        assert_eq!(listed["files"][0]["parameters"][0]["name"], "width");
        let mut s = Session::default();
        let before = s.doc.features.len();
        let missing = dir.join("openscad.exe");
        let err = s
            .execute(
                "scad.insert",
                &json!({"path": dir.join("bracket.scad").to_string_lossy(), "openscad": missing.to_string_lossy(), "parameters": {"width": 8}}),
            )
            .unwrap_err()
            .to_string();
        assert!(err.contains("not found") || err.contains("OpenSCAD"), "{err}");
        assert_eq!(s.doc.features.len(), before);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn a_cube_stl_from_the_importer_is_an_editable_solid() {
        let c = |i: usize| [(i & 1) as f32 * 10.0, ((i >> 1) & 1) as f32 * 10.0, ((i >> 2) & 1) as f32 * 10.0];
        let quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];
        let mut tris = Vec::new();
        for q in quads {
            tris.push([c(q[0]), c(q[1]), c(q[2])]);
            tris.push([c(q[0]), c(q[2]), c(q[3])]);
        }
        let mut bytes = vec![b' '; 80];
        bytes.extend((tris.len() as u32).to_le_bytes());
        for t in &tris {
            for x in [0.0f32, 0.0, 0.0].iter().chain(t.iter().flatten()) {
                bytes.extend(x.to_le_bytes());
            }
            bytes.extend([0, 0]);
        }
        let mut s = Session::default();
        let mut parameters = serde_json::Map::new();
        parameters.insert("width".into(), json!(12));
        let out = import_stl(
            &mut s,
            &bytes,
            Some("Bracket"),
            solidcraft_doc::ScadSource { path: "bracket.scad".into(), parameters, openscad: None, segments: Some(32), fa: None, fs: None },
        )
        .unwrap();
        assert_eq!(out["component_name"], "Bracket");
        assert_eq!(out["solid"], json!(true));
        let id = out["feature"].as_u64().unwrap();
        let solidcraft_doc::FeatureKind::MeshImport { solid: Some(solid), .. } = &s.doc.feature(id).unwrap().kind else {
            panic!("the import is a solid");
        };
        let saved = solid.scad.as_ref().unwrap();
        assert_eq!(saved.path, "bracket.scad");
        assert_eq!(saved.parameters["width"], json!(12));
        assert_eq!(saved.segments, Some(32));
        let components = s.doc.components.len();
        let features = s.doc.features.len();
        let err = s.execute("scad.revise", &json!({"feature": id, "path": "bracket.scad"})).unwrap_err().to_string();
        assert!(err.contains("bracket.scad") || err.contains("OpenSCAD") || err.contains("not found"), "{err}");
        assert_eq!(s.doc.components.len(), components);
        assert_eq!(s.doc.features.len(), features);
        let name = out["bodies"][0].as_str().unwrap().to_string();
        {
            let state = s.model.state();
            let body = &state.body(&name).unwrap().body;
            assert!(!body.is_mesh(), "OpenSCAD output must be a solid, so faces can highlight");
            assert_eq!(body.faces(0.01).unwrap().len(), 6);
        }
        s.active_component = out["component"].as_u64().unwrap();
        s.execute("solid.fillet", &json!({"body": name, "edges": [[5.0, 0.0, 0.0]], "radius": 1})).unwrap();
        assert!(s.model.results.iter().all(|r| r.error.is_none()), "{:?}", s.model.results);
    }
}
