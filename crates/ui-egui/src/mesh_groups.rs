//! Coloured face groups while Convert STL is placing solids in the open design.
//! Confirm closes the dialog, and this preview with it.

use serde_json::Value;
use solidcraft_engine::Session;
use solidcraft_engine::geom::{Aabb3, Vec3};
use solidcraft_engine::kernel::{ConvertOptions, FaceGroup};

use crate::gpu::GpuScene;

pub struct Groups {
    pub scene: GpuScene,
    pub bounds: Aabb3,
}

/// The import command inside a preview job, including an edit's `timeline.redefine`.
fn import_params(cmds: &[(String, Value)]) -> Option<&Value> {
    cmds.iter().find_map(|(command, params)| {
        if command == "mesh.import" {
            return Some(params);
        }
        if command == "timeline.redefine" && params.get("command").and_then(Value::as_str) == Some("mesh.import") {
            return params.get("params");
        }
        None
    })
}

pub fn build(session: &Session, cmds: &[(String, Value)]) -> Option<Result<Groups, String>> {
    let params = import_params(cmds)?;
    Some(build_params(session, params))
}

fn build_params(session: &Session, params: &Value) -> Result<Groups, String> {
    let tolerance = eval(session, params.get("tolerance").and_then(Value::as_str).unwrap_or("0.05 mm"), solidcraft_engine::doc::expr::Kind::Length)?;
    let angle = eval(session, params.get("angle").and_then(Value::as_str).unwrap_or("15 deg"), solidcraft_engine::doc::expr::Kind::Angle)?;
    let mesh_percent = params.get("mesh").and_then(Value::as_f64).unwrap_or(100.0);
    let flag = |key: &str| params.get(key).and_then(Value::as_bool).unwrap_or(true);
    let options = ConvertOptions {
        tolerance,
        angle,
        mesh_percent,
        cylinders: flag("cylinders"),
        cones: flag("cones"),
        tori: flag("tori"),
        spheres: flag("spheres"),
        threads: flag("threads"),
        freeform: flag("freeform"),
    };
    let origin = origin(session, params.get("origin"))?;
    let mut paths = Vec::new();
    if let Some(one) = params.get("path").and_then(Value::as_str) {
        paths.push(one.to_string());
    }
    if let Some(more) = params.get("paths").and_then(Value::as_array) {
        for item in more {
            if let Some(path) = item.as_str() {
                paths.push(path.to_string());
            }
        }
    }
    let recursive = params.get("recursive").and_then(Value::as_bool).unwrap_or(false);
    let mut files = Vec::new();
    for path in &paths {
        expand_stls(path, recursive, &mut files)?;
    }
    files.sort();
    files.dedup();
    if files.is_empty() {
        return Err("no STL files".into());
    }
    let mut scene = GpuScene::default();
    let mut bounds = Aabb3::EMPTY;
    for path in &files {
        draw_file(&mut scene, &mut bounds, path, &options, origin)?;
    }
    if bounds.is_empty() {
        return Err("no triangles to preview".into());
    }
    Ok(Groups { scene, bounds })
}

fn draw_file(scene: &mut GpuScene, bounds: &mut Aabb3, path: &str, options: &ConvertOptions, origin: Vec3) -> Result<(), String> {
    let bytes = solidcraft_engine::io::vfs::read(path, 128 << 20).map_err(|e| e.to_string())?;
    let tris = solidcraft_engine::io::read_stl(&bytes).map_err(|e| e.to_string())?;
    if tris.is_empty() {
        return Err(format!("{path} has no triangles"));
    }
    let mut positions = Vec::with_capacity(tris.len().saturating_mul(3));
    let mut triangles = Vec::with_capacity(tris.len());
    for tri in &tris {
        let base = u32::try_from(positions.len()).map_err(|_| "too many vertices".to_string())?;
        for corner in tri {
            positions.push(Vec3::new(corner[0] as f64, corner[1] as f64, corner[2] as f64));
        }
        triangles.push([base, base + 1, base + 2]);
    }
    let body = solidcraft_engine::kernel::mesh_body(&positions, &triangles).map_err(|e| e.to_string())?;
    let groups = solidcraft_engine::kernel::label_mesh(&body, options).map_err(|e| e.to_string())?;
    for (triangle, group) in groups.triangles.iter().zip(&groups.groups) {
        let Some(corners) = triangle_corners(&groups.positions, triangle) else { continue };
        let offset = [corners[0] + origin, corners[1] + origin, corners[2] + origin];
        let Some(normal) = (offset[1] - offset[0]).cross(offset[2] - offset[0]).normalized() else { continue };
        let color = group_color(*group);
        for corner in offset {
            bounds.add(corner);
            scene.tri(corner.to_f32(), normal.to_f32(), color);
        }
    }
    Ok(())
}

fn triangle_corners(positions: &[Vec3], triangle: &[u32; 3]) -> Option<[Vec3; 3]> {
    Some([*positions.get(triangle[0] as usize)?, *positions.get(triangle[1] as usize)?, *positions.get(triangle[2] as usize)?])
}

pub fn group_color(group: FaceGroup) -> [u8; 4] {
    let rgb = match group {
        FaceGroup::Plane => [70, 130, 210],
        FaceGroup::Cylinder => [40, 190, 180],
        FaceGroup::Cone => [230, 140, 40],
        FaceGroup::Sphere => [230, 200, 50],
        FaceGroup::Freeform => [220, 90, 160],
        FaceGroup::LeftOut => [140, 90, 70],
    };
    [rgb[0], rgb[1], rgb[2], 255]
}

fn origin(session: &Session, value: Option<&Value>) -> Result<Vec3, String> {
    let mut coords = [0.0, 0.0, 0.0];
    if let Some(raw) = value.and_then(Value::as_array) {
        for (index, item) in raw.iter().take(3).enumerate() {
            let expr = if let Some(number) = item.as_f64() {
                format!("{number} mm")
            } else if let Some(text) = item.as_str() {
                text.to_string()
            } else {
                "0 mm".to_string()
            };
            coords[index] = eval(session, &expr, solidcraft_engine::doc::expr::Kind::Length)?;
        }
    }
    Ok(Vec3::new(coords[0], coords[1], coords[2]))
}

fn expand_stls(path: &str, recursive: bool, out: &mut Vec<String>) -> Result<(), String> {
    if out.len() > 200 {
        return Err("at most 200 STL files".into());
    }
    let file = std::path::Path::new(path);
    if file.is_file() {
        out.push(path.to_string());
        return Ok(());
    }
    if !file.is_dir() {
        return Err(format!("{path} was not found"));
    }
    let entries = std::fs::read_dir(file).map_err(|e| format!("{path}: {e}"))?;
    for entry in entries {
        let entry = entry.map_err(|e| format!("{path}: {e}"))?;
        let child = entry.path();
        let text = child.display().to_string();
        let stl = child.extension().is_some_and(|ext| ext.eq_ignore_ascii_case("stl"));
        if child.is_file() && stl {
            out.push(text);
        } else if recursive && child.is_dir() {
            expand_stls(&text, true, out)?;
        }
        if out.len() > 200 {
            return Err("at most 200 STL files".into());
        }
    }
    Ok(())
}

fn eval(session: &Session, expr: &str, kind: solidcraft_engine::doc::expr::Kind) -> Result<f64, String> {
    session.doc.eval(expr, kind).map_err(|e| e.to_string())
}
