use super::*;
use solidcraft_doc::{Document, FeatureKind, MeshData};
use solidcraft_geom::Vec3;

fn imported_box() -> Session {
    let body = solidcraft_kernel::box_solid(Vec3::ZERO, Vec3::new(10., 20., 30.)).unwrap();
    let mesh = body.tessellate(0.01).unwrap();
    let mut doc = Document::default();
    doc.add_feature(
        FeatureKind::MeshImport {
            file: "public-safe-box.stl".into(),
            solid: None,
            meshes: vec![MeshData {
                name: "Mesh".into(),
                positions: mesh.positions.iter().flat_map(|p| [p.x, p.y, p.z]).collect(),
                triangles: mesh.triangles.iter().flatten().copied().collect(),
                color: Some([0.2, 0.5, 0.8]),
            }],
        },
        Some("Imported mesh"),
    )
    .unwrap();
    Session::new(doc)
}

#[test]
fn insert_mesh_as_solid_reconstructs_stl_and_exports_step() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 10, "width": 20, "height": 30, "operation": "new", "body_name": "Stock"}));
    let path = std::env::temp_dir().join(format!("solidcraft-insert-mesh-solid-{}.stl", std::process::id()));
    run(&mut s, "file.save_mesh", json!({"path": &path, "bodies": ["Stock"]}));
    run(&mut s, "file.new", json!({"name": "Assembly"}));
    let r = run(&mut s, "file.insert_mesh_solid", json!({"path": path.to_string_lossy(), "tolerance": "0.05 mm"}));
    assert_eq!(r["solid"], json!(true));
    assert_eq!(r["converted"].as_array().map(|a| a.len()), Some(1));
    let name = r["bodies"][0].as_str().unwrap().to_owned();
    let state = s.model.state();
    let body = &state.body(&name).unwrap().body;
    assert!(!body.is_mesh());
    assert_eq!(body.faces(0.01).unwrap().len(), 6);
    let step = std::env::temp_dir().join(format!("solidcraft-insert-mesh-solid-{}.step", std::process::id()));
    run(&mut s, "file.export", json!({"path": &step, "bodies": [name]}));
    let text = std::fs::read_to_string(&step).unwrap();
    assert!(text.contains("ISO-10303-21"), "{text}");
    assert!(text.contains("MANIFOLD_SOLID_BREP") || text.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"), "{text}");
    run(&mut s, "solid.fillet", json!({"body": name, "edges": [[0, 10, 30]], "radius": 1}));
    assert!(s.model.results.iter().all(|r| r.error.is_none()), "{:?}", s.model.results);
    let _ = std::fs::remove_file(path);
    let _ = std::fs::remove_file(step);
}

#[test]
fn insert_mesh_convert_flag_matches_insert_mesh_solid() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 8, "width": 8, "height": 8, "operation": "new", "body_name": "Cube"}));
    let path = std::env::temp_dir().join(format!("solidcraft-insert-mesh-convert-{}.stl", std::process::id()));
    run(&mut s, "file.save_mesh", json!({"path": &path, "bodies": ["Cube"]}));
    run(&mut s, "file.new", json!({"name": "Via flag"}));
    let r = run(&mut s, "file.insert_mesh", json!({"path": path.to_string_lossy(), "convert": true, "tolerance": "0.05 mm"}));
    assert_eq!(r["solid"], json!(true));
    let name = r["bodies"][0].as_str().unwrap();
    assert!(!s.model.state().body(name).unwrap().body.is_mesh());
    let _ = std::fs::remove_file(path);
}

#[test]
fn insert_mesh_as_solid_refuses_unsupported_mesh_without_changing_the_design() {
    let mut s = Session::default();
    // A tetrahedron is closed but not a constant-section prism.
    let mesh = MeshData {
        name: "Tet".into(),
        positions: vec![0., 0., 0., 10., 0., 0., 5., 8., 0., 5., 3., 7.],
        triangles: vec![0, 1, 2, 0, 1, 3, 1, 2, 3, 0, 2, 3],
        color: None,
    };
    let bytes = {
        // Write a tiny ASCII STL of the tetrahedron.
        let mut out = String::from("solid tet\n");
        let p = |i: usize| [mesh.positions[i * 3], mesh.positions[i * 3 + 1], mesh.positions[i * 3 + 2]];
        for tri in mesh.triangles.chunks(3) {
            let (a, b, c) = (p(tri[0] as usize), p(tri[1] as usize), p(tri[2] as usize));
            out.push_str("facet normal 0 0 0\nouter loop\n");
            for v in [a, b, c] {
                out.push_str(&format!("vertex {} {} {}\n", v[0], v[1], v[2]));
            }
            out.push_str("endloop\nendfacet\n");
        }
        out.push_str("endsolid tet\n");
        out.into_bytes()
    };
    let path = std::env::temp_dir().join(format!("solidcraft-insert-mesh-solid-tet-{}.stl", std::process::id()));
    std::fs::write(&path, bytes).unwrap();
    let before = s.doc.to_json();
    assert!(s.execute("file.insert_mesh_solid", &json!({"path": path.to_string_lossy()})).is_err());
    assert_eq!(s.doc.to_json(), before);
    let _ = std::fs::remove_file(path);
}

#[test]
fn mesh_conversion_is_parametric_undoable_and_editable_after_cold_reopen() {
    let mut s = imported_box();
    run(&mut s, "parameters.add", json!({"name":"size","expression":"1","unit":""}));
    run(&mut s, "solid.scale", json!({"bodies":["Mesh"],"factor":"size","name":"Mesh scale"}));
    run(&mut s, "mesh.convert", json!({"body":"Mesh","tolerance":"0.01 mm","name":"Reconstructed prism"}));
    let check = |s: &Session| {
        let state = s.model.state();
        let body = &state.body("Mesh").unwrap().body;
        assert!(!body.is_mesh());
        assert_eq!(body.faces(0.01).unwrap().len(), 6);
        assert_eq!(body.color(), Some([0.2, 0.5, 0.8]));
        assert!(s.model.results.iter().all(|r| r.error.is_none()), "{:?}", s.model.results);
    };
    check(&s);
    assert!((volume(&mut s) - 6000.).abs() < 1e-6);
    run(&mut s, "edit.undo", json!({}));
    assert!(s.model.state().body("Mesh").unwrap().body.is_mesh());
    run(&mut s, "edit.redo", json!({}));
    check(&s);
    run(&mut s, "parameters.change", json!({"name":"size","expression":"2"}));
    check(&s);
    assert!((volume(&mut s) - 48000.).abs() < 1e-5);
    let mut cold = Session::new(Document::from_json(&s.doc.to_json()).unwrap());
    check(&cold);
    assert!((volume(&mut cold) - 48000.).abs() < 1e-5);
    run(&mut cold, "solid.fillet", json!({"body":"Mesh","edges":[[0,20,60]],"radius":2}));
    assert!(volume(&mut cold) < 48000.);
    assert!(cold.model.results.iter().all(|r| r.error.is_none()));
}

#[test]
fn failed_mesh_conversions_leave_the_design_unchanged() {
    let mut s = imported_box();
    let before = s.doc.to_json();
    for params in [
        json!({}),
        json!({"body":"Missing"}),
        json!({"body":"Mesh","tolerance":0}),
        json!({"body":"Mesh","tolerance":"unknown"}),
        json!({"body":"Mesh","tolerance":"2 mm"}),
    ] {
        assert!(s.execute("mesh.convert", &params).is_err());
        assert_eq!(s.doc.to_json(), before);
        assert!(s.model.state().body("Mesh").unwrap().body.is_mesh());
    }
}

#[test]
fn convert_uses_the_selected_mesh_body_when_body_is_omitted() {
    let mut s = imported_box();
    s.selection = vec![crate::Sel::Body { name: "Mesh".into() }];
    run(&mut s, "mesh.convert", json!({"tolerance":"0.01 mm"}));
    let state = s.model.state();
    let body = &state.body("Mesh").unwrap().body;
    assert!(!body.is_mesh());
    assert_eq!(body.faces(0.01).unwrap().len(), 6);
}

#[test]
fn convert_without_body_or_selection_is_refused() {
    let mut s = imported_box();
    let before = s.doc.to_json();
    assert!(s.execute("mesh.convert", &json!({"tolerance":"0.01 mm"})).is_err());
    assert_eq!(s.doc.to_json(), before);
    assert!(s.model.state().body("Mesh").unwrap().body.is_mesh());
}

#[test]
fn an_explicit_body_wins_over_the_selection() {
    let mut s = imported_box();
    s.selection = vec![crate::Sel::Body { name: "Missing".into() }];
    run(&mut s, "mesh.convert", json!({"body":"Mesh","tolerance":"0.01 mm"}));
    assert!(!s.model.state().body("Mesh").unwrap().body.is_mesh());
}

#[test]
fn exported_binary_stl_with_a_bore_reconstructs_analytic_faces() {
    let mut s = Session::default();
    run(&mut s, "sketch.create", json!({"plane":"XY","name":"Section"}));
    run(&mut s, "sketch.line", json!({"points":[[0,0],[40,0],[40,20],[20,20],[20,30],[0,30]],"closed":true}));
    run(&mut s, "sketch.circle.center", json!({"center":[10,10],"radius":4}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch":"Section","profiles":[{"point":[30,10]}],"distance":15,"body_name":"Bracket"}));
    let path = std::env::temp_dir().join(format!("solidcraft-mesh-convert-{}.stl", std::process::id()));
    run(&mut s, "file.save_mesh", json!({"path":path}));
    run(&mut s, "doc.open", json!({"path":path}));
    let name = s.model.state().bodies[0].name.clone();
    run(&mut s, "mesh.convert", json!({"body":name,"tolerance":"0.05 mm"}));
    let state = s.model.state();
    let body = &state.bodies[0].body;
    assert!(!body.is_mesh());
    assert!(body.validity().is_empty());
    assert_eq!(solidcraft_kernel::measure(body).unwrap().merged.face_types.get("cylinder"), Some(&1));
    assert_eq!(body.faces(0.01).unwrap().len(), 10);
    std::fs::remove_file(path).unwrap();
}

/// #784: STL is closed as written where faces meet (a fillet, a bored hole): every edge of the
/// file, keyed by its exact f32 ends, is shared by exactly two triangles, as in 3MF.
#[test]
fn stl_export_is_closed_where_faces_meet() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 30, "width": 20, "height": 10, "operation": "new", "body_name": "BoxFillet"}));
    run(&mut s, "solid.fillet", json!({"body": "BoxFillet", "edges": [[15, 0, 10]], "radius": 2}));
    run(&mut s, "solid.cylinder", json!({"diameter": 30, "height": 12, "base": [60, 0, 0], "operation": "new", "body_name": "CylHole"}));
    run(&mut s, "solid.hole", json!({"position": [60, 0, 12], "diameter": 6}));
    let state = s.model.state();
    for name in ["BoxFillet", "CylHole"] {
        let body = &state.bodies.iter().find(|b| b.name == name).unwrap().body;
        let expected = solidcraft_kernel::measure(body).unwrap().volume;
        for format in [solidcraft_io::Format::StlBinary, solidcraft_io::Format::StlAscii] {
            let bytes = solidcraft_io::export(&state, &[name.to_string()], format, name).unwrap();
            let tris = solidcraft_io::read_stl(&bytes).unwrap();
            let key = |p: [f32; 3]| p.map(f32::to_bits);
            let mut edges = std::collections::HashMap::new();
            for t in &tris {
                for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                    let (a, b) = (key(a), key(b));
                    *edges.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
                }
            }
            let open = edges.values().filter(|n| **n != 2).count();
            assert_eq!(open, 0, "{name} {format:?}: {open} edges not shared by exactly two triangles");
            let p = |v: [f32; 3]| Vec3::new(v[0] as f64, v[1] as f64, v[2] as f64);
            let v: f64 = tris.iter().map(|t| p(t[0]).dot(p(t[1]).cross(p(t[2]))) / 6.0).sum();
            assert!((v - expected).abs() <= expected * 1e-3, "{name} {format:?}: STL volume {v}, body {expected}");
            let mf = solidcraft_io::export(&state, &[name.to_string()], solidcraft_io::Format::ThreeMf, name).unwrap();
            let (objs, _) = solidcraft_io::read_3mf(&mf).unwrap();
            assert_eq!(tris.len(), objs.iter().map(|o| o.triangles.len()).sum::<usize>(), "{name} {format:?}: STL and 3MF triangles");
        }
    }
}

#[test]
fn save_mesh_rejects_unknown_format_and_slicer_without_writing() {
    let mut s = imported_box();
    let path = std::env::temp_dir().join(format!("solvecraft-save-mesh-{}.stl", std::process::id()));
    let _ = std::fs::remove_file(&path);
    for params in [json!({"path":path,"format":"nope"}), json!({"path":path,"slicer":"no such slicer anywhere"})] {
        assert!(s.execute("file.save_mesh", &params).is_err());
        assert!(!path.exists());
    }
}

#[test]
fn save_mesh_writes_stl_and_3mf() {
    let mut s = imported_box();
    let dir = std::env::temp_dir();
    for (ext, format) in [("stl", None), ("3mf", None), ("dat", Some("3mf"))] {
        let path = dir.join(format!("solvecraft-save-mesh-{}.{ext}", std::process::id()));
        let mut params = json!({"path": path});
        if let Some(f) = format {
            params["format"] = json!(f);
        }
        s.execute("file.save_mesh", &params).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        // A 3MF file is a zip archive; a binary STL starts with its 80-byte header.
        assert_eq!(bytes.starts_with(b"PK"), ext == "3mf" || format == Some("3mf"), "{ext}");
        std::fs::remove_file(path).unwrap();
    }
}

#[test]
fn mesh_convert_records_recognition_choices() {
    let mut s = imported_box();
    run(
        &mut s,
        "mesh.convert",
        json!({"body":"Mesh","tolerance":"0.01 mm","angle":"2 deg","mesh":80,"cylinders":false,"cones":false,"tori":false,"spheres":false,"threads":false,"freeform":true}),
    );
    let kind = s.doc.features.iter().find_map(|f| match &f.kind {
        FeatureKind::MeshConvert { angle, mesh, cylinders, cones, tori, spheres, threads, freeform, .. } => {
            Some((angle.as_str().to_string(), *mesh, *cylinders, *cones, *tori, *spheres, *threads, *freeform))
        }
        _ => None,
    });
    assert_eq!(kind, Some(("2 deg".into(), 80.0, false, false, false, false, false, true)));
    assert!(!s.model.state().body("Mesh").unwrap().body.is_mesh());
    assert!(s.execute("mesh.convert", &json!({"body":"Mesh","mesh":0})).is_err());
}

#[test]
fn installed_slicers_are_listed() {
    let mut s = imported_box();
    let r = s.execute("file.slicers", &json!({})).unwrap();
    assert!(r["slicers"].is_array());
}

#[test]
fn convert_files_writes_one_step_per_stl_and_leaves_the_design() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 10, "width": 20, "height": 30, "operation": "new", "body_name": "Stock"}));
    let dir = std::env::temp_dir().join(format!("solidcraft-stl-folder-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(dir.join("nested")).unwrap();
    let stl = dir.join("a.stl");
    run(&mut s, "file.save_mesh", json!({"path": &stl, "bodies": ["Stock"]}));
    std::fs::copy(&stl, dir.join("nested").join("b.stl")).unwrap();
    std::fs::write(dir.join("note.txt"), b"not a mesh").unwrap();
    let out = dir.join("step");
    std::fs::create_dir_all(&out).unwrap();
    let features = s.doc.features.len();
    let flat = s.execute("mesh.convert_files", &json!({"path": &dir, "output": &out, "angle": "15 deg"})).unwrap();
    assert_eq!(flat["written"].as_array().map(Vec::len), Some(1), "{flat}");
    assert_eq!(s.doc.features.len(), features);
    let step = std::fs::read(out.join("a.step")).unwrap();
    assert!(step.windows(12).any(|window| window == b"ISO-10303-21"), "STEP header missing");
    let both = s.execute("mesh.convert_files", &json!({"path": &dir, "output": &out, "recursive": true})).unwrap();
    assert_eq!(both["written"].as_array().map(Vec::len), Some(2), "{both}");
    assert!(out.join("b.step").is_file());
    assert!(s.execute("mesh.convert_files", &json!({"path": dir.join("note.txt")})).is_err());
    assert!(s.execute("mesh.convert_files", &json!({})).is_err());
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn import_stl_places_a_solid_in_the_design() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 10, "width": 20, "height": 30, "operation": "new", "body_name": "Stock"}));
    let path = std::env::temp_dir().join(format!("solidcraft-import-stl-{}.stl", std::process::id()));
    run(&mut s, "file.save_mesh", json!({"path": &path, "bodies": ["Stock"]}));
    run(&mut s, "file.new", json!({"name": "Placed"}));
    let r = run(&mut s, "mesh.import", json!({"path": path.to_string_lossy(), "origin": ["100 mm", "0 mm", "0 mm"], "tolerance": "0.05 mm"}));
    assert!(r["feature"].as_u64().is_some(), "{r}");
    let state = s.model.state();
    let body = state.bodies.iter().find(|b| !b.body.is_mesh()).expect("a solid");
    let mesh = body.body.tessellate(0.2).unwrap();
    let min_x = mesh.positions.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    assert!(min_x > 90.0, "origin should shift the solid, min x {min_x}");
    let solid = s.doc.features.iter().find_map(|f| match &f.kind {
        FeatureKind::MeshImport { solid: Some(solid), .. } => Some(solid),
        _ => None,
    });
    assert_eq!(solid.map(|item| item.origin[0].as_str()), Some("100 mm"));
    let _ = std::fs::remove_file(path);
}
