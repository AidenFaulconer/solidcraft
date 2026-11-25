//! The Move window's main value: the one the canvas box shows and the keyboard types into.
//! It follows the move type, because each type uses a different value.
use serde_json::json;
use solidcraft_engine::Session;

use super::*;
use crate::Services;
use crate::dialogs::Dialog;

fn move_dialog(mode: usize) -> Dialog {
    let mut s = Session::default();
    s.execute("solid.box", &json!({"length": 20, "width": 20, "height": 20})).unwrap();
    let mut app = SolveApp::new(s, Services::default());
    app.start("solid.move");
    let mut d = app.dialog.clone().unwrap();
    d.kind = kind(mode);
    d.inputs = inputs(mode);
    if let Some(inp) = d.inputs.first_mut() {
        inp.items = vec![Sel::Body { name: "Body1".into() }];
    }
    d
}

#[test]
fn the_main_value_follows_the_move_type() {
    for mode in [FREE, TRANSLATE, POINT_TO_POSITION] {
        let mut d = move_dialog(mode);
        let got = d.primary().map(|p| (p.0, p.1));
        assert_eq!(got, Some(("X", ValueKind::Length)), "{} types a distance", TYPES[mode]);
    }

    let mut d = move_dialog(ROTATE);
    let got = d.primary().map(|p| (p.0, p.1));
    assert_eq!(got, Some(("Angle", ValueKind::Angle)), "Rotate types an angle, not an X distance");

    let mut d = move_dialog(POINT_TO_POINT);
    assert!(d.primary().is_none(), "Point to Point is two picks with nothing to type");

    let mut d = move_dialog(ALIGN);
    assert!(d.primary().is_none(), "Align Frames edits its offset beside the frame, not in the canvas box");
}

#[test]
fn the_main_value_writes_through_to_the_command() {
    let mut d = move_dialog(ROTATE);
    if let Some((_, _, value)) = d.primary() {
        *value = "90 deg".into();
    }
    let Kind::Move { angle, .. } = &d.kind else { panic!("not a move") };
    assert_eq!(angle, "90 deg", "the box edits the angle itself, not a copy of it");

    let mut d = move_dialog(TRANSLATE);
    if let Some((_, _, value)) = d.primary() {
        *value = "30 mm".into();
    }
    let Kind::Move { x, y, z, .. } = &d.kind else { panic!("not a move") };
    assert_eq!((x.as_str(), y.as_str(), z.as_str()), ("30 mm", "0 mm", "0 mm"), "the box edits X alone");
}

/// Free Move and Translate carry the triad. The other three have no manipulator yet, so their
/// value box is the only way to drive them: it must still have somewhere to sit.
#[test]
fn every_move_type_has_somewhere_to_put_its_value_box() {
    let mut s = Session::default();
    s.execute("solid.box", &json!({"length": 20, "width": 20, "height": 20})).unwrap();
    let app = SolveApp::new(s, Services::default());
    for mode in [FREE, TRANSLATE, ROTATE, POINT_TO_POINT, POINT_TO_POSITION, ALIGN] {
        let d = move_dialog(mode);
        let has_triad = triad(&app, &d).is_some();
        assert_eq!(has_triad, matches!(mode, FREE | TRANSLATE), "{} triad", TYPES[mode]);
        let at = triad(&app, &d).map(|t| t.center).or_else(|| crate::canvas::anchor(&app, &d).map(|(base, _)| base));
        assert!(at.is_some(), "{} has nowhere to put its value box", TYPES[mode]);
    }
}

/// Point to Point takes the origin point as From or To (#3696): a click on the origin dot picks
/// it, and the body moves from its corner onto the origin.
#[test]
fn point_to_point_takes_the_origin_point() {
    let mut h = crate::scenario::Harness::new();
    h.app.ui.show_origin = true;
    h.app.run("solid.box", json!({"length": 20, "width": 20, "height": 20})).unwrap();
    h.app.run("solid.move", json!({"bodies": ["Body1"], "translate": ["30 mm", "20 mm", "0 mm"]})).unwrap();
    h.app.cam.distance = 200.0;
    h.settle();
    let hi = h.app.session.model.state().body("Body1").unwrap().mesh().bounds().max;
    h.app.start("solid.move");
    {
        let d = h.app.dialog.as_mut().unwrap();
        d.kind = kind(POINT_TO_POINT);
        d.inputs = inputs(POINT_TO_POINT);
        d.inputs[0].items = vec![Sel::Body { name: "Body1".into() }];
        d.active = 1;
    }
    let corner = h.call("ui.at", json!({ "world": [hi.x, hi.y, hi.z] }))["result"].clone();
    h.step(&json!({ "click": corner })).unwrap();
    h.app.dialog.as_mut().unwrap().active = 2;
    let origin = h.call("ui.at", json!({ "world": [0, 0, 0] }))["result"].clone();
    h.step(&json!({ "click": origin })).unwrap();
    let d = h.app.dialog.clone().unwrap();
    assert_eq!(d.inputs[2].items, vec![Sel::Vertex { body: String::new(), point: Vec3::ZERO }], "the origin is the To point");
    let p = params(&h.app, &d).unwrap();
    assert_eq!(p["translate"], json!([mm(-hi.x), mm(-hi.y), mm(-hi.z)]), "the corner goes onto the origin");
}

fn frame_json(origin: [f64; 3], x: [f64; 3], z: [f64; 3]) -> serde_json::Value {
    json!({"origin": origin, "x": x, "z": z})
}

fn top_face(app: &SolveApp) -> usize {
    let mesh = app.session.model.state().body("Body1").unwrap().mesh().clone();
    mesh.tri_face
        .iter()
        .zip(&mesh.triangles)
        .find_map(|(face, t)| {
            let [a, b, c] = mesh.tri(t)?;
            (a.z > 19.0 && b.z > 19.0 && c.z > 19.0).then_some(*face as usize)
        })
        .unwrap()
}

/// The cursor over the middle of a face gets the centre; over a corner, that corner. Crossing
/// the nearest edge reverses the secondary axis. Both are measured on screen.
#[test]
fn align_frames_follow_the_cursor_across_a_face() {
    use crate::viewport::{Hit, projection};
    let mut s = Session::default();
    s.execute("solid.box", &json!({"length": 20, "width": 20, "height": 20})).unwrap();
    let mut app = SolveApp::new(s, Services::default());
    app.cam.target = Vec3::new(10.0, 10.0, 10.0);
    app.cam.distance = 300.0;
    app.cam.fov = 0.0;
    app.cam = app.cam.looking_from(Vec3::Z);
    let rect = egui::Rect::from_min_size(egui::pos2(0.0, 0.0), egui::vec2(800.0, 600.0));
    let proj = projection(&app, rect);
    let face = top_face(&app);
    let at = |cursor: Vec3| {
        let hit = Hit::Face { body: "Body1".into(), index: face, point: cursor };
        infer(&app.session, &proj, proj.to_screen(cursor).unwrap(), &hit).unwrap().frame
    };
    let centre = at(Vec3::new(10.0, 10.0, 20.0));
    assert!(centre.origin.dist(Vec3::new(10.0, 10.0, 20.0)) < 0.5, "centre {:?}", centre.origin);
    let corner = at(Vec3::new(20.0, 20.0, 20.0));
    assert!(corner.origin.dist(Vec3::new(20.0, 20.0, 20.0)) < 0.5, "corner {:?}", corner.origin);
    let inside = at(Vec3::new(19.0, 12.0, 20.0));
    let outside = at(Vec3::new(21.0, 12.0, 20.0));
    assert!(inside.origin.dist(outside.origin) < 0.5, "same corner {:?} {:?}", inside.origin, outside.origin);
    assert!(inside.x.dot(outside.x) < -0.5, "across the edge {:?} {:?}", inside.x, outside.x);
}

/// Two frames become one rigid move, including a turn of the frame after it was picked.
#[test]
fn align_frames_turn_a_body_onto_the_other_frame() {
    let mut s = Session::default();
    s.execute("solid.box", &json!({"length": 20, "width": 20, "height": 20})).unwrap();
    let mut app = SolveApp::new(s, Services::default());
    app.start("solid.move");
    let mut d = app.dialog.clone().unwrap();
    d.kind = kind(ALIGN);
    d.inputs = inputs(ALIGN);
    d.inputs[0].items = vec![Sel::Body { name: "Body1".into() }];
    d.extra.insert("from_frame".into(), frame_json([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    d.extra.insert("to_frame".into(), frame_json([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]));
    let p = params(&app, &d).unwrap();
    assert_eq!(p["angle"], json!("90 deg"), "the frames differ by a quarter turn");
    app.run("solid.move", p).unwrap();
    let mesh = app.session.model.state().body("Body1").unwrap().mesh();
    assert!(mesh.positions.iter().any(|q| q.dist(Vec3::new(-20.0, 0.0, 0.0)) < 1e-3), "the +Y corner turns onto -X");
    assert!(mesh.positions.iter().all(|q| q.dist(Vec3::new(20.0, 0.0, 0.0)) > 1.0), "the +X corner leaves");

    let mut d = app.dialog.clone().unwrap();
    d.kind = kind(ALIGN);
    d.inputs = inputs(ALIGN);
    d.inputs[0].items = vec![Sel::Body { name: "Body1".into() }];
    if let Kind::Move { from_x, .. } = &mut d.kind {
        *from_x = "10 mm".into();
    }
    d.extra.insert("from_frame".into(), frame_json([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    d.extra.insert("to_frame".into(), frame_json([10.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    let p = params(&app, &d).unwrap();
    assert_eq!(p["translate"], json!(["0 mm", "0 mm", "0 mm"]), "an offset of the from frame is the same shift as moving the to frame");
}
