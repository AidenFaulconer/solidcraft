//! Move/Copy: the move types (Free Move, Translate, Rotate, Point to Point, Point to Position,
//! Align Frames), their rows and inputs, the triad on the canvas and the solid.move they make.
//! Nothing moves until a value is typed, the triad is dragged, or Align Frames has two frames.
//!
//! Align Frames places a triad on each pick. On a face it sits on the face centre or the corner
//! nearest the cursor on screen, and its secondary axis follows the cursor around the nearest
//! edge. After the pick, the frame stays put and can be offset and turned.

use crate::selection::SelectionExt as _;
use crate::ui_kit::{choice_row, expr_row};
use serde_json::{Value, json};
use solidcraft_engine::Sel;
use solidcraft_engine::Session;
use solidcraft_engine::doc::expr::Kind as ValueKind;
use solidcraft_engine::geom::{self, AlignFrame, Vec2, Vec3};

use crate::SolveApp;
use crate::dialogs::{Dialog, Kind};
use crate::gizmo::{self, Change, Drag};
use crate::selection::{AXES, BODIES, EDGES, FACES, ORIGIN, SelInput, VERTICES};

pub const TYPES: [&str; 6] = ["Free Move", "Translate", "Rotate", "Point to Point", "Point to Position", "Align Frames"];
pub const FREE: usize = 0;
pub const TRANSLATE: usize = 1;
pub const ROTATE: usize = 2;
pub const POINT_TO_POINT: usize = 3;
pub const POINT_TO_POSITION: usize = 4;
pub const ALIGN: usize = 5;
const POINTS: u16 = VERTICES | FACES | EDGES | ORIGIN;

/// A fresh Move dialog of a type (all values zero).
pub fn kind(mode: usize) -> Kind {
    let z = || "0 mm".to_string();
    let a = || "0 deg".to_string();
    Kind::Move {
        mode,
        x: z(),
        y: z(),
        z: z(),
        rx: a(),
        ry: a(),
        rz: a(),
        angle: a(),
        from_x: z(),
        from_y: z(),
        from_z: z(),
        from_turn: a(),
        from_flip: false,
        to_x: z(),
        to_y: z(),
        to_z: z(),
        to_turn: a(),
        to_flip: false,
    }
}

/// The selection inputs of a move type (the bodies first).
pub fn inputs(mode: usize) -> Vec<SelInput> {
    let mut v = vec![SelInput::new("Bodies", BODIES, true)];
    match mode {
        FREE => v.push(SelInput::new("Pivot", POINTS, false)),
        ROTATE => v.push(SelInput::new("Axis", AXES | EDGES, false)),
        POINT_TO_POINT => {
            v.push(SelInput::new("From", POINTS, false));
            v.push(SelInput::new("To", POINTS, false));
        }
        POINT_TO_POSITION => v.push(SelInput::new("Point", POINTS, false)),
        ALIGN => {
            v.push(SelInput::new("From Frame", POINTS, false));
            v.push(SelInput::new("To Frame", POINTS, false));
        }
        _ => {}
    }
    v
}

/// The Move Type row (above the inputs); switching keeps the bodies and resets the values.
pub fn type_row(d: &mut Dialog, ui: &mut egui::Ui) {
    let Kind::Move { mode, .. } = d.kind else { return };
    let mut m = mode;
    choice_row(ui, "Move Type", "move_type", &mut m, &TYPES);
    if m != mode {
        let bodies = d.inputs.first().map(|i| i.items.clone()).unwrap_or_default();
        d.kind = kind(m);
        d.inputs = inputs(m);
        if let Some(i) = d.inputs.first_mut() {
            i.items = bodies;
        }
        d.extra.remove("pivot");
        d.extra.remove("from_frame");
        d.extra.remove("to_frame");
        d.active = if bodies_empty(d) { 0 } else { 1.min(d.inputs.len() - 1) };
    }
}

fn bodies_empty(d: &Dialog) -> bool {
    d.inputs.first().is_none_or(|i| i.items.is_empty())
}

/// The value rows after the inputs; true when Enter was pressed in one.
pub fn rows(ui: &mut egui::Ui, d: &mut Dialog) -> bool {
    let mode = match &d.kind {
        Kind::Move { mode, .. } => *mode,
        _ => return false,
    };
    let mut enter = false;
    match mode {
        FREE | TRANSLATE => {
            let Kind::Move { mode, x, y, z, rx, ry, rz, .. } = &mut d.kind else { return false };
            enter |= expr_row(ui, "X Distance", x);
            enter |= expr_row(ui, "Y Distance", y);
            enter |= expr_row(ui, "Z Distance", z);
            if *mode == FREE {
                enter |= expr_row(ui, "X Angle", rx);
                enter |= expr_row(ui, "Y Angle", ry);
                enter |= expr_row(ui, "Z Angle", rz);
            }
        }
        ROTATE => {
            let Kind::Move { angle, .. } = &mut d.kind else { return false };
            enter |= expr_row(ui, "Angle", angle);
        }
        POINT_TO_POSITION => {
            let Kind::Move { x, y, z, .. } = &mut d.kind else { return false };
            enter |= expr_row(ui, "X", x);
            enter |= expr_row(ui, "Y", y);
            enter |= expr_row(ui, "Z", z);
        }
        ALIGN => enter |= frame_rows(ui, d),
        _ => {}
    }
    enter
}

/// Offset and turn for a frame that has already been picked. Hidden until then, and foldable.
fn frame_rows(ui: &mut egui::Ui, d: &mut Dialog) -> bool {
    let mut enter = false;
    if d.inputs.get(1).is_none_or(|i| i.items.is_empty()) && d.inputs.get(2).is_none_or(|i| i.items.is_empty()) {
        crate::ui_kit::text(
            ui,
            "Pick a face, edge or vertex for each frame. It follows the cursor, then stays so you can offset and turn it.",
            crate::ui_kit::Text::Hint("align_frames"),
        );
    }
    enter |= frame_section(ui, d, 1, "move_from_frame", "From frame");
    enter |= frame_section(ui, d, 2, "move_to_frame", "To frame");
    enter
}

fn frame_section(ui: &mut egui::Ui, d: &mut Dialog, slot: usize, id: &str, title: &str) -> bool {
    if d.inputs.get(slot).is_none_or(|i| i.items.is_empty()) {
        return false;
    }
    crate::ui_kit::section(ui, id, title, true, |ui| {
        let Kind::Move { from_x, from_y, from_z, from_turn, from_flip, to_x, to_y, to_z, to_turn, to_flip, .. } = &mut d.kind else {
            return false;
        };
        let (x, y, z, turn, flip) = if slot == 1 { (from_x, from_y, from_z, from_turn, from_flip) } else { (to_x, to_y, to_z, to_turn, to_flip) };
        let mut enter = false;
        enter |= expr_row(ui, "Offset X", x);
        enter |= expr_row(ui, "Offset Y", y);
        enter |= expr_row(ui, "Offset Z", z);
        enter |= expr_row(ui, "Turn", turn);
        if crate::ui_kit::checkbox_row(ui, id, "Flip primary", *flip) {
            *flip = !*flip;
        }
        enter
    })
    .unwrap_or(false)
}

fn sel_point(s: Option<&Sel>) -> Option<Vec3> {
    match s? {
        Sel::Vertex { point, .. } | Sel::Face { point, .. } | Sel::Edge { point, .. } => Some(*point),
        _ => None,
    }
}

fn items(d: &Dialog, i: usize) -> &[Sel] {
    crate::selection::items(&d.inputs, i)
}

/// The centre of the picked bodies where they are before this move.
fn bodies_center(app: &SolveApp, d: &Dialog) -> Option<Vec3> {
    let st = match d.editing {
        Some((id, _)) => app.session.model.state_before(id),
        None => app.session.model.state(),
    };
    let mut bb: Option<(Vec3, Vec3)> = None;
    for n in items(d, 0).body_names() {
        let Some(b) = st.body(&n) else { continue };
        let x = b.mesh().bounds();
        bb = Some(match bb {
            None => (x.min, x.max),
            Some((lo, hi)) => (
                Vec3::new(lo.x.min(x.min.x), lo.y.min(x.min.y), lo.z.min(x.min.z)),
                Vec3::new(hi.x.max(x.max.x), hi.y.max(x.max.y), hi.z.max(x.max.z)),
            ),
        });
    }
    bb.map(|(lo, hi)| (lo + hi) * 0.5)
}

/// What a Free Move turns about: the picked pivot, else the kept one (editing), else the
/// bodies' centre.
fn pivot(app: &SolveApp, d: &Dialog) -> Option<Vec3> {
    if let Some(p) = sel_point(items(d, 1).first()) {
        return Some(p);
    }
    if let Some(a) = d.extra.get("pivot").and_then(|v| serde_json::from_value::<[f64; 3]>(v.clone()).ok()) {
        return Some(Vec3::new(a[0], a[1], a[2]));
    }
    bodies_center(app, d)
}

fn len(app: &SolveApp, s: &str) -> Result<f64, String> {
    app.session.doc.eval(s, ValueKind::Length).map_err(|e| e.to_string()).and_then(|v| if v.is_finite() { Ok(v) } else { Err("not a number".into()) })
}

fn ang(app: &SolveApp, s: &str) -> Result<f64, String> {
    app.session.doc.eval(s, ValueKind::Angle).map_err(|e| e.to_string()).and_then(|v| if v.is_finite() { Ok(v) } else { Err("not a number".into()) })
}

fn mm(v: f64) -> String {
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    format!("{} mm", if s == "-0" || s.is_empty() { "0" } else { s })
}

fn deg(v: f64) -> String {
    let s = format!("{:.4}", v.to_degrees());
    let s = s.trim_end_matches('0').trim_end_matches('.');
    format!("{} deg", if s == "-0" || s.is_empty() { "0" } else { s })
}

/// The solid.move parameters (bodies, translate, axis, angle).
pub fn params(app: &SolveApp, d: &Dialog) -> Result<Value, String> {
    let Kind::Move { mode, x, y, z, rx, ry, rz, angle, .. } = &d.kind else { return Err("not a move".into()) };
    let bodies = items(d, 0).body_names();
    if bodies.is_empty() {
        return Err("select bodies first".into());
    }
    let need_point = |i: usize, what: &str| sel_point(items(d, i).first()).ok_or(format!("select {what} first"));
    let num = |t: Vec3| json!([mm(t.x), mm(t.y), mm(t.z)]);
    let mut p = json!({ "bodies": bodies });
    match *mode {
        TRANSLATE => p["translate"] = json!([x, y, z]),
        FREE => {
            let a = [ang(app, rx)?, ang(app, ry)?, ang(app, rz)?];
            let turned: Vec<usize> = (0..3).filter(|i| a[*i].abs() > 1e-12).collect();
            if turned.is_empty() {
                p["translate"] = json!([x, y, z]);
                // A rotation kept from an edited move about another axis.
                for k in ["axis", "angle"] {
                    if let Some(v) = d.extra.get(k) {
                        p[k] = v.clone();
                    }
                }
            } else {
                let piv = pivot(app, d).ok_or("select bodies first")?;
                let r = gizmo::mul(&gizmo::mul(&gizmo::rotation(Vec3::Z, a[2]), &gizmo::rotation(Vec3::Y, a[1])), &gizmo::rotation(Vec3::X, a[0]));
                let (axis, th) = gizmo::axis_angle(&r);
                match turned.as_slice() {
                    // One turn about the origin: the typed values stay as they are.
                    [i] if piv.len() < 1e-9 => {
                        p["translate"] = json!([x, y, z]);
                        p["axis"] = json!([[1, 0, 0], [0, 1, 0], [0, 0, 1]][*i]);
                        p["angle"] = json!([rx, ry, rz][*i]);
                    }
                    _ => {
                        let t = Vec3::new(len(app, x)?, len(app, y)?, len(app, z)?);
                        p["translate"] = num(piv - gizmo::apply(&r, piv) + t);
                        p["axis"] = json!([axis.x, axis.y, axis.z]);
                        p["angle"] = json!(deg(th));
                    }
                }
            }
        }
        ROTATE => {
            let (o, a) = items(d, 1).first().and_then(|s| axis_line(app, s)).ok_or("select an axis or a straight edge first")?;
            let th = ang(app, angle)?;
            let r = gizmo::rotation(a, th);
            p["translate"] = num(o - gizmo::apply(&r, o));
            p["axis"] = json!([a.x, a.y, a.z]);
            p["angle"] = json!(angle);
        }
        POINT_TO_POINT => {
            let (from, to) = (need_point(1, "the point to move from")?, need_point(2, "the point to move to")?);
            p["translate"] = num(to - from);
        }
        ALIGN => {
            let from = placed_frame(app, d, 1, "from")?;
            let to = placed_frame(app, d, 2, "to")?;
            let place = from.map_onto(to).ok_or("the frames do not define a move")?;
            p["translate"] = num(place.translate);
            if place.angle.abs() > 1e-12 {
                p["axis"] = json!([place.axis.x, place.axis.y, place.axis.z]);
                p["angle"] = json!(deg(place.angle));
            }
        }
        _ => {
            let from = need_point(1, "the point to move")?;
            let at = Vec3::new(len(app, x)?, len(app, y)?, len(app, z)?);
            p["translate"] = num(at - from);
        }
    }
    if d.extra.get("copy").and_then(Value::as_bool).unwrap_or(false) {
        p["copy"] = json!(true);
    }
    Ok(p)
}

/// An axis line: origin axes and sketch lines, or a straight body edge.
fn axis_line(app: &SolveApp, s: &Sel) -> Option<(Vec3, Vec3)> {
    if let Some(a) = crate::dialogs::axis_of(app, s) {
        return Some(a);
    }
    let Sel::Edge { body, index, .. } = s else { return None };
    let st = app.session.model.state();
    let m = st.body(body)?.mesh();
    let e = m.edges.get(*index)?;
    let (a, b) = (*e.first()?, *e.last()?);
    let dir = (b - a).normalized()?;
    // Straight: every point on the chord.
    e.iter().all(|p| (*p - a).cross(dir).len() < 1e-6 * (1.0 + (b - a).len())).then_some((a, dir))
}

/// Where the triad sits: the pivot carried by the move so far (Free Move and Translate only).
pub fn triad(app: &SolveApp, d: &Dialog) -> Option<gizmo::Triad> {
    let Kind::Move { mode, x, y, z, .. } = &d.kind else { return None };
    if !matches!(*mode, FREE | TRANSLATE) || bodies_empty(d) {
        return None;
    }
    let t = Vec3::new(len(app, x).unwrap_or(0.0), len(app, y).unwrap_or(0.0), len(app, z).unwrap_or(0.0));
    Some(gizmo::Triad { center: pivot(app, d)? + t, translate: true, rotate: *mode == FREE })
}

/// Apply a triad drag to the dialog's values (added to the values when the drag began).
pub fn drag(app: &SolveApp, d: &mut Dialog, g: Drag) {
    let Kind::Move { x, y, z, rx, ry, rz, .. } = &d.kind else { return };
    if g.started || !d.extra.contains_key("drag0") {
        let v = [len(app, x), len(app, y), len(app, z), ang(app, rx), ang(app, ry), ang(app, rz)].map(|r| r.unwrap_or(0.0));
        d.extra.insert("drag0".into(), json!(v));
    }
    let v0: [f64; 6] = d.extra.get("drag0").and_then(|v| serde_json::from_value(v.clone()).ok()).unwrap_or_default();
    let Kind::Move { x, y, z, rx, ry, rz, .. } = &mut d.kind else { return };
    match g.change {
        Change::Translate(t) => {
            *x = mm(v0[0] + t.x);
            *y = mm(v0[1] + t.y);
            *z = mm(v0[2] + t.z);
        }
        Change::Rotate { axis, angle } => {
            let slot = match axis {
                0 => rx,
                1 => ry,
                _ => rz,
            };
            *slot = deg(v0.get(3 + axis.min(2)).copied().unwrap_or(0.0) + angle);
        }
    }
    if g.done {
        d.extra.remove("drag0");
    }
}

/// The dialog for editing a move: Free Move with its translation and, about a world axis
/// through the origin, its angle (another axis is kept as it is).
pub fn for_feature(
    d: &mut Dialog,
    bodies: &[String],
    translate: &[solidcraft_engine::doc::Expr; 3],
    axis: Option<Vec3>,
    angle: Option<&solidcraft_engine::doc::Expr>,
    copy: bool,
) {
    let [x, y, z] = translate.clone().map(String::from);
    let mut k = kind(FREE);
    if let Kind::Move { x: kx, y: ky, z: kz, rx, ry, rz, .. } = &mut k {
        (*kx, *ky, *kz) = (x, y, z);
        if let Some(a) = angle {
            let dir = axis.unwrap_or(Vec3::Z);
            let world = [Vec3::X, Vec3::Y, Vec3::Z].iter().position(|w| dir.cross(*w).len() < 1e-9 && dir.dot(*w) > 0.0);
            match world {
                Some(0) => *rx = a.to_string(),
                Some(1) => *ry = a.to_string(),
                Some(_) => *rz = a.to_string(),
                None => {
                    d.extra.insert("axis".into(), json!([dir.x, dir.y, dir.z]));
                    d.extra.insert("angle".into(), json!(a));
                }
            }
            // The stored move turns about the origin.
            d.extra.insert("pivot".into(), json!([0.0, 0.0, 0.0]));
        }
    }
    d.kind = k;
    d.inputs = inputs(FREE);
    if let Some(inp) = d.inputs.first_mut() {
        inp.items = bodies.iter().map(|n| Sel::Body { name: n.clone() }).collect();
    }
    if copy {
        d.extra.insert("copy".into(), json!(true));
    }
}

/// Triads to draw: frames already picked (with their offset and turn) and, while choosing, the
/// one under the cursor plus the snap points it is choosing among.
pub struct FramePreview {
    pub frames: Vec<AlignFrame>,
    pub marks: Vec<Vec3>,
}

pub fn preview(app: &SolveApp, d: &Dialog, proj: &crate::viewport::Proj) -> FramePreview {
    let mut out = FramePreview { frames: Vec::new(), marks: Vec::new() };
    if !aligning(d) {
        return out;
    }
    for slot in [1, 2] {
        if let Some(frame) = placed_frame(app, d, slot, "").ok() {
            out.frames.push(frame);
        }
    }
    if matches!(d.active, 1 | 2)
        && let Some(cursor) = app.viewport.mouse
        && let Some(hit) = app.viewport.hover.as_ref()
        && let Some(inferred) = infer(&app.session, proj, cursor, hit)
    {
        let fresh = out.frames.iter().all(|f| f.origin.dist(inferred.frame.origin) > 0.2);
        if fresh {
            out.frames.push(inferred.frame);
        }
        out.marks = inferred.marks;
    }
    out
}

/// A world point the hit names, used when the pointer position was not recorded.
pub fn hit_point(hit: &crate::viewport::Hit) -> Option<Vec3> {
    use crate::viewport::Hit;
    match hit {
        Hit::Face { point, .. } | Hit::Vertex { point, .. } | Hit::Plane { point, .. } => Some(*point),
        Hit::Edge { mid, .. } => Some(*mid),
        _ => None,
    }
}

/// Remember the frame under the cursor for the input that was just picked. A second click that
/// clears the input forgets it.
pub fn capture(session: &Session, d: &mut Dialog, slot: usize, hit: &crate::viewport::Hit, proj: &crate::viewport::Proj, cursor: Option<egui::Pos2>) {
    if !aligning(d) {
        return;
    }
    let Some(key) = frame_key(slot) else { return };
    if d.inputs.get(slot).is_none_or(|i| i.items.is_empty()) {
        d.extra.remove(key);
        return;
    }
    let Some(cursor) = cursor else { return };
    let Some(inferred) = infer(session, proj, cursor, hit) else { return };
    d.extra.insert(key.into(), frame_value(&inferred.frame));
}

/// Drop a frame whose pick was cleared.
pub fn forget_frame(d: &mut Dialog, slot: usize) {
    if aligning(d)
        && let Some(key) = frame_key(slot)
    {
        d.extra.remove(key);
    }
}

fn aligning(d: &Dialog) -> bool {
    matches!(d.kind, Kind::Move { mode: ALIGN, .. })
}

fn frame_key(slot: usize) -> Option<&'static str> {
    match slot {
        1 => Some("from_frame"),
        2 => Some("to_frame"),
        _ => None,
    }
}

fn frame_value(frame: &AlignFrame) -> Value {
    let p = |v: Vec3| json!([v.x, v.y, v.z]);
    json!({"origin": p(frame.origin), "x": p(frame.x), "z": p(frame.z)})
}

fn read3(v: &Value) -> Option<Vec3> {
    let a = v.as_array()?;
    let p = Vec3::new(a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?);
    p.is_finite().then_some(p)
}

fn load_frame(d: &Dialog, slot: usize) -> Option<AlignFrame> {
    let v = d.extra.get(frame_key(slot)?)?;
    AlignFrame::orthonormal(read3(v.get("origin")?)?, read3(v.get("x")?)?, read3(v.get("z")?)?)
}

fn placed_frame(app: &SolveApp, d: &Dialog, slot: usize, which: &str) -> Result<AlignFrame, String> {
    let base = load_frame(d, slot).ok_or_else(|| format!("select the {which} frame first"))?;
    let Kind::Move { from_x, from_y, from_z, from_turn, from_flip, to_x, to_y, to_z, to_turn, to_flip, .. } = &d.kind else {
        return Err("not a move".into());
    };
    let (x, y, z, turn, flip) = if slot == 1 { (from_x, from_y, from_z, from_turn, *from_flip) } else { (to_x, to_y, to_z, to_turn, *to_flip) };
    base.adjusted(Vec3::new(len(app, x)?, len(app, y)?, len(app, z)?), ang(app, turn)?, flip)
        .ok_or_else(|| "frame offset is not a number".to_string())
}

fn screen_at(proj: &crate::viewport::Proj, cursor: egui::Pos2) -> geom::Screen {
    let (right, up, back) = proj.cam.basis();
    let w = f64::from(proj.rect.width()).max(1.0);
    let h = f64::from(proj.rect.height()).max(1.0);
    let nx = (f64::from(cursor.x) - f64::from(proj.rect.left())) / w * 2.0 - 1.0;
    let ny = 1.0 - (f64::from(cursor.y) - f64::from(proj.rect.top())) / h * 2.0;
    let hh = proj.cam.half_height();
    let aspect = w / h;
    let distance = proj.cam.distance.max(1e-6);
    let perspective = proj.cam.fov > 0.0;
    let cursor = if perspective { Vec2::new(nx * hh * aspect / distance, ny * hh / distance) } else { Vec2::new(nx * hh * aspect, ny * hh) };
    geom::Screen { eye: proj.cam.eye(), forward: -back, right, up, cursor, perspective }
}

/// The frame a pick stands for, from the cursor and the camera.
pub(crate) fn infer(session: &Session, proj: &crate::viewport::Proj, cursor: egui::Pos2, hit: &crate::viewport::Hit) -> Option<geom::Inferred> {
    use crate::viewport::Hit;
    let screen = screen_at(proj, cursor);
    let state = session.world_state();
    match hit {
        Hit::Face { body, index, .. } => {
            let mesh = state.body(body)?.mesh();
            on_mesh_face(&mesh, *index, &screen)
        }
        Hit::Edge { body, index, .. } => geom::on_edge(state.body(body)?.mesh().edges.get(*index)?, &screen),
        Hit::Vertex { body, point } if body.is_empty() => {
            let z = if screen.eye.z >= point.z { Vec3::Z } else { -Vec3::Z };
            AlignFrame::orthonormal(*point, Vec3::X, z).map(|frame| geom::Inferred { frame, marks: vec![*point] })
        }
        Hit::Vertex { body, point } => {
            let mesh = state.body(body)?.mesh();
            if let Some(face) = owning_face(&mesh, *point, screen.eye) {
                return on_mesh_face(&mesh, face, &screen);
            }
            let z = (screen.eye - *point).normalized().unwrap_or(Vec3::Z);
            AlignFrame::orthonormal(*point, Vec3::X, z).map(|frame| geom::Inferred { frame, marks: vec![*point] })
        }
        _ => None,
    }
}

fn on_mesh_face(mesh: &geom::Mesh, face: usize, screen: &geom::Screen) -> Option<geom::Inferred> {
    let (normal, points, segments) = face_parts(mesh, face)?;
    geom::on_face(&points, &segments, normal, screen)
}

fn face_parts(mesh: &geom::Mesh, face: usize) -> Option<(Vec3, Vec<Vec3>, Vec<(Vec3, Vec3)>)> {
    let id = u32::try_from(face).ok()?;
    let mut area = 0.0;
    let mut normal = Vec3::ZERO;
    let mut centroid = Vec3::ZERO;
    let mut seen = 0usize;
    for (t, tf) in mesh.triangles.iter().zip(&mesh.tri_face) {
        if *tf != id {
            continue;
        }
        seen += 1;
        if seen > 100_000 {
            break;
        }
        let Some([a, b, c]) = mesh.tri(t) else { continue };
        let n = (b - a).cross(c - a);
        let weight = n.len();
        if weight < 1e-12 {
            continue;
        }
        area += weight;
        normal += n;
        centroid += (a + b + c) * weight;
    }
    if area < 1e-12 {
        return None;
    }
    let normal = normal.normalized()?;
    let centroid = centroid / (area * 3.0);
    let tol = mesh.bounds().diagonal().max(1.0) * 1e-6;
    let mut points = vec![centroid];
    let mut segments = Vec::new();
    for (ei, faces) in mesh.edge_faces.iter().enumerate().take(20_000) {
        if mesh.seams.get(ei).copied().unwrap_or(false) || !faces.iter().any(|f| *f == id) {
            continue;
        }
        let Some(edge) = mesh.edges.get(ei) else { continue };
        if edge.len() < 2 {
            continue;
        }
        if let Some((center, _)) = geom::circle_through(edge) {
            push_unique(&mut points, center, tol);
            push_segments(&mut segments, edge, false);
        } else if let (Some(&a), Some(&b)) = (edge.first(), edge.last()) {
            let straight = (b - a).normalized().is_some_and(|d| edge.iter().all(|p| (*p - a).cross(d).len() < tol.max(1e-4) * a.dist(b).max(1.0)));
            push_unique(&mut points, a, tol);
            push_unique(&mut points, b, tol);
            push_segments(&mut segments, edge, straight);
        }
        if points.len() >= 256 {
            break;
        }
    }
    Some((normal, points, segments))
}

fn owning_face(mesh: &geom::Mesh, point: Vec3, eye: Vec3) -> Option<usize> {
    let tol = mesh.bounds().diagonal().max(1.0) * 1e-5;
    let mut best: Option<(f64, usize)> = None;
    let mut seen = Vec::new();
    for (ei, edge) in mesh.edges.iter().enumerate().take(20_000) {
        if !edge.iter().take(4096).any(|p| p.dist(point) <= tol.max(1e-4)) {
            continue;
        }
        for face in mesh.edge_faces.get(ei).into_iter().flatten().copied() {
            if seen.contains(&face) {
                continue;
            }
            seen.push(face);
            let Some(n) = one_face_normal(mesh, face) else { continue };
            let score = n.dot(eye - point);
            if best.is_none_or(|(s, _)| score > s) {
                best = Some((score, face as usize));
            }
        }
        if seen.len() > 64 {
            break;
        }
    }
    best.map(|(_, face)| face)
}

fn one_face_normal(mesh: &geom::Mesh, face: u32) -> Option<Vec3> {
    let mut n = Vec3::ZERO;
    let mut seen = 0usize;
    for (t, tf) in mesh.triangles.iter().zip(&mesh.tri_face) {
        if *tf != face {
            continue;
        }
        let Some([a, b, c]) = mesh.tri(t) else { continue };
        n += (b - a).cross(c - a);
        seen += 1;
        if seen > 64 {
            break;
        }
    }
    n.normalized()
}

fn push_unique(points: &mut Vec<Vec3>, p: Vec3, tol: f64) {
    if points.len() >= 256 || !p.is_finite() || points.iter().any(|q| q.dist(p) <= tol) {
        return;
    }
    points.push(p);
}

fn push_segments(segments: &mut Vec<(Vec3, Vec3)>, edge: &[Vec3], straight: bool) {
    if straight {
        if let (Some(&a), Some(&b)) = (edge.first(), edge.last())
            && segments.len() < 2048
        {
            segments.push((a, b));
        }
        return;
    }
    for w in edge.windows(2) {
        if segments.len() >= 2048 {
            return;
        }
        if w[0].is_finite() && w[1].is_finite() {
            segments.push((w[0], w[1]));
        }
    }
}

#[cfg(test)]
#[path = "dialogs_move_tests.rs"]
mod tests;
