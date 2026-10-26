//! OpenSCAD library: choose a `.scad` file, edit the parameters written at the top of it,
//! and insert the mesh OpenSCAD makes. The script stays a file on disk. The design stores
//! the mesh, not a live link back to the script.

use crate::ui_kit::{self, choice_row, text_row};
use serde_json::{Map, Value, json};

use crate::SolveApp;
use crate::dialogs::Kind;

const SHOWN: usize = 16;

/// One Customizer value the dialog can edit.
#[derive(Clone, Debug)]
pub struct ParamRow {
    pub name: String,
    pub kind: String,
    pub group: String,
    pub description: String,
    pub text: String,
    pub options: Vec<String>,
    pub option: usize,
}

/// One script in the folder.
#[derive(Clone, Debug)]
pub struct Script {
    pub name: String,
    pub path: String,
    pub params: Vec<ParamRow>,
    pub extra: usize,
}

/// The OpenSCAD library dialog.
#[derive(Clone, Debug)]
pub struct Dlg {
    pub dir: String,
    pub filter: String,
    pub script: usize,
    pub files: Vec<Script>,
    /// Program path the user typed. Empty: use the OpenSCAD found on PATH.
    pub openscad: String,
    pub found: String,
    pub name: String,
    /// `$fn`, empty when the script's own value should stand.
    pub fn_text: String,
    pub note: String,
    pub truncated: bool,
}

impl Default for Dlg {
    fn default() -> Self {
        Self {
            dir: String::new(),
            filter: String::new(),
            script: 0,
            files: Vec::new(),
            openscad: String::new(),
            found: String::new(),
            name: String::new(),
            fn_text: String::new(),
            note: String::new(),
            truncated: false,
        }
    }
}

pub fn start(id: &str) -> Option<(Kind, Vec<crate::selection::SelInput>)> {
    (id == "scad.insert").then(|| (Kind::Scad(Dlg::default()), Vec::new()))
}

struct Loaded {
    dir: String,
    files: Vec<Script>,
    script: usize,
    found: String,
    truncated: bool,
}

fn load(app: &mut SolveApp, path: &str) -> Result<Loaded, String> {
    let path = path.trim();
    if path.is_empty() {
        return Err("Choose a folder of .scad files, or one .scad file.".into());
    }
    let v = app.session.execute("scad.list", &json!({"path": path})).map_err(|e| e.to_string())?;
    let files: Vec<Script> = v.get("files").and_then(Value::as_array).map(|rows| rows.iter().filter_map(script_from).collect()).unwrap_or_default();
    let selected = v.get("selected").and_then(Value::as_str).unwrap_or("");
    let script = files.iter().position(|f| f.name == selected).unwrap_or(0);
    Ok(Loaded {
        dir: v.get("path").and_then(Value::as_str).unwrap_or(path).to_string(),
        found: v.get("openscad").and_then(Value::as_str).unwrap_or("").to_string(),
        truncated: v.get("truncated").and_then(Value::as_bool).unwrap_or(false),
        files,
        script,
    })
}

fn apply_load(k: &mut Dlg, loaded: Result<Loaded, String>) {
    match loaded {
        Ok(loaded) => {
            k.dir = loaded.dir;
            k.script = loaded.script.min(loaded.files.len().saturating_sub(1));
            k.files = loaded.files;
            k.found = loaded.found;
            k.truncated = loaded.truncated;
            k.note.clear();
        }
        Err(error) => k.note = error,
    }
}

fn script_from(v: &Value) -> Option<Script> {
    let name = v.get("name")?.as_str()?.to_string();
    let path = v.get("path")?.as_str()?.to_string();
    let all: Vec<ParamRow> =
        v.get("parameters").and_then(Value::as_array).map(|rows| rows.iter().filter_map(param_from).collect()).unwrap_or_default();
    let visible: Vec<ParamRow> = all.into_iter().filter(|p| p.group != "Hidden").collect();
    let extra = visible.len().saturating_sub(SHOWN);
    Some(Script { name, path, params: visible.into_iter().take(SHOWN).collect(), extra })
}

fn param_from(v: &Value) -> Option<ParamRow> {
    let name = v.get("name")?.as_str()?.to_string();
    let kind = v.get("kind")?.as_str()?.to_string();
    let text = match v.get("value")? {
        Value::Bool(on) => if *on { "true" } else { "false" }.to_string(),
        Value::Number(n) => n.to_string(),
        Value::String(s) => s.clone(),
        other => other.to_string(),
    };
    let options: Vec<String> = v
        .get("options")
        .and_then(Value::as_array)
        .map(|rows| rows.iter().map(|v| if let Value::String(s) = v { s.clone() } else { v.to_string() }).collect())
        .unwrap_or_default();
    let option = options.iter().position(|o| o == &text).unwrap_or(0);
    Some(ParamRow {
        name,
        kind,
        group: v.get("group").and_then(Value::as_str).unwrap_or("Parameters").to_string(),
        description: v.get("description").and_then(Value::as_str).unwrap_or("").to_string(),
        text,
        options,
        option,
    })
}

fn matches_filter(name: &str, filter: &str) -> bool {
    filter.is_empty() || name.to_ascii_lowercase().contains(&filter.to_ascii_lowercase())
}

/// The dialog's rows. Enter in a text field confirms.
pub fn rows(app: &mut SolveApp, ui: &mut egui::Ui, k: &mut Dlg) -> bool {
    let mut enter = false;
    let (_changed, submitted) = text_row(ui, "Library", &mut k.dir, "folder or .scad file");
    enter |= submitted;
    ui.horizontal(|ui| {
        if ui_kit::button(ui, "Load", ui_kit::Button::Normal).clicked() {
            let dir = k.dir.clone();
            let loaded = load(app, &dir);
            apply_load(k, loaded);
        }
        if ui_kit::button(ui, "Browse", ui_kit::Button::Normal).clicked() {
            app.pick_open_for(&["scad"], |app, path| {
                let Some(path) = path else { return };
                let loaded = load(app, &path);
                let Some(k) = dialog(app) else { return };
                apply_load(k, loaded);
            });
        }
    });
    if k.found.is_empty() {
        ui_kit::text(ui, "OpenSCAD was not found on PATH. Install it, or set the program path.", ui_kit::Text::Note("scad_missing"));
    } else if k.openscad.trim().is_empty() {
        ui_kit::text(ui, &format!("Using {}", k.found), ui_kit::Text::Note("scad_found"));
    }
    let (_, submitted) = text_row(ui, "Program", &mut k.openscad, "openscad.exe, if it is not on PATH");
    enter |= submitted;
    let (changed, submitted) = text_row(ui, "Filter", &mut k.filter, "part of a file name");
    enter |= submitted;
    if changed && !k.files.get(k.script).is_some_and(|f| matches_filter(&f.name, &k.filter)) {
        k.script = k.files.iter().position(|f| matches_filter(&f.name, &k.filter)).unwrap_or(0);
    }
    let shown: Vec<usize> = k.files.iter().enumerate().filter(|(_, f)| matches_filter(&f.name, &k.filter)).map(|(i, _)| i).collect();
    if shown.is_empty() {
        ui_kit::text(ui, "Load a folder to list its .scad files.", ui_kit::Text::Note("scad_empty"));
    } else {
        let names: Vec<String> = shown.iter().filter_map(|i| k.files.get(*i).map(|f| f.name.clone())).collect();
        let refs: Vec<&str> = names.iter().map(String::as_str).collect();
        let mut pos = shown.iter().position(|i| *i == k.script).unwrap_or(0);
        choice_row(ui, "Script", "scad_script", &mut pos, &refs);
        if let Some(index) = shown.get(pos).copied() {
            k.script = index;
        }
    }
    if k.truncated {
        ui_kit::text(ui, "The folder has more scripts than this list shows.", ui_kit::Text::Note("scad_truncated"));
    }
    let script = k.script;
    let mut group = String::new();
    let count = k.files.get(script).map(|f| f.params.len()).unwrap_or(0);
    for index in 0..count {
        let Some(param) = k.files.get_mut(script).and_then(|f| f.params.get_mut(index)) else { continue };
        if param.group != group {
            group.clone_from(&param.group);
            ui_kit::text(ui, &group, ui_kit::Text::Note("scad_group"));
        }
        enter |= param_row(ui, param);
    }
    if let Some(extra) = k.files.get(script).map(|f| f.extra).filter(|extra| *extra > 0) {
        ui_kit::text(
            ui,
            &format!("{extra} more parameters keep the file's values. Pass them to scad.insert to change them."),
            ui_kit::Text::Note("scad_extra"),
        );
    }
    let (_, submitted) = text_row(ui, "Curve segments", &mut k.fn_text, "$fn, empty keeps the file");
    enter |= submitted;
    let (_, submitted) = text_row(ui, "Name", &mut k.name, "component name");
    enter |= submitted;
    ui_kit::text(
        ui,
        "OK runs OpenSCAD and inserts a mesh in the active component. Changing these values later does not rebuild that mesh.",
        ui_kit::Text::Note("scad_mesh"),
    );
    if !k.note.is_empty() {
        ui_kit::text(ui, &k.note, ui_kit::Text::Note("scad_note"));
    }
    enter
}

fn param_row(ui: &mut egui::Ui, param: &mut ParamRow) -> bool {
    let label = row_label(param);
    if param.kind == "bool" {
        let mut on = param.text == "true";
        ui_kit::row(ui, &label, |ui| {
            if ui_kit::checkbox(ui, "", &mut on).changed() {
                param.text = if on { "true" } else { "false" }.to_string();
            }
        });
        return false;
    }
    if param.kind == "enum" && !param.options.is_empty() {
        if param.option >= param.options.len() {
            param.option = 0;
        }
        let id = param.name.clone();
        let options = param.options.clone();
        let refs: Vec<&str> = options.iter().map(String::as_str).collect();
        choice_row(ui, &label, &id, &mut param.option, &refs);
        return false;
    }
    text_row(ui, &label, &mut param.text, &param.description).1
}

fn row_label(param: &ParamRow) -> String {
    if param.description.is_empty() { param.name.clone() } else { format!("{} — {}", param.name, param.description) }
}

fn dialog(app: &mut SolveApp) -> Option<&mut Dlg> {
    match &mut app.dialog.as_mut()?.kind {
        Kind::Scad(k) => Some(k),
        _ => None,
    }
}

fn param_value(param: &ParamRow) -> Result<Value, String> {
    match param.kind.as_str() {
        "bool" => Ok(json!(param.text == "true")),
        "int" => {
            let n: i64 = param.text.trim().parse().map_err(|_| format!("{} must be a whole number", param.name))?;
            Ok(json!(n))
        }
        "number" => {
            let n: f64 = param.text.trim().parse().map_err(|_| format!("{} must be a number", param.name))?;
            if !n.is_finite() {
                return Err(format!("{} must be a finite number", param.name));
            }
            Ok(json!(n))
        }
        "string" => Ok(json!(param.text)),
        "enum" => {
            let choice = param.options.get(param.option).ok_or_else(|| format!("choose a value for {}", param.name))?;
            if let Some(n) = choice.parse::<i64>().ok().filter(|_| choice.chars().all(|c| c.is_ascii_digit() || c == '-')) {
                return Ok(json!(n));
            }
            if let Some(n) =
                choice.parse::<f64>().ok().filter(|n| n.is_finite() && choice.chars().all(|c| c.is_ascii_digit() || matches!(c, '.' | '-' | '+')))
            {
                return Ok(json!(n));
            }
            Ok(json!(choice))
        }
        _ => Err(format!("{} cannot be edited here", param.name)),
    }
}

/// The command OK runs.
pub fn commands(k: &Dlg) -> Result<Vec<(String, Value)>, String> {
    let file = k.files.get(k.script).filter(|f| matches_filter(&f.name, &k.filter)).ok_or("choose a .scad file")?;
    let mut parameters = Map::new();
    for param in &file.params {
        parameters.insert(param.name.clone(), param_value(param)?);
    }
    let mut body = json!({"path": file.path, "parameters": parameters});
    if let Some(program) = Some(k.openscad.trim()).filter(|p| !p.is_empty()) {
        body["openscad"] = json!(program);
    }
    if let Some(name) = Some(k.name.trim()).filter(|n| !n.is_empty()) {
        body["name"] = json!(name);
    }
    if !k.fn_text.trim().is_empty() {
        let n: u64 = k.fn_text.trim().parse().map_err(|_| "Curve segments must be a whole number from 0 to 1024")?;
        if n > 1024 {
            return Err("Curve segments must be a whole number from 0 to 1024".into());
        }
        body["fn"] = json!(n);
    }
    Ok(vec![("scad.insert".into(), body)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialogs::Kind;

    fn sample() -> Dlg {
        let mut k = Dlg::default();
        k.files.push(Script {
            name: "bracket.scad".into(),
            path: "bracket.scad".into(),
            extra: 0,
            params: vec![
                ParamRow {
                    name: "width".into(),
                    kind: "int".into(),
                    group: "Size".into(),
                    description: String::new(),
                    text: "10".into(),
                    options: Vec::new(),
                    option: 0,
                },
                ParamRow {
                    name: "shape".into(),
                    kind: "enum".into(),
                    group: "Size".into(),
                    description: String::new(),
                    text: "round".into(),
                    options: vec!["round".into(), "square".into()],
                    option: 1,
                },
            ],
        });
        k.fn_text = "32".into();
        k.name = "Bracket".into();
        k
    }

    #[test]
    fn ok_sends_the_chosen_script_and_its_parameters() {
        let (id, body) = commands(&sample()).unwrap().into_iter().next().unwrap();
        assert_eq!(id, "scad.insert");
        assert_eq!(body["path"], "bracket.scad");
        assert_eq!(body["parameters"]["width"], 10);
        assert_eq!(body["parameters"]["shape"], "square");
        assert_eq!(body["fn"], 32);
        assert_eq!(body["name"], "Bracket");
    }

    #[test]
    fn the_toolbar_command_opens_this_dialog() {
        let h = crate::scenario::Harness::new();
        let d = crate::dialogs::Dialog::for_command(&h.app, "scad.insert").unwrap();
        assert!(matches!(d.kind, Kind::Scad(_)));
    }
}
