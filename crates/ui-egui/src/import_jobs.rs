//! STL and OpenSCAD imports that take long enough to leave the window.
//!
//! Confirm closes the dialog and queues the command. One job runs at a time, off the
//! window thread. The viewport keeps a line of progress, and another import can be
//! queued while the first is still fitting.

use std::collections::VecDeque;
use std::panic::AssertUnwindSafe;

use serde_json::Value;
use solidcraft_engine::Session;

use crate::SolveApp;
use crate::dialogs::{Dialog, Kind};

const MAX_QUEUED: usize = 32;

struct Job {
    label: String,
    command: String,
    params: Value,
    /// The worker's session becomes the design. A STEP-file conversion only writes files.
    adopt: bool,
}

struct Running {
    job: Job,
    revision: u64,
    started: f64,
    done: std::sync::mpsc::Receiver<Result<(Session, Value), String>>,
}

#[derive(Default)]
pub struct Queue {
    waiting: VecDeque<Job>,
    running: Option<Running>,
}

impl Queue {
    pub fn busy(&self) -> bool {
        self.running.is_some() || !self.waiting.is_empty()
    }
}

/// Confirm for these dialogs queues the work and closes the dialog.
pub fn wants(d: &Dialog) -> bool {
    matches!(d.kind, Kind::MeshConvertFiles { .. } | Kind::Scad(_))
}

/// Queue commands already built from the dialog. Call this after an edit has put the
/// timeline marker back, so the worker copies the design as it will be kept.
pub fn enqueue_commands(app: &mut SolveApp, commands: Vec<(String, Value)>) -> Result<(), String> {
    if app.imports.waiting.len() + usize::from(app.imports.running.is_some()) + commands.len() > MAX_QUEUED {
        return Err(format!("at most {MAX_QUEUED} imports can be queued"));
    }
    for (command, params) in commands {
        let adopt = command != "mesh.convert_files";
        app.imports.waiting.push_back(Job { label: label(&command, &params), command, params, adopt });
    }
    kick(app);
    let waiting = app.imports.waiting.len();
    let running = app.imports.running.is_some();
    if running && waiting > 0 {
        app.set_status(format!("Queued. {waiting} waiting behind the one already running."), false);
    }
    Ok(())
}

pub fn tick(app: &mut SolveApp, ctx: &egui::Context) {
    let Some(running) = app.imports.running.as_ref() else {
        kick(app);
        return;
    };
    let outcome = match running.done.try_recv() {
        Ok(outcome) => Some(outcome),
        Err(std::sync::mpsc::TryRecvError::Empty) => {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
            return;
        }
        Err(std::sync::mpsc::TryRecvError::Disconnected) => None,
    };
    let Some(running) = app.imports.running.take() else { return };
    match outcome {
        Some(Ok((_copy, _value))) if running.job.adopt && app.session.revision != running.revision => {
            app.imports.waiting.push_front(running.job);
            app.set_status("The design changed, so the import will run again.", false);
        }
        Some(Ok((copy, value))) => {
            if running.job.adopt {
                app.session.accept_computed(copy);
            }
            let result = Ok(value);
            app.finish_command(&running.job.command, &result);
            app.set_status(finished(&running.job, result.as_ref().ok()), false);
        }
        Some(Err(error)) => {
            app.finish_command(&running.job.command, &Err(error));
        }
        None => {
            app.set_status(format!("{} stopped unexpectedly", running.job.label), true);
        }
    }
    kick(app);
    if app.imports.busy() {
        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}

/// A line at the foot of the view while an import is running or waiting.
pub fn show(app: &SolveApp, ctx: &egui::Context) {
    let Some(text) = progress_text(app) else { return };
    let Some(view) = app.viewport.rect else { return };
    let t = crate::theme::Tokens::get();
    let lift = if crate::open_meshing::active(app) { 36.0 } else { 0.0 };
    let at = egui::pos2(view.left() + 12.0, view.bottom() - 12.0 - lift);
    egui::Area::new(egui::Id::new("import_jobs"))
        .order(egui::Order::Foreground)
        .pivot(egui::Align2::LEFT_BOTTOM)
        .fixed_pos(at)
        .interactable(false)
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style()).fill(t.panel).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(egui::RichText::new(text).color(t.text));
                });
            });
        });
}

fn kick(app: &mut SolveApp) {
    if app.imports.running.is_some() {
        return;
    }
    let Some(job) = app.imports.waiting.pop_front() else { return };
    let revision = app.session.revision;
    let mut copy = app.session.scratch();
    let (tx, rx) = std::sync::mpsc::channel();
    let command = job.command.clone();
    let params = job.params.clone();
    let spawned = std::thread::Builder::new().name("import".into()).spawn(move || {
        let outcome = std::panic::catch_unwind(AssertUnwindSafe(|| copy.execute(&command, &params).map_err(|e| e.to_string())));
        let sent = match outcome {
            Ok(Ok(value)) => Ok((copy, value)),
            Ok(Err(error)) => Err(error),
            Err(_) => Err(format!("{command} stopped unexpectedly")),
        };
        let _ = tx.send(sent);
    });
    if spawned.is_err() {
        app.imports.waiting.push_front(job);
        app.set_status("Could not start the import.", true);
        return;
    }
    app.imports.running = Some(Running { job, revision, started: crate::now_ms(), done: rx });
}

fn progress_text(app: &SolveApp) -> Option<String> {
    let running = app.imports.running.as_ref()?;
    let seconds = ((crate::now_ms() - running.started) / 1000.0).floor().max(0.0);
    let waiting = app.imports.waiting.len();
    let behind = if waiting == 0 { String::new() } else { format!(" · {waiting} waiting") };
    Some(format!("{} {} — {seconds:.0} s{behind}", verb(&running.job.command), running.job.label))
}

fn verb(command: &str) -> &'static str {
    match command {
        "mesh.convert_files" => "Writing STEP for",
        "scad.insert" | "scad.revise" => "Running OpenSCAD for",
        _ => "Importing",
    }
}

fn finished(job: &Job, value: Option<&Value>) -> String {
    if job.command == "mesh.convert_files" {
        let n = value.and_then(|v| v.get("written")).and_then(Value::as_array).map(Vec::len).unwrap_or(0);
        return format!("Wrote {n} STEP file(s) for {}", job.label);
    }
    if job.command == "scad.revise" {
        return format!("Rebuilt {}", job.label);
    }
    format!("Imported {}", job.label)
}

fn label(command: &str, params: &Value) -> String {
    if command == "timeline.redefine" {
        let inner_command = params.get("command").and_then(Value::as_str).unwrap_or("mesh.import");
        return params.get("params").map(|inner| label(inner_command, inner)).unwrap_or_else(|| "STL import".into());
    }
    let mut names = Vec::new();
    if let Some(path) = params.get("path").and_then(Value::as_str) {
        names.push(file_name(path));
    }
    if let Some(more) = params.get("paths").and_then(Value::as_array) {
        for item in more {
            if let Some(path) = item.as_str() {
                let name = file_name(path);
                if !names.contains(&name) {
                    names.push(name);
                }
            }
        }
    }
    match names.as_slice() {
        [] => {
            if command == "scad.insert" || command == "scad.revise" {
                "OpenSCAD".into()
            } else {
                "STL".into()
            }
        }
        [one] => one.clone(),
        [first, rest @ ..] => format!("{first} and {} more", rest.len()),
    }
}

fn file_name(path: &str) -> String {
    std::path::Path::new(path)
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .filter(|name| !name.is_empty())
        .unwrap_or_else(|| path.to_string())
}
