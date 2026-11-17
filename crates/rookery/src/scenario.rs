//! Scenario harness v0 (row 4.7): a Lua file that names a tree, how long to
//! run it, and a `check(log)` that decides pass or fail.
//!
//! ```lua
//! -- lua/scenarios/patrol_4x4.lua
//! ci = true                          -- run in CI (scripts/run-scenarios.sh)
//! tree = "../trees/patrol.lua"       -- relative to this file
//! ticks = 10000                      -- 50 Hz ticks (200 s of sim time)
//! function setup(world) end          -- optional; world = { rows, cols }
//! function check(log)                -- log: see below
//!   return ok, "message"
//! end
//! ```
//!
//! `log` is a plain table built after the run:
//! `{ ticks, sim_time, visited = { {r, c} | "exit" ... }, visited_cells,
//!    errors = { "leaf: msg", ... }, max_box_displacement, final_q }`.
//! `visited` lists arrivals in order; `visited_cells` is the number of
//! distinct cells among them.
//!
//! The scenario runs in its own Lua state, apart from the tree's, so a
//! scenario can never reach into the blackboard.

use std::path::{Path, PathBuf};

use mast::route::Pose;
use mlua::{Function, Lua, Table, Value};

/// A loaded scenario file.
pub struct Scenario {
    lua: Lua,
    path: PathBuf,
    /// The tree to run, resolved against the scenario's directory.
    pub tree: Option<PathBuf>,
    /// How many 50 Hz ticks to run.
    pub ticks: Option<u64>,
    /// Whether CI runs it.
    pub ci: bool,
}

/// What the harness hands to `check(log)`.
#[derive(Debug, Clone, PartialEq)]
pub struct RunLog {
    pub ticks: u64,
    pub sim_time: f64,
    pub visited: Vec<Pose>,
    pub errors: Vec<String>,
    pub max_box_displacement: f64,
    pub final_q: [f64; 4],
    /// Cells whose box moved more than 1 mm during the run (row 6.5).
    pub moved_boxes: Vec<(usize, usize)>,
}

impl Scenario {
    pub fn load(path: &Path) -> Result<Self, String> {
        let lua = Lua::new();
        let src = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        lua.load(&src)
            .set_name(path.display().to_string())
            .exec()
            .map_err(|e| format!("{}: {e}", path.display()))?;
        let g = lua.globals();
        let base = path.parent().unwrap_or(Path::new("."));
        let tree = g
            .get::<Option<String>>("tree")
            .map_err(|e| format!("{}: tree: {e}", path.display()))?
            .map(|t| base.join(t));
        let ticks = g
            .get::<Option<u64>>("ticks")
            .map_err(|e| format!("{}: ticks: {e}", path.display()))?;
        let ci = g.get::<Option<bool>>("ci").ok().flatten().unwrap_or(false);
        if g.get::<Option<Function>>("check").ok().flatten().is_none() {
            return Err(format!(
                "{}: a scenario must define check(log)",
                path.display()
            ));
        }
        Ok(Self {
            lua,
            path: path.to_path_buf(),
            tree,
            ticks,
            ci,
        })
    }

    /// Call `setup(world)` if the scenario defines it.
    pub fn setup(&self, rows: usize, cols: usize) -> Result<(), String> {
        let Some(f) = self
            .lua
            .globals()
            .get::<Option<Function>>("setup")
            .ok()
            .flatten()
        else {
            return Ok(());
        };
        let world = self.lua.create_table().map_err(|e| e.to_string())?;
        world.set("rows", rows).map_err(|e| e.to_string())?;
        world.set("cols", cols).map_err(|e| e.to_string())?;
        f.call::<()>(world)
            .map_err(|e| format!("{}: setup: {e}", self.path.display()))
    }

    /// Call `check(log)` and return `(ok, message)`.
    pub fn check(&self, log: &RunLog) -> Result<(bool, String), String> {
        let f: Function = self.lua.globals().get("check").map_err(|e| e.to_string())?;
        let t = self.to_lua(log).map_err(|e| e.to_string())?;
        let (ok, msg): (Value, Option<String>) = f
            .call(t)
            .map_err(|e| format!("{}: check: {e}", self.path.display()))?;
        let ok = matches!(ok, Value::Boolean(true));
        Ok((ok, msg.unwrap_or_default()))
    }

    fn to_lua(&self, log: &RunLog) -> mlua::Result<Table> {
        let lua = &self.lua;
        let t = lua.create_table()?;
        t.set("ticks", log.ticks)?;
        t.set("sim_time", log.sim_time)?;
        let visited = lua.create_table()?;
        let mut cells: Vec<(usize, usize)> = Vec::new();
        for (i, p) in log.visited.iter().enumerate() {
            let v: Value = match *p {
                Pose::Cell(r, c) => {
                    if !cells.contains(&(r, c)) {
                        cells.push((r, c));
                    }
                    Value::Table(lua.create_sequence_from([r, c])?)
                }
                Pose::Exit => Value::String(lua.create_string("exit")?),
                Pose::Transfer(r, c) => {
                    let t = lua.create_table()?;
                    t.set("transfer", lua.create_sequence_from([r, c])?)?;
                    Value::Table(t)
                }
                Pose::Home => Value::String(lua.create_string("home")?),
            };
            visited.set(i + 1, v)?;
        }
        t.set("visited", visited)?;
        t.set("visited_cells", cells.len())?;
        t.set(
            "errors",
            lua.create_sequence_from(log.errors.iter().cloned())?,
        )?;
        t.set("max_box_displacement", log.max_box_displacement)?;
        t.set("final_q", lua.create_sequence_from(log.final_q)?)?;
        let moved = lua.create_table()?;
        for (i, (r, c)) in log.moved_boxes.iter().enumerate() {
            moved.set(i + 1, lua.create_sequence_from([*r, *c])?)?;
        }
        t.set("moved_boxes", moved)?;
        Ok(t)
    }
}

/// Cells whose box moved more than 1 mm between two qpos vectors. The
/// boxes' free joints come first in qpos, row-major (`box_0_0, box_0_1, …`).
pub fn moved_boxes(first: &[f64], last: &[f64], rows: usize, cols: usize) -> Vec<(usize, usize)> {
    (0..rows * cols)
        .filter(|&b| {
            (0..3).any(|k| {
                let i = 7 * b + k;
                i < first.len() && i < last.len() && (last[i] - first[i]).abs() > 0.001
            })
        })
        .map(|b| (b / cols, b % cols))
        .collect()
}

/// The largest position change of any free-joint body between two qpos
/// vectors. The scene builder emits the free joints first (7 values each:
/// boxes, then crawfish, then shells) and the crane's 4 slides last, so the
/// free bodies are the first `(nq - 4) / 7` blocks. Parked shells lie still,
/// so they only count once the lifecycle moves one (7.5).
pub fn max_box_displacement(first: &[f64], last: &[f64]) -> f64 {
    let n = first.len().min(last.len());
    let boxes = n.saturating_sub(4) / 7;
    (0..boxes)
        .flat_map(|b| (0..3).map(move |k| 7 * b + k))
        .map(|i| (last[i] - first[i]).abs())
        .fold(0.0, f64::max)
}
