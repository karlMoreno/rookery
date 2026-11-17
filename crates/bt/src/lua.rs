//! The `mlua` bridge (row 4.4): tree files and Lua leaves.
//!
//! ```text
//!   tree.lua ──run with Sequence/Fallback/…/Leaf globals──▶ spec table
//!        └─▶ build(spec) ──▶ Box<dyn Node> (nodes.rs composites + LuaLeaf)
//!
//!   LuaLeaf::tick(bb)
//!     every 10 ticks: mtime changed on two checks in a row? → reload (keep old code on error)
//!     view = read-only table { sim_time, gantry_q, gantry_target, route_len, goal, events }
//!            + bb:go_to(pose)   ← the only write, a goal pose (Home() / Exit() / Cell(r, c))
//!     tick(view, args…) via mlua (a Lua error comes back as Err, never a panic)
//!       Ok("success" | "failure" | "running") → Status
//!       Err(e) → LeafError { leaf, message } + warn! → Status::Failure
//! ```
//!
//! [Band 1] Errors are contained through `mlua::Result`, which wraps Lua's
//! own protected call. Nothing here relies on `catch_unwind`, because Rust
//! panics abort on the Air.

use std::cell::RefCell;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::time::SystemTime;

use mast::route::Pose;
use mlua::{Function, Lua, MultiValue, Table, Value};

use crate::nodes::{Fallback, Inverter, Parallel, Repeat, Sequence};
use crate::{Blackboard, BtError, LeafError, Node, Status};

/// Leaf errors, shared between the leaves of one tree and the tree.
pub(crate) type ErrorSink = Rc<RefCell<Vec<LeafError>>>;

/// How often a leaf checks its file for changes, in ticks.
pub const RELOAD_CHECK_TICKS: u64 = 10;

/// Lua source that defines the tree-file constructors and the pose helpers.
/// They only build plain tables ("specs"); Rust turns specs into nodes.
const PRELUDE: &str = r#"
function Sequence(children) return { kind = "Sequence", children = children } end
function Fallback(children) return { kind = "Fallback", children = children } end
function Parallel(t)
  local children = {}
  for i, c in ipairs(t) do children[i] = c end
  return { kind = "Parallel", children = children,
           success = t.success or #children, failure = t.failure or 1 }
end
function Inverter(child) return { kind = "Inverter", child = child } end
function Repeat(n, child) return { kind = "Repeat", n = n, child = child } end
function Leaf(path, ...) return { kind = "Leaf", path = path, args = table.pack(...) } end
function Home() return { pose = "Home" } end
function Exit() return { pose = "Exit" } end
function Cell(r, c) return { pose = "Cell", r = r, c = c } end
function Transfer(r, c) return { pose = "Transfer", r = r, c = c } end
"#;

/// Load `path` as a tree file and build its nodes.
pub(crate) fn load_tree(
    path: &Path,
    lua: &Lua,
    sink: &ErrorSink,
) -> Result<Box<dyn Node>, BtError> {
    let lua_err = |message: String| BtError::Lua {
        path: path.to_path_buf(),
        message,
    };
    lua.load(PRELUDE)
        .set_name("bt prelude")
        .exec()
        .map_err(|e| lua_err(e.to_string()))?;
    let src = std::fs::read_to_string(path).map_err(|source| BtError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let spec: Value = lua
        .load(&src)
        .set_name(path.display().to_string())
        .eval()
        .map_err(|e| lua_err(e.to_string()))?;
    let base = path.parent().unwrap_or(Path::new("."));
    build(&spec, base, lua, sink).map_err(lua_err)
}

fn build(spec: &Value, base: &Path, lua: &Lua, sink: &ErrorSink) -> Result<Box<dyn Node>, String> {
    let t = match spec {
        Value::Table(t) => t,
        other => return Err(format!("expected a node, got {}", other.type_name())),
    };
    let kind: String = t
        .get("kind")
        .map_err(|_| "a table without `kind` is not a node".to_string())?;
    let children = |key: &str| -> Result<Vec<Box<dyn Node>>, String> {
        let list: Table = t.get(key).map_err(|e| format!("{kind}: {e}"))?;
        list.sequence_values::<Value>()
            .map(|v| {
                v.map_err(|e| e.to_string())
                    .and_then(|v| build(&v, base, lua, sink))
            })
            .collect()
    };
    let child = || -> Result<Box<dyn Node>, String> {
        let v: Value = t.get("child").map_err(|e| e.to_string())?;
        build(&v, base, lua, sink)
    };
    let uint = |key: &str| -> Result<usize, String> {
        let n: i64 = t.get(key).map_err(|e| format!("{kind}.{key}: {e}"))?;
        usize::try_from(n).map_err(|_| format!("{kind}.{key} must be ≥ 0, got {n}"))
    };
    Ok(match kind.as_str() {
        "Sequence" => Box::new(Sequence::new(children("children")?)),
        "Fallback" => Box::new(Fallback::new(children("children")?)),
        "Parallel" => Box::new(Parallel::new(
            children("children")?,
            uint("success")?,
            uint("failure")?,
        )),
        "Inverter" => Box::new(Inverter::new(child()?)),
        "Repeat" => Box::new(Repeat::new(uint("n")?, child()?)),
        "Leaf" => {
            let rel: String = t.get("path").map_err(|e| format!("Leaf: {e}"))?;
            let args: Table = t.get("args").map_err(|e| e.to_string())?;
            let n: usize = args.get::<Option<usize>>("n").ok().flatten().unwrap_or(0);
            let args = (1..=n)
                .map(|i| args.get::<Value>(i).map_err(|e| e.to_string()))
                .collect::<Result<_, _>>()?;
            Box::new(LuaLeaf::load(
                base.join(rel),
                args,
                lua.clone(),
                sink.clone(),
            )?)
        }
        other => return Err(format!("unknown node kind `{other}`")),
    })
}

/// A leaf whose body is a Lua file defining `tick(bb, …)`.
pub struct LuaLeaf {
    name: String,
    path: PathBuf,
    args: Vec<Value>,
    lua: Lua,
    tick_fn: Function,
    loaded_mtime: Option<SystemTime>,
    pending_mtime: Option<SystemTime>,
    ticks: u64,
    sink: ErrorSink,
}

impl LuaLeaf {
    fn load(path: PathBuf, args: Vec<Value>, lua: Lua, sink: ErrorSink) -> Result<Self, String> {
        let name = path
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        let tick_fn = compile(&lua, &path).map_err(|e| format!("leaf `{name}`: {e}"))?;
        Ok(Self {
            name,
            loaded_mtime: mtime(&path),
            path,
            args,
            lua,
            tick_fn,
            pending_mtime: None,
            ticks: 0,
            sink,
        })
    }

    fn fail(&self, message: String) -> Status {
        tracing::warn!(leaf = %self.name, error = %message, "lua leaf failed");
        self.sink.borrow_mut().push(LeafError {
            leaf: self.name.clone(),
            message,
        });
        Status::Failure
    }

    /// Hot reload: on every RELOAD_CHECK_TICKS-th tick, compare the file's
    /// mtime. A new mtime seen on two checks in a row triggers a reload
    /// (debounce, so a half-written file is not loaded). A reload that fails
    /// keeps the old code and records an error.
    fn maybe_reload(&mut self) {
        if !self.ticks.is_multiple_of(RELOAD_CHECK_TICKS) {
            return;
        }
        let now = mtime(&self.path);
        if now == self.loaded_mtime {
            self.pending_mtime = None;
            return;
        }
        if self.pending_mtime != now {
            self.pending_mtime = now;
            return;
        }
        self.pending_mtime = None;
        self.loaded_mtime = now;
        match compile(&self.lua, &self.path) {
            Ok(f) => {
                tracing::info!(leaf = %self.name, "lua leaf reloaded");
                self.tick_fn = f;
            }
            Err(e) => {
                self.fail(format!("reload failed, keeping the old code: {e}"));
            }
        }
    }
}

impl Node for LuaLeaf {
    fn tick(&mut self, bb: &mut Blackboard) -> Status {
        self.ticks += 1;
        self.maybe_reload();
        let goal = Rc::new(RefCell::new(None::<Option<Pose>>));
        let result = view(&self.lua, bb, goal.clone()).and_then(|view| {
            let mut args = vec![Value::Table(view)];
            args.extend(self.args.iter().cloned());
            self.tick_fn.call::<Value>(MultiValue::from_vec(args))
        });
        // Apply the goal write only after the call returns, so a leaf that
        // errors halfway leaves the blackboard as it was.
        let status = match result {
            Ok(v) => match status_of(&v) {
                Ok(s) => s,
                Err(e) => return self.fail(e),
            },
            Err(e) => return self.fail(e.to_string()),
        };
        let pending = goal.borrow_mut().take();
        if let Some(g) = pending
            && let Ok(key) = bb.try_register::<Option<Pose>>("gantry_goal")
        {
            bb.set(&key, g);
        }
        status
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Compile a leaf file in its own environment (globals fall through to `_G`)
/// and return its `tick` function.
fn compile(lua: &Lua, path: &Path) -> Result<Function, String> {
    let src = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let env = lua.create_table().map_err(|e| e.to_string())?;
    let meta = lua.create_table().map_err(|e| e.to_string())?;
    meta.set("__index", lua.globals())
        .map_err(|e| e.to_string())?;
    env.set_metatable(Some(meta));
    lua.load(&src)
        .set_name(path.display().to_string())
        .set_environment(env.clone())
        .exec()
        .map_err(|e| e.to_string())?;
    env.get::<Function>("tick")
        .map_err(|_| format!("{} does not define `tick(bb, …)`", path.display()))
}

fn mtime(path: &Path) -> Option<SystemTime> {
    std::fs::metadata(path).and_then(|m| m.modified()).ok()
}

fn status_of(v: &Value) -> Result<Status, String> {
    let s = match v {
        Value::String(s) => s.to_string_lossy().to_ascii_lowercase(),
        other => {
            return Err(format!(
                "tick must return \"success\", \"failure\" or \"running\", got {}",
                other.type_name()
            ));
        }
    };
    match s.as_str() {
        "success" => Ok(Status::Success),
        "failure" => Ok(Status::Failure),
        "running" => Ok(Status::Running),
        other => Err(format!(
            "tick returned \"{other}\"; expected success, failure or running"
        )),
    }
}

/// The read-only blackboard view a leaf sees, plus `bb:go_to(pose)`.
fn view(
    lua: &Lua,
    bb: &Blackboard,
    goal: Rc<RefCell<Option<Option<Pose>>>>,
) -> mlua::Result<Table> {
    let data = lua.create_table()?;
    if let Ok(Some(t)) = bb.get_named::<f64>("sim_time") {
        data.set("sim_time", *t)?;
    }
    for key in ["gantry_q", "gantry_target"] {
        if let Ok(Some(q)) = bb.get_named::<[f64; 4]>(key) {
            data.set(key, lua.create_sequence_from(q.iter().copied())?)?;
        }
    }
    if let Ok(Some(n)) = bb.get_named::<usize>("route_len") {
        data.set("route_len", *n)?;
    }
    if let Ok(Some(g)) = bb.get_named::<Option<Pose>>("gantry_goal") {
        data.set("goal", pose_to_lua(lua, *g)?)?;
    }
    // Row 7.5: recent events, oldest first, each { seq, t, kind, tray, r, c }
    // with r and c counted from 0 like Cell(r, c).
    let events = lua.create_table()?;
    if let Ok(Some(q)) = bb.get_named::<std::collections::VecDeque<crate::BbEvent>>("events") {
        for (i, e) in q.iter().enumerate() {
            let t = lua.create_table()?;
            t.set("seq", e.seq)?;
            t.set("t", e.t)?;
            t.set("kind", e.kind)?;
            t.set("tray", e.tray)?;
            t.set("r", e.r)?;
            t.set("c", e.c)?;
            events.set(i + 1, t)?;
        }
    }
    data.set("events", events)?;
    data.set(
        "go_to",
        lua.create_function(move |_, (_this, pose): (Value, Value)| {
            let p = pose_from_lua(&pose).map_err(mlua::Error::runtime)?;
            *goal.borrow_mut() = Some(Some(p));
            Ok(())
        })?,
    )?;

    let proxy = lua.create_table()?;
    let meta = lua.create_table()?;
    let known: Vec<&str> = vec![
        "sim_time",
        "gantry_q",
        "gantry_target",
        "route_len",
        "goal",
        "go_to",
        "events",
    ];
    let d = data.clone();
    meta.set(
        "__index",
        lua.create_function(move |_, (_t, k): (Value, String)| {
            let v: Value = d.get(k.as_str())?;
            if v.is_nil() && !known.contains(&k.as_str()) {
                return Err(mlua::Error::runtime(format!("blackboard has no key `{k}`")));
            }
            Ok(v)
        })?,
    )?;
    meta.set(
        "__newindex",
        lua.create_function(|_, (_t, k): (Value, Value)| -> mlua::Result<()> {
            let k = k.to_string().unwrap_or_default();
            Err(mlua::Error::runtime(format!(
                "blackboard is read-only (tried to set `{k}`); use bb:go_to(Home() | Exit() | Cell(r, c))"
            )))
        })?,
    )?;
    proxy.set_metatable(Some(meta));
    Ok(proxy)
}

fn pose_to_lua(lua: &Lua, p: Option<Pose>) -> mlua::Result<Value> {
    let Some(p) = p else { return Ok(Value::Nil) };
    let t = lua.create_table()?;
    match p {
        Pose::Home => t.set("pose", "Home")?,
        Pose::Exit => t.set("pose", "Exit")?,
        Pose::Cell(r, c) => {
            t.set("pose", "Cell")?;
            t.set("r", r)?;
            t.set("c", c)?;
        }
        Pose::Transfer(r, c) => {
            t.set("pose", "Transfer")?;
            t.set("r", r)?;
            t.set("c", c)?;
        }
    }
    Ok(Value::Table(t))
}

fn pose_from_lua(v: &Value) -> Result<Pose, String> {
    let bad = || {
        format!(
            "bb:go_to expects Home(), Exit(), Cell(r, c) or Transfer(r, c), got {}",
            v.type_name()
        )
    };
    let Value::Table(t) = v else {
        return Err(bad());
    };
    let kind: String = t.get("pose").map_err(|_| bad())?;
    match kind.as_str() {
        "Home" => Ok(Pose::Home),
        "Exit" => Ok(Pose::Exit),
        "Cell" | "Transfer" => {
            let idx = |k: &str| -> Result<usize, String> {
                let n: i64 = t.get(k).map_err(|_| format!("{kind} needs integer {k}"))?;
                usize::try_from(n).map_err(|_| format!("{kind} {k} must be ≥ 0, got {n}"))
            };
            let (r, c) = (idx("r")?, idx("c")?);
            Ok(if kind == "Cell" {
                Pose::Cell(r, c)
            } else {
                Pose::Transfer(r, c)
            })
        }
        other => Err(format!("bb:go_to: unknown pose `{other}`")),
    }
}
