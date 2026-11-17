//! `bt`: behavior-tree engine, typed Blackboard and the `mlua` bridge.
//!
//! Row 4.1 declared this surface. Row 4.2 added the composites
//! ([`nodes`]), 4.3 the Blackboard, and 4.4 (Lua bridge) fills in the rest without changing a
//! signature. The Phase 4 gates (`tests/gates/tests/phase4_*.rs`) are
//! written against [`Tree`], [`Blackboard`] and [`BbSchema`].
//!
//! Patterns:
//! - **Composite**: Sequence, Fallback, Parallel and the decorators are
//!   [`Node`]s that hold child nodes, so a whole tree ticks through one
//!   [`Node::tick`]. Why: trees become data that Lua can compose and
//!   hot-reload, while the engine that runs them stays small, typed Rust.
//! - **Typed Blackboard**: a key is a [`BbKey<T>`], so asking for the wrong
//!   type is a compile error in Rust (and a named error in Lua, 4.3).
//!
//! ```text
//!               «trait» Node
//!         + tick(&mut Blackboard) -> Status {Success, Failure, Running}
//!         + reset()   + name()
//!               △
//!   ┌───────────┼──────────────┬──────────────┐
//!   Sequence  Fallback     Parallel       Inverter / Repeat      LuaLeaf (4.4)
//!   (memory)  (memory)  {succ, fail thr}   decorators            tick(bb) in Lua
//!
//!   Tree ── root: Box<dyn Node> ── load(tree.lua, &Lua) / tick(&mut Blackboard)
//!   Blackboard ── BbKey<T> ── BbSchema { sim_time, gantry_q, gantry_target, route_len, gantry_goal }
//! ```
//!
//! ## Tree files (contract fixed in 4.1)
//!
//! A tree file is Lua that returns a node built from these globals, which
//! the loader provides:
//!
//! ```lua
//! return Repeat(0, Sequence {          -- Repeat(0, …) repeats forever
//!   Leaf("../leaves/goto_cell.lua", 0, 0),
//!   Leaf("../leaves/at_target.lua"),
//! })
//! ```
//!
//! `Sequence{…}`, `Fallback{…}`, `Parallel{success = n, failure = m, …}`,
//! `Inverter(node)`, `Repeat(n, node)`, `Leaf(path, args…)`. A leaf path is
//! relative to the tree file. A leaf file defines `function tick(bb, …)`,
//! which returns `"success"`, `"failure"` or `"running"`.
//!
//! **Lua sets goals, never raw targets** ([Band 1] lesson, see #19): the only
//! write a leaf has is a goal pose (`bb:goto(pose)`). Rust expands it with
//! `mast::route` and plays it through `mast::clamp`.

pub mod nodes;

use std::any::{Any, TypeId};
use std::marker::PhantomData;
use std::path::{Path, PathBuf};

use mast::route::Pose;
use mlua::Lua;

/// Bumped on any change to [`BbSchema`]'s keys or types.
pub const SCHEMA_VERSION: u32 = 1;

/// The result of ticking a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Success,
    Failure,
    Running,
}

/// One node of a behavior tree (pattern: **Composite**).
pub trait Node {
    /// Run one tick against the blackboard.
    fn tick(&mut self, bb: &mut Blackboard) -> Status;
    /// Forget any `Running` memory, so the next tick starts fresh. It
    /// propagates to the children.
    fn reset(&mut self) {}
    /// A name for logs: the leaf file stem, or the composite kind.
    fn name(&self) -> &str;
}

/// Index of a registered blackboard entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KeyId(u32);

/// A typed handle to one blackboard entry. Holding a `BbKey<T>` is the only
/// way to read or write that entry, and only as a `T`.
#[derive(Debug)]
pub struct BbKey<T> {
    id: KeyId,
    _type: PhantomData<fn() -> T>,
}

impl<T> Clone for BbKey<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for BbKey<T> {}

impl<T> BbKey<T> {
    pub fn id(&self) -> KeyId {
        self.id
    }
}

/// One blackboard entry: its name, its declared type and its value, if set.
struct Entry {
    name: String,
    type_id: TypeId,
    type_name: &'static str,
    value: Option<Box<dyn Any + Send>>,
}

/// Why a by-name blackboard access failed. Lua sees these as named errors (4.4).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BbError {
    /// No key has this name.
    UnknownKey { key: String },
    /// The key holds a different type than the caller asked for.
    WrongType {
        key: String,
        expected: &'static str,
        found: &'static str,
    },
}

impl std::fmt::Display for BbError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownKey { key } => write!(f, "blackboard has no key `{key}`"),
            Self::WrongType {
                key,
                expected,
                found,
            } => {
                write!(f, "blackboard key `{key}` holds {found}, not {expected}")
            }
        }
    }
}

impl std::error::Error for BbError {}

/// The Rust-owned store that trees read and write (pattern: **Typed
/// Blackboard**).
///
/// Entries live in a `Vec` indexed by [`KeyId`], each tagged with the
/// `TypeId` it was registered with. A [`BbKey<T>`] can only be made by
/// [`Blackboard::register`], so typed access through a key is checked at
/// compile time. A wrong type does not compile (see `tests/ui`). Access by
/// name, which is how Lua reaches entries, is checked at run time and fails
/// with a named [`BbError`].
///
/// ```text
///   register::<T>("name") ──▶ KeyId(i) ──▶ entries[i] = { name, TypeId::of::<T>(), value }
///   get(&BbKey<T>)        ──▶ entries[i].value.downcast_ref::<T>()   (cannot mismatch)
///   get_named::<T>("name")──▶ TypeId check ──▶ Ok(&T) | Err(WrongType { key, expected, found })
/// ```
#[derive(Default)]
pub struct Blackboard {
    entries: Vec<Entry>,
}

impl std::fmt::Debug for Blackboard {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Blackboard")
            .field(
                "keys",
                &self
                    .entries
                    .iter()
                    .map(|e| (&e.name, e.type_name))
                    .collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl Blackboard {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register an entry called `name` holding a `T`, and return its key.
    /// Registering the same name and type again returns the same key.
    ///
    /// # Panics
    /// If `name` is already registered with a different type. That is a bug
    /// in the schema, found at startup, never at tick time. Use
    /// [`Blackboard::try_register`] to get an error instead.
    pub fn register<T: Any + Send>(&mut self, name: &str) -> BbKey<T> {
        match self.try_register(name) {
            Ok(key) => key,
            Err(e) => panic!("{e}"),
        }
    }

    /// Like [`Blackboard::register`], but a type conflict is an error.
    pub fn try_register<T: Any + Send>(&mut self, name: &str) -> Result<BbKey<T>, BbError> {
        let want = TypeId::of::<T>();
        if let Some(i) = self.entries.iter().position(|e| e.name == name) {
            let e = &self.entries[i];
            if e.type_id != want {
                return Err(BbError::WrongType {
                    key: name.to_string(),
                    expected: std::any::type_name::<T>(),
                    found: e.type_name,
                });
            }
            return Ok(Self::key(i));
        }
        self.entries.push(Entry {
            name: name.to_string(),
            type_id: want,
            type_name: std::any::type_name::<T>(),
            value: None,
        });
        Ok(Self::key(self.entries.len() - 1))
    }

    fn key<T>(i: usize) -> BbKey<T> {
        BbKey {
            id: KeyId(u32::try_from(i).expect("fewer than 2^32 blackboard keys")),
            _type: PhantomData,
        }
    }

    fn entry(&self, id: KeyId) -> Option<&Entry> {
        self.entries.get(id.0 as usize)
    }

    /// The value at `key`, if it has been set.
    pub fn get<T: Any + Send>(&self, key: &BbKey<T>) -> Option<&T> {
        self.entry(key.id)?.value.as_ref()?.downcast_ref::<T>()
    }

    /// Set the value at `key`.
    pub fn set<T: Any + Send>(&mut self, key: &BbKey<T>, value: T) {
        if let Some(e) = self.entries.get_mut(key.id.0 as usize) {
            e.value = Some(Box::new(value));
        }
    }

    /// The value of the entry called `name`, read as a `T`. Lua access goes
    /// through this, so a wrong type is a named error, not a panic.
    pub fn get_named<T: Any + Send>(&self, name: &str) -> Result<Option<&T>, BbError> {
        let e = self
            .entries
            .iter()
            .find(|e| e.name == name)
            .ok_or_else(|| BbError::UnknownKey {
                key: name.to_string(),
            })?;
        if e.type_id != TypeId::of::<T>() {
            return Err(BbError::WrongType {
                key: name.to_string(),
                expected: std::any::type_name::<T>(),
                found: e.type_name,
            });
        }
        Ok(e.value.as_ref().and_then(|v| v.downcast_ref::<T>()))
    }

    /// The registered key names, in registration order.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.entries.iter().map(|e| e.name.as_str())
    }
}

/// The keys every Rookery tree shares (version [`SCHEMA_VERSION`]).
///
/// Snapshot keys are written by Rust each tick and are read-only to trees.
/// `gantry_goal` is the only key a tree writes ([Band 1] lesson: goals, not
/// targets).
#[derive(Debug, Clone, Copy)]
pub struct BbSchema {
    /// Simulated time, seconds.
    pub sim_time: BbKey<f64>,
    /// Measured joint positions (rail_y, mast_z, arm_x, hook_lift).
    pub gantry_q: BbKey<[f64; 4]>,
    /// Commanded joint targets after `mast::clamp`, for logs only.
    pub gantry_target: BbKey<[f64; 4]>,
    /// Moves left in the current route. 0 means idle.
    pub route_len: BbKey<usize>,
    /// The pose the tree wants. Rust expands it with `mast::route`.
    pub gantry_goal: BbKey<Option<Pose>>,
}

impl BbSchema {
    /// Register every schema key on `bb`. `gantry_goal` starts as `None`
    /// (no goal) and `route_len` as 0, so a tree always reads a value.
    pub fn register(bb: &mut Blackboard) -> Self {
        let schema = Self {
            sim_time: bb.register("sim_time"),
            gantry_q: bb.register("gantry_q"),
            gantry_target: bb.register("gantry_target"),
            route_len: bb.register("route_len"),
            gantry_goal: bb.register("gantry_goal"),
        };
        bb.set(&schema.gantry_goal, None);
        bb.set(&schema.route_len, 0);
        schema
    }

    /// Fill every snapshot key from `snap`. The Integrator calls this once
    /// per tick, before the tree ticks (4.5). It never touches `gantry_goal`.
    pub fn write_snapshot(&self, bb: &mut Blackboard, snap: &GantrySnapshot) {
        bb.set(&self.sim_time, snap.sim_time);
        bb.set(&self.gantry_q, snap.gantry_q);
        bb.set(&self.gantry_target, snap.gantry_target);
        bb.set(&self.route_len, snap.route_len);
    }
}

/// What Rust knows about the crane at the start of a tick, as plain values,
/// so `bt` needs no dependency on `sim`. The Integrator builds it from the
/// sim snapshot and the route queue.
///
/// Per-box poses (`box_pose(name)` in v5) are left out until a leaf needs
/// them: Phase 4's patrol only reads the crane.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct GantrySnapshot {
    pub sim_time: f64,
    pub gantry_q: [f64; 4],
    pub gantry_target: [f64; 4],
    pub route_len: usize,
}

/// A leaf's failure, kept for the run log and the Phase 4 gate.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LeafError {
    /// The leaf's name (its file stem).
    pub leaf: String,
    /// The Lua error message.
    pub message: String,
}

/// Why a tree could not be loaded.
#[derive(Debug)]
pub enum BtError {
    /// The tree or a leaf file could not be read.
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    /// Lua rejected the tree file, or it did not return a node.
    Lua { path: PathBuf, message: String },
}

impl std::fmt::Display for BtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io { path, source } => write!(f, "{}: {source}", path.display()),
            Self::Lua { path, message } => write!(f, "{}: {message}", path.display()),
        }
    }
}

impl std::error::Error for BtError {}

/// A loaded behavior tree.
pub struct Tree {
    root: Box<dyn Node>,
    errors: Vec<LeafError>,
}

impl std::fmt::Debug for Tree {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Tree")
            .field("root", &self.root.name())
            .finish()
    }
}

impl Tree {
    /// Load the tree file at `path` (see "Tree files" above), using `lua`.
    pub fn load(path: impl AsRef<Path>, lua: &Lua) -> Result<Tree, BtError> {
        let _ = (path.as_ref(), lua);
        todo!("rows 4.2 + 4.4: run the tree file with the node constructors, build the nodes")
    }

    /// Tick the root once. A Lua error in a leaf makes that leaf return
    /// `Failure` and is recorded in [`Tree::take_errors`]; it never panics.
    pub fn tick(&mut self, bb: &mut Blackboard) -> Status {
        let _ = (bb, &mut self.root);
        todo!("rows 4.2 + 4.4: tick the root, collect leaf errors")
    }

    /// Leaf errors since the last call, oldest first.
    pub fn take_errors(&mut self) -> Vec<LeafError> {
        std::mem::take(&mut self.errors)
    }
}
