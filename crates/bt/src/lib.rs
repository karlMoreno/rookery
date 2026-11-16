//! `bt`: behavior-tree engine, typed Blackboard and the `mlua` bridge.
//!
//! Row 4.1 declares this surface over `todo!()`. Rows 4.2 (composites), 4.3
//! (Blackboard) and 4.4 (Lua bridge) fill it in without changing a
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

use std::any::Any;
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

/// The Rust-owned store that trees read and write (pattern: **Typed
/// Blackboard**). Row 4.3 implements it.
#[derive(Default)]
pub struct Blackboard {
    entries: Vec<(String, Box<dyn Any + Send>)>,
}

impl std::fmt::Debug for Blackboard {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Blackboard")
            .field(
                "keys",
                &self.entries.iter().map(|(n, _)| n).collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl Blackboard {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register an entry called `name` holding a `T`, and return its key.
    pub fn register<T: Any + Send>(&mut self, name: &str) -> BbKey<T> {
        let _ = (name, &self.entries);
        todo!("row 4.3: store the name and TypeId, return the key")
    }

    /// The value at `key`, if it has been set.
    pub fn get<T: Any + Send>(&self, key: &BbKey<T>) -> Option<&T> {
        let _ = key;
        todo!("row 4.3: look up and downcast")
    }

    /// Set the value at `key`.
    pub fn set<T: Any + Send>(&mut self, key: &BbKey<T>, value: T) {
        let _ = (key, value);
        todo!("row 4.3: store the value")
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
    /// Register every schema key on `bb`.
    pub fn register(bb: &mut Blackboard) -> Self {
        let _ = bb;
        todo!("row 4.3: register the schema keys")
    }
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
