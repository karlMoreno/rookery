//! `sim`: the safe Rust face of the MuJoCo C API.
//!
//! Pattern: **Facade**. Everything else in Rookery talks to [`Sim`]
//! (`load`, `step`, `snapshot`, `apply`, `body_index`) and never to MuJoCo
//! directly. How: `Sim` will own the raw `mjModel`/`mjData` pointers
//! privately (row 1.6) and hand out only plain Rust values ([`Snapshot`],
//! [`BodyState`]) and take only plain Rust values ([`Targets`]). Why: it
//! confines every `unsafe` block to this crate, where each one carries a
//! `// SAFETY:` invariant and its own tests, so the rest of the workspace
//! stays 100% safe Rust and a MuJoCo upgrade touches one crate.
//!
//! ```text
//!   rookery / gates ──uses──▶ «facade» Sim
//!                             ─────────────────────────────────────────
//!                             + load(path) -> Result<Sim, SimError>
//!                             + step(&mut self)
//!                             + snapshot(&self) -> Snapshot
//!                             + apply(&mut self, &Targets) -> Result<(), SimError>
//!                             + body_index(&self, name) -> Option<usize>
//!                               │ creates            │ reads
//!                               ▼                    ▼
//!                          Snapshot ◆──── 0..* BodyState      Targets
//!                          time: f64        name, pos, quat   Vec<f64> → ctrl
//!                               │
//!                               ▼ unsafe, private (rows 1.5–1.6)
//!                             ffi.rs (bindgen over mujoco.h)
//! ```
//!
//! Row 1.4 declares this surface with every body left as `todo!()`; rows
//! 1.5 (bindings) and 1.6 (safe facade) fill it in without changing a
//! signature. The Phase 1 gate (`tests/gates/tests/phase1_drop.rs`) is
//! written against exactly this surface, so it compiles now and fails at
//! the first `todo!()` until 1.6 lands.

use std::path::Path;

// Raw bindgen output, private to this crate (row 1.5). Only `Sim` (row 1.6)
// will call into it.
mod ffi;

/// Everything that can go wrong at the facade boundary.
///
/// Errors are returned, never panicked, so a bad scene file or a wrong
/// target vector from a behavior can't take the process down (1.8 review
/// checklist: "every error path returns rather than panics").
#[derive(Debug, thiserror::Error)]
pub enum SimError {
    /// `mj_loadXML` rejected the file; the message is MuJoCo's error buffer.
    #[error("failed to load MJCF: {0}")]
    Load(String),
    /// A [`Targets`] vector whose length is not the model's `nu`.
    #[error("target length {got} does not match actuator count {expected}")]
    TargetLen { expected: usize, got: usize },
}

/// Pose of one MuJoCo body in world coordinates, copied out of `mjData`.
#[derive(Debug, Clone, PartialEq)]
pub struct BodyState {
    /// Body name from the MJCF (`""` for an unnamed body; body 0 is `world`).
    pub name: String,
    /// World position `xpos`, metres.
    pub pos: [f64; 3],
    /// World orientation `xquat`, unit quaternion `[w, x, y, z]`.
    pub quat: [f64; 4],
}

/// A typed, owned copy of the sim state at one instant.
///
/// Owned on purpose: callers (Lua via the blackboard, the run log) can keep
/// it after the next `step` without borrowing into MuJoCo's memory.
#[derive(Debug, Clone, PartialEq)]
pub struct Snapshot {
    /// Simulated time `mjData.time`, seconds.
    pub time: f64,
    /// One entry per MuJoCo body, indexed by body id: `bodies[0]` is the
    /// world body, and [`Sim::body_index`] returns an index into this `Vec`.
    pub bodies: Vec<BodyState>,
}

/// Actuator commands for one step, written to `mjData.ctrl` by
/// [`Sim::apply`]. Its length must equal the model's actuator count `nu`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Targets(pub Vec<f64>);

/// The Facade itself: one loaded MuJoCo model plus its simulation data.
///
/// Row 1.6 replaces the placeholder field with
/// `{ model: *mut mjModel, data: *mut mjData, body_names: Vec<String> }`
/// and a `Drop` that frees `data` then `model`. Fields stay private so no
/// caller can reach the raw pointers.
#[derive(Debug)]
pub struct Sim {
    _private: (),
}

impl Sim {
    /// Load an MJCF file and allocate its data (`mj_loadXML` + `mj_makeData`).
    ///
    /// # Errors
    /// [`SimError::Load`] when the file is missing or MuJoCo rejects it.
    pub fn load(path: impl AsRef<Path>) -> Result<Sim, SimError> {
        let _ = path.as_ref();
        todo!("row 1.6: mj_loadXML, mj_makeData, cache body names")
    }

    /// Advance the physics by one `timestep` (`mj_step`).
    pub fn step(&mut self) {
        todo!("row 1.6: mj_step")
    }

    /// Copy `time`, `xpos` and `xquat` for every body into a [`Snapshot`].
    pub fn snapshot(&self) -> Snapshot {
        todo!("row 1.6: read time, xpos, xquat for every body")
    }

    /// Write `targets` to `mjData.ctrl` for the next step.
    ///
    /// # Errors
    /// [`SimError::TargetLen`] when `targets.0.len() != nu`; `ctrl` is left
    /// unchanged in that case.
    pub fn apply(&mut self, targets: &Targets) -> Result<(), SimError> {
        let _ = targets;
        todo!("row 1.6: check len == nu, write ctrl")
    }

    /// Index of the body called `name` in [`Snapshot::bodies`] (equal to its
    /// MuJoCo body id), or `None` if the model has no such body.
    pub fn body_index(&self, name: &str) -> Option<usize> {
        let _ = name;
        todo!("row 1.6: look up the cached body names")
    }
}
