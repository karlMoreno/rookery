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
//! Row 1.4 declared this surface, row 1.5 generated the bindings, and row
//! 1.6 filled in [`Sim`] (in `src/sim.rs`) without changing a signature.
//! The Phase 1 gate (`tests/gates/tests/phase1_drop.rs`) is written against
//! exactly this surface.

#![deny(unsafe_op_in_unsafe_fn)]

// Raw bindgen output, private to this crate (row 1.5). Only `sim.rs` calls
// into it.
mod ffi;
// The safe facade and every `unsafe` block that touches MuJoCo (row 1.6).
mod sim;

pub use sim::Sim;

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
    /// A qpos vector whose length is not the model's `nq`.
    #[error("qpos length {got} does not match nq {expected}")]
    QposLen { expected: usize, got: usize },
    /// [`Sim::set_body_pose`] named a body the model does not have.
    #[error("no body named `{0}`")]
    UnknownBody(String),
    /// [`Sim::set_body_pose`] named a body that is not on a single free joint.
    #[error("body `{0}` has no free joint of its own, so it cannot be placed")]
    NotFree(String),
    /// A gravity-compensation value that is not a finite number.
    #[error("gravcomp {0} is not a finite number")]
    BadGravcomp(f64),
    /// The model was compiled with no gravity-compensated body, so MuJoCo
    /// skips gravity compensation entirely and a new value would do nothing.
    #[error("the model has no gravcomp body, so gravity compensation is off")]
    NoGravcomp,
    /// A quaternion with zero (or non-finite) length, which names no rotation.
    #[error("quaternion {0:?} has no direction")]
    BadQuat([f64; 4]),
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
    /// Magnitude of the external contact force on the body, newtons: the
    /// force part of `cfrc_ext`, which `Sim::step` keeps current (row 5.3).
    pub contact_force: f64,
    /// Rotation about world X, radians, from `quat` (Z-Y-X Euler angles).
    pub roll: f64,
    /// Rotation about world Y, radians, from `quat` (Z-Y-X Euler angles).
    pub pitch: f64,
}

/// `(roll, pitch)` in radians from a unit quaternion `[w, x, y, z]`
/// (Z-Y-X Euler angles). Pitch is clamped at ±90° so it never goes NaN.
pub fn roll_pitch(q: [f64; 4]) -> (f64, f64) {
    let [w, x, y, z] = q;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    (roll, pitch)
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
