//! `mast`: stacker-crane kinematics, independent of the farm.
//!
//! Row 3.1 declares this surface over `todo!()`. Rows 3.3 (kinematics and
//! Command) and 3.4 (clamps) fill it in without changing a signature. The
//! Phase 3 gate (`tests/gates/tests/phase3_clamp.rs`) is written against
//! [`clamp`] and [`limits`], so it compiles now and fails until 3.4.
//!
//! Patterns:
//! - **Command**: every gantry move is a [`MoveCommand`] that runs across
//!   ticks ([`Command::tick`] reports [`CmdStatus`]) and can produce its
//!   reverse ([`Command::undo`]). Why: Phase 5 sequences a lift out of
//!   commands, and Phase 6 reverses a half-done transfer with `undo`.
//! - **Strategy**: [`Hook`] (a passive J-hook now, active later) is a trait,
//!   so swapping hardware is a new impl, not an edit.
//!
//! ```text
//!            «trait» Command                         «trait» Hook
//!   + tick(&mut GantryState, &GantryObs, dt)       + engage_dz() -> f64
//!       -> CmdStatus {Running, Done, Failed}       + reach() -> f64
//!   + undo() -> Box<dyn Command>                          △
//!            △                                      PassiveHook (3.3)
//!       MoveCommand { axis, goal, start }
//!            │ every new target goes through
//!            ▼
//!   clamp(prev, next, dt, &ShelfWindows) -> JointTargets ── reads ── limits.rs (protected)
//! ```
//!
//! `mast` never depends on `sim` or `rookery-core`. The Integrator fills
//! [`GantryObs`] from the sim snapshot, and CI checks `cargo tree -p mast`.

pub mod limits;

pub use limits::ShelfWindows;

/// The crane's four prismatic joints, in the order used by every `[f64; 4]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Axis {
    /// `rail_y`: the truck along the floor rail.
    RailY,
    /// `mast_z`: the lift carriage up the mast.
    MastZ,
    /// `arm_x`: the arm reaching toward the trays.
    ArmX,
    /// `hook_lift`: the hook's own short stroke.
    HookLift,
}

impl Axis {
    /// All axes, in index order.
    pub const ALL: [Axis; 4] = [Axis::RailY, Axis::MastZ, Axis::ArmX, Axis::HookLift];

    /// Position of this axis in a `[f64; 4]`.
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// Position targets for the four joints, indexed by [`Axis::index`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct JointTargets(pub [f64; 4]);

/// What the crane is currently commanded to do.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GantryState {
    pub targets: JointTargets,
}

/// What the crane observes, filled by the Integrator from the sim snapshot,
/// so `mast` never depends on `sim`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GantryObs {
    /// Measured joint positions, indexed by [`Axis::index`].
    pub q: [f64; 4],
    /// Contact force on the end effector, newtons.
    pub ee_contact: f64,
}

/// Why a command could not finish.
#[derive(Debug, Clone, PartialEq)]
pub enum MastError {
    /// The joint did not reach its goal within the allowed ticks.
    Timeout { axis: Axis },
    /// The goal lies outside the joint's range in [`limits`].
    OutOfRange { axis: Axis, goal: f64 },
}

impl std::fmt::Display for MastError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout { axis } => write!(f, "{axis:?} did not reach its goal in time"),
            Self::OutOfRange { axis, goal } => {
                write!(f, "{axis:?} goal {goal} is outside its range")
            }
        }
    }
}

impl std::error::Error for MastError {}

/// The result of one [`Command::tick`].
#[derive(Debug, Clone, PartialEq)]
pub enum CmdStatus {
    Running,
    Done,
    Failed(MastError),
}

/// One unit of crane motion that runs across ticks (pattern: **Command**).
pub trait Command {
    /// Advance by `dt` seconds: update `st.targets` (always through [`clamp`])
    /// and report whether the command is still running.
    fn tick(&mut self, st: &mut GantryState, obs: &GantryObs, dt: f64) -> CmdStatus;
    /// The command that reverses this one, from wherever it started.
    fn undo(&self) -> Box<dyn Command>;
}

/// How close the *measured* joint position must be to `goal` before a
/// [`MoveCommand`] reports `Done`, metres. `Done` needs both: the target has
/// reached `goal` exactly, and `|obs.q - goal| <= DONE_TOL`. The second
/// check catches actuator lag and gravity sag. Not a safety limit, so it
/// lives here and not in `limits.rs`.
pub const DONE_TOL: f64 = 0.002;

/// Drive one axis to `goal` at no more than its `MAX_VEL`.
///
/// Field names are fixed here for rows 3.3–3.5: `goal` is where the axis
/// goes and `start` is where it was. The plan's 3.1 text says `target`; that
/// word now means only the per-tick value in [`JointTargets`].
#[derive(Debug, Clone, PartialEq)]
pub struct MoveCommand {
    pub axis: Axis,
    pub goal: f64,
    /// The axis target when the first tick ran, so [`Command::undo`] knows
    /// where to go back to. `None` until then.
    start: Option<f64>,
}

impl MoveCommand {
    pub fn new(axis: Axis, goal: f64) -> Self {
        Self {
            axis,
            goal,
            start: None,
        }
    }
}

impl Command for MoveCommand {
    fn tick(&mut self, st: &mut GantryState, obs: &GantryObs, dt: f64) -> CmdStatus {
        let _ = (st, obs, dt, self.start);
        todo!("row 3.3: step toward goal at MAX_VEL through clamp, Done within tolerance")
    }

    fn undo(&self) -> Box<dyn Command> {
        todo!("row 3.3: a MoveCommand back to `start`")
    }
}

/// The end effector (pattern: **Strategy**). A passive J-hook in Phase 3.
pub trait Hook {
    /// How far the hook must rise under a handle to engage it, metres.
    fn engage_dz(&self) -> f64;
    /// Horizontal reach of the hook past the arm's tip, metres.
    fn reach(&self) -> f64;
}

/// Make `next` safe to send, given the previous targets `prev` and the tick
/// length `dt` (seconds):
///
/// 1. every axis within its range in [`limits::RANGES`];
/// 2. every axis moves at most `limits::MAX_VEL[i] * dt` from `prev`;
/// 3. the arm extends beyond [`limits::ARM_RETRACTED`] only while the mast is
///    inside a window of `windows` taller than `ARM_THICKNESS + hook_lift`.
///
/// `prev` is assumed to satisfy all three already, being the previous output.
/// From an illegal `prev`, rules 2 and 3 can conflict, and no output can
/// satisfy both.
///
/// How rule 3 is kept without breaking rule 2 (settled in row 3.1): when the
/// arm is extended (in `prev` or `next`), a mast move or hook lift that would
/// leave every admitting window is refused, and mast Z and hook lift are held
/// at `prev`. The arm is never snapped to `ARM_RETRACTED`, because that could
/// move it faster than `MAX_VEL · dt`. The arm may only extend once the mast
/// is already in a window.
pub fn clamp(
    prev: &JointTargets,
    next: JointTargets,
    dt: f64,
    windows: &ShelfWindows,
) -> JointTargets {
    let _ = (prev, next, dt, windows);
    todo!("row 3.4: range, velocity and shelf-window clamps")
}
