//! `mast`: stacker-crane kinematics, independent of the farm.
//!
//! Row 3.1 declared this surface. Row 3.3 filled in the kinematics
//! ([`kinematics`]) and [`MoveCommand`], and row 3.4 fills in [`clamp`],
//! without changing a signature. The
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

pub mod kinematics;
pub mod limits;

use serde::{Deserialize, Serialize};

pub use kinematics::{SceneMeta, cell_pose, exit_pose, home};
pub use limits::ShelfWindows;

/// The crane's four prismatic joints, in the order used by every `[f64; 4]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MoveCommand {
    pub axis: Axis,
    pub goal: f64,
    /// The axis target when the first tick ran, so [`Command::undo`] knows
    /// where to go back to. `None` until then.
    #[serde(default)]
    start: Option<f64>,
    /// Ticks spent waiting for the measured position after the target
    /// reached `goal`. Not logged.
    #[serde(skip)]
    settling: u32,
}

/// How long a [`MoveCommand`] waits for the measured position to settle
/// within [`DONE_TOL`] once its target has reached `goal`, in seconds. After
/// that it reports [`MastError::Timeout`]. A critically damped axis settles
/// in under 1 s (row 3.2 test), so 2 s is generous.
pub const SETTLE_TIMEOUT: f64 = 2.0;

impl MoveCommand {
    pub fn new(axis: Axis, goal: f64) -> Self {
        Self {
            axis,
            goal,
            start: None,
            settling: 0,
        }
    }

    /// Where the axis was when the first tick ran, once it has run.
    pub fn start(&self) -> Option<f64> {
        self.start
    }
}

impl Command for MoveCommand {
    fn tick(&mut self, st: &mut GantryState, obs: &GantryObs, dt: f64) -> CmdStatus {
        let i = self.axis.index();
        let range = limits::RANGES[i];
        if !range.contains(self.goal) {
            return CmdStatus::Failed(MastError::OutOfRange {
                axis: self.axis,
                goal: self.goal,
            });
        }
        let current = st.targets.0[i];
        self.start.get_or_insert(current);

        // Step the target toward the goal by at most MAX_VEL · dt. Row 3.4
        // routes this through `clamp` (range, rate, shelf windows); until
        // then the command keeps to its own axis's rate limit.
        let max_step = limits::MAX_VEL[i] * dt.max(0.0);
        let delta = (self.goal - current).clamp(-max_step, max_step);
        st.targets.0[i] = if (self.goal - current).abs() <= max_step {
            self.goal // land exactly on the goal, with no float residue
        } else {
            current + delta
        };

        if st.targets.0[i] != self.goal {
            return CmdStatus::Running;
        }
        // The target is at the goal. Done once the measured joint agrees,
        // which catches actuator lag and gravity sag (settled in 3.1).
        if (obs.q[i] - self.goal).abs() <= DONE_TOL {
            return CmdStatus::Done;
        }
        self.settling += 1;
        if f64::from(self.settling) * dt > SETTLE_TIMEOUT {
            CmdStatus::Failed(MastError::Timeout { axis: self.axis })
        } else {
            CmdStatus::Running
        }
    }

    /// A move back to where this one started. Before the first tick there is
    /// nothing to reverse, so the undo is a command that is done at once.
    fn undo(&self) -> Box<dyn Command> {
        match self.start {
            Some(start) => Box::new(MoveCommand::new(self.axis, start)),
            None => Box::new(Noop),
        }
    }
}

/// A command that is done immediately: the undo of a move that never ran.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Noop;

impl Command for Noop {
    fn tick(&mut self, _st: &mut GantryState, _obs: &GantryObs, _dt: f64) -> CmdStatus {
        CmdStatus::Done
    }

    fn undo(&self) -> Box<dyn Command> {
        Box::new(Noop)
    }
}

/// Every command the crane ran, with the tick it started on (rule 6: a bug
/// report is a run id). It serialises to JSON for the run log.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct CommandLog(pub Vec<(u64, MoveCommand)>);

impl CommandLog {
    /// Record `cmd` as started on `tick`.
    pub fn push(&mut self, tick: u64, cmd: MoveCommand) {
        self.0.push((tick, cmd));
    }
}

/// The end effector (pattern: **Strategy**). A passive J-hook in Phase 3.
pub trait Hook {
    /// How far the hook must rise under a handle to engage it, metres.
    fn engage_dz(&self) -> f64;
    /// Horizontal reach of the hook past the arm's tip, metres.
    fn reach(&self) -> f64;
}

/// The Phase 3 hook: a passive J-hook. A finger runs along +X from the arm
/// tip with an upturned tip, so it slides under a handle and catches it
/// when lifted. It has no actuator of its own beyond `hook_lift`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PassiveHook;

impl Hook for PassiveHook {
    /// From the engage height, the finger rises half the box-top-to-handle
    /// gap to touch the handle, then a little more to carry it. 2 cm is
    /// within `HOOK_LIFT` (3 cm), as the preview showed (1.2 cm of lift
    /// plus 1 cm of clearance).
    fn engage_dz(&self) -> f64 {
        0.02
    }

    fn reach(&self) -> f64 {
        kinematics::HOOK_FINGER
    }
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
