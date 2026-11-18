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
pub mod lift;
pub mod limits;
pub mod route;
pub mod transfer;

use serde::{Deserialize, Serialize};

pub use kinematics::{SceneMeta, cell_pose, exit_pose, exit_slot_pose, home};
pub use lift::{LIFT_HEIGHT, LiftSequence, MacroCommand};
pub use limits::ShelfWindows;
pub use transfer::{SlotAllocator, Transfer};

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

/// What the crane is currently commanded to do, and the shelf windows its
/// commands must respect. Every target a [`Command`] writes goes through
/// [`clamp`] with these windows (row 3.4). Build them from the scene with
/// [`kinematics::shelf_windows`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GantryState {
    pub targets: JointTargets,
    pub windows: ShelfWindows,
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
    /// [`clamp`] held the axis (the shelf rule) for longer than
    /// [`SETTLE_TIMEOUT`], so the move cannot make progress from here.
    Blocked { axis: Axis },
}

impl std::fmt::Display for MastError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Timeout { axis } => write!(f, "{axis:?} did not reach its goal in time"),
            Self::OutOfRange { axis, goal } => {
                write!(f, "{axis:?} goal {goal} is outside its range")
            }
            Self::Blocked { axis } => write!(f, "{axis:?} is held by the shelf rule"),
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
    /// Ticks in a row on which `clamp` let the target make no progress. Not logged.
    #[serde(skip)]
    blocked: u32,
    /// Fraction of the axis's `MAX_VEL` to move at, in (0, 1]. 1 by default.
    /// A slow segment (the lift's engage) asks for less; `clamp` still caps
    /// every step at `MAX_VEL`, so this can only slow a move down (row 5.5).
    #[serde(default = "full_speed")]
    pub speed: f64,
    /// Current commanded speed along the axis, m/s, for the acceleration
    /// ramp. Starts at 0. Not logged.
    #[serde(skip)]
    vel: f64,
}

/// Acceleration limit per axis (rail, mast, arm, hook), m/s². Every move
/// ramps up and down at this rate instead of jumping to full speed: a box
/// hanging from the saddle slides along its handle under abrupt starts and
/// stops (row 6.6 probe), and undo after a release then misses its cell.
/// Not a safety limit (clamp still caps every step at MAX_VEL), so it lives
/// here and not in limits.rs.
pub const MAX_ACCEL: [f64; 4] = [0.5, 0.5, 0.5, 0.1];

fn full_speed() -> f64 {
    1.0
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
            blocked: 0,
            speed: 1.0,
            vel: 0.0,
        }
    }

    /// The same move at `speed` × `MAX_VEL` (clamped to (0, 1]).
    pub fn with_speed(mut self, speed: f64) -> Self {
        self.speed = if speed.is_finite() {
            speed.clamp(0.01, 1.0)
        } else {
            1.0
        };
        self
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

        // Ask for the goal outright and let `clamp` do the rest: range,
        // MAX_VEL · dt, and the shelf rule (row 3.4). Within one step, clamp
        // passes the goal through untouched, so the target lands on it exactly.
        let prev = st.targets;
        let mut want = prev;
        // Trapezoid profile: accelerate at MAX_ACCEL up to speed × MAX_VEL,
        // and slow down so as to stop at the goal (v ≤ √(2·a·remaining)).
        let dt_ = dt.max(0.0);
        let remaining = (self.goal - current).abs();
        let a = MAX_ACCEL[i];
        let vmax = self.speed * limits::MAX_VEL[i];
        self.vel = (self.vel + a * dt_)
            .min(vmax)
            .min((2.0 * a * remaining).sqrt());
        let step = self.vel * dt_;
        want.0[i] = if remaining <= step || remaining <= 1e-9 {
            self.goal
        } else {
            current + step.copysign(self.goal - current)
        };
        st.targets = clamp(&prev, want, dt, &st.windows);

        if st.targets.0[i] == self.goal {
            self.vel = 0.0; // arrived: the next move starts from rest
        }
        if st.targets.0[i] != self.goal {
            if st.targets.0[i] == current && dt > 0.0 {
                self.blocked += 1;
                if f64::from(self.blocked) * dt > SETTLE_TIMEOUT {
                    return CmdStatus::Failed(MastError::Blocked { axis: self.axis });
                }
            } else {
                self.blocked = 0;
            }
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
            Some(start) => Box::new(MoveCommand::new(self.axis, start).with_speed(self.speed)),
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
/// 4. (row 6.3) the rail moves only while the arm is at or below
///    [`limits::ARM_RETRACTED`] in both `prev` and the output;
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
///
/// A non-finite `next` value (NaN, ±∞) is treated as "stay at `prev`", and
/// a negative or non-finite `dt` as 0, so nothing moves.
///
/// ```text
///   next ──range──▶ ──rate (|Δ| ≤ MAX_VEL·dt)──▶ ──shelf rule──▶ out
///                                                 arm out & not admitted?
///                                                   prev arm out → hold mast_z, hook_lift at prev
///                                                   prev arm in  → arm stops at ARM_RETRACTED
/// ```
pub fn clamp(
    prev: &JointTargets,
    next: JointTargets,
    dt: f64,
    windows: &ShelfWindows,
) -> JointTargets {
    use limits::{ARM_RETRACTED, MAX_VEL, RANGES};
    let dt = if dt.is_finite() && dt > 0.0 { dt } else { 0.0 };
    let mut out = *prev;
    for i in 0..4 {
        let p = prev.0[i];
        let n = next.0[i];
        let n = if n.is_finite() {
            n.clamp(RANGES[i].lo, RANGES[i].hi)
        } else {
            p
        };
        let max = MAX_VEL[i] * dt;
        // Pass `n` through unchanged when it is within one step, so a goal
        // inside the step is hit exactly, with no float residue.
        out.0[i] = if (n - p).abs() <= max {
            n
        } else {
            p + max.copysign(n - p)
        };
    }

    let (z, x, h) = (
        Axis::MastZ.index(),
        Axis::ArmX.index(),
        Axis::HookLift.index(),
    );
    if out.0[x] > ARM_RETRACTED && !windows.admits(out.0[z], out.0[h]) {
        if prev.0[x] > ARM_RETRACTED {
            // The arm is already out, so prev is admitted (the invariant).
            // Refuse the mast and lift moves that would leave the window.
            // Holding them is always within the rate bound.
            out.0[z] = prev.0[z];
            out.0[h] = prev.0[h];
        } else {
            // The arm wants to extend outside a window: let it go no
            // further than retracted. prev ≤ ARM_RETRACTED < out, so this
            // step is shorter than the one the rate clamp allowed.
            out.0[x] = ARM_RETRACTED;
        }
    }

    // Carry safety (row 6.3): the truck may run along the rail only with the
    // arm retracted, so a hanging box is never swept sideways through the
    // other boxes. Holding the rail at prev is always within range and rate.
    let y = Axis::RailY.index();
    if out.0[y] != prev.0[y] && (prev.0[x] > ARM_RETRACTED || out.0[x] > ARM_RETRACTED) {
        out.0[y] = prev.0[y];
    }
    out
}
