//! Safe routes between named poses (moved here from `rookery` in row 4.1,
//! so the scripted tour and the behavior tree share one route builder).
//!
//! Pattern: a small **Builder** of command sequences. A route never sweeps
//! the hook through a handle ([Band 1] lesson from the 3.5 tour):
//!
//! ```text
//!   leave (if the arm is out):  ArmX → arm − PRE   finger out from under the handle
//!                               MastZ → travel     above every handle
//!                               ArmX → FOLDED      fold the fork
//!   approach Cell / Exit:       MastZ → travel
//!                               RailY → y
//!                               ArmX → arm − PRE   finger in front of the handle, inside the box
//!                               MastZ → engage     hook envelope centred under the handle
//!                               ArmX → arm         handle over mid-finger
//!   Home:                       MastZ → 0.45, RailY → 0 (fork already folded)
//! ```
//!
//! Moves whose goal equals the planned position are skipped.

use serde::{Deserialize, Serialize};

use crate::kinematics::{self, FOLDED, HOOK_FINGER, SceneMeta};
use crate::limits::ARM_RETRACTED;
use crate::{Axis, JointTargets, MoveCommand};

/// How far back from the engage pose the finger waits before sliding under
/// the handle: its tip clears the handle by about 5 mm, metres.
pub const PRE: f64 = HOOK_FINGER / 2.0 + 0.015;
/// Clearance of the hook (finger, upturned tip and radius) above the
/// tallest handle while travelling, metres.
pub const TRAVEL_CLEAR: f64 = 0.03;

/// A named crane pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Pose {
    Home,
    /// Engaged under the handle of the box in cell (row, col).
    Cell(usize, usize),
    /// Engaged under the handle of a box on the exit tray.
    Exit,
    /// Engaged under the handle of a box in exit slot `i` (row 8.4).
    ExitSlot(usize),
    /// Engaged under the handle of the box in cell (row, col) of tray `t`
    /// (Phase 10). `Cell(r, c)` is tray 0.
    CellOn(usize, usize, usize),
    /// The transfer job for cell (row, col) of tray `t` (Phase 10).
    TransferOn(usize, usize, usize),
    /// Not a place but a job (row 6.5): move the box in cell (row, col) to
    /// the exit tray and return home. `route` refuses it, and the crane
    /// player runs `Transfer::build` instead.
    Transfer(usize, usize),
}

/// Why a route could not be built.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RouteError {
    /// The cell is outside the scene's grid.
    NoSuchCell {
        r: usize,
        c: usize,
        rows: usize,
        cols: usize,
    },
    /// The pose is a job, not a place to route to (`Pose::Transfer`).
    NotARoute(Pose),
    /// The scene has no exit slot `slot` (it has `slots`).
    NoSuchSlot { slot: usize, slots: usize },
    /// The scene has no tray `tray` (it has `trays`).
    NoSuchTray { tray: usize, trays: usize },
}

impl std::fmt::Display for RouteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoSuchCell { r, c, rows, cols } => {
                write!(f, "Cell({r}, {c}) is outside the {rows}x{cols} grid")
            }
            Self::NotARoute(p) => write!(f, "{p:?} is a job, not a place to route to"),
            Self::NoSuchTray { tray, trays } => {
                write!(
                    f,
                    "tray {tray} does not exist; the scene has {trays} tray(s)"
                )
            }
            Self::NoSuchSlot { slot, slots } => {
                write!(
                    f,
                    "ExitSlot({slot}) does not exist; the scene has {slots} exit slot(s)"
                )
            }
        }
    }
}

impl std::error::Error for RouteError {}

/// The hook height that clears every handle in the scene, on the tray and on the exit tray.
pub fn travel_z(meta: &SceneMeta) -> f64 {
    let handle_top = meta.box_size[2] + meta.handle_height;
    let exit_handle_top = meta.derived.exit_pose[2] + handle_top;
    handle_top.max(exit_handle_top) + TRAVEL_CLEAR
}

/// The single-axis moves that take the crane from `from` to `to` safely.
pub fn route(
    from: JointTargets,
    to: Pose,
    meta: &SceneMeta,
) -> Result<Vec<MoveCommand>, RouteError> {
    let mut plan = Plan {
        at: from,
        cmds: Vec::new(),
    };
    plan.goto(to, meta)?;
    Ok(plan.cmds)
}

/// How far above an engage height a rack approach goes (row 10.5).
pub const APPROACH_RISE: f64 = 0.06;
/// Room kept below a window's top, metres.
pub const WINDOW_EPS: f64 = 0.005;

/// On a rack, the approach height for a target engaged at mast `z`:
/// `APPROACH_RISE` above it, capped just under the top of the window that
/// holds `z`. Off a rack, or outside every window, `z` itself.
pub fn approach_z(z: f64, meta: &SceneMeta) -> f64 {
    window_top(z, meta).map_or(z, |hi| (z + APPROACH_RISE).min(hi - WINDOW_EPS).max(z))
}

/// The top of the shelf window that holds mast height `z`, on a rack.
pub fn window_top(z: f64, meta: &SceneMeta) -> Option<f64> {
    if !meta.is_rack() {
        return None;
    }
    let w = crate::limits::ShelfWindows::from_meta(meta).ok()?;
    w.0.iter().find(|r| r.contains(z)).map(|r| r.hi)
}

/// Where the crane ends after `cmds`, starting from `from`.
pub fn end_of(from: JointTargets, cmds: &[MoveCommand]) -> JointTargets {
    let mut at = from;
    for c in cmds {
        at.0[c.axis.index()] = c.goal;
    }
    at
}

/// The builder: where the crane will be after the commands so far.
struct Plan {
    at: JointTargets,
    cmds: Vec<MoveCommand>,
}

impl Plan {
    fn go(&mut self, axis: Axis, goal: f64) {
        let i = axis.index();
        if self.at.0[i] != goal {
            self.cmds.push(MoveCommand::new(axis, goal));
            self.at.0[i] = goal;
        }
    }

    fn goto(&mut self, pose: Pose, meta: &SceneMeta) -> Result<(), RouteError> {
        let travel = travel_z(meta);
        // On a rack (Phase 10) the arm only ever moves inside a shelf window:
        // it leaves at the height it worked at, and changes level retracted.
        let rack = meta.is_rack();
        // Leave: back the finger out from under any handle, rise, retract.
        if self.at.0[Axis::ArmX.index()] > ARM_RETRACTED {
            let back = (self.at.0[Axis::ArmX.index()] - PRE).max(FOLDED);
            self.go(Axis::ArmX, back);
            if !rack {
                self.go(Axis::MastZ, travel.max(self.at.0[Axis::MastZ.index()]));
            }
            self.go(Axis::ArmX, FOLDED);
        }
        if let Pose::Transfer(..) | Pose::TransferOn(..) = pose {
            return Err(RouteError::NotARoute(pose));
        }
        let cell = |t: usize, r: usize, c: usize| -> Result<JointTargets, RouteError> {
            if t >= meta.trays {
                return Err(RouteError::NoSuchTray {
                    tray: t,
                    trays: meta.trays,
                });
            }
            kinematics::cell_pose_on(t, r, c, meta).ok_or(RouteError::NoSuchCell {
                r,
                c,
                rows: meta.rows,
                cols: meta.cols,
            })
        };
        let target = match pose {
            Pose::Transfer(..) | Pose::TransferOn(..) => unreachable!("refused above"),
            Pose::Home => {
                let h = kinematics::home();
                self.go(Axis::MastZ, h.0[Axis::MastZ.index()]);
                self.go(Axis::RailY, h.0[Axis::RailY.index()]);
                self.go(Axis::HookLift, h.0[Axis::HookLift.index()]);
                return Ok(());
            }
            Pose::Cell(r, c) => cell(0, r, c)?,
            Pose::CellOn(t, r, c) => cell(t, r, c)?,
            Pose::Exit => kinematics::exit_pose(meta),
            Pose::ExitSlot(i) => {
                kinematics::exit_slot_pose(i, meta).ok_or(RouteError::NoSuchSlot {
                    slot: i,
                    slots: meta.derived.exit_slots.len(),
                })?
            }
        };
        if rack {
            // Arm retracted in the aisle: go to an approach height inside the
            // target's window, along the rail, reach in short of the handle,
            // drop to the engage height, then slide under. The approach
            // height clears the exit tray's plate, which on the bottom shelf
            // sits above the engage height (found in 10.5: reaching in at
            // engage height drove the hook into the plate).
            let [y, z, x, _] = target.0;
            self.go(Axis::MastZ, approach_z(z, meta));
            self.go(Axis::RailY, y);
            self.go(Axis::ArmX, (x - PRE).max(FOLDED));
            self.go(Axis::MastZ, z);
            self.go(Axis::ArmX, x);
            return Ok(());
        }
        let [y, z, x, _] = target.0;
        self.go(Axis::MastZ, travel);
        self.go(Axis::RailY, y);
        self.go(Axis::ArmX, (x - PRE).max(FOLDED));
        self.go(Axis::MastZ, z);
        self.go(Axis::ArmX, x);
        Ok(())
    }
}
