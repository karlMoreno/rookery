//! Tour scripts (row 3.5): a RON list of [`Step`]s, expanded into the
//! single-axis [`MoveCommand`]s that [`crate::tour::ScriptHooks`] plays.
//!
//! ```ron
//! // examples/tour.ron
//! [Goto(Home), Goto(Cell(0, 0)), Goto(Cell(3, 3)), Goto(Exit), Goto(Home)]
//! ```
//!
//! A script names poses, not numbers, so it keeps working when the scene
//! changes. Poses come from `mast::kinematics` and `rookery.json`. `Move`
//! is the escape hatch for one raw `MoveCommand`.
//!
//! Pattern: the expansion is a small **Builder** of command sequences. Each
//! `Goto` becomes a route that never sweeps the hook through a handle:
//!
//! ```text
//!   leave (if the arm is out):  ArmX → arm − PRE   finger out from under the handle
//!                               MastZ → travel     above every handle
//!                               ArmX → 0           retract
//!   approach Cell / Exit:       MastZ → travel
//!                               RailY → y
//!                               ArmX → arm − PRE   finger in front of the handle, inside the box
//!                               MastZ → engage     between the box top and the handle bottom
//!                               ArmX → arm         handle over mid-finger
//!   Home:                       MastZ → 0.45, RailY → 0, HookLift → 0
//! ```
//!
//! Moves whose goal equals the planned position are skipped.

use std::path::Path;

use mast::kinematics::{self, HOOK_FINGER};
use mast::limits::ARM_RETRACTED;
use mast::{Axis, JointTargets, MoveCommand, SceneMeta};
use serde::Deserialize;

/// How far back from the engage pose the finger waits before sliding under
/// the handle: its tip clears the handle by about 5 mm, metres.
pub const PRE: f64 = HOOK_FINGER / 2.0 + 0.015;
/// Clearance of the hook (finger, upturned tip and radius) above the
/// tallest handle while travelling, metres.
pub const TRAVEL_CLEAR: f64 = 0.03;

/// A named crane pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum Pose {
    Home,
    /// Engaged under the handle of the box in cell (row, col).
    Cell(usize, usize),
    /// Engaged under the handle of a box on the exit tray.
    Exit,
}

/// One line of a tour script.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub enum Step {
    /// Go to a named pose by a safe route.
    Goto(Pose),
    /// One raw single-axis move: (axis, goal).
    Move(Axis, f64),
}

/// Parse a RON script: a list of [`Step`]s.
pub fn parse(text: &str) -> Result<Vec<Step>, String> {
    ron::from_str(text).map_err(|e| format!("bad tour script: {e}"))
}

/// Read and parse a script file.
pub fn load(path: &Path) -> Result<Vec<Step>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    parse(&text).map_err(|e| format!("{}: {e}", path.display()))
}

/// The hook height that clears every handle in the scene, on the tray and on the exit tray.
pub fn travel_z(meta: &SceneMeta) -> f64 {
    let handle_top = meta.box_size[2] + meta.handle_height;
    let exit_handle_top = meta.derived.exit_pose[2] + handle_top;
    handle_top.max(exit_handle_top) + TRAVEL_CLEAR
}

/// Expand `steps`, starting from `start`, into the command list to play.
pub fn expand(
    steps: &[Step],
    start: JointTargets,
    meta: &SceneMeta,
) -> Result<Vec<MoveCommand>, String> {
    let mut plan = Plan {
        at: start,
        cmds: Vec::new(),
    };
    for step in steps {
        match *step {
            Step::Move(axis, goal) => plan.go(axis, goal),
            Step::Goto(pose) => plan.goto(pose, meta)?,
        }
    }
    Ok(plan.cmds)
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

    fn goto(&mut self, pose: Pose, meta: &SceneMeta) -> Result<(), String> {
        let travel = travel_z(meta);
        // Leave: back the finger out from under any handle, rise, retract.
        if self.at.0[Axis::ArmX.index()] > ARM_RETRACTED {
            let back = (self.at.0[Axis::ArmX.index()] - PRE).max(0.0);
            self.go(Axis::ArmX, back);
            self.go(Axis::MastZ, travel.max(self.at.0[Axis::MastZ.index()]));
            self.go(Axis::ArmX, 0.0);
        }
        let target = match pose {
            Pose::Home => {
                let h = kinematics::home();
                self.go(Axis::MastZ, h.0[Axis::MastZ.index()]);
                self.go(Axis::RailY, h.0[Axis::RailY.index()]);
                self.go(Axis::HookLift, h.0[Axis::HookLift.index()]);
                return Ok(());
            }
            Pose::Cell(r, c) => kinematics::cell_pose(r, c, meta).ok_or_else(|| {
                format!(
                    "Cell({r}, {c}) is outside the {}x{} grid",
                    meta.rows, meta.cols
                )
            })?,
            Pose::Exit => kinematics::exit_pose(meta),
        };
        let [y, z, x, _] = target.0;
        self.go(Axis::MastZ, travel);
        self.go(Axis::RailY, y);
        self.go(Axis::ArmX, (x - PRE).max(0.0));
        self.go(Axis::MastZ, z);
        self.go(Axis::ArmX, x);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const META: &str = include_str!("../../../scene/generated/rookery.json");
    const TOUR: &str = include_str!("../../../examples/tour.ron");

    fn meta() -> SceneMeta {
        SceneMeta::from_json(META).expect("rookery.json parses")
    }

    #[test]
    fn the_tour_parses_and_ends_at_home() {
        let steps = parse(TOUR).expect("tour.ron parses");
        assert_eq!(steps.first(), Some(&Step::Goto(Pose::Home)));
        assert_eq!(steps.last(), Some(&Step::Goto(Pose::Home)));
        let m = meta();
        let cmds = expand(&steps, kinematics::home(), &m).expect("expands");
        let mut at = kinematics::home();
        for c in &cmds {
            at.0[c.axis.index()] = c.goal;
        }
        assert_eq!(at, kinematics::home());
    }

    #[test]
    fn a_route_reaches_the_engage_pose_and_only_descends_in_front_of_the_handle() {
        let m = meta();
        let cmds = expand(&[Step::Goto(Pose::Cell(3, 3))], kinematics::home(), &m).unwrap();
        let goal = kinematics::cell_pose(3, 3, &m).unwrap();
        let last = cmds.last().unwrap();
        assert_eq!((last.axis, last.goal), (Axis::ArmX, goal.0[2]));
        let descend = cmds
            .iter()
            .position(|c| c.axis == Axis::MastZ && c.goal == goal.0[1])
            .unwrap();
        let before = &cmds[descend - 1];
        assert_eq!(before.axis, Axis::ArmX);
        assert!(
            before.goal < goal.0[2],
            "descends only with the finger in front of the handle"
        );
    }

    #[test]
    fn travel_clears_every_handle() {
        let m = meta();
        let z = travel_z(&m);
        assert!(z > m.derived.exit_pose[2] + m.box_size[2] + m.handle_height);
    }

    #[test]
    fn a_cell_outside_the_grid_is_an_error() {
        assert!(expand(&[Step::Goto(Pose::Cell(9, 0))], kinematics::home(), &meta()).is_err());
    }

    #[test]
    fn raw_moves_pass_through() {
        let steps = parse("[Move(MastZ, 0.3)]").unwrap();
        let cmds = expand(&steps, kinematics::home(), &meta()).unwrap();
        assert_eq!(cmds, vec![MoveCommand::new(Axis::MastZ, 0.3)]);
    }
}
