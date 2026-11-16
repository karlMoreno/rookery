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
//! The route for each `Goto` comes from `mast::route` (moved there in row
//! 4.1, so the behavior tree uses the same one).

use std::path::Path;

use mast::route::{self, Pose};
use mast::{Axis, JointTargets, MoveCommand, SceneMeta};
use serde::Deserialize;

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

/// Expand `steps`, starting from `start`, into the command list to play.
pub fn expand(
    steps: &[Step],
    start: JointTargets,
    meta: &SceneMeta,
) -> Result<Vec<MoveCommand>, String> {
    let mut at = start;
    let mut cmds = Vec::new();
    for step in steps {
        let more = match *step {
            Step::Move(axis, goal) if at.0[axis.index()] != goal => {
                vec![MoveCommand::new(axis, goal)]
            }
            Step::Move(..) => Vec::new(),
            Step::Goto(pose) => route::route(at, pose, meta).map_err(|e| e.to_string())?,
        };
        at = route::end_of(at, &more);
        cmds.extend(more);
    }
    Ok(cmds)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mast::kinematics;

    const META: &str = include_str!("../../../scene/generated/rookery.json");
    const TOUR: &str = include_str!("../../../examples/tour.ron");

    fn meta() -> SceneMeta {
        SceneMeta::from_json(META).expect("rookery.json parses")
    }

    #[test]
    fn the_tour_parses_and_ends_at_home() {
        let steps = parse(TOUR).expect("tour.ron parses");
        assert_eq!(steps.first(), Some(&Step::Goto(Pose::Home)));
        assert!(expand(&[Step::Goto(Pose::Cell(9, 0))], kinematics::home(), &meta()).is_err());
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
    fn raw_moves_pass_through() {
        let steps = parse("[Move(MastZ, 0.3)]").unwrap();
        let cmds = expand(&steps, kinematics::home(), &meta()).unwrap();
        assert_eq!(cmds, vec![MoveCommand::new(Axis::MastZ, 0.3)]);
    }
}
