//! The lift (Phase 5): composing `MoveCommand`s into a `MacroCommand`.
//!
//! Row 5.4 declares this surface over `todo!()`, and row 5.5 fills it in.
//! The Phase 5 gate (`tests/gates/tests/phase5_lift.rs`) is written against
//! [`LiftSequence::build`] and [`LIFT_HEIGHT`].
//!
//! Pattern: **Command composition**. A [`MacroCommand`] is itself a
//! [`Command`]: it runs its steps in order, and its `undo` is the completed
//! steps' undos in reverse. Phase 6 builds a whole transfer the same way.
//!
//! ```text
//!   LiftSequence::build(engaged, lift_height) = MacroCommand [
//!     HookLift → +engage_dz        slow: the finger takes the handle's weight
//!     MastZ    → +lift_height      the box leaves the water
//!   ]
//!   undo → MacroCommand [ MastZ back, HookLift back ]
//! ```

use crate::{CmdStatus, Command, GantryObs, GantryState, JointTargets};

/// How far the mast rises after the hook has taken the handle, metres.
/// The box starts on the tray floor with water at 0.05 m, so 8 cm clears
/// it by a few centimetres. Inside the shelf window [0.085, 0.80], so the
/// shelf rule allows it with the arm out.
pub const LIFT_HEIGHT: f64 = 0.08;

/// Steps run in order, each to `Done`.
pub struct MacroCommand {
    steps: Vec<Box<dyn Command>>,
    done: usize,
}

impl MacroCommand {
    pub fn new(steps: Vec<Box<dyn Command>>) -> Self {
        Self { steps, done: 0 }
    }

    /// How many steps have finished.
    pub fn completed(&self) -> usize {
        self.done
    }
}

impl Command for MacroCommand {
    fn tick(&mut self, st: &mut GantryState, obs: &GantryObs, dt: f64) -> CmdStatus {
        let _ = (st, obs, dt, &mut self.steps, self.done);
        todo!("row 5.5: run the current step to Done, then advance")
    }

    fn undo(&self) -> Box<dyn Command> {
        todo!("row 5.5: the completed steps' undos, reversed")
    }
}

/// Builds the lift from an engaged pose.
pub struct LiftSequence;

impl LiftSequence {
    /// The lift for a crane engaged at `engaged` (finger under the handle):
    /// hook up by `PassiveHook::engage_dz`, then mast up by `lift_height`.
    pub fn build(engaged: JointTargets, lift_height: f64) -> MacroCommand {
        let _ = (engaged, lift_height);
        todo!("row 5.5: HookLift +engage_dz (slow), then MastZ +lift_height")
    }
}
