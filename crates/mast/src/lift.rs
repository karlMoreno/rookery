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
//!     MastZ → +engage_dz              slow: the finger takes the handle's weight
//!     MastZ → +engage_dz+lift_height  the box leaves the water
//!   ]
//!   undo → MacroCommand [ MastZ back, MastZ back ]
//!
//! The fork's hook is rigid (crane swap, D1), so the mast does the engage
//! that the old arm's `hook_lift` stroke did, at the same 1 cm/s.
//! ```

use crate::limits::MAX_VEL;
use crate::{
    Axis, CmdStatus, Command, GantryObs, GantryState, Hook, JointTargets, MoveCommand, PassiveHook,
};

/// How far the mast rises after the hook has taken the handle, metres.
/// The box starts on the tray floor with water at 0.05 m, so 8 cm clears
/// it by a few centimetres. Inside the shelf window [0.085, 0.80], so the
/// shelf rule allows it with the arm out.
pub const LIFT_HEIGHT: f64 = 0.08;

/// How fast the hook rises into the handle during the engage, m/s. The old
/// arm's hook stroke engaged at 0.5 × 0.02 m/s; the mast keeps that pace.
pub const ENGAGE_VEL: f64 = 0.01;
/// Fraction of the mast's `MAX_VEL` for the engage (hook takes the handle's
/// weight) and for setting a box down: `ENGAGE_VEL` / 0.15 m/s.
pub const ENGAGE_SPEED: f64 = ENGAGE_VEL / MAX_VEL[1];
/// Fraction of `MAX_VEL` for the mast rise with the box hanging. Full speed
/// is fine once the hook is a 30 mm saddle: in the probe, 100 offset lifts
/// peaked at 0.83° roll (row 5.5).
pub const LIFT_SPEED: f64 = 1.0;

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

    /// How many steps there are in total.
    pub fn steps(&self) -> usize {
        self.steps.len()
    }
}

impl Command for MacroCommand {
    fn tick(&mut self, st: &mut GantryState, obs: &GantryObs, dt: f64) -> CmdStatus {
        // Several instant steps may finish in one tick; a Running or Failed
        // step stops the walk, and Failed fails the whole macro.
        while self.done < self.steps.len() {
            match self.steps[self.done].tick(st, obs, dt) {
                CmdStatus::Done => self.done += 1,
                other => return other,
            }
        }
        CmdStatus::Done
    }

    /// The completed steps' undos in reverse order, plus the undo of a step
    /// that had started but not finished (it knows where it began), so a
    /// half-done lift lowers back exactly to where it started.
    fn undo(&self) -> Box<dyn Command> {
        let started = (self.done + 1).min(self.steps.len());
        Box::new(MacroCommand::new(
            self.steps[..started]
                .iter()
                .rev()
                .map(|s| s.undo())
                .collect(),
        ))
    }
}

/// Builds the lift from an engaged pose.
pub struct LiftSequence;

impl LiftSequence {
    /// The lift for a crane engaged at `engaged` (finger under the handle):
    /// mast up slowly by `PassiveHook::engage_dz`, then up by `lift_height`.
    pub fn build(engaged: JointTargets, lift_height: f64) -> MacroCommand {
        let engage_z = engaged.0[Axis::MastZ.index()] + PassiveHook.engage_dz();
        MacroCommand::new(vec![
            Box::new(MoveCommand::new(Axis::MastZ, engage_z).with_speed(ENGAGE_SPEED)),
            Box::new(MoveCommand::new(Axis::MastZ, engage_z + lift_height).with_speed(LIFT_SPEED)),
        ])
    }
}
