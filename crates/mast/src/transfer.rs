//! A full transfer, cell to exit tray (Phase 6).
//!
//! Row 6.1 declares this surface over `todo!()`; rows 6.2, 6.3 and 6.6 fill
//! it in. The Phase 6 gate (`tests/gates/tests/phase6_transfer.rs`) is
//! written against [`Transfer::build`].
//!
//! Pattern: **Command composition**, like the lift. A transfer is one
//! [`MacroCommand`], so a fault can be undone with its `undo()`, which
//! reverses every completed step plus the one in progress.
//!
//! ```text
//!   Transfer::build(from, r, c, meta) = MacroCommand [
//!     route(from → Cell(r, c))            engage: finger under the handle
//!     LiftSequence                         hook +2 cm, mast +8 cm
//!     loaded route → above the exit        travel height clears the box's underside (6.3)
//!     LowerUntilContact                    the box settles on the exit cradle (6.6)
//!     release                              hook down, finger out from under the handle
//!     route → Home
//!   ]
//! ```

use crate::kinematics::{self, SceneMeta};
use crate::lift::{LIFT_HEIGHT, LiftSequence, MacroCommand};
use crate::route::{self, Pose, RouteError, travel_z};
use crate::{Axis, Command, JointTargets, MoveCommand};

/// Fraction of `MAX_VEL` for horizontal moves with a box hanging. The handle
/// can slide along the saddle in Y, so rail moves at full speed swing the box.
pub const CARRY_SPEED: f64 = 0.3;

/// Builds transfers.
pub struct Transfer;

impl Transfer {
    /// The transfer of the box in cell `(r, c)` to the exit tray, starting
    /// with the crane at `from`, ending at home.
    ///
    /// # Errors
    /// [`RouteError::NoSuchCell`] when `(r, c)` is outside the grid.
    pub fn build(
        from: JointTargets,
        r: usize,
        c: usize,
        meta: &SceneMeta,
    ) -> Result<MacroCommand, RouteError> {
        let mut steps: Vec<Box<dyn Command>> = Vec::new();
        let mut at = from;
        let push =
            |steps: &mut Vec<Box<dyn Command>>, cmds: Vec<MoveCommand>, at: &mut JointTargets| {
                *at = route::end_of(*at, &cmds);
                steps.extend(cmds.into_iter().map(|c| Box::new(c) as Box<dyn Command>));
            };

        // 1. Engage: the safe route to the finger under the handle.
        let engage = route::route(at, Pose::Cell(r, c), meta)?;
        push(&mut steps, engage, &mut at);
        let engaged = at;

        // 2. Lift: hook takes the handle (slow), mast raises the box clear of the water.
        let lift = LiftSequence::build(engaged, LIFT_HEIGHT);
        at.0[Axis::HookLift.index()] += crate::Hook::engage_dz(&crate::PassiveHook);
        at.0[Axis::MastZ.index()] += LIFT_HEIGHT;
        steps.push(Box::new(lift));

        // 3. Carry: rise to travel height, retract, run the rail, reach over
        //    the exit tray. The box hangs about 9 cm below the hook, and at
        //    travel height its underside clears every handle and the exit
        //    tray. The rail only moves with the arm retracted (6.3).
        let exit = kinematics::exit_pose(meta);
        let travel = travel_z(meta);
        let carry = vec![
            MoveCommand::new(Axis::MastZ, travel.max(at.0[Axis::MastZ.index()])),
            MoveCommand::new(Axis::ArmX, 0.0).with_speed(CARRY_SPEED),
            MoveCommand::new(Axis::RailY, exit.0[Axis::RailY.index()]).with_speed(CARRY_SPEED),
            MoveCommand::new(Axis::ArmX, exit.0[Axis::ArmX.index()]).with_speed(CARRY_SPEED),
            // 4. Lower to the exit engage height, still holding the box
            //    (hook lifted), so it hangs just above the tray.
            //    6.6 replaces this with LowerUntilContact.
            MoveCommand::new(Axis::MastZ, exit.0[Axis::MastZ.index()]),
            // 5. Release: hook down sets the box on the tray, leaving the
            //    finger centred under the handle with room to back out.
            MoveCommand::new(Axis::HookLift, 0.0).with_speed(crate::lift::ENGAGE_SPEED),
        ];
        push(&mut steps, carry, &mut at);

        // 6. Home: the route backs the finger out, rises and retracts.
        let home = route::route(at, Pose::Home, meta)?;
        push(&mut steps, home, &mut at);
        Ok(MacroCommand::new(steps))
    }
}
