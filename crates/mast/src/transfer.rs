//! A full transfer, cell to exit tray (Phase 6), or into an exit slot (8.4).
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

/// Clearance between a carried box's underside and the handles of boxes
/// already waiting on the exit tray, metres (row 8.5).
pub const LOADED_CLEAR: f64 = 0.02;

/// Mast height for carrying a box over a full exit tray (row 8.5).
///
/// Retracting the arm sweeps the hanging box across the exit tray at the
/// cell's Y, and with slots another box may already sit there. Its handle
/// top is `exit z + box height + handle height`. A hanging box's underside
/// is `mast − engage_z(0)` (the mast height that engages a box resting at
/// z = 0), so the mast must be at least that plus the clearance. Found in
/// 8.5: at the plain travel height, carrying box (3, 1) knocked the box in
/// slot 0 (same Y) off the tray.
pub fn loaded_travel_z(meta: &SceneMeta) -> f64 {
    let exit_top = meta.derived.exit_pose[2] + meta.box_size[2] + meta.handle_height;
    travel_z(meta).max(meta.engage_z(0.0) + exit_top + LOADED_CLEAR)
}

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
        Self::build_to(from, r, c, None, meta)
    }

    /// The same transfer into exit slot `slot` (row 8.4), or onto the middle
    /// of the exit tray with `None`.
    ///
    /// # Errors
    /// [`RouteError::NoSuchCell`] for a cell outside the grid and
    /// [`RouteError::NoSuchSlot`] for a slot the scene doesn't have.
    pub fn build_to(
        from: JointTargets,
        r: usize,
        c: usize,
        slot: Option<usize>,
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
        let exit = match slot {
            None => kinematics::exit_pose(meta),
            Some(i) => kinematics::exit_slot_pose(i, meta).ok_or(RouteError::NoSuchSlot {
                slot: i,
                slots: meta.derived.exit_slots.len(),
            })?,
        };
        // With slots, other boxes may wait on the exit tray: carry above them.
        // Without, the Phase 6 height (the gate's tuning) is unchanged.
        let travel = if slot.is_some() {
            loaded_travel_z(meta)
        } else {
            travel_z(meta)
        };
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

/// Hands out exit slots (row 8.4): the lowest free one, so boxes fill from
/// the middle of the exit tray outwards. `free` returns a slot when its box
/// has been harvested from the tray (or a transfer into it was undone).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlotAllocator {
    taken: Vec<bool>,
}

impl SlotAllocator {
    /// `n` free slots.
    pub fn new(n: usize) -> Self {
        Self {
            taken: vec![false; n],
        }
    }

    /// The lowest free slot, now taken, or `None` when the tray is full.
    pub fn take(&mut self) -> Option<usize> {
        let i = self.taken.iter().position(|t| !t)?;
        self.taken[i] = true;
        Some(i)
    }

    /// Give slot `i` back. Freeing a free or unknown slot does nothing.
    pub fn free(&mut self, i: usize) {
        if let Some(t) = self.taken.get_mut(i) {
            *t = false;
        }
    }

    /// Slots in use.
    pub fn in_use(&self) -> usize {
        self.taken.iter().filter(|t| **t).count()
    }

    /// Slots in all.
    pub fn len(&self) -> usize {
        self.taken.len()
    }

    /// Whether there are no slots at all.
    pub fn is_empty(&self) -> bool {
        self.taken.is_empty()
    }
}
