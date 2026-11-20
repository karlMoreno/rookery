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
//!     LiftSequence                         mast +2 cm slowly (engage), then +8 cm
//!     loaded route → above the exit        travel height clears the box's underside (6.3)
//!     LowerUntilContact                    the box settles on the exit cradle (6.6)
//!     release                              mast down 2 cm slowly, finger out from under the handle
//!     route → Home
//!   ]
//! ```

use crate::kinematics::{self, FOLDED, SceneMeta};
use crate::lift::{ENGAGE_SPEED, LIFT_HEIGHT, LiftSequence, MacroCommand};
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

    /// The transfer of the box in cell (`r`, `c`) of tray `t` (Phase 10)
    /// into exit slot `slot`. On one tray it is [`Transfer::build_to`]. On a
    /// rack every move stays inside a shelf window:
    ///
    /// ```text
    ///   route → CellOn(t, r, c)        mast to the shelf's engage height, rail, reach in
    ///   LiftSequence                   hook +2 cm, mast +8 cm: still under the shelf above
    ///   arm in (carry speed)           the box slides out over its tray's front wall
    ///   mast → the exit's engage height, rail → the slot's Y   (arm retracted, in the aisle)
    ///   arm out over the slot, hook down: the box is set down
    ///   route → Home
    /// ```
    ///
    /// # Errors
    /// [`RouteError::NoSuchTray`], [`RouteError::NoSuchCell`] or
    /// [`RouteError::NoSuchSlot`].
    pub fn build_on(
        from: JointTargets,
        t: usize,
        r: usize,
        c: usize,
        slot: Option<usize>,
        meta: &SceneMeta,
    ) -> Result<MacroCommand, RouteError> {
        if !meta.is_rack() {
            if t != 0 {
                return Err(RouteError::NoSuchTray {
                    tray: t,
                    trays: meta.trays,
                });
            }
            return Self::build_to(from, r, c, slot, meta);
        }
        let mut steps: Vec<Box<dyn Command>> = Vec::new();
        let mut at = from;
        let push =
            |steps: &mut Vec<Box<dyn Command>>, cmds: Vec<MoveCommand>, at: &mut JointTargets| {
                *at = route::end_of(*at, &cmds);
                steps.extend(cmds.into_iter().map(|c| Box::new(c) as Box<dyn Command>));
            };
        let engage = route::route(at, Pose::CellOn(t, r, c), meta)?;
        push(&mut steps, engage, &mut at);
        let engaged = at;
        let engage_dz = crate::Hook::engage_dz(&crate::PassiveHook);
        let lift = LiftSequence::build(engaged, LIFT_HEIGHT);
        at.0[Axis::MastZ.index()] += engage_dz + LIFT_HEIGHT;
        steps.push(Box::new(lift));
        let exit = match slot {
            None => kinematics::exit_pose(meta),
            Some(i) => kinematics::exit_slot_pose(i, meta).ok_or(RouteError::NoSuchSlot {
                slot: i,
                slots: meta.derived.exit_slots.len(),
            })?,
        };
        // Before sliding the box out, rise as high as the shelf allows: on
        // the bottom shelf the box passes over the exit tray's plate, whose
        // top is level with a just-lifted box's underside (row 10.5).
        let exit_reach_z = route::window_top(exit.0[Axis::MastZ.index()], meta)
            .map_or(exit.0[Axis::MastZ.index()], |hi| {
                (hi - route::WINDOW_EPS).max(exit.0[Axis::MastZ.index()])
            });
        let lifted = at.0[Axis::MastZ.index()];
        let clear = route::window_top(lifted, meta)
            .map_or(lifted, |hi| (hi - route::WINDOW_EPS).max(lifted));
        let carry = vec![
            MoveCommand::new(Axis::MastZ, clear),
            MoveCommand::new(Axis::ArmX, FOLDED).with_speed(CARRY_SPEED),
            // Reach over the slot as high as shelf 0's window allows: a
            // hanging box sits about 2 cm lower on the hook than a resting
            // one, so at the exit's engage height it would strike the plate's
            // edge. Then set it down.
            MoveCommand::new(Axis::MastZ, exit_reach_z).with_speed(CARRY_SPEED),
            MoveCommand::new(Axis::RailY, exit.0[Axis::RailY.index()]).with_speed(CARRY_SPEED),
            MoveCommand::new(Axis::ArmX, exit.0[Axis::ArmX.index()]).with_speed(CARRY_SPEED),
            // Hold the box engage_dz above the engage height, so it hangs just
            // over the tray, then set it down slowly with the mast.
            MoveCommand::new(Axis::MastZ, exit.0[Axis::MastZ.index()] + engage_dz),
            MoveCommand::new(Axis::MastZ, exit.0[Axis::MastZ.index()]).with_speed(ENGAGE_SPEED),
        ];
        push(&mut steps, carry, &mut at);
        let home = route::route(at, Pose::Home, meta)?;
        push(&mut steps, home, &mut at);
        Ok(MacroCommand::new(steps))
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
        let engage_dz = crate::Hook::engage_dz(&crate::PassiveHook);

        // 2. Lift: hook takes the handle (slow), mast raises the box clear of the water.
        let lift = LiftSequence::build(engaged, LIFT_HEIGHT);
        at.0[Axis::MastZ.index()] += engage_dz + LIFT_HEIGHT;
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
        // The heights are for the old arm, whose hook stroke held a carried
        // box engage_dz above the mast. The fork's hook is the mast, so carry
        // engage_dz higher to keep the same clearances.
        let travel = engage_dz
            + if slot.is_some() {
                loaded_travel_z(meta)
            } else {
                travel_z(meta)
            };
        let carry = vec![
            MoveCommand::new(Axis::MastZ, travel.max(at.0[Axis::MastZ.index()])),
            MoveCommand::new(Axis::ArmX, FOLDED).with_speed(CARRY_SPEED),
            MoveCommand::new(Axis::RailY, exit.0[Axis::RailY.index()]).with_speed(CARRY_SPEED),
            MoveCommand::new(Axis::ArmX, exit.0[Axis::ArmX.index()]).with_speed(CARRY_SPEED),
            // 4. Lower to engage_dz above the exit engage height, still
            //    holding the box, so it hangs just above the tray.
            MoveCommand::new(Axis::MastZ, exit.0[Axis::MastZ.index()] + engage_dz),
            // 5. Release: the mast sets the box on the tray slowly, leaving
            //    the finger centred under the handle with room to back out.
            //    (The old arm did this with its hook stroke.)
            MoveCommand::new(Axis::MastZ, exit.0[Axis::MastZ.index()]).with_speed(ENGAGE_SPEED),
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

    /// The first free slot in `order` (row 10.5: the rack fills the slots
    /// that block nothing first), now taken, or `None`.
    pub fn take_from(&mut self, order: &[usize]) -> Option<usize> {
        let i = order
            .iter()
            .copied()
            .find(|&i| self.taken.get(i) == Some(&false))?;
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
