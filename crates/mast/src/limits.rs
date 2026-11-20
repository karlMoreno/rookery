//! Safety constants for the stacker crane (rule 5). **Protected path:** only
//! Karl edits this file (CODEOWNERS, gate-guard). Agents may propose a change
//! only in a separate PR that changes nothing else (rule 9).
//!
//! Every target reaches the sim through [`crate::clamp`], which enforces
//! these numbers. Lua can neither read nor write them.
//!
//! Joint coordinates, in metres, in the scene frame from `scene/generated/rookery.json`
//! (the tray floor's top is z = 0, the grid is centred, rows run along +X, and the
//! floor rail runs along Y at x = `rail_line_x` ≈ -0.64):
//!
//! ```text
//!   axis       joint      meaning                                   range
//!   RailY      rail_y     truck position along the floor rail (Y)   -0.40 .. 0.40
//!   MastZ      mast_z     lift-carriage (hook) height (Z)            0.05 .. 0.85
//!   ArmX       arm_x      fork extension from the mast toward +X     0.07 .. 0.85
//!   HookLift   hook_lift  locked: the fork's hook is rigid (Z)       0.00 .. 0.00
//! ```
//!
//! Where the numbers come from (Phase 2 scene, 4x4 cells, 0.15 m pitch):
//! - `RAIL_Y`: cell centres span y = ±0.225 and the rail is 0.90 m long
//!   (±0.45), so the truck stays 5 cm inside the rail ends.
//! - `MAST_Z`: the throwaway preview showed a top-shelf pick needs about 0.80 m
//!   (0.763 m plus margin). Phase 1 trays only need ~0.2 m. Raised to 0.85 m
//!   for the fork (crane swap): with rack3's shelf gap at 0.17 the top shelf
//!   sits at 0.61 m, its lift ends at 0.80 m, and the box still has to rise
//!   over its tray's front wall. The real crane's 1.05 m mast reaches 0.86 m.
//! - `ARM_X`: the reach arm is a two-stage telescoping fork (crane swap, rows
//!   2.3-2.5; hardware in `hardware/crane`, preset `rookery_tray.toml`). Folded,
//!   the hook's tip sits 0.11 m in front of the rail line, short of the exit
//!   tray's aisle edge at 0.17 m, so the mast may move with the fork folded.
//!   Extended, it reaches 0.85 m: the far row needs 0.845 m (rail x ≈ -0.64,
//!   set back 0.17 m from the exit tray so a box on the folded fork clears it,
//!   to x = +0.225, minus half the finger). The old straight arm went 0.00 .. 0.85.
//! - `HOOK_LIFT`: 0. The fork's hook is rigid, as on the real crane: it takes a
//!   handle by raising the mast (`lift::ENGAGE_SPEED`), not with a stroke of its
//!   own. The joint stays in the scene, locked, so targets keep four axes.
//!   It was 0.00 .. 0.03 with the old arm.
//! - `MAX_VEL`: the preview's tuned speeds (cart.toml `[motion]`).
//!
//! Gravity sag: these are *target* limits. The mast's position actuator sags
//! under load unless the scene compensates, so row 3.2 sets `gravcomp="1"` on
//! the moving crane bodies, and `MoveCommand` checks the measured `q`, not
//! just the target, before reporting `Done` (see `DONE_TOL` in `lib.rs`).

/// A closed interval `lo..=hi`. `const`-constructible so limits are constants.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Range {
    pub lo: f64,
    pub hi: f64,
}

impl Range {
    pub const fn new(lo: f64, hi: f64) -> Self {
        Self { lo, hi }
    }

    /// True when `lo <= x <= hi`.
    pub fn contains(&self, x: f64) -> bool {
        self.lo <= x && x <= self.hi
    }

    /// `hi - lo`.
    pub fn height(&self) -> f64 {
        self.hi - self.lo
    }
}

/// An axis-aligned box `min..=max` per coordinate (x, y, z).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// Truck position along the floor rail, metres.
pub const RAIL_Y: Range = Range::new(-0.40, 0.40);
/// Lift-carriage height, metres.
pub const MAST_Z: Range = Range::new(0.05, 0.85);
/// Fork extension toward +X, metres: folded .. fully extended.
pub const ARM_X: Range = Range::new(0.07, 0.85);
/// Hook stroke, metres. Locked: the fork's hook engages with the mast.
pub const HOOK_LIFT: Range = Range::new(0.00, 0.00);

/// Ranges indexed like [`crate::Axis`]: rail, mast, arm, hook.
pub const RANGES: [Range; 4] = [RAIL_Y, MAST_Z, ARM_X, HOOK_LIFT];

/// Maximum speed per axis (rail, mast, arm, hook), m/s. A target may move
/// at most `MAX_VEL[i] * dt` per tick.
pub const MAX_VEL: [f64; 4] = [0.30, 0.15, 0.20, 0.02];

/// Where the hook may ever be, in the scene frame: from the rail to just past
/// the far tray wall, the length of the rail, from the ground to above the top shelf.
pub const WORKSPACE: Aabb = Aabb {
    min: [-0.62, -0.45, -0.01],
    max: [0.35, 0.45, 0.90],
};

/// Vertical thickness of the arm beam and hook, metres. The arm may extend
/// only into a gap taller than this.
pub const ARM_THICKNESS: f64 = 0.04;
/// Arm extensions at or below this count as retracted: clear of every tray,
/// so the mast may move freely, metres. Just past the folded fork (0.07): the
/// hook's tip then stays behind the exit tray's aisle edge (0.17).
pub const ARM_RETRACTED: f64 = 0.075;
/// How far the fork's top rises above the hook, metres: a 45 mm shank from the
/// hook to the fork's underside plus 27 mm of fork (rail, block, bar). Under a
/// shelf the fork, not the handle, is the tallest thing (fit study D4).
pub const FORK_ABOVE_HOOK: f64 = 0.072;

/// Mast heights at which the arm may extend: one window per shelf, derived
/// from the scene (Phase 3 has one tray, Phase 10 three). With the hook
/// lifted by `hook_lift`, a window counts only if its height exceeds
/// `ARM_THICKNESS + hook_lift`, the vertical envelope of the arm and hook.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ShelfWindows(pub Vec<Range>);

impl ShelfWindows {
    /// True when `mast_z` lies inside a window the arm and a hook lifted by
    /// `hook_lift` fit through.
    pub fn admits(&self, mast_z: f64, hook_lift: f64) -> bool {
        self.0
            .iter()
            .any(|w| w.height() > ARM_THICKNESS + hook_lift && w.contains(mast_z))
    }
}
