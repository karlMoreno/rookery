//! Safety constants for the stacker crane (rule 5). **Protected path:** only
//! Karl edits this file (CODEOWNERS, gate-guard). Agents may propose a change
//! only in a separate PR that changes nothing else (rule 9).
//!
//! Every target reaches the sim through [`crate::clamp`], which enforces
//! these numbers. Lua can neither read nor write them.
//!
//! Joint coordinates, in metres, in the scene frame from `scene/generated/rookery.json`
//! (the tray floor's top is z = 0, the grid is centred, rows run along +X, and the
//! floor rail runs along Y at x = `rail_line_x` ≈ -0.59):
//!
//! ```text
//!   axis       joint      meaning                                   range
//!   RailY      rail_y     truck position along the floor rail (Y)   -0.40 .. 0.40
//!   MastZ      mast_z     lift-carriage (hook) height (Z)            0.05 .. 0.80
//!   ArmX       arm_x      arm extension from the mast toward +X      0.00 .. 0.85
//!   HookLift   hook_lift  hook stroke above the carriage (Z)         0.00 .. 0.03
//! ```
//!
//! Where the numbers come from (Phase 2 scene, 4x4 cells, 0.15 m pitch):
//! - `RAIL_Y`: cell centres span y = ±0.225 and the rail is 0.90 m long
//!   (±0.45), so the truck stays 5 cm inside the rail ends.
//! - `MAST_Z`: the throwaway preview showed a top-shelf pick needs about 0.80 m
//!   (0.763 m plus margin). Phase 1 trays only need ~0.2 m. The range is sized
//!   for Phase 10's three stacked trays, so it never has to grow.
//! - `ARM_X`: from the rail (x ≈ -0.59) to the far row (x = +0.225) is 0.815 m,
//!   and the exit tray (x ≈ -0.395) is 0.195 m, so 0.85 m reaches both with margin.
//! - `HOOK_LIFT`: 3 cm. With the arm out, the arm and hook need
//!   `ARM_THICKNESS + hook_lift` of clear height, which is 0.07 m at full lift and fits
//!   Phase 2's 0.08 m shelf gap. A 0.10 m lift would not (kinematics study).
//!   The preview engaged a handle with 1.2 cm of lift plus 1 cm of clearance.
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
pub const MAST_Z: Range = Range::new(0.05, 0.80);
/// Arm extension toward +X, metres.
pub const ARM_X: Range = Range::new(0.00, 0.85);
/// Hook stroke, metres.
pub const HOOK_LIFT: Range = Range::new(0.00, 0.03);

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
/// so the mast may move freely, metres.
pub const ARM_RETRACTED: f64 = 0.05;

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
