//! Shelf windows from the scene (row 10.3): the mast heights at which the
//! arm may reach into each shelf of a rack.
//!
//! Row 10.1 declares this surface over `todo!()`. `limits.rs` keeps the
//! absolute envelope and stays untouched; the windows are derived from
//! `rookery.json` at start-up and checked against that envelope, so a scene
//! that asks for the impossible is refused before anything moves.
//!
//! ```text
//!   rookery.json ─ shelf bases, box/handle heights, shelf gap ─▶ ShelfWindows::from_meta
//!        each window ⊂ MAST_Z and taller than ARM_THICKNESS, or a named WindowError
//! ```

use crate::kinematics::{HOOK_CLEARANCE, SceneMeta};
use crate::limits::{ARM_THICKNESS, FORK_ABOVE_HOOK, HOOK_LIFT, MAST_Z, Range, ShelfWindows};

/// Room kept between the lifted box's handle top (or the fork's top) and
/// the floor of the tray above, metres.
pub const UNDER_SHELF_MARGIN: f64 = 0.01;
/// Tray floor thickness, metres (the builder's `TRAY_WALL`).
pub const TRAY_FLOOR: f64 = 0.01;

/// Why a scene's shelf windows were refused.
#[derive(Debug, Clone, PartialEq)]
pub enum WindowError {
    /// Window `shelf` reaches outside `limits::MAST_Z`.
    OutsideMast { shelf: usize, lo: f64, hi: f64 },
    /// Window `shelf` is no taller than the arm (`limits::ARM_THICKNESS`).
    TooThin { shelf: usize, height: f64 },
}

impl std::fmt::Display for WindowError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OutsideMast { shelf, lo, hi } => {
                write!(
                    f,
                    "shelf {shelf}'s window {lo:.3}..{hi:.3} m reaches outside the mast's range"
                )
            }
            Self::TooThin { shelf, height } => {
                write!(
                    f,
                    "shelf {shelf}'s window is {height:.3} m tall; the arm needs more"
                )
            }
        }
    }
}

impl std::error::Error for WindowError {}

impl ShelfWindows {
    /// One window per shelf of the scene, lowest first.
    ///
    /// # Errors
    /// A window outside `MAST_Z` or no taller than `ARM_THICKNESS`.
    ///
    /// ```text
    ///   window t = [ base_t + box height + HOOK_CLEARANCE ,   the finger clears the box walls
    ///                base_{t+1} − floor − (handle top − engage height + max hook lift) − margin ]
    ///                                         the lifted box's handle clears the tray above
    ///   the top shelf's window runs to MAST_Z.hi
    /// ```
    ///
    /// One tray gives the Phase 3 window, `[box top + clearance, MAST_Z.hi]`.
    pub fn from_meta(meta: &SceneMeta) -> Result<ShelfWindows, WindowError> {
        let engage_above_base = meta.engage_z(0.0);
        let handle_top = meta.box_size[2] + meta.handle_height;
        let mut out = Vec::with_capacity(meta.trays);
        for t in 0..meta.trays.max(1) {
            let base = meta.shelf_base(t).unwrap_or(0.0);
            let lo = base + meta.box_size[2] + HOOK_CLEARANCE;
            let hi = match meta.shelf_base(t + 1) {
                // The tallest thing above the hook decides: the lifted
                // handle, or the fork, which rides FORK_ABOVE_HOOK over it
                // (crane swap, fit study D4).
                Some(next) => {
                    next - TRAY_FLOOR
                        - (handle_top - engage_above_base + HOOK_LIFT.hi).max(FORK_ABOVE_HOOK)
                        - UNDER_SHELF_MARGIN
                }
                None => MAST_Z.hi,
            };
            if lo < MAST_Z.lo || hi > MAST_Z.hi {
                return Err(WindowError::OutsideMast { shelf: t, lo, hi });
            }
            if hi - lo <= ARM_THICKNESS {
                return Err(WindowError::TooThin {
                    shelf: t,
                    height: hi - lo,
                });
            }
            out.push(Range::new(lo, hi));
        }
        Ok(ShelfWindows(out))
    }
}
