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

use crate::JointTargets;
use crate::kinematics::SceneMeta;
use crate::lift::MacroCommand;
use crate::route::RouteError;

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
        let _ = (from, r, c, meta);
        todo!("row 6.2: engage route, lift, loaded route, lower, release, home")
    }
}
