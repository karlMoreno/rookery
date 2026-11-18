//! Unattended runs (Phase 8): one tray, a behavior tree, a seed, and the
//! world's clock, with no one at the controls. The Phase 8 gate and
//! `rookery run --scenario week_1tray.lua` both go through [`run`].
//!
//! Row 8.1 declares this surface over `todo!()`. 8.2 (detector and events),
//! 8.4 (exit slots) and 8.5 (`autonomy.lua`) fill it in without changing a
//! signature.
//!
//! ```text
//!   run(args) ─▶ Sim::load(scene) + World::new(seed) + ShellMover + OracleDetector
//!            ─▶ Runner<BtHooks> under the fast-forward clock until bio_seconds
//!            ─▶ AutonomyReport { events, boxes_in_slots, errors, physics_seconds }
//! ```

use std::path::PathBuf;

use rookery_core::{CellId, Event};

/// What to run.
#[derive(Debug, Clone)]
pub struct AutonomyArgs {
    /// The farm scene (`scene/generated/rookery.xml`); its metadata is the
    /// `.json` beside it.
    pub scene: PathBuf,
    /// The behavior tree (`lua/trees/autonomy.lua`).
    pub tree: PathBuf,
    /// World seed: the same seed gives the same molts.
    pub seed: u64,
    /// Biological seconds to run.
    pub bio_seconds: f64,
    /// Where to write the run log (`<log>/<id>/…`), or nowhere.
    pub log: Option<PathBuf>,
}

/// What happened.
#[derive(Debug, Clone, PartialEq)]
pub struct AutonomyReport {
    /// Every lifecycle event with the biological time it happened at, in
    /// order: `Molted`, `WindowClosing`, `ShellEaten`, and from 8.2
    /// `BoxLifted` and `WindowMissed`.
    pub events: Vec<(f64, Event)>,
    /// Cells whose box rests in an exit slot when the run ends (8.4).
    pub boxes_in_slots: Vec<CellId>,
    /// Leaf, route, move and shell errors, as in the run log.
    pub errors: Vec<String>,
    /// Physics time simulated, seconds (the CI cost).
    pub physics_seconds: f64,
}

/// Run one tray unattended for `args.bio_seconds` of biological time.
///
/// # Errors
/// A scene, tree or log that cannot be loaded or written.
pub fn run(args: &AutonomyArgs) -> Result<AutonomyReport, String> {
    let _ = args;
    todo!("row 8.5: the autonomy run")
}
