//! Benchmark rows (Phase 12): one sweep point run unattended and boiled down
//! to one CSV row with a fixed column order.
//!
//! Row 12.1 declares this surface over `todo!()`; 12.2 (molt rate, metrics)
//! fills it in. `scene/scene/bench.py` (12.3) builds each point's scene in a
//! temp directory and asks `rookery` for the row; the Phase 12 gate checks
//! that the same point and seed give the same row, byte for byte.
//!
//! ```text
//!   Point { scene, molt_rate, detector, seed, days } ─▶ autonomy::run_with ─▶ metrics ─▶ CSV row
//! ```

use std::path::PathBuf;

use rookery_core::DetectorKind;

/// The columns, in order. Changing them changes every results file.
pub const CSV_HEADER: &str = "scene,trays,cells,molt_rate,detector,seed,days,molts,missed_windows,\
latency_mean_s,latency_p95_s,handlings_per_animal,precision,recall,physics_s";

/// One sweep point.
#[derive(Debug, Clone, PartialEq)]
pub struct Point {
    /// The scene (its metadata is the `.json` beside it).
    pub scene: PathBuf,
    /// Multiplier on the molt rate: 1 is the default cycle, 3 is about an
    /// eyestalk-ablated one (every stage a third as long).
    pub molt_rate: f64,
    pub detector: DetectorKind,
    pub seed: u64,
    /// Biological days to run (the counted horizon; an 8 h tail follows).
    pub days: f64,
}

/// Run `point` with `lua/trees/autonomy.lua` and return its CSV row, in
/// [`CSV_HEADER`]'s order.
///
/// # Errors
/// A scene or tree that cannot be loaded.
pub fn row(point: &Point) -> Result<String, String> {
    let _ = point;
    todo!("row 12.2: molt rate and run metrics")
}
