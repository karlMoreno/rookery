//! The camera's view of each cell (Phase 11).
//!
//! Row 11.1 declares this surface over `todo!()`. 11.2 (ray-cast depth in
//! `sim`), 11.3 (a view per cell and the sampling schedule) and 11.4 (the
//! image processing) fill it in without changing a signature.
//!
//! ```text
//!   sim ── Sim::depth_image(view of cell) ──▶ height band ──▶ components ──▶ bodies_seen
//!   world ─ is a shed shell in that box? ──────────────────────────────────▶ shell_in_box
//! ```

use std::path::Path;

use rookery_core::CellId;

/// One cell seen once: what the camera counted, and the truth.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraSample {
    pub cell: CellId,
    /// Biological time of the sample, seconds.
    pub t: f64,
    /// Whether a shed shell lies in this cell's box (the truth, from the sim).
    pub shell_in_box: bool,
    /// Bodies the camera counted on the box floor.
    pub bodies_seen: usize,
}

/// Run `scene` with no harvesting for `bio_seconds` on `seed`, sample every
/// cell's view on the camera schedule, and return every sample. Shells come
/// and go with the molts, so the samples hold both answers.
///
/// # Errors
/// A scene that cannot be loaded.
pub fn fidelity(scene: &Path, seed: u64, bio_seconds: f64) -> Result<Vec<CameraSample>, String> {
    let _ = (scene, seed, bio_seconds);
    todo!("rows 11.2-11.4: depth views and counting")
}
