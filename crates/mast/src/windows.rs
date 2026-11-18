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

use crate::kinematics::SceneMeta;
use crate::limits::ShelfWindows;

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
    pub fn from_meta(meta: &SceneMeta) -> Result<ShelfWindows, WindowError> {
        let _ = meta;
        todo!("row 10.3: windows from the rack's shelves")
    }
}
