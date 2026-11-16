//! Where the crane must be to engage a box's handle (row 3.3).
//!
//! Inputs come from `scene/generated/rookery.json` (read into [`SceneMeta`]),
//! so `mast` follows the scene without depending on it. Outputs are
//! [`JointTargets`] in the `limits.rs` coordinates. Since 3.2 those are
//! exactly the MJCF joint values.
//!
//! ```text
//!   side view (X →), one box, hook engaged under the handle
//!
//!            handle_r_c (capsule along Y, radius 6 mm)
//!               ( o )            ← handle bottom = box_h + handle_height − 12 mm
//!    hook ──[=======]╮          ← finger along +X at mast_z, tip turns up
//!   ┌──────────────────────┐    ← box top, box_h
//!   │        box_r_c       │
//!   └──────────────────────┘    z = 0 (tray floor)
//!
//!   mast_z (engage) = midway between the box top and the handle bottom
//!   arm_x           = cell_x − rail_line_x − HOOK_FINGER / 2   (handle over mid-finger)
//!   rail_y          = cell_y
//! ```
//!
//! These are *engage* poses. How the crane gets there (travel above the
//! handles, descend in front of the target handle, slide under it) is a
//! sequence of `MoveCommand`s, and Phase 5 builds it.

use std::path::Path;

use serde::Deserialize;

use crate::JointTargets;
use crate::limits::{MAST_Z, Range, ShelfWindows};

/// Hook finger length along +X from the arm tip, metres. Matches
/// `HOOK_FINGER` in `scene/scene/builder.py`.
pub const HOOK_FINGER: f64 = 0.04;
/// Handle capsule diameter, metres. Matches `2 * HANDLE_RADIUS` in
/// `scene/scene/builder.py`.
pub const HANDLE_DIAMETER: f64 = 0.012;
/// Home pose (rail_y, mast_z, arm_x, hook_lift). Matches `GANTRY_HOME` in
/// `scene/scene/builder.py`, which is the MJCF `ref` and the `home` keyframe.
pub const HOME: JointTargets = JointTargets([0.0, 0.45, 0.0, 0.0]);

/// The parts of `rookery.json` the kinematics needs.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct SceneMeta {
    pub rows: usize,
    pub cols: usize,
    pub cell_pitch: f64,
    /// (width X, depth Y, height Z).
    pub box_size: [f64; 3],
    pub handle_height: f64,
    pub derived: Derived,
}

/// Placements the scene resolved (`"derived"` in `rookery.json`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct Derived {
    /// Centre of the exit tray's top surface (x, y, z).
    pub exit_pose: [f64; 3],
    /// X of the floor rail.
    pub rail_line_x: f64,
}

/// Why scene metadata could not be read.
#[derive(Debug)]
pub enum MetaError {
    Io(std::io::Error),
    Json(serde_json::Error),
}

impl std::fmt::Display for MetaError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "cannot read scene metadata: {e}"),
            Self::Json(e) => write!(f, "bad scene metadata: {e}"),
        }
    }
}

impl std::error::Error for MetaError {}

impl SceneMeta {
    /// Parse the text of `rookery.json`. Unknown keys are ignored.
    pub fn from_json(text: &str) -> Result<Self, MetaError> {
        serde_json::from_str(text).map_err(MetaError::Json)
    }

    /// Read and parse `rookery.json`.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, MetaError> {
        let text = std::fs::read_to_string(path).map_err(MetaError::Io)?;
        Self::from_json(&text)
    }

    /// (x, y) of cell `(r, c)` in the tray frame. This is the same formula as
    /// `SceneSpec.cell_center`: the grid is centred and rows run along +X.
    pub fn cell_center(&self, r: usize, c: usize) -> (f64, f64) {
        let x = (r as f64 - (self.rows as f64 - 1.0) / 2.0) * self.cell_pitch;
        let y = (c as f64 - (self.cols as f64 - 1.0) / 2.0) * self.cell_pitch;
        (x, y)
    }

    /// Hook height that puts the finger midway between the box top and the
    /// handle's underside, for a box resting at `base_z`.
    fn engage_z(&self, base_z: f64) -> f64 {
        let box_top = base_z + self.box_size[2];
        let handle_bottom = box_top + self.handle_height - HANDLE_DIAMETER;
        (box_top + handle_bottom) / 2.0
    }

    fn arm_for(&self, x: f64) -> f64 {
        x - self.derived.rail_line_x - HOOK_FINGER / 2.0
    }
}

/// Engage pose under the handle of the box in cell `(r, c)`, or `None`
/// if the cell is outside the grid.
pub fn cell_pose(r: usize, c: usize, meta: &SceneMeta) -> Option<JointTargets> {
    if r >= meta.rows || c >= meta.cols {
        return None;
    }
    let (x, y) = meta.cell_center(r, c);
    Some(JointTargets([y, meta.engage_z(0.0), meta.arm_for(x), 0.0]))
}

/// Engage pose under the handle of a box sitting on the exit tray.
pub fn exit_pose(meta: &SceneMeta) -> JointTargets {
    let [x, y, z] = meta.derived.exit_pose;
    JointTargets([y, meta.engage_z(z), meta.arm_for(x), 0.0])
}

/// Shelf windows for the scene: the mast heights at which the arm may
/// extend over a tray.
///
/// Phase 3 has one tray and nothing above it, so there is one window from
/// just above the box walls (box top + [`HOOK_CLEARANCE`], so the finger
/// clears the walls as it slides in) to the top of the mast. Phase 10 adds a
/// window per shelf, capped by the underside of the shelf above.
pub fn shelf_windows(meta: &SceneMeta) -> ShelfWindows {
    let lo = meta.box_size[2] + HOOK_CLEARANCE;
    ShelfWindows(vec![Range::new(lo, MAST_Z.hi)])
}

/// Gap kept between the hook finger's centreline and a box's wall top when
/// the arm is out: the finger radius (4 mm) plus 1 mm, metres.
pub const HOOK_CLEARANCE: f64 = 0.005;

/// Where the hook is in the scene frame for `t`: `(rail_line_x + arm_x,
/// rail_y, mast_z + hook_lift)`. Used to check `limits::WORKSPACE`.
pub fn forward(t: &JointTargets, meta: &SceneMeta) -> [f64; 3] {
    let [y, z, x, h] = t.0;
    [meta.derived.rail_line_x + x, y, z + h]
}

/// The home pose: truck centred, carriage at 0.45 m, arm retracted, hook down.
pub fn home() -> JointTargets {
    HOME
}
