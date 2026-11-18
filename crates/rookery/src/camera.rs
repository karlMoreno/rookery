//! The camera's view of each cell (Phase 11).
//!
//! Each cell has a top-down depth view of its box's floor: one ray per pixel
//! (`Sim::ray`, row 11.2), straight down from above the box, so no OpenGL is
//! involved and the Air and CI see the same thing. The rays look through the
//! water, the handle bar and its posts, the way a depth camera with a
//! refraction model would. `rookery_core::vision` (row 11.4) then counts the
//! bodies lying on the floor.
//!
//! ```text
//!   eye ─┬─┬─┬─  RES × RES rays over the box's inside, from EYE_ABOVE the floor
//!        ▼ ▼ ▼   (on the rack that is below the shelf above)
//!   ── crawfish ── shell ──  box floor ─▶ heights ─▶ band ─▶ components ─▶ bodies_seen
//! ```

use std::path::Path;

use mast::SceneMeta;
use rookery_core::vision::{self, HeightImage};
use rookery_core::{CellId, LifecycleParams, World};
use sim::{Sim, Targets};

use crate::shells::ShellMover;

/// Pixels per side of a view: about 3.4 mm per pixel over a 0.108 m box.
pub const RES: usize = 32;
/// Height of the eye above the box floor, metres: above the handle (0.125),
/// under the next shelf (0.275 on the rack).
pub const EYE_ABOVE: f64 = 0.20;
/// What lies on the floor: from 3 mm (above the floor's own noise) to 4 cm
/// (above the crawfish, 24 mm tall, and well below the handle).
pub const BAND: (f64, f64) = (0.003, 0.04);
/// Smaller components are specks at a wall's edge, not bodies.
pub const MIN_PX: usize = 10;
/// Box floor thickness and wall, metres (the builder's BOX_FLOOR, BOX_WALL).
const BOX_FLOOR: f64 = 0.004;
const BOX_WALL: f64 = 0.003;
/// How often to sample when no lifecycle event comes sooner (fidelity runs).
const SAMPLE_EVERY: f64 = 6.0 * 3600.0;

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

/// Every cell's camera.
pub struct Cameras {
    meta: SceneMeta,
    /// Geoms the rays look through: the water, handles and handle posts.
    skip: Vec<usize>,
}

impl Cameras {
    pub fn new(sim: &Sim, meta: &SceneMeta) -> Cameras {
        let skip = sim
            .geom_names()
            .iter()
            .enumerate()
            .filter(|(_, n)| {
                n.starts_with("water") || n.starts_with("handle_") || n.starts_with("post_")
            })
            .map(|(i, _)| i)
            .collect();
        Cameras {
            meta: meta.clone(),
            skip,
        }
    }

    /// The heights above `cell`'s box floor, `RES × RES`, row-major.
    pub fn heights(&self, sim: &Sim, cell: CellId) -> HeightImage {
        let (cx, cy) = self.meta.cell_center(cell.r.into(), cell.c.into());
        let floor = self.meta.shelf_base(cell.tray.into()).unwrap_or(0.0) + BOX_FLOOR;
        let eye = floor + EYE_ABOVE;
        // Stay a pixel inside the walls so they never fill the edge.
        let half = self.meta.box_size[0].min(self.meta.box_size[1]) / 2.0 - BOX_WALL - 0.004;
        let mut z = Vec::with_capacity(RES * RES);
        for j in 0..RES {
            for i in 0..RES {
                let x = cx - half + 2.0 * half * (i as f64 + 0.5) / RES as f64;
                let y = cy - half + 2.0 * half * (j as f64 + 0.5) / RES as f64;
                let h = sim
                    .ray([x, y, eye], [0.0, 0.0, -1.0], &self.skip)
                    .map_or(f64::NAN, |(d, _)| eye - d - floor);
                z.push(h);
            }
        }
        HeightImage { w: RES, h: RES, z }
    }

    /// How many bodies lie on `cell`'s box floor.
    pub fn bodies(&self, sim: &Sim, cell: CellId) -> usize {
        vision::count_bodies(&self.heights(sim, cell), BAND.0, BAND.1, MIN_PX)
    }
}

/// Run `scene` with no harvesting for `bio_seconds` on `seed`, sample every
/// cell's view after each lifecycle event (and every 6 h without one), and
/// return every sample. Shells come and go with the molts, so the samples
/// hold both answers.
///
/// # Errors
/// A scene that cannot be loaded.
pub fn fidelity(scene: &Path, seed: u64, bio_seconds: f64) -> Result<Vec<CameraSample>, String> {
    let meta_path = scene.with_extension("json");
    let meta = SceneMeta::load(&meta_path).map_err(|e| e.to_string())?;
    let mut sim = Sim::load(scene).map_err(|e| e.to_string())?;
    sim.apply(&Targets(mast::kinematics::home().0.to_vec()))
        .map_err(|e| e.to_string())?;
    let cells = crate::cells::all(&meta);
    let mut world = World::new(LifecycleParams::default(), seed, &cells);
    let mut mover = ShellMover::subscribe(&sim, &meta_path, &meta, &mut world)?
        .ok_or("the scene has no shells, so there is nothing to see")?;
    for _ in 0..200 {
        sim.step(); // settle, and fill the poses the rays read
    }
    let cams = Cameras::new(&sim, &meta);
    let n = cells.len();
    let (mut t, mut out) = (0.0, Vec::new());
    while t < bio_seconds {
        let dt = (world.time_to_next_transition() + 1e-6)
            .min(SAMPLE_EVERY)
            .min(bio_seconds - t);
        world.step(dt);
        t += dt;
        mover.apply(&mut sim, t);
        for _ in 0..50 {
            sim.step(); // a placed shell settles on the floor
        }
        let q = sim.qpos();
        for (k, cell) in cells.iter().enumerate() {
            // The truth: the shell's free joint is up in its box, not parked
            // under the tray.
            let shell_z = q.get(7 * (2 * n + k) + 2).copied().unwrap_or(-1.0);
            out.push(CameraSample {
                cell: *cell,
                t,
                shell_in_box: shell_z > -0.05,
                bodies_seen: cams.bodies(&sim, *cell),
            });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn repo(rel: &str) -> PathBuf {
        PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(rel)
    }

    #[test]
    fn the_camera_sees_the_crawfish_and_then_its_shell() {
        for scene in ["scene/generated/rookery.xml", "scene/generated/rack3.xml"] {
            let scene = repo(scene);
            let meta = SceneMeta::load(scene.with_extension("json")).unwrap();
            let mut sim = Sim::load(&scene).unwrap();
            for _ in 0..200 {
                sim.step();
            }
            let cams = Cameras::new(&sim, &meta);
            let cell = *crate::cells::all(&meta).last().unwrap(); // the top shelf on the rack
            assert_eq!(
                cams.bodies(&sim, cell),
                1,
                "{}: the crawfish alone",
                scene.display()
            );
            // The floor reads 0 where nothing lies on it.
            let img = cams.heights(&sim, cell);
            assert!(
                img.z[0].abs() < 1e-3,
                "{}: corner height {}",
                scene.display(),
                img.z[0]
            );
            // Put the shell in, as the mover does.
            let tag = crate::cells::tag(cell);
            let (x, y) = meta.cell_center(cell.r.into(), cell.c.into());
            let floor = meta.shelf_base(cell.tray.into()).unwrap();
            sim.set_gravcomp(&format!("shell_{tag}"), 0.0).unwrap();
            sim.set_body_pose(
                &format!("shell_{tag}"),
                [x, y + 0.03, floor + 0.004 + 0.008],
                [1.0, 0.0, 0.0, 0.0],
            )
            .unwrap();
            for _ in 0..200 {
                sim.step();
            }
            assert_eq!(
                cams.bodies(&sim, cell),
                2,
                "{}: crawfish and shell",
                scene.display()
            );
            assert_eq!(
                cams.heights(&sim, cell),
                cams.heights(&sim, cell),
                "the same view twice"
            );
        }
    }
}
