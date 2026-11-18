//! Unattended runs (Phase 8): one tray, a behavior tree, a seed, and the
//! world's clock, with no one at the controls. The Phase 8 gate and
//! `rookery run --scenario week_1tray.lua` both go through [`run`].
//!
//! Row 8.1 declared this surface; 8.2 (detector and events), 8.4 (exit
//! slots) and 8.5 (`autonomy.lua`, this body) filled it in without changing
//! a signature.
//!
//! ```text
//!   run(args) ─▶ Sim::load(scene) + World::new(seed) + ShellMover + OracleDetector
//!            ─▶ Runner<BtHooks> under the fast-forward clock until bio_seconds
//!            ─▶ AutonomyReport { events, boxes_in_slots, errors, physics_seconds }
//! ```

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::num::NonZeroUsize;
use std::path::PathBuf;

use mast::kinematics::{self, shelf_windows};
use mast::{GantryState, SceneMeta};
use rookery_core::{CellId, Event, LifecycleParams, World};
use sim::Sim;

use crate::bt_hooks::BtHooks;
use crate::runner::Runner;
use crate::shells::ShellMover;

/// The behavior tree's rate, ticks per second.
const TICK_HZ: f64 = 50.0;
/// Most physics time one run may simulate, seconds. A transfer takes about
/// 30 s, so this covers a month of molts with room to spare; past it the
/// run stops early rather than hang.
pub const MAX_PHYSICS_SECONDS: f64 = 900.0;
/// A box is in its exit slot when its floor's centre is this close to the
/// slot (metres, horizontally) and within 1 cm of the tray top.
const IN_SLOT_TOL: f64 = 0.02;

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
    let meta_path = args.scene.with_extension("json");
    let meta = SceneMeta::load(&meta_path).map_err(|e| e.to_string())?;
    let lua = mlua::Lua::new();
    let tree = bt::Tree::load(&args.tree, &lua).map_err(|e| e.to_string())?;
    let mut sim = Sim::load(&args.scene).map_err(|e| e.to_string())?;
    sim.apply(&sim::Targets(vec![0.0; 4])).map_err(|_| {
        format!(
            "{}: expected the four crane actuators",
            args.scene.display()
        )
    })?;
    let dt_phys = sim.timestep();
    let every = (1.0 / (TICK_HZ * dt_phys)).round().max(1.0) as usize;
    let dt = every as f64 * dt_phys;

    let qpos: Box<dyn Write> = match &args.log {
        Some(dir) => {
            fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
            Box::new(BufWriter::new(
                File::create(dir.join("qpos.csv")).map_err(|e| e.to_string())?,
            ))
        }
        None => Box::new(std::io::sink()),
    };
    let cells: Vec<CellId> = (0..meta.rows * meta.cols)
        .map(|i| CellId {
            tray: 0,
            r: (i / meta.cols) as u8,
            c: (i % meta.cols) as u8,
        })
        .collect();
    let mut world = World::new(LifecycleParams::default(), args.seed, &cells);
    let mover = ShellMover::subscribe(&sim, &meta_path, &meta, &mut world)?;
    let state = GantryState {
        targets: kinematics::home(),
        windows: shelf_windows(&meta),
    };
    let hooks = BtHooks::new(&sim, tree, state, meta.clone(), dt, qpos)?
        .with_world(world)
        .with_shells(mover);
    let mut runner = Runner::new(
        sim,
        hooks,
        NonZeroUsize::new(every).unwrap_or(NonZeroUsize::MIN),
    );

    // The clock (row 7.2): physics while the crane works, jumps while it idles.
    let bio_end = args.bio_seconds.max(0.0);
    let max_steps = (MAX_PHYSICS_SECONDS / dt_phys) as usize;
    let mut steps = 0usize;
    while runner.hooks().bio_time() < bio_end - 1e-9 && steps < max_steps {
        let left = bio_end - runner.hooks().bio_time();
        if runner.hooks().idle() && left > dt {
            runner.hooks_mut().fast_forward(left - dt);
        }
        runner.run(every);
        steps += every;
    }

    let q = runner.sim().qpos();
    let hooks = runner.hooks();
    let mut errors = hooks.errors.clone();
    if steps >= max_steps && hooks.bio_time() < bio_end - 1e-9 {
        errors.push(format!(
            "clock: stopped at {:.0} s of biology after {MAX_PHYSICS_SECONDS:.0} s of physics",
            hooks.bio_time()
        ));
    }
    let boxes_in_slots = hooks
        .slot_of
        .iter()
        .filter(|(cell, slot)| {
            let i = 7 * (usize::from(cell.r) * meta.cols + usize::from(cell.c));
            let (Some(b), Some(p)) = (q.get(i..i + 3), meta.derived.exit_slots.get(*slot)) else {
                return false;
            };
            (b[0] - p[0]).hypot(b[1] - p[1]) < IN_SLOT_TOL && (b[2] - p[2]).abs() < 0.01
        })
        .map(|(cell, _)| *cell)
        .collect();
    let report = AutonomyReport {
        events: hooks.bio_events.clone(),
        boxes_in_slots,
        errors,
        physics_seconds: steps as f64 * dt_phys,
    };
    if let Some(dir) = &args.log {
        let summary = serde_json::json!({
            "scene": args.scene.display().to_string(),
            "tree": args.tree.display().to_string(),
            "seed": args.seed,
            "bio_time": hooks.bio_time(),
            "physics_seconds": report.physics_seconds,
            "events": report.events.iter().map(|&(t, e)| {
                let (kind, c) = crate::bt_hooks::describe(e);
                serde_json::json!({"t": t, "kind": kind, "tray": c.tray, "r": c.r, "c": c.c})
            }).collect::<Vec<_>>(),
            "slots": hooks.slot_of.iter().map(|(c, i)| serde_json::json!({"r": c.r, "c": c.c, "slot": i})).collect::<Vec<_>>(),
            "boxes_in_slots": report.boxes_in_slots.iter().map(|c| [c.r, c.c]).collect::<Vec<_>>(),
            "errors": report.errors,
        });
        fs::write(
            dir.join("summary.json"),
            serde_json::to_string_pretty(&summary).unwrap_or_default() + "\n",
        )
        .map_err(|e| e.to_string())?;
    }
    Ok(report)
}
