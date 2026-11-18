//! `rookery`: the binary that wires the crates together.
//!
//! `cargo run -p rookery -- <scene.xml>` loads a scene, prints its body count
//! and body names, and runs 2 s of sim time headless through the
//! [`runner::Runner`] (pattern: **Template Method**, see `runner.rs`) with
//! [`runner::NoHooks`]. If the scene has a body named `box` (the Phase 1 drop
//! scene), it then prints that body's final height.
//!
//! - Phase 2 demo: `rookery scene/generated/rookery.xml` gives `nbody 54`
//!   (world, tray_0, 16 boxes, 16 crawfish, 16 shells, crane_truck/carriage/arm/hook;
//!   the exit tray, rails and shell park pad are world geoms).
//! - Phase 1 demo: `rookery scene/generated/drop.xml` ends with `box z 0.049892`.
//!
//! ```text
//!   main ──▶ Sim::load(scene) ──▶ snapshot().bodies → names
//!                └─▶ Runner::new(sim, NoHooks, 1) ──▶ run(2 s / timestep)
//!                                                       │
//!   print Report { nbody, bodies, box z? } ◀────────────┘
//! ```

use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use sim::Sim;

use rookery::runner::{NoHooks, Runner};

/// Sim time the Phase 1 demo runs for, in seconds.
const DURATION_S: f64 = 2.0;
/// The body whose height the Phase 1 demo prints, when the scene has one.
const BODY: &str = "box";

#[derive(Debug, Parser)]
#[command(
    version,
    about = "Run a Rookery scene headless",
    args_conflicts_with_subcommands = true
)]
struct Cli {
    #[command(subcommand)]
    cmd: Option<Cmd>,
    /// MJCF scene to load and summarise, e.g. scene/generated/rookery.xml
    scene: Option<PathBuf>,
}

#[derive(Debug, Subcommand)]
enum Cmd {
    /// Play a tour script (--script) or a behavior tree (--tree) on the crane,
    /// logging every tick (Phase 3 and Phase 4 demos).
    Run(tour_cmd::RunArgs),
    /// Run N offset lifts on cell (1, 1) and print their margins as JSON
    /// (row 5.6; used by scene/scene/sweep.py).
    Lift {
        /// Scene to load. Its metadata is the .json beside it.
        #[arg(long, default_value = "scene/generated/rookery.xml")]
        scene: PathBuf,
        /// Number of trials (seeds 0..N, the same seeds as the Phase 5 gate)
        #[arg(long, default_value_t = 20)]
        seeds: u64,
        /// Mast speed during the lift, as a fraction of MAX_VEL
        #[arg(long, default_value_t = mast::lift::LIFT_SPEED)]
        lift_speed: f64,
    },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match (&cli.cmd, &cli.scene) {
        (Some(Cmd::Run(args)), _) => tour_cmd::run(args),
        (
            Some(Cmd::Lift {
                scene,
                seeds,
                lift_speed,
            }),
            _,
        ) => rookery::lift_trials::summary(scene, *seeds, *lift_speed)
            .map(|v| serde_json::to_string(&v).unwrap_or_default() + "\n"),
        (None, Some(scene)) => run(scene).map(|r| r.to_string()),
        (None, None) => Err("give a scene to load, or `rookery run --script <tour.ron>`".into()),
    };
    match result {
        Ok(report) => {
            print!("{report}");
            ExitCode::SUCCESS
        }
        Err(msg) => {
            eprintln!("rookery: {msg}");
            ExitCode::FAILURE
        }
    }
}

/// What one run prints, built as a value so `run` stays free of I/O and
/// testable.
///
/// ```text
///   nbody 54
///   bodies world tray_0 box_0_0 ... box_3_3
///   box z 0.049892        (only when the scene has a body named `box`)
/// ```
#[derive(Debug)]
struct Report {
    /// Body names in MuJoCo id order, `world` first.
    bodies: Vec<String>,
    /// Final `z` of [`BODY`], if the scene has one (the Phase 1 demo).
    box_z: Option<f64>,
}

impl std::fmt::Display for Report {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "nbody {}", self.bodies.len())?;
        writeln!(f, "bodies {}", self.bodies.join(" "))?;
        if let Some(z) = self.box_z {
            writeln!(f, "{BODY} z {z:.6}")?;
        }
        Ok(())
    }
}

/// Load the scene, list its bodies, run [`DURATION_S`] of sim time, and
/// report the final `z` of [`BODY`] when there is one. Errors come back as
/// a message. Nothing here panics on bad input.
fn run(scene: &Path) -> Result<Report, String> {
    let sim = Sim::load(scene).map_err(|e| e.to_string())?;
    let bodies: Vec<String> = sim.snapshot().bodies.into_iter().map(|b| b.name).collect();
    let body = sim.body_index(BODY);
    let dt = sim.timestep();
    if !dt.is_finite() || dt <= 0.0 {
        return Err(format!(
            "{}: timestep {dt} is not a positive number",
            scene.display()
        ));
    }
    // Rounded so 2.0 / 0.001 gives exactly 2,000 steps despite float error.
    let steps = (DURATION_S / dt).round() as usize;

    let mut runner = Runner::new(sim, NoHooks, NonZeroUsize::MIN);
    runner.run(steps);
    let box_z = body.map(|i| runner.sim().snapshot().bodies[i].pos[2]);
    Ok(Report { bodies, box_z })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rookery::{bt_hooks, lift_trials};

    fn cli(rel: &str) -> PathBuf {
        PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(rel)
    }

    #[test]
    fn generated_farm_lists_world_tray_and_sixteen_boxes() {
        let r = run(&cli("scene/generated/rookery.xml")).expect("generated scene loads");
        assert_eq!(r.bodies[0], "world");
        assert!(r.bodies.iter().any(|b| b == "tray_0"));
        let boxes = r.bodies.iter().filter(|b| b.starts_with("box_")).count();
        assert_eq!(boxes, 16);
        assert_eq!(r.box_z, None, "the farm has no body named `box`");
    }

    #[test]
    fn drop_scene_still_prints_the_phase1_height() {
        let r = run(&cli("scene/generated/drop.xml")).expect("drop.xml loads");
        assert_eq!(r.bodies, ["world", "box"]);
        let z = r.box_z.expect("drop.xml has `box`");
        assert!((z - 0.05).abs() < 0.001, "z = {z}");
        assert!(r.to_string().ends_with(&format!("box z {z:.6}\n")));
    }

    #[test]
    fn a_tour_engages_a_handle_and_returns_home_without_disturbing_any_box() {
        let dir = std::env::temp_dir().join(format!("rookery-tour-test-{}", std::process::id()));
        let script = dir.join("short.ron");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(&script, "[Goto(Cell(3, 3)), Goto(Home)]").unwrap();
        let args = tour_cmd::RunArgs {
            script: Some(script),
            tree: None,
            seconds: 60.0,
            scenario: None,
            ticks: None,
            fault_at: None,
            seed: 7,
            log: dir.clone(),
            scene: cli("scene/generated/rookery.xml"),
            id: Some("t".into()),
        };
        let report = tour_cmd::run(&args).expect("tour completes at home");
        assert!(report.contains("Completed"), "{report}");
        let csv = std::fs::read_to_string(dir.join("t/qpos.csv")).unwrap();
        let rows: Vec<Vec<f64>> = csv
            .lines()
            .skip(1)
            .map(|l| l.split(',').map(|v| v.parse().unwrap()).collect())
            .collect();
        let (first, last) = (&rows[0], &rows[rows.len() - 1]);
        let nboxes = (first.len() - 1 - 4) / 7;
        for b in 0..nboxes {
            for k in 0..3 {
                let i = 1 + 7 * b + k;
                assert!(
                    (last[i] - first[i]).abs() < 0.001,
                    "box {b} moved {}",
                    last[i] - first[i]
                );
            }
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A BtHooks runner on the committed scene with `tree` (a Lua tree source)
    /// whose leaves live in the same temp dir.
    fn bt_runner(
        name: &str,
        tree: &str,
        leaves: &[(&str, &str)],
    ) -> Runner<bt_hooks::BtHooks<std::io::Sink>> {
        let dir = std::env::temp_dir().join(format!("rookery-bt-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        for (file, src) in leaves {
            std::fs::write(dir.join(file), src).unwrap();
        }
        std::fs::write(dir.join("tree.lua"), tree).unwrap();
        let tree = bt::Tree::load(dir.join("tree.lua"), &mlua::Lua::new()).expect("tree loads");
        let scene = cli("scene/generated/rookery.xml");
        let meta = mast::SceneMeta::load(scene.with_extension("json")).unwrap();
        let sim = Sim::load(&scene).unwrap();
        let state = mast::GantryState {
            targets: mast::home(),
            windows: mast::kinematics::shelf_windows(&meta),
        };
        let hooks = bt_hooks::BtHooks::new(&sim, tree, state, meta, 0.02, std::io::sink()).unwrap();
        Runner::new(sim, hooks, NonZeroUsize::new(20).unwrap())
    }

    #[test]
    fn the_tree_ticks_every_20_physics_steps() {
        let mut r = bt_runner(
            "rate",
            r#"return Leaf("idle.lua")"#,
            &[("idle.lua", r#"function tick(bb) return "running" end"#)],
        );
        r.run(1_000);
        assert_eq!(r.hooks().tree_ticks, 50);
    }

    #[test]
    fn an_off_grid_goal_is_a_named_error_and_the_crane_holds() {
        let mut r = bt_runner(
            "offgrid",
            r#"return Leaf("bad.lua")"#,
            &[(
                "bad.lua",
                r#"function tick(bb) bb:go_to(Cell(9, 0)) return "running" end"#,
            )],
        );
        r.run(200);
        let h = r.hooks();
        assert!(
            h.errors.iter().any(|e| e.contains("outside the 4x4 grid")),
            "{:?}",
            h.errors
        );
        assert_eq!(h.targets(), mast::home(), "the crane must hold");
    }

    #[test]
    fn a_tree_goal_drives_the_crane_to_the_cell() {
        let mut r = bt_runner(
            "drive",
            r#"return Leaf("go.lua")"#,
            &[(
                "go.lua",
                r#"function tick(bb) bb:go_to(Cell(1, 2)) return "running" end"#,
            )],
        );
        r.run(20_000); // 20 s of sim time
        let h = r.hooks();
        assert!(h.errors.is_empty(), "{:?}", h.errors);
        let meta = mast::SceneMeta::load(cli("scene/generated/rookery.json")).unwrap();
        let goal = mast::cell_pose(1, 2, &meta).unwrap();
        assert_eq!(h.targets(), goal, "targets reach the engage pose exactly");
        assert_eq!(h.crane().route_len(), 0);
    }

    #[test]
    fn a_lift_trial_on_the_committed_scene_passes_with_margin() {
        let scene = cli("scene/generated/rookery.xml");
        let meta = mast::SceneMeta::load(scene.with_extension("json")).unwrap();
        let t = lift_trials::trial(&scene, &meta, 0, mast::lift::LIFT_SPEED);
        assert!(t.passed(), "{t:?}");
        assert!(t.max_roll_deg < 2.0 && t.clearance_m > 0.02, "{t:?}");
    }

    #[test]
    fn a_fault_mid_transfer_puts_the_box_back_in_its_cell() {
        // Row 6.4: the crane player runs a whole Transfer macro, and
        // abort_active() swaps it for its undo halfway through the carry.
        let mut r = bt_runner(
            "fault",
            r#"return Leaf("idle.lua")"#,
            &[("idle.lua", r#"function tick(bb) return "running" end"#)],
        );
        let meta = mast::SceneMeta::load(cli("scene/generated/rookery.json")).unwrap();
        let b = r.sim().body_index("box_1_2").unwrap();
        r.run(20); // xpos is filled by the first step
        let start = r.sim().snapshot().bodies[b].pos;
        let t = mast::Transfer::build(mast::home(), 1, 2, &meta).unwrap();
        r.hooks_mut()
            .crane_mut()
            .enqueue_command(Box::new(t), "transfer (1, 2)");
        // Run until the box has been lifted well clear, then fault.
        let mut lifted = false;
        for _ in 0..3_000 {
            r.run(20);
            let z = r.sim().snapshot().bodies[b].pos[2];
            if z > meta.water_depth + 0.02 {
                lifted = true;
                break;
            }
        }
        assert!(lifted, "the box was never lifted");
        r.run(20 * 50); // 1 s more, so the carry is under way
        assert!(
            r.hooks_mut().crane_mut().abort_active(),
            "a command was active"
        );
        for _ in 0..3_000 {
            r.run(20);
            if r.hooks().crane().route_len() == 0 {
                break;
            }
        }
        r.run(20 * 50); // settle
        let p = r.sim().snapshot().bodies[b].pos;
        let off = ((p[0] - start[0]).powi(2) + (p[1] - start[1]).powi(2)).sqrt();
        assert!(
            off < 0.005 && (p[2] - start[2]).abs() < 0.005,
            "box {off:.4} m from its cell, z {:.4}",
            p[2]
        );
        assert!(
            r.hooks()
                .crane()
                .events
                .iter()
                .any(|(_, e)| e.starts_with("fault"))
        );
    }

    #[test]
    fn a_quiet_sim_week_fast_forwards_in_under_a_minute() {
        let dir = std::env::temp_dir().join(format!("rookery-week-{}", std::process::id()));
        let args = tour_cmd::RunArgs {
            script: None,
            tree: Some(cli("lua/trees/idle.lua")),
            seconds: 7.0 * 86_400.0,
            log: dir.clone(),
            scene: cli("scene/generated/rookery.xml"),
            id: Some("w".into()),
            scenario: None,
            ticks: None,
            fault_at: None,
            seed: 7,
        };
        let t0 = std::time::Instant::now();
        tour_cmd::run(&args).expect("week runs");
        let wall = t0.elapsed().as_secs_f64();
        let summary: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("w/summary.json")).unwrap())
                .unwrap();
        let bio = summary["bio_time"].as_f64().unwrap();
        assert!(bio >= 7.0 * 86_400.0 - 1.0, "only {bio} s of biology");
        assert!(wall < 60.0, "a quiet week took {wall:.1} s of wall time");
        let molts = summary["bio_events"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|e| e.as_str().unwrap().contains("Molted"))
            .count();
        assert!(molts > 0, "no molts in a week");
        // Row 7.5: every molt put a shell in its box, and nothing else moved.
        let shells = summary["shells"].as_array().unwrap();
        assert_eq!(shells.len(), molts, "{shells:?}");
        assert!(
            shells
                .iter()
                .all(|s| s.as_str().unwrap().contains("inside: true")),
            "{shells:?}"
        );
        assert!(summary["max_box_displacement"].as_f64().unwrap() < 0.002);
        // Row 7.7: the same events as data, for scene.plots molt_hist.
        let events = summary["events"].as_array().unwrap();
        assert_eq!(
            events.len(),
            summary["bio_events"].as_array().unwrap().len()
        );
        assert_eq!(
            events.iter().filter(|e| e["kind"] == "Molted").count(),
            molts
        );
        assert!(summary["lifecycle"]["interval_mean"].as_f64().unwrap() > 20.0 * 86_400.0);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn while_the_crane_is_busy_biology_keeps_pace_with_physics() {
        let mut r = bt_runner(
            "busy",
            r#"return Leaf("go.lua")"#,
            &[(
                "go.lua",
                r#"function tick(bb) bb:go_to(Transfer(1, 2)) return "running" end"#,
            )],
        );
        let meta = mast::SceneMeta::load(cli("scene/generated/rookery.json")).unwrap();
        let cells: Vec<rookery_core::CellId> = (0..16)
            .map(|i| rookery_core::CellId {
                tray: 0,
                r: i / 4,
                c: i % 4,
            })
            .collect();
        let _ = meta;
        let world = rookery_core::World::new(rookery_core::LifecycleParams::default(), 7, &cells);
        let hooks = std::mem::replace(r.hooks_mut(), placeholder_hooks());
        *r.hooks_mut() = hooks.with_world(world);
        r.run(20 * 500); // 10 s: the transfer is under way the whole time
        let sim_t = r.sim().snapshot().time;
        assert!(!r.hooks().idle(), "the transfer should still be running");
        assert!(
            (r.hooks().bio_time() - sim_t).abs() < 1e-6,
            "bio {} vs sim {sim_t}",
            r.hooks().bio_time()
        );
    }

    /// A throwaway BtHooks, only to swap the real one out of a Runner.
    fn placeholder_hooks() -> bt_hooks::BtHooks<std::io::Sink> {
        let r = bt_runner(
            "placeholder",
            r#"return Leaf("idle.lua")"#,
            &[("idle.lua", r#"function tick(bb) return "running" end"#)],
        );
        r.into_hooks()
    }

    #[test]
    fn missing_scene_is_an_error_not_a_panic() {
        assert!(run(&cli("no/such.xml")).is_err());
    }
}

/// `rookery run`: the Phase 3 demo.
mod tour_cmd {
    use std::fs::{self, File};
    use std::io::BufWriter;
    use std::num::NonZeroUsize;
    use std::path::PathBuf;
    use std::time::{SystemTime, UNIX_EPOCH};

    use mast::kinematics::{self, shelf_windows};
    use mast::{DONE_TOL, GantryState, SceneMeta};
    use sim::Sim;

    use rookery::runner::Runner;
    use rookery::script;
    use rookery::tour::{Outcome, ScriptHooks};

    /// Commands tick at 50 Hz (every 20 physics steps at 1 ms), the BT rate from Phase 4.
    const TICK_HZ: f64 = 50.0;
    /// Upper bound on physics time in one run, seconds.
    const MAX_SECONDS: f64 = 300.0;
    /// Upper bound on biological time in one run (fast-forward makes long
    /// runs cheap), seconds: one year.
    const MAX_BIO_SECONDS: f64 = 366.0 * 86_400.0;

    #[derive(Debug, clap::Args)]
    pub struct RunArgs {
        /// Tour script (RON), e.g. examples/tour.ron
        #[arg(long, conflicts_with_all = ["tree", "scenario"], required_unless_present_any = ["tree", "scenario"])]
        pub script: Option<PathBuf>,
        /// Behavior tree (Lua), e.g. lua/trees/patrol.lua (row 4.5)
        #[arg(long)]
        pub tree: Option<PathBuf>,
        /// With --tree: sim time to run, seconds (--ticks or the scenario wins)
        #[arg(long, default_value_t = 60.0)]
        pub seconds: f64,
        /// Scenario (Lua): names a tree and ticks, and checks the run (row 4.7)
        #[arg(long)]
        pub scenario: Option<PathBuf>,
        /// Seed for the animals' lifecycles (row 7.2)
        #[arg(long, default_value_t = 7)]
        pub seed: u64,
        /// 50 Hz ticks to run (overrides --seconds and the scenario's ticks)
        #[arg(long)]
        pub ticks: Option<u64>,
        /// Inject a fault on this command tick: the active command is replaced
        /// by its undo (row 6.4)
        #[arg(long)]
        pub fault_at: Option<u64>,
        /// Directory for run logs: <log>/<run id>/{qpos.csv, commands.json, summary.json}
        #[arg(long, default_value = "runs")]
        pub log: PathBuf,
        /// Scene to load. Its metadata is the .json beside it.
        #[arg(long, default_value = "scene/generated/rookery.xml")]
        pub scene: PathBuf,
        /// Run id (default: tour-<unix seconds>)
        #[arg(long)]
        pub id: Option<String>,
    }

    pub fn run(args: &RunArgs) -> Result<String, String> {
        let Some(script_path) = &args.script else {
            return tree_run(args);
        };
        let meta = SceneMeta::load(args.scene.with_extension("json")).map_err(|e| e.to_string())?;
        let steps = script::load(script_path)?;
        let start = kinematics::home();
        let cmds = script::expand(&steps, start, &meta)?;
        let ncmds = cmds.len();

        let mut sim = Sim::load(&args.scene).map_err(|e| e.to_string())?;
        if sim.apply(&sim::Targets(vec![0.0; 4])).is_err() {
            return Err(format!(
                "{}: expected the four crane actuators",
                args.scene.display()
            ));
        }
        let dt_phys = sim.timestep();
        let every = (1.0 / (TICK_HZ * dt_phys)).round().max(1.0) as usize;
        let dt = every as f64 * dt_phys;

        let id = args.id.clone().unwrap_or_else(|| {
            let secs = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0);
            format!("tour-{secs}")
        });
        let dir = args.log.join(&id);
        fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let qpos = BufWriter::new(File::create(dir.join("qpos.csv")).map_err(|e| e.to_string())?);

        let state = GantryState {
            targets: start,
            windows: shelf_windows(&meta),
        };
        let mut hooks = ScriptHooks::new(&sim, state, cmds, &meta, dt, qpos)?;
        hooks.fault_at = args.fault_at;
        let every_nz = NonZeroUsize::new(every).unwrap_or(NonZeroUsize::MIN);
        let mut runner = Runner::new(sim, hooks, every_nz);

        let max_steps = (MAX_SECONDS / dt_phys) as usize;
        let chunk = every * 50; // 1 s at a time
        let mut stepped = 0;
        while stepped < max_steps && !runner.hooks().finished() {
            runner.run(chunk);
            stepped += chunk;
        }
        runner.run(chunk); // 1 s more, so the final pose is settled and logged
        runner.hooks_mut().close();

        let hooks = runner.hooks();
        if let Some(e) = hooks.io_error() {
            return Err(format!("writing qpos.csv: {e}"));
        }
        let final_t = hooks.targets();
        let b = runner.sim().snapshot();
        let q = {
            let id = |n: &str| runner.sim().body_index(n).expect("crane body");
            [
                b.bodies[id("crane_truck")].pos[1],
                b.bodies[id("crane_carriage")].pos[2],
                b.bodies[id("crane_arm")].pos[0] - meta.derived.rail_line_x,
                b.bodies[id("crane_hook")].pos[2] - b.bodies[id("crane_carriage")].pos[2],
            ]
        };
        let home = kinematics::home().0;
        let err = q
            .iter()
            .zip(home)
            .map(|(a, h)| (a - h).abs())
            .fold(0.0, f64::max);
        let outcome = hooks
            .outcome
            .clone()
            .unwrap_or(Outcome::Unfinished { remaining: ncmds });
        let at_home = err <= DONE_TOL;

        fs::write(
            dir.join("commands.json"),
            serde_json::to_string_pretty(hooks.log()).map_err(|e| e.to_string())? + "\n",
        )
        .map_err(|e| e.to_string())?;
        let summary = serde_json::json!({
            "id": id,
            "script": script_path.display().to_string(),
            "scene": args.scene.display().to_string(),
            "commands": ncmds,
            "outcome": format!("{outcome:?}"),
            "sim_time": b.time,
            "final_q": q,
            "final_targets": final_t.0,
            "home": home,
            "max_error_from_home": err,
            "at_home": at_home,
        });
        fs::write(
            dir.join("summary.json"),
            serde_json::to_string_pretty(&summary).unwrap_or_default() + "\n",
        )
        .map_err(|e| e.to_string())?;

        let report = format!(
            "run {id}: {ncmds} commands, {outcome:?}, {:.2} s sim\n\
             final q [{:.4}, {:.4}, {:.4}, {:.4}], max error from home {:.4} m ({})\n\
             log {}\n",
            b.time,
            q[0],
            q[1],
            q[2],
            q[3],
            err,
            if at_home {
                "within tolerance"
            } else {
                "NOT within tolerance"
            },
            dir.display()
        );
        match (outcome, at_home) {
            (Outcome::Completed, true) => Ok(report),
            _ => Err(report),
        }
    }

    /// `rookery run --tree`: tick a behavior tree at 50 Hz for `--seconds`
    /// (row 4.5). Writes the same run log as a tour, plus the tree's errors.
    pub fn tree_run(args: &RunArgs) -> Result<String, String> {
        use rookery::bt_hooks::BtHooks;

        let scenario = match &args.scenario {
            Some(p) => Some(rookery::scenario::Scenario::load(p)?),
            None => None,
        };
        let tree_owned = args
            .tree
            .clone()
            .or_else(|| scenario.as_ref().and_then(|s| s.tree.clone()))
            .ok_or("give --script, --tree, or a --scenario that names a tree")?;
        let tree_path = &tree_owned;
        let meta = SceneMeta::load(args.scene.with_extension("json")).map_err(|e| e.to_string())?;
        let (hooks_meta_rows, hooks_meta_cols) = (meta.rows, meta.cols);
        if let Some(s) = &scenario {
            s.setup(meta.rows, meta.cols)?;
        }
        let lua = mlua::Lua::new();
        let tree = bt::Tree::load(tree_path, &lua).map_err(|e| e.to_string())?;

        let mut sim = Sim::load(&args.scene).map_err(|e| e.to_string())?;
        if sim.apply(&sim::Targets(vec![0.0; 4])).is_err() {
            return Err(format!(
                "{}: expected the four crane actuators",
                args.scene.display()
            ));
        }
        let dt_phys = sim.timestep();
        let every = (1.0 / (TICK_HZ * dt_phys)).round().max(1.0) as usize;
        let dt = every as f64 * dt_phys;

        let id = args.id.clone().unwrap_or_else(|| {
            let secs = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0);
            format!("tree-{secs}")
        });
        let dir = args.log.join(&id);
        fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let qpos = BufWriter::new(File::create(dir.join("qpos.csv")).map_err(|e| e.to_string())?);

        let state = GantryState {
            targets: kinematics::home(),
            windows: shelf_windows(&meta),
        };
        let cells: Vec<rookery_core::CellId> = (0..meta.rows * meta.cols)
            .map(|i| rookery_core::CellId {
                tray: 0,
                r: (i / meta.cols) as u8,
                c: (i % meta.cols) as u8,
            })
            .collect();
        let mut world =
            rookery_core::World::new(rookery_core::LifecycleParams::default(), args.seed, &cells);
        let mover = rookery::shells::ShellMover::subscribe(
            &sim,
            &args.scene.with_extension("json"),
            &meta,
            &mut world,
        )?;
        let mut hooks = BtHooks::new(&sim, tree, state, meta, dt, qpos)?
            .with_world(world)
            .with_shells(mover);
        hooks.fault_at = args.fault_at;
        let every_nz = NonZeroUsize::new(every).unwrap_or(NonZeroUsize::MIN);
        let first_qpos = sim.qpos();
        let mut runner = Runner::new(sim, hooks, every_nz);
        let ticks = args
            .ticks
            .or_else(|| scenario.as_ref().and_then(|s| s.ticks));
        let seconds = ticks.map_or(args.seconds, |t| t as f64 * dt);
        // The clock (row 7.2): `seconds` is biological time. While the crane
        // is busy, physics runs and biology keeps pace; while it is idle,
        // biology jumps to the next stage change, then one physics tick lets
        // the tree react. The Runner decides, never Lua.
        let bio_end = seconds.clamp(0.0, MAX_BIO_SECONDS);
        let max_phys_steps = (MAX_SECONDS / dt_phys) as usize;
        let mut phys_steps = 0usize;
        while runner.hooks().bio_time() < bio_end - 1e-9 && phys_steps < max_phys_steps {
            let left = bio_end - runner.hooks().bio_time();
            if runner.hooks().idle() && left > dt {
                runner.hooks_mut().fast_forward(left - dt);
            }
            runner.run(every);
            phys_steps += every;
        }
        // Boxes and crawfish: shells move by design (row 7.5).
        let box_moved = rookery::scenario::max_box_displacement(
            &first_qpos,
            &runner.sim().qpos(),
            2 * hooks_meta_rows * hooks_meta_cols,
        );

        let hooks = runner.hooks();
        if let Some(e) = hooks.crane().io_error() {
            return Err(format!("writing qpos.csv: {e}"));
        }
        let t = runner.sim().snapshot().time;
        fs::write(
            dir.join("commands.json"),
            serde_json::to_string_pretty(&hooks.crane().log).map_err(|e| e.to_string())? + "\n",
        )
        .map_err(|e| e.to_string())?;
        let summary = serde_json::json!({
            "id": id,
            "tree": tree_path.display().to_string(),
            "scene": args.scene.display().to_string(),
            "sim_time": t,
            "tree_ticks": hooks.tree_ticks,
            "last_status": format!("{:?}", hooks.last_status),
            "goal": format!("{:?}", hooks.goal()),
            "final_targets": hooks.targets().0,
            "errors": hooks.errors,
            "arrivals": hooks.arrivals.iter().map(|p| format!("{p:?}")).collect::<Vec<_>>(),
            "bio_time": hooks.bio_time(),
            "bio_events": hooks.bio_events.iter().map(|(t, e)| format!("{t:.0} {e:?}")).collect::<Vec<_>>(),
            // Row 7.7: the same events as data, and the configured molt
            // interval, for `scene/scene/plots.py molt_hist`.
            "events": hooks.bio_events.iter().map(|&(t, e)| {
                let (kind, c) = rookery::bt_hooks::describe(e);
                serde_json::json!({"t": t, "kind": kind, "tray": c.tray, "r": c.r, "c": c.c})
            }).collect::<Vec<_>>(),
            "lifecycle": {
                "seed": args.seed,
                "interval_mean": rookery_core::LifecycleParams::default().interval_mean(),
                "interval_sigma": rookery_core::LifecycleParams::default().interval_sigma(),
            },
            "max_box_displacement": box_moved,
            "shells": hooks.shells.as_ref().map(|m| m.checks.iter().map(|c| format!("{c:?}")).collect::<Vec<_>>()),
        });
        fs::write(
            dir.join("summary.json"),
            serde_json::to_string_pretty(&summary).unwrap_or_default() + "\n",
        )
        .map_err(|e| e.to_string())?;
        let report = format!(
            "run {id}: tree {}, {} ticks over {:.2} s sim, last status {:?}, goal {:?}, {} error(s)\nlog {}\n",
            tree_path.display(),
            hooks.tree_ticks,
            t,
            hooks.last_status,
            hooks.goal(),
            hooks.errors.len(),
            dir.display()
        );
        if let Some(s) = &scenario {
            let log = rookery::scenario::RunLog {
                ticks: hooks.tree_ticks,
                sim_time: t,
                visited: hooks.arrivals.clone(),
                errors: hooks.errors.clone(),
                max_box_displacement: box_moved,
                final_q: hooks.crane().observe(runner.sim()).q,
                moved_boxes: rookery::scenario::moved_boxes(
                    &first_qpos,
                    &runner.sim().qpos(),
                    hooks_meta_rows,
                    hooks_meta_cols,
                ),
                events: hooks
                    .bio_events
                    .iter()
                    .map(|&(t, e)| {
                        let (kind, cell) = rookery::bt_hooks::describe(e);
                        (t, kind, cell)
                    })
                    .collect(),
                shells: hooks
                    .shells
                    .as_ref()
                    .map(|m| m.checks.clone())
                    .unwrap_or_default(),
            };
            let (ok, msg) = s.check(&log)?;
            let verdict = format!(
                "scenario {}{}: {} — {msg}\n",
                args.scenario
                    .as_ref()
                    .map(|p| p.display().to_string())
                    .unwrap_or_default(),
                if s.ci { " (ci)" } else { "" },
                if ok { "PASS" } else { "FAIL" }
            );
            fs::write(dir.join("scenario.txt"), &verdict).map_err(|e| e.to_string())?;
            return if ok {
                Ok(report + &verdict)
            } else {
                Err(report + &verdict)
            };
        }
        if hooks.errors.is_empty() {
            Ok(report)
        } else {
            Err(format!(
                "{report}errors:\n  {}\n",
                hooks.errors.join("\n  ")
            ))
        }
    }
}
