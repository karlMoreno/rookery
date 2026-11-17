//! `rookery`: the binary that wires the crates together.
//!
//! `cargo run -p rookery -- <scene.xml>` loads a scene, prints its body count
//! and body names, and runs 2 s of sim time headless through the
//! [`runner::Runner`] (pattern: **Template Method**, see `runner.rs`) with
//! [`runner::NoHooks`]. If the scene has a body named `box` (the Phase 1 drop
//! scene), it then prints that body's final height.
//!
//! - Phase 2 demo: `rookery scene/generated/rookery.xml` gives `nbody 38`
//!   (world, tray_0, 16 boxes, 16 crawfish, crane_truck/carriage/arm/hook; the exit tray
//!   and rails are world geoms).
//! - Phase 1 demo: `rookery scene/generated/drop.xml` ends with `box z 0.049892`.
//!
//! ```text
//!   main ──▶ Sim::load(scene) ──▶ snapshot().bodies → names
//!                └─▶ Runner::new(sim, NoHooks, 1) ──▶ run(2 s / timestep)
//!                                                       │
//!   print Report { nbody, bodies, box z? } ◀────────────┘
//! ```

mod bt_hooks;
mod crane;
mod runner;
mod scenario;
mod script;
mod tour;

use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clap::{Parser, Subcommand};
use sim::Sim;

use runner::{NoHooks, Runner};

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
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    let result = match (&cli.cmd, &cli.scene) {
        (Some(Cmd::Run(args)), _) => tour_cmd::run(args),
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
///   nbody 38
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

    use crate::runner::Runner;
    use crate::script;
    use crate::tour::{Outcome, ScriptHooks};

    /// Commands tick at 50 Hz (every 20 physics steps at 1 ms), the BT rate from Phase 4.
    const TICK_HZ: f64 = 50.0;
    /// Upper bound on a tour's sim time, seconds.
    const MAX_SECONDS: f64 = 300.0;

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
        /// 50 Hz ticks to run (overrides --seconds and the scenario's ticks)
        #[arg(long)]
        pub ticks: Option<u64>,
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
        let hooks = ScriptHooks::new(&sim, state, cmds, &meta, dt, qpos)?;
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
        use crate::bt_hooks::BtHooks;

        let scenario = match &args.scenario {
            Some(p) => Some(crate::scenario::Scenario::load(p)?),
            None => None,
        };
        let tree_owned = args
            .tree
            .clone()
            .or_else(|| scenario.as_ref().and_then(|s| s.tree.clone()))
            .ok_or("give --script, --tree, or a --scenario that names a tree")?;
        let tree_path = &tree_owned;
        let meta = SceneMeta::load(args.scene.with_extension("json")).map_err(|e| e.to_string())?;
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
        let hooks = BtHooks::new(&sim, tree, state, meta, dt, qpos)?;
        let every_nz = NonZeroUsize::new(every).unwrap_or(NonZeroUsize::MIN);
        let first_qpos = sim.qpos();
        let mut runner = Runner::new(sim, hooks, every_nz);
        let ticks = args
            .ticks
            .or_else(|| scenario.as_ref().and_then(|s| s.ticks));
        let seconds = ticks.map_or(args.seconds, |t| t as f64 * dt);
        let steps = (seconds.clamp(0.0, MAX_SECONDS) / dt_phys).round() as usize;
        runner.run(steps);
        let box_moved = crate::scenario::max_box_displacement(&first_qpos, &runner.sim().qpos());

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
            "max_box_displacement": box_moved,
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
            let log = crate::scenario::RunLog {
                ticks: hooks.tree_ticks,
                sim_time: t,
                visited: hooks.arrivals.clone(),
                errors: hooks.errors.clone(),
                max_box_displacement: box_moved,
                final_q: hooks.crane().observe(runner.sim()).q,
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
