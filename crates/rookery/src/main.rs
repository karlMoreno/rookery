//! `rookery`: the binary that wires the crates together.
//!
//! `cargo run -p rookery -- <scene.xml>` loads a scene, prints its body count
//! and body names, and runs 2 s of sim time headless through the
//! [`runner::Runner`] (pattern: **Template Method**, see `runner.rs`) with
//! [`runner::NoHooks`]. If the scene has a body named `box` (the Phase 1 drop
//! scene), it then prints that body's final height.
//!
//! - Phase 2 demo: `rookery scene/generated/rookery.xml` gives `nbody 22`
//!   (world, tray_0, 16 boxes, crane_truck/carriage/arm/hook; the exit tray
//!   and rails are world geoms).
//! - Phase 1 demo: `rookery scene/generated/drop.xml` ends with `box z 0.049892`.
//!
//! ```text
//!   main ──▶ Sim::load(scene) ──▶ snapshot().bodies → names
//!                └─▶ Runner::new(sim, NoHooks, 1) ──▶ run(2 s / timestep)
//!                                                       │
//!   print Report { nbody, bodies, box z? } ◀────────────┘
//! ```

mod runner;

use std::num::NonZeroUsize;
use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;
use sim::Sim;

use runner::{NoHooks, Runner};

/// Sim time the Phase 1 demo runs for, in seconds.
const DURATION_S: f64 = 2.0;
/// The body whose height the Phase 1 demo prints, when the scene has one.
const BODY: &str = "box";

#[derive(Debug, Parser)]
#[command(version, about = "Run a Rookery scene headless")]
struct Cli {
    /// MJCF scene to load, e.g. scene/generated/rookery.xml
    scene: PathBuf,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match run(&cli) {
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
///   nbody 22
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
fn run(cli: &Cli) -> Result<Report, String> {
    let sim = Sim::load(&cli.scene).map_err(|e| e.to_string())?;
    let bodies: Vec<String> = sim.snapshot().bodies.into_iter().map(|b| b.name).collect();
    let body = sim.body_index(BODY);
    let dt = sim.timestep();
    if !dt.is_finite() || dt <= 0.0 {
        return Err(format!(
            "{}: timestep {dt} is not a positive number",
            cli.scene.display()
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

    fn cli(rel: &str) -> Cli {
        Cli {
            scene: PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(rel),
        }
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
    fn missing_scene_is_an_error_not_a_panic() {
        assert!(run(&cli("no/such.xml")).is_err());
    }
}
