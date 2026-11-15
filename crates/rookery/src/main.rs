//! `rookery`: the binary that wires the crates together.
//!
//! Phase 1 demo: `cargo run -p rookery -- examples/drop.xml` loads the scene,
//! runs 2 s of sim time headless through the [`runner::Runner`] (pattern:
//! **Template Method**, see `runner.rs`) with [`runner::NoHooks`], and prints
//! the final height of body `box`.
//!
//! ```text
//!   main ──▶ Sim::load(scene) ──▶ Runner::new(sim, NoHooks, 1) ──▶ run(2 s / timestep)
//!                                                                    │
//!                          println!("{z}") ◀── snapshot().bodies[box].pos[2]
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
/// The body whose height the demo prints.
const BODY: &str = "box";

#[derive(Debug, Parser)]
#[command(version, about = "Run a Rookery scene headless")]
struct Cli {
    /// MJCF scene to load, e.g. examples/drop.xml
    scene: PathBuf,
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match run(&cli) {
        Ok(z) => {
            println!("{z:.6}");
            ExitCode::SUCCESS
        }
        Err(msg) => {
            eprintln!("rookery: {msg}");
            ExitCode::FAILURE
        }
    }
}

/// Load, run for [`DURATION_S`], and return the final `z` of [`BODY`].
/// Errors come back as a message; nothing here panics on bad input.
fn run(cli: &Cli) -> Result<f64, String> {
    let sim = Sim::load(&cli.scene).map_err(|e| e.to_string())?;
    let body = sim
        .body_index(BODY)
        .ok_or_else(|| format!("{}: no body named `{BODY}`", cli.scene.display()))?;
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
    Ok(runner.sim().snapshot().bodies[body].pos[2])
}
