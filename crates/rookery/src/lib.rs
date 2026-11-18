//! `rookery` as a library: the Integrator's parts, so the binary and the
//! phase gates drive the same code (row 8.1).
//!
//! ```text
//!   main.rs (CLI: summarise, run --script/--tree/--scenario, lift)
//!      │ uses
//!   lib.rs ── autonomy unattended runs: the Phase 8 gate's entry point
//!          ── runner   Template Method: physics at 1 kHz, hooks at 50 Hz
//!          ── crane    MoveCommand player, sim → GantryObs, qpos log
//!          ── bt_hooks the behavior tree drives the crane through goals
//!          ── shells   the shell mover (Observer of the World's bus)
//!          ── scenario Lua scenarios and the run log they check
//!          ── tour / script / lift_trials   the Phase 3 and 5 drivers
//!      ▲ dev-dependency
//!   tests/gates (Phase 8 on)
//! ```

pub mod autonomy;
pub mod bt_hooks;
pub mod crane;
pub mod lift_trials;
pub mod runner;
pub mod scenario;
pub mod script;
pub mod shells;
pub mod tour;
