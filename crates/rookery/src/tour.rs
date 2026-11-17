//! Playing a tour through the [`Runner`](crate::runner::Runner) (row 3.5).
//!
//! [`ScriptHooks`] implements the Template Method's hooks on top of the
//! shared [`Crane`] player:
//!
//! ```text
//!   pre_step  (every 1 ms)  apply the current JointTargets to the actuators
//!   post_step (every 1 ms)  –
//!   tick      (every 20 ms) log qpos → tick the current MoveCommand
//!                           (through mast::clamp) → next command when done
//! ```
//!
//! Commands run one after another. A `Failed` command stops the tour (the
//! crane then holds its targets), and the run log records why.

use std::io::Write;

use mast::{CommandLog, GantryState, JointTargets, MoveCommand, SceneMeta};
use sim::Sim;

use crate::crane::{Crane, Step};
use crate::runner::Hooks;

/// What a finished tour reports.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// Every command reported `Done`.
    Completed,
    /// A command failed: its index in the plan and the reason.
    Failed { index: usize, reason: String },
    /// Time ran out with commands still queued.
    Unfinished { remaining: usize },
}

/// Hooks that play a pre-expanded list of commands and log every tick.
pub struct ScriptHooks<W: Write> {
    crane: Crane<W>,
    pub outcome: Option<Outcome>,
    /// Fault injection (row 6.4): on this command tick, the active command is
    /// replaced by its undo and the rest of the script is dropped.
    pub fault_at: Option<u64>,
}

impl<W: Write> ScriptHooks<W> {
    /// `dt` is the command tick length in seconds (tick_every × timestep).
    pub fn new(
        sim: &Sim,
        state: GantryState,
        cmds: Vec<MoveCommand>,
        meta: &SceneMeta,
        dt: f64,
        qpos_out: W,
    ) -> Result<Self, String> {
        let mut crane = Crane::new(sim, state, meta, dt, qpos_out)?;
        crane.enqueue(cmds);
        Ok(Self {
            crane,
            outcome: None,
            fault_at: None,
        })
    }

    /// Commands as they started.
    pub fn log(&self) -> &CommandLog {
        &self.crane.log
    }

    /// The commanded targets right now.
    pub fn targets(&self) -> JointTargets {
        self.crane.targets()
    }

    /// The first write error on the qpos log, if any.
    pub fn io_error(&self) -> Option<&std::io::Error> {
        self.crane.io_error()
    }

    /// Whether the tour has stopped (completed or failed).
    pub fn finished(&self) -> bool {
        self.outcome.is_some()
    }

    /// Mark the tour unfinished if time ran out first.
    pub fn close(&mut self) {
        if self.outcome.is_none() {
            self.outcome = Some(Outcome::Unfinished {
                remaining: self.crane.route_len(),
            });
        }
    }
}

impl<W: Write> Hooks for ScriptHooks<W> {
    fn pre_step(&mut self, sim: &mut Sim) {
        self.crane.apply(sim);
    }

    fn tick(&mut self, sim: &mut Sim) {
        if self.fault_at == Some(self.crane.ticks() + 1) {
            self.crane.abort_active();
        }
        let step = self.crane.tick(sim);
        if self.outcome.is_some() {
            return;
        }
        match step {
            Step::Idle => self.outcome = Some(Outcome::Completed),
            Step::Running | Step::Done => {}
            Step::Failed(reason) => {
                self.outcome = Some(Outcome::Failed {
                    index: self.crane.started - 1,
                    reason,
                });
            }
        }
    }
}
