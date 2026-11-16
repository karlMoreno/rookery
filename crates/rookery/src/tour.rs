//! Playing a tour through the [`Runner`](crate::runner::Runner) (row 3.5).
//!
//! [`ScriptHooks`] implements the Template Method's hooks:
//!
//! ```text
//!   pre_step  (every 1 ms)  apply the current JointTargets to the actuators
//!   post_step (every 1 ms)  –
//!   tick      (every 20 ms) read GantryObs from the sim → tick the current
//!                           MoveCommand (through mast::clamp) → log qpos
//! ```
//!
//! Commands run one after another. A `Failed` command stops the tour (the
//! crane then holds its targets), and the run log records why.
//! `mast` never sees the sim: this module turns body poses into
//! [`GantryObs`].

use std::collections::VecDeque;
use std::io::Write;

use mast::{CmdStatus, Command, CommandLog, GantryObs, GantryState, MoveCommand, SceneMeta};
use sim::{Sim, Targets};

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

/// Hooks that play a list of commands and log every tick.
pub struct ScriptHooks<W: Write> {
    state: GantryState,
    queue: VecDeque<MoveCommand>,
    current: Option<(usize, MoveCommand)>,
    next_index: usize,
    dt: f64,
    rail_line_x: f64,
    ticks: u64,
    /// Commands as they started (rule 6).
    pub log: CommandLog,
    /// `time,qpos...` CSV, one row per tick.
    qpos_out: W,
    pub outcome: Option<Outcome>,
    /// Body ids of the crane, looked up once.
    ids: CraneIds,
    io_error: Option<std::io::Error>,
}

#[derive(Debug, Clone, Copy)]
struct CraneIds {
    truck: usize,
    carriage: usize,
    arm: usize,
    hook: usize,
}

impl<W: Write> ScriptHooks<W> {
    /// `dt` is the command tick length in seconds (tick_every × timestep).
    pub fn new(
        sim: &Sim,
        state: GantryState,
        cmds: Vec<MoveCommand>,
        meta: &SceneMeta,
        dt: f64,
        mut qpos_out: W,
    ) -> Result<Self, String> {
        let id = |n: &str| {
            sim.body_index(n)
                .ok_or_else(|| format!("scene has no crane body `{n}`"))
        };
        let ids = CraneIds {
            truck: id("crane_truck")?,
            carriage: id("crane_carriage")?,
            arm: id("crane_arm")?,
            hook: id("crane_hook")?,
        };
        let nq = sim.qpos().len();
        let header: Vec<String> = std::iter::once("time".to_string())
            .chain((0..nq).map(|i| format!("q{i}")))
            .collect();
        writeln!(qpos_out, "{}", header.join(",")).map_err(|e| e.to_string())?;
        Ok(Self {
            state,
            queue: cmds.into(),
            current: None,
            next_index: 0,
            dt,
            rail_line_x: meta.derived.rail_line_x,
            ticks: 0,
            log: CommandLog::default(),
            qpos_out,
            outcome: None,
            ids,
            io_error: None,
        })
    }

    /// Joint positions from body poses: truck y, carriage z, arm x relative
    /// to the rail, hook z relative to the carriage.
    fn observe(&self, sim: &Sim) -> GantryObs {
        let b = sim.snapshot().bodies;
        let id = self.ids;
        GantryObs {
            q: [
                b[id.truck].pos[1],
                b[id.carriage].pos[2],
                b[id.arm].pos[0] - self.rail_line_x,
                b[id.hook].pos[2] - b[id.carriage].pos[2],
            ],
            ee_contact: 0.0,
        }
    }

    /// The commanded targets right now.
    pub fn targets(&self) -> mast::JointTargets {
        self.state.targets
    }

    /// The first write error on the qpos log, if any.
    pub fn io_error(&self) -> Option<&std::io::Error> {
        self.io_error.as_ref()
    }

    fn log_qpos(&mut self, sim: &Sim) {
        if self.io_error.is_some() {
            return;
        }
        let mut row = format!("{:.6}", sim.snapshot().time);
        for q in sim.qpos() {
            row.push_str(&format!(",{q:.6}"));
        }
        if let Err(e) = writeln!(self.qpos_out, "{row}") {
            self.io_error = Some(e);
        }
    }

    /// Whether the tour has stopped (completed or failed).
    pub fn finished(&self) -> bool {
        self.outcome.is_some()
    }

    /// Mark the tour unfinished if time ran out first.
    pub fn close(&mut self) {
        if self.outcome.is_none() {
            let remaining = self.queue.len() + usize::from(self.current.is_some());
            self.outcome = Some(Outcome::Unfinished { remaining });
        }
    }
}

impl<W: Write> Hooks for ScriptHooks<W> {
    fn pre_step(&mut self, sim: &mut Sim) {
        // Errors cannot happen here: the scene has exactly the four crane
        // actuators, checked in run_tour before the first step.
        let _ = sim.apply(&Targets(self.state.targets.0.to_vec()));
    }

    fn tick(&mut self, sim: &mut Sim) {
        self.ticks += 1;
        self.log_qpos(sim);
        if self.outcome.is_some() {
            return;
        }
        if self.current.is_none() {
            match self.queue.pop_front() {
                Some(cmd) => {
                    self.log.push(self.ticks, cmd.clone());
                    self.current = Some((self.next_index, cmd));
                    self.next_index += 1;
                }
                None => {
                    self.outcome = Some(Outcome::Completed);
                    return;
                }
            }
        }
        let obs = self.observe(sim);
        if let Some((index, cmd)) = self.current.as_mut() {
            match cmd.tick(&mut self.state, &obs, self.dt) {
                CmdStatus::Running => {}
                CmdStatus::Done => self.current = None,
                CmdStatus::Failed(e) => {
                    self.outcome = Some(Outcome::Failed {
                        index: *index,
                        reason: e.to_string(),
                    });
                }
            }
        }
    }
}
