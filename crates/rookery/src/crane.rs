//! The crane player shared by the scripted tour (3.5) and the behavior
//! tree (4.5): a queue of [`MoveCommand`]s played through `mast::clamp`,
//! the sim → [`GantryObs`] translation, and the per-tick `qpos` log.
//!
//! Pattern: plain composition. [`ScriptHooks`](crate::tour::ScriptHooks)
//! and [`BtHooks`](crate::bt_hooks::BtHooks) each *own* a `Crane` and
//! differ only in where commands come from: a pre-expanded script, or goals
//! the tree sets.
//!
//! ```text
//!   Crane::tick(sim):  log qpos → (idle? pull next command) → tick it with obs
//!                      → Step { Idle | Running | Done | Failed(reason) }
//!   Crane::apply(sim): write state.targets to the four actuators (every 1 ms)
//! ```

use std::collections::VecDeque;
use std::io::Write;

use mast::{
    CmdStatus, Command, CommandLog, GantryObs, GantryState, JointTargets, MoveCommand, SceneMeta,
};
use sim::{Sim, Targets};

/// What one [`Crane::tick`] did.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// Nothing to play.
    Idle,
    /// A command is in progress.
    Running,
    /// A command finished this tick.
    Done,
    /// A command failed this tick, and the queue was dropped.
    Failed(String),
}

#[derive(Debug, Clone, Copy)]
struct CraneIds {
    truck: usize,
    carriage: usize,
    arm: usize,
    hook: usize,
}

/// A queued command: a logged single-axis move, or any other command
/// (a macro such as `Transfer`) with a label for the event log.
// Used by the tests now, and by Pose::Transfer goals in row 6.5.
#[cfg_attr(not(test), allow(dead_code))]
enum Queued {
    Move(MoveCommand),
    Other(Box<dyn Command>, String),
}

/// Plays commands on the crane and logs every tick.
pub struct Crane<W: Write> {
    state: GantryState,
    queue: VecDeque<Queued>,
    current: Option<Box<dyn Command>>,
    dt: f64,
    rail_line_x: f64,
    ids: CraneIds,
    ticks: u64,
    /// How many commands have started.
    pub started: usize,
    /// Notable events by tick: macros starting, faults (row 6.4).
    pub events: Vec<(u64, String)>,
    /// Commands as they started (rule 6).
    pub log: CommandLog,
    qpos_out: W,
    io_error: Option<std::io::Error>,
}

impl<W: Write> Crane<W> {
    /// `dt` is the command tick length in seconds. Writes the qpos CSV header.
    pub fn new(
        sim: &Sim,
        state: GantryState,
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
            queue: VecDeque::new(),
            current: None,
            dt,
            rail_line_x: meta.derived.rail_line_x,
            ids,
            ticks: 0,
            started: 0,
            events: Vec::new(),
            log: CommandLog::default(),
            qpos_out,
            io_error: None,
        })
    }

    /// Append single-axis moves to the queue. Each is logged when it starts.
    pub fn enqueue(&mut self, cmds: impl IntoIterator<Item = MoveCommand>) {
        self.queue.extend(cmds.into_iter().map(Queued::Move));
    }

    /// Append any command, such as a whole `Transfer` macro (row 6.4).
    /// `label` goes into `events` when it starts.
    // Used by the tests now, and by Pose::Transfer goals in row 6.5.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn enqueue_command(&mut self, cmd: Box<dyn Command>, label: impl Into<String>) {
        self.queue.push_back(Queued::Other(cmd, label.into()));
    }

    /// Fault (row 6.4): replace the active command with its `undo()` and
    /// drop everything queued behind it. For a macro, the undo reverses its
    /// completed steps and the one in progress, so the crane puts back what
    /// it was carrying. Returns false when nothing was active.
    pub fn abort_active(&mut self) -> bool {
        let Some(cmd) = self.current.take() else {
            return false;
        };
        self.current = Some(cmd.undo());
        self.queue.clear();
        self.events.push((
            self.ticks,
            "fault: active command replaced by its undo".into(),
        ));
        true
    }

    /// Drop the queued commands. The one in progress finishes, so the hook
    /// is never left halfway along a route step.
    pub fn clear_queue(&mut self) {
        self.queue.clear();
    }

    /// Command ticks so far.
    pub fn ticks(&self) -> u64 {
        self.ticks
    }

    /// Commands left, the current one included.
    pub fn route_len(&self) -> usize {
        self.queue.len() + usize::from(self.current.is_some())
    }

    /// Whether a command is in progress.
    pub fn has_current(&self) -> bool {
        self.current.is_some()
    }

    /// The commanded targets right now.
    pub fn targets(&self) -> JointTargets {
        self.state.targets
    }

    /// The first write error on the qpos log, if any.
    pub fn io_error(&self) -> Option<&std::io::Error> {
        self.io_error.as_ref()
    }

    /// Joint positions from body poses: truck y, carriage z, arm x relative
    /// to the rail, hook z relative to the carriage.
    pub fn observe(&self, sim: &Sim) -> GantryObs {
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

    /// Send the current targets to the actuators. The scene has exactly the
    /// four crane actuators (checked before the run), so this cannot fail.
    pub fn apply(&self, sim: &mut Sim) {
        let _ = sim.apply(&Targets(self.state.targets.0.to_vec()));
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

    /// One command tick: log, start the next command if idle, advance it.
    pub fn tick(&mut self, sim: &Sim) -> Step {
        self.ticks += 1;
        self.log_qpos(sim);
        if self.current.is_none() {
            match self.queue.pop_front() {
                Some(Queued::Move(cmd)) => {
                    self.log.push(self.ticks, cmd.clone());
                    self.started += 1;
                    self.current = Some(Box::new(cmd));
                }
                Some(Queued::Other(cmd, label)) => {
                    self.events.push((self.ticks, format!("start: {label}")));
                    self.started += 1;
                    self.current = Some(cmd);
                }
                None => return Step::Idle,
            }
        }
        let obs = self.observe(sim);
        let Some(cmd) = self.current.as_mut() else {
            return Step::Idle;
        };
        match cmd.tick(&mut self.state, &obs, self.dt) {
            CmdStatus::Running => Step::Running,
            CmdStatus::Done => {
                self.current = None;
                Step::Done
            }
            CmdStatus::Failed(e) => {
                self.current = None;
                self.queue.clear();
                Step::Failed(e.to_string())
            }
        }
    }
}
