//! Ticking the behavior tree in the loop (row 4.5).
//!
//! [`BtHooks`] implements the Template Method's hooks: physics at 1 kHz,
//! the tree at 50 Hz (`tick_every = 20`), never inside `step`.
//!
//! ```text
//!   pre_step (1 ms)   crane.apply(sim)                       targets → actuators
//!   tick (20 ms)      obs = crane.observe(sim)
//!                     schema.write_snapshot(bb, {sim_time, q, targets, route_len})
//!                     tree.tick(bb)                           Lua leaves read bb, may bb:go_to(pose)
//!                     new goal? → drop the queue (the current move finishes)
//!                     idle and a goal pending? → mast::route(targets → goal) → enqueue
//!                     crane.tick(sim)                         one MoveCommand through mast::clamp
//! ```
//!
//! [Band 1] Lua sets goals, never raw targets. A goal off the grid is a
//! named error, and the crane holds. A route step that fails drops the
//! rest of the route, so the tree can set a new goal.

use std::io::Write;

use bt::{BbSchema, Blackboard, GantrySnapshot, Status, Tree};
use mast::route::{self, Pose};
use mast::{DONE_TOL, GantryState, JointTargets, SceneMeta};
use sim::Sim;

use crate::crane::{Crane, Step};
use crate::runner::Hooks;

/// Hooks that let a behavior tree drive the crane through goals.
pub struct BtHooks<W: Write> {
    crane: Crane<W>,
    tree: Tree,
    bb: Blackboard,
    schema: BbSchema,
    meta: SceneMeta,
    last_goal: Option<Pose>,
    pending: Option<Pose>,
    /// How many times the tree has ticked.
    pub tree_ticks: u64,
    /// The root's status after the latest tick.
    pub last_status: Option<Status>,
    /// Every error so far, oldest first: leaf errors as `leaf: message`,
    /// route and command errors as `route: …` / `move: …`.
    pub errors: Vec<String>,
    /// Goals the crane arrived at, in order (row 4.7): the route empty and
    /// every measured joint within DONE_TOL of its target. One entry per goal.
    pub arrivals: Vec<Pose>,
    arrived: bool,
    /// Fault injection (row 6.4): on this command tick, the active command is
    /// replaced by its undo.
    pub fault_at: Option<u64>,
}

impl<W: Write> BtHooks<W> {
    pub fn new(
        sim: &Sim,
        tree: Tree,
        state: GantryState,
        meta: SceneMeta,
        dt: f64,
        qpos_out: W,
    ) -> Result<Self, String> {
        let crane = Crane::new(sim, state, &meta, dt, qpos_out)?;
        let mut bb = Blackboard::new();
        let schema = BbSchema::register(&mut bb);
        Ok(Self {
            crane,
            tree,
            bb,
            schema,
            meta,
            last_goal: None,
            pending: None,
            tree_ticks: 0,
            last_status: None,
            errors: Vec::new(),
            arrivals: Vec::new(),
            arrived: false,
            fault_at: None,
        })
    }

    /// The commanded targets right now.
    pub fn targets(&self) -> JointTargets {
        self.crane.targets()
    }

    /// The crane player (for the command log and qpos errors).
    pub fn crane(&self) -> &Crane<W> {
        &self.crane
    }

    /// The crane player, mutably (to enqueue commands directly).
    // Used by the tests now, and by Pose::Transfer goals in row 6.5.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn crane_mut(&mut self) -> &mut Crane<W> {
        &mut self.crane
    }

    /// The goal the tree last set.
    pub fn goal(&self) -> Option<Pose> {
        self.last_goal
    }
}

impl<W: Write> Hooks for BtHooks<W> {
    fn pre_step(&mut self, sim: &mut Sim) {
        self.crane.apply(sim);
    }

    fn tick(&mut self, sim: &mut Sim) {
        let obs = self.crane.observe(sim);
        let snap = GantrySnapshot {
            sim_time: sim.snapshot().time,
            gantry_q: obs.q,
            gantry_target: self.crane.targets().0,
            route_len: self.crane.route_len() + usize::from(self.pending.is_some()),
        };
        self.schema.write_snapshot(&mut self.bb, &snap);

        self.last_status = Some(self.tree.tick(&mut self.bb));
        self.tree_ticks += 1;
        for e in self.tree.take_errors() {
            self.errors.push(format!("{}: {}", e.leaf, e.message));
        }

        let goal = self.bb.get(&self.schema.gantry_goal).copied().flatten();
        if goal != self.last_goal {
            self.last_goal = goal;
            self.arrived = false;
            self.crane.clear_queue();
            self.pending = goal;
        }
        if let Some(pose) = self.pending
            && !self.crane.has_current()
        {
            self.pending = None;
            match route::route(self.crane.targets(), pose, &self.meta) {
                Ok(cmds) => self.crane.enqueue(cmds),
                Err(e) => self.errors.push(format!("route: goal {pose:?}: {e}")),
            }
        }

        if self.fault_at == Some(self.crane.ticks() + 1) && self.crane.abort_active() {
            self.errors.push(format!(
                "fault: injected at tick {}",
                self.crane.ticks() + 1
            ));
        }
        if let Step::Failed(reason) = self.crane.tick(sim) {
            self.errors.push(format!("move: {reason}"));
        }

        // Arrival: the goal's route is done and the joints have settled.
        if let Some(pose) = self.last_goal
            && !self.arrived
            && self.pending.is_none()
            && self.crane.route_len() == 0
        {
            let q = self.crane.observe(sim).q;
            let t = self.crane.targets().0;
            if q.iter().zip(t).all(|(a, b)| (a - b).abs() <= DONE_TOL) {
                self.arrived = true;
                self.arrivals.push(pose);
            }
        }
    }
}
