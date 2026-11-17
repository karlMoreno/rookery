//! Ticking the behavior tree in the loop (row 4.5).
//!
//! [`BtHooks`] implements the Template Method's hooks: physics at 1 kHz,
//! the tree at 50 Hz (`tick_every = 20`), never inside `step`.
//!
//! ```text
//!   pre_step (1 ms)   crane.apply(sim)                       targets → actuators
//!   tick (20 ms)      world.step(20 ms) → events → bb `events` + run log (7.2, 7.5)
//!                     shells.apply(sim)                       move shed shells (7.5)
//!                     obs = crane.observe(sim)
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

use bt::{BbEvent, BbSchema, Blackboard, GantrySnapshot, Status, Tree};
use mast::route::{self, Pose};
use mast::{DONE_TOL, GantryState, JointTargets, SceneMeta};
use rookery_core::{CellId, Event, World};
use sim::Sim;

use crate::crane::{Crane, Step};
use crate::runner::Hooks;
use crate::shells::ShellMover;

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
    /// The animals (row 7.2). Biology advances with every physics tick, and
    /// jumps ahead in `fast_forward` while the crane is idle.
    world: Option<World>,
    /// Biological seconds elapsed.
    bio_time: f64,
    /// Lifecycle events with the biological time they happened at.
    pub bio_events: Vec<(f64, Event)>,
    /// Command tick length, seconds.
    tick_dt: f64,
    /// Consecutive ticks the crane has had nothing to do.
    idle_ticks: u32,
    /// Moves shed shells on `Molted` / `ShellEaten` (row 7.5).
    pub shells: Option<ShellMover>,
    /// Events handed to the tree so far (the `seq` of the last one).
    event_seq: u64,
}

/// Ticks the crane must sit idle before the clock may fast-forward (0.5 s at
/// 50 Hz), so a tree between two goals, or one that hasn't ticked yet, gets
/// the chance to ask for something first.
pub const IDLE_TICKS_BEFORE_FF: u32 = 25;

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
            world: None,
            bio_time: 0.0,
            bio_events: Vec::new(),
            tick_dt: dt,
            idle_ticks: 0,
            shells: None,
            event_seq: 0,
        })
    }

    /// Give the hooks a shell mover, already subscribed to the world's bus
    /// (row 7.5).
    pub fn with_shells(mut self, mover: Option<ShellMover>) -> Self {
        self.shells = mover;
        self
    }

    /// Log events at the current biological time and hand them to the tree
    /// through the `events` key (row 7.5).
    fn record(&mut self, events: Vec<Event>) {
        for e in events {
            self.event_seq += 1;
            let (kind, cell) = describe(e);
            self.schema.push_event(
                &mut self.bb,
                BbEvent {
                    seq: self.event_seq,
                    t: self.bio_time,
                    kind,
                    tray: cell.tray,
                    r: cell.r,
                    c: cell.c,
                },
            );
            self.bio_events.push((self.bio_time, e));
        }
    }

    /// Give the hooks a World to run (row 7.2).
    pub fn with_world(mut self, world: World) -> Self {
        self.world = Some(world);
        self
    }

    /// Biological seconds elapsed.
    pub fn bio_time(&self) -> f64 {
        self.bio_time
    }

    /// Whether the crane has nothing to do: no command, nothing queued, no
    /// goal waiting to be planned. Only then may the clock fast-forward.
    pub fn idle(&self) -> bool {
        self.crane_idle() && self.idle_ticks >= IDLE_TICKS_BEFORE_FF
    }

    fn crane_idle(&self) -> bool {
        self.crane.route_len() == 0 && self.pending.is_none()
    }

    /// Fast-forward (row 7.2): with the crane idle, skip physics and advance
    /// biology straight to the next stage change, capped at `max_dt`.
    /// Returns the seconds skipped (0 if the crane is busy or there's no world).
    pub fn fast_forward(&mut self, max_dt: f64) -> f64 {
        if !self.idle() {
            return 0.0;
        }
        let Some(world) = self.world.as_mut() else {
            return 0.0;
        };
        // A hair past the transition, so it is crossed in this step.
        let dt = (world.time_to_next_transition() + 1e-6).min(max_dt.max(0.0));
        if dt <= 0.0 {
            return 0.0;
        }
        let events = world.step(dt);
        self.bio_time += dt;
        // The shell mover's inbox has these too; it acts on the next tick,
        // which the clock always runs right after a jump.
        self.record(events);
        dt
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

/// An event's name (as Lua sees it) and its cell.
pub fn describe(e: Event) -> (&'static str, CellId) {
    match e {
        Event::Molted(c) => ("Molted", c),
        Event::WindowClosing(c) => ("WindowClosing", c),
        Event::ShellEaten(c) => ("ShellEaten", c),
        Event::BoxLifted(c) => ("BoxLifted", c),
        Event::WindowMissed(c) => ("WindowMissed", c),
    }
}

impl<W: Write> Hooks for BtHooks<W> {
    fn pre_step(&mut self, sim: &mut Sim) {
        self.crane.apply(sim);
    }

    fn tick(&mut self, sim: &mut Sim) {
        // Real time: biology advances with the physics tick (row 7.2).
        self.bio_time += self.tick_dt;
        if let Some(world) = self.world.as_mut() {
            let events = world.step(self.tick_dt);
            self.record(events);
        }
        // Shells move before the snapshot, so the tree and physics see them.
        if let Some(mover) = self.shells.as_mut() {
            let errors = mover.apply(sim, self.bio_time);
            self.errors.extend(errors);
        }
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
            if let Pose::Transfer(r, c) = pose {
                // A job, not a place (row 6.5): one Transfer macro, so a
                // fault can undo it as a whole.
                match mast::Transfer::build(self.crane.targets(), r, c, &self.meta) {
                    Ok(t) => self
                        .crane
                        .enqueue_command(Box::new(t), format!("transfer ({r}, {c})")),
                    Err(e) => self.errors.push(format!("route: goal {pose:?}: {e}")),
                }
            } else {
                match route::route(self.crane.targets(), pose, &self.meta) {
                    Ok(cmds) => self.crane.enqueue(cmds),
                    Err(e) => self.errors.push(format!("route: goal {pose:?}: {e}")),
                }
            }
        }

        if self.fault_at == Some(self.crane.ticks() + 1) && self.crane.abort_active() {
            self.errors.push(format!(
                "fault: injected at tick {}",
                self.crane.ticks() + 1
            ));
        }
        let step = self.crane.tick(sim);
        self.idle_ticks = if self.crane_idle() {
            self.idle_ticks.saturating_add(1)
        } else {
            0
        };
        if let Step::Failed(reason) = step {
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
