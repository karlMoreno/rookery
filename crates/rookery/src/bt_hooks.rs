//! Ticking the behavior tree in the loop (row 4.5).
//!
//! [`BtHooks`] implements the Template Method's hooks: physics at 1 kHz,
//! the tree at 50 Hz (`tick_every = 20`), never inside `step`.
//!
//! ```text
//!   pre_step (1 ms)   crane.apply(sim)                       targets → actuators
//!   tick (20 ms)      world.step(20 ms) → events → bb `events` + run log (7.2, 7.5)
//!                     detector → bb `detected` (soonest first)  (8.3)
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

use bt::{BbDetected, BbEvent, BbSchema, Blackboard, GantrySnapshot, Status, Tree};
use mast::route::{self, Pose};
use mast::{DONE_TOL, GantryState, JointTargets, SceneMeta};
use rookery_core::{CellId, CellObs, DetectorKind, Event, MoltDetector, World};
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
    /// Decides which cells have molted (row 8.3; pattern: **Strategy**).
    detector: Box<dyn MoltDetector>,
    /// Exit slots (row 8.4), when the scene has them: a transfer goes into
    /// the lowest free one.
    slots: Option<mast::SlotAllocator>,
    /// Which exit slot each transferred cell's box went to, in order.
    pub slot_of: Vec<(CellId, usize)>,
    /// Every 20 ms block of physics so far (row 8.6): the fast-forward jump
    /// before it and the targets held through it. A replay needs nothing else.
    pub blocks: Vec<Block>,
    pending_ff: f64,
    block_targets: [f64; 4],
    /// Takes harvested boxes off the exit tray and frees their slots (8.7).
    worker: crate::exit_worker::ExitWorker,
    /// Cells whose box the worker found resting in its exit slot when it
    /// came to clear it: delivered, even though the tray is empty later.
    pub delivered: Vec<CellId>,
}

/// Whether `cell`'s box rests in exit slot `slot` (rows 8.4, 8.7): its
/// floor's centre within 2 cm of the slot, horizontally, and within 1 cm of
/// the tray top. Boxes come first in qpos, row-major.
pub fn box_in_slot(qpos: &[f64], meta: &SceneMeta, cell: CellId, slot: usize) -> bool {
    let i = 7 * (usize::from(cell.r) * meta.cols + usize::from(cell.c));
    let (Some(b), Some(p)) = (qpos.get(i..i + 3), meta.derived.exit_slots.get(slot)) else {
        return false;
    };
    (b[0] - p[0]).hypot(b[1] - p[1]) < 0.02 && (b[2] - p[2]).abs() < 0.01
}

/// One block of physics as `targets.csv` records it (row 8.6).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Block {
    /// Biological seconds fast-forwarded just before the block (0 if none).
    pub ff: f64,
    /// Actuator targets held for the whole block (rail_y, mast_z, arm_x, hook_lift).
    pub targets: [f64; 4],
}

/// Harvest every cell whose box has risen above [`LIFTED_Z`] (row 8.2) and
/// return the `BoxLifted` events, once per cell. Shared by the live run
/// and the replay, so both decide the same way. Boxes come first in qpos,
/// in the world's (row-major) cell order.
pub fn harvest_lifted(world: &mut World, qpos: &[f64]) -> Vec<Event> {
    let cells: Vec<CellId> = world.animals().iter().map(|(c, _)| *c).collect();
    cells
        .into_iter()
        .enumerate()
        .filter(|&(i, _)| qpos.get(7 * i + 2).is_some_and(|z| *z > LIFTED_Z))
        .filter_map(|(_, c)| world.harvest(c))
        .collect()
}

/// Ticks the crane must sit idle before the clock may fast-forward (0.5 s at
/// 50 Hz), so a tree between two goals, or one that hasn't ticked yet, gets
/// the chance to ask for something first.
pub const IDLE_TICKS_BEFORE_FF: u32 = 25;

/// A transferred box counts as lifted out of its cell (`BoxLifted`, row
/// 8.2) once its floor is this far above the tray floor, metres. The hook
/// lifts a box about 15 cm to carry it, and a box at rest sits within a
/// millimetre of 0.
pub const LIFTED_Z: f64 = 0.02;

impl<W: Write> BtHooks<W> {
    pub fn new(
        sim: &Sim,
        tree: Tree,
        state: GantryState,
        meta: SceneMeta,
        dt: f64,
        qpos_out: W,
    ) -> Result<Self, String> {
        let start_targets = state.targets.0;
        let crane = Crane::new(sim, state, &meta, dt, qpos_out)?;
        let n_slots = meta.derived.exit_slots.len();
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
            detector: DetectorKind::default().build(),
            slots: (n_slots > 0).then(|| mast::SlotAllocator::new(n_slots)),
            slot_of: Vec::new(),
            blocks: Vec::new(),
            pending_ff: 0.0,
            block_targets: start_targets,
            worker: Default::default(),
            delivered: Vec::new(),
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
        self.worker.note(&events, self.bio_time);
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
        self.pending_ff += dt;
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
        Event::Harvested(c) => ("Harvested", c),
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
        // Row 8.3: what the detector sees, for the tree's `detected` key.
        if let Some(world) = self.world.as_ref() {
            let obs = CellObs::default(); // the oracle reads the world, not the sim
            let mut cells = Vec::new();
            for (i, (cell, _)) in world.animals().iter().enumerate() {
                if self.detector.detect(*cell, &obs, world).is_some() {
                    cells.push(BbDetected {
                        tray: cell.tray,
                        r: cell.r,
                        c: cell.c,
                        closes_in: world.window_left(i).unwrap_or(0.0),
                    });
                }
            }
            self.schema.write_detected(&mut self.bb, cells);
        }
        // Shells move before the snapshot, so the tree and physics see them.
        if let Some(mover) = self.shells.as_mut() {
            let errors = mover.apply(sim, self.bio_time);
            self.errors.extend(errors);
        }
        // Row 8.7: the worker clears boxes harvested 2 h ago and frees their
        // slots, before the tree plans the next transfer.
        if let Some(world) = self.world.as_mut() {
            let q = sim.qpos();
            for c in self.worker.due_now(self.bio_time) {
                if let Some(&(_, i)) = self.slot_of.iter().rev().find(|(sc, _)| *sc == c)
                    && box_in_slot(&q, &self.meta, c, i)
                {
                    self.delivered.push(c);
                }
            }
            let cleared = self.worker.apply(sim, world, self.bio_time);
            for e in &cleared {
                if let (Event::Harvested(c), Some(a)) = (e, self.slots.as_mut())
                    && let Some(&(_, i)) = self.slot_of.iter().rev().find(|(sc, _)| sc == c)
                {
                    a.free(i);
                }
            }
            self.record(cleared);
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
                // Row 8.4: into the lowest free exit slot, when the scene
                // has slots; onto the middle of the exit tray otherwise.
                let slot = match self.slots.as_mut() {
                    None => Ok(None),
                    Some(a) => a.take().map(Some).ok_or(()),
                };
                let built = slot
                    .map_err(|()| "every exit slot is full".to_string())
                    .and_then(|slot| {
                        mast::Transfer::build_to(self.crane.targets(), r, c, slot, &self.meta)
                            .map(|t| (t, slot))
                            .map_err(|e| {
                                if let (Some(a), Some(i)) = (self.slots.as_mut(), slot) {
                                    a.free(i);
                                }
                                e.to_string()
                            })
                    });
                match built {
                    Ok((t, slot)) => {
                        if let Some(i) = slot {
                            self.slot_of.push((
                                CellId {
                                    tray: 0,
                                    r: r as u8,
                                    c: c as u8,
                                },
                                i,
                            ));
                        }
                        let label = match slot {
                            Some(i) => format!("transfer ({r}, {c}) to exit slot {i}"),
                            None => format!("transfer ({r}, {c})"),
                        };
                        self.crane.enqueue_command(Box::new(t), label);
                    }
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

        // Row 8.2: a box has left its cell, so its animal is harvested. The
        // world publishes BoxLifted once. Any box counts, not only the
        // goal's, so a replay (8.6), which has no goals, decides the same.
        if let Some(world) = self.world.as_mut() {
            let lifted = harvest_lifted(world, &sim.qpos());
            self.record(lifted);
        }
        // Row 8.6: the block of physics that just ran, for targets.csv.
        self.blocks.push(Block {
            ff: std::mem::take(&mut self.pending_ff),
            targets: self.block_targets,
        });
        self.block_targets = self.crane.targets().0;

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
