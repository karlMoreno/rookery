//! The exit-tray worker (row 8.7): a person who takes harvested boxes off
//! the exit tray, so its six slots never fill up over a long run.
//!
//! [`HARVEST_DELAY`] after a box is lifted out of its cell (`BoxLifted`) it
//! has long since been set down in its slot, so the worker carries it, with
//! the soft-shell crawfish and any shed shell inside, to a basket off to the
//! side of the farm, publishes `Harvested(cell)` and frees the slot.
//!
//! ```text
//!   BoxLifted(r_c) at t ─▶ worker.expect(r_c, t + 2 h)
//!   tick at bio ≥ due  ─▶ box_r_c, crawfish_r_c, shell_r_c ─set_body_pose─▶ basket k
//!                     ─▶ Harvested(r_c) on the bus, slot freed for the next transfer
//! ```
//!
//! The live run and `rookery replay` both use it, with the same timing, so a
//! replay reproduces the `Harvested` events too.

use rookery_core::{CellId, Event, HOUR, World};
use sim::Sim;

/// How long after `BoxLifted` the worker clears the box, biological seconds.
/// A transfer takes about 30 s, so the box is long settled in its slot.
pub const HARVEST_DELAY: f64 = 2.0 * HOUR;

/// Where cleared boxes go: a row on the ground beside the farm, one cell
/// pitch apart, out of the crane's reach (the rail runs at x ≈ -0.59 and
/// the tray ends at x = +0.31).
const BASKET_X: f64 = 0.9;
const BASKET_PITCH: f64 = 0.15;
const BASKET_Z: f64 = -0.01; // the ground plane

/// Clears harvested boxes off the exit tray.
#[derive(Debug, Default)]
pub struct ExitWorker {
    /// Boxes to clear, with when.
    due: Vec<(CellId, f64)>,
    /// Boxes cleared so far (each takes the next basket place).
    cleared: usize,
}

impl ExitWorker {
    /// Note the events just recorded at biological time `t`: each
    /// `BoxLifted` is a box to clear later.
    pub fn note(&mut self, events: &[Event], t: f64) {
        for e in events {
            if let Event::BoxLifted(c) = e {
                self.due.push((*c, t + HARVEST_DELAY));
            }
        }
    }

    /// The cells whose boxes [`ExitWorker::apply`] would clear at time `t`.
    pub fn due_now(&self, t: f64) -> Vec<CellId> {
        self.due
            .iter()
            .filter(|(_, d)| *d <= t)
            .map(|(c, _)| *c)
            .collect()
    }

    /// Clear every box that is due at biological time `t`: move it (and its
    /// animal and shell) to the basket and publish `Harvested`. Returns the
    /// events, in the order the boxes were lifted.
    pub fn apply(&mut self, sim: &mut Sim, world: &mut World, t: f64) -> Vec<Event> {
        let (now, later): (Vec<_>, Vec<_>) = self.due.drain(..).partition(|(_, d)| *d <= t);
        self.due = later;
        let mut out = Vec::new();
        for (cell, _) in now {
            let k = self.cleared as f64;
            self.cleared += 1;
            let basket = [
                BASKET_X + BASKET_PITCH * (k % 4.0),
                -0.3 + BASKET_PITCH * (k / 4.0).floor(),
                BASKET_Z,
            ];
            let tag = format!("{}_{}", cell.r, cell.c);
            let snap = sim.snapshot();
            let Some(b) = sim.body_index(&format!("box_{tag}")) else {
                continue;
            };
            let box_pos = snap.bodies[b].pos;
            for name in [
                format!("box_{tag}"),
                format!("crawfish_{tag}"),
                format!("shell_{tag}"),
            ] {
                let Some(i) = sim.body_index(&name) else {
                    continue;
                };
                let body = &snap.bodies[i];
                // Shells still parked stay parked; only what rides in the box moves.
                if name.starts_with("shell_") && body.pos[2] < -0.05 {
                    continue;
                }
                let pos = [
                    basket[0] + body.pos[0] - box_pos[0],
                    basket[1] + body.pos[1] - box_pos[1],
                    basket[2] + body.pos[2] - box_pos[2],
                ];
                let _ = sim.set_body_pose(&name, pos, body.quat);
            }
            let e = Event::Harvested(cell);
            world.bus.publish(0, e);
            out.push(e);
        }
        out
    }
}
