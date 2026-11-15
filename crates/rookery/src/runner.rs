//! The simulation loop (row 1.7).
//!
//! Pattern: **Template Method**. [`Runner::run`] fixes the skeleton of one
//! step, `pre_step → step → post_step`, plus `tick` every `tick_every`
//! steps. Phases change behaviour by implementing [`Hooks`], never by editing
//! the loop. Why: the order of reads, writes and physics is decided once,
//! here, so a later phase (the BT engine on `tick`, the run log on
//! `post_step`) cannot get it subtly wrong, and every phase shares one loop
//! that the gates exercise.
//!
//! Rust has no inheritance, so the "abstract class" is a trait with default
//! (no-op) methods, and the template method lives on a struct generic over it.
//!
//! ```text
//!   ┌──────────────────────────────┐        ┌───────────────────────────┐
//!   │ Runner<H: Hooks>             │ hooks  │ «trait» Hooks             │
//!   ├──────────────────────────────┤◆──────▶├───────────────────────────┤
//!   │ - sim: Sim                   │        │ + pre_step(&mut Sim)  {}  │
//!   │ - hooks: H                   │        │ + post_step(&mut Sim) {}  │
//!   │ - tick_every: NonZeroUsize   │        │ + tick(&mut Sim)      {}  │
//!   ├──────────────────────────────┤        └─────────────▲─────────────┘
//!   │ + run(steps)   «template»    │                      │ implements
//!   │ + sim() / hooks()            │                ┌─────┴─────┐
//!   └──────────────────────────────┘                │ NoHooks   │ (Phase 1)
//!                                                   └───────────┘
//!   run(steps): for i in 1..=steps {
//!       hooks.pre_step(sim); sim.step(); hooks.post_step(sim);
//!       if i % tick_every == 0 { hooks.tick(sim) }
//!   }
//! ```

use std::num::NonZeroUsize;

use sim::Sim;

/// The overridable steps of the loop. Every method defaults to a no-op, so an
/// implementation overrides only what its phase needs.
pub trait Hooks {
    /// Before physics: write commands (e.g. `Sim::apply`).
    fn pre_step(&mut self, _sim: &mut Sim) {}
    /// After physics: read state (e.g. `Sim::snapshot`).
    fn post_step(&mut self, _sim: &mut Sim) {}
    /// Every `tick_every` steps, after `post_step`: slower decisions (the BT
    /// engine from Phase 4). A no-op in Phase 1.
    fn tick(&mut self, _sim: &mut Sim) {}
}

/// Hooks that do nothing: the Phase 1 demo just lets physics run.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoHooks;

impl Hooks for NoHooks {}

/// Owns the [`Sim`] and the [`Hooks`] and drives the loop.
#[derive(Debug)]
pub struct Runner<H: Hooks> {
    sim: Sim,
    hooks: H,
    /// `NonZeroUsize` so `i % tick_every` can never divide by zero.
    tick_every: NonZeroUsize,
}

impl<H: Hooks> Runner<H> {
    pub fn new(sim: Sim, hooks: H, tick_every: NonZeroUsize) -> Self {
        Self {
            sim,
            hooks,
            tick_every,
        }
    }

    /// The template method: run `steps` physics steps with the hooks around
    /// each one. Not overridable on purpose.
    pub fn run(&mut self, steps: usize) {
        for i in 1..=steps {
            self.hooks.pre_step(&mut self.sim);
            self.sim.step();
            self.hooks.post_step(&mut self.sim);
            if i % self.tick_every == 0 {
                self.hooks.tick(&mut self.sim);
            }
        }
    }

    /// Read access to the sim, e.g. for a final snapshot.
    pub fn sim(&self) -> &Sim {
        &self.sim
    }

    /// Read access to the hooks, e.g. for what they recorded.
    // Used by the tests now; later phases read hook state through it.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn hooks(&self) -> &H {
        &self.hooks
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DROP_XML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/drop.xml");

    /// Records each hook call with the sim time it saw.
    #[derive(Default)]
    struct Recorder {
        calls: Vec<(&'static str, f64)>,
    }

    impl Hooks for Recorder {
        fn pre_step(&mut self, sim: &mut Sim) {
            self.calls.push(("pre", sim.snapshot().time));
        }
        fn post_step(&mut self, sim: &mut Sim) {
            self.calls.push(("post", sim.snapshot().time));
        }
        fn tick(&mut self, sim: &mut Sim) {
            self.calls.push(("tick", sim.snapshot().time));
        }
    }

    fn runner(tick_every: usize) -> Runner<Recorder> {
        let sim = Sim::load(DROP_XML).expect("drop.xml loads");
        let every = NonZeroUsize::new(tick_every).expect("non-zero");
        Runner::new(sim, Recorder::default(), every)
    }

    #[test]
    fn hooks_run_in_template_order_around_the_step() {
        let mut r = runner(1);
        r.run(2);
        let names: Vec<_> = r.hooks().calls.iter().map(|(n, _)| *n).collect();
        assert_eq!(names, ["pre", "post", "tick", "pre", "post", "tick"]);
        // pre sees the time before the step, post the time after it.
        let (_, t_pre) = r.hooks().calls[0];
        let (_, t_post) = r.hooks().calls[1];
        assert!(t_post > t_pre, "post_step ran before the physics step");
    }

    #[test]
    fn tick_fires_every_tick_every_steps() {
        let mut r = runner(3);
        r.run(10);
        let count = |name| r.hooks().calls.iter().filter(|(n, _)| *n == name).count();
        assert_eq!(count("pre"), 10);
        assert_eq!(count("post"), 10);
        assert_eq!(count("tick"), 3); // after steps 3, 6, 9
    }

    #[test]
    fn no_hooks_still_advances_time() {
        let sim = Sim::load(DROP_XML).expect("drop.xml loads");
        let dt = sim.timestep();
        let mut r = Runner::new(sim, NoHooks, NonZeroUsize::MIN);
        r.run(5);
        assert!((r.sim().snapshot().time - 5.0 * dt).abs() < 1e-12);
    }
}
