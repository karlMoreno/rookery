//! `rookery-core`: the farm's domain model.
//!
//! Row 7.1 declares this surface over `todo!()`. Rows 7.3 (state machine),
//! 7.5 (event bus) and 7.6 (biology) fill it in without changing a
//! signature. The Phase 7 gate (`tests/gates/tests/phase7_lifecycle.rs`,
//! part A) is written against [`World`] and [`LifecycleParams`].
//!
//! Patterns:
//! - **State**: the crawfish lifecycle
//!   `Intermolt → Premolt → Molting → SoftShell → Hardened → Intermolt`,
//!   clock-driven, with each stage's length drawn from [`LifecycleParams`].
//! - **Observer**: an [`EventBus`] (`Molted`, `ShellEaten`, …) so the shell
//!   mover, the run log and the behaviour tree subscribe instead of polling.
//! - **Strategy**: [`MoltDetector`], an oracle now and a camera in Phase 11.
//!
//! ```text
//!   World { params, rng: ChaCha8Rng(seed), animals: Vec<(CellId, Crawfish)>, bus }
//!     step(bio_dt) ─▶ each Crawfish::step ─▶ Option<Event> ─▶ bus.publish ─▶ subscribers + log
//!
//!   «State» Intermolt ─▶ Premolt ─▶ Molting ─(Molted)─▶ SoftShell ─(WindowClosing)─▶ Hardened ─▶ Intermolt
//! ```
//!
//! Times are seconds of simulated (biological) time. `rookery-core` depends
//! on no other Rookery crate. The Integrator turns sim snapshots into
//! [`CellObs`], so it never needs `sim`, and `mast` never depends on it
//! (a CI check since Phase 3).

use rand_chacha::ChaCha8Rng;

/// A 50 Hz tick count, or a biology step count, used to stamp events.
pub type Tick = u64;

/// One second of simulated time.
pub const SECOND: f64 = 1.0;
pub const HOUR: f64 = 3600.0 * SECOND;
pub const DAY: f64 = 24.0 * HOUR;

/// The stages of one molt cycle (pattern: **State**).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MoltStage {
    Intermolt,
    Premolt,
    Molting,
    SoftShell,
    Hardened,
}

/// How long each stage lasts, in seconds. Intermolt is lognormal with the
/// given mean and sigma; the other stages are lognormal with a 10%
/// coefficient of variation around their means (row 7.3).
///
/// Defaults are juvenile red swamp crawfish, from the v5 notes: a molt cycle
/// of about 30 days, a soft-shell window of a few hours.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LifecycleParams {
    pub intermolt_mean: f64,
    pub intermolt_sigma: f64,
    pub premolt_mean: f64,
    pub molting_mean: f64,
    pub soft_window: f64,
    pub hardened_mean: f64,
    /// How long a shed shell lies in the box before the animal eats it.
    pub shell_delay: f64,
}

impl Default for LifecycleParams {
    fn default() -> Self {
        Self {
            intermolt_mean: 24.0 * DAY,
            intermolt_sigma: 4.0 * DAY,
            premolt_mean: 3.0 * DAY,
            molting_mean: HOUR,
            soft_window: 6.0 * HOUR,
            hardened_mean: 2.0 * DAY,
            shell_delay: DAY,
        }
    }
}

impl LifecycleParams {
    /// Expected time from one molt to the next: the sum of the stage means.
    pub fn interval_mean(&self) -> f64 {
        todo!("row 7.3: sum of the stage means")
    }

    /// Expected standard deviation of the molt-to-molt interval, from the
    /// stage variances (the stages are independent).
    pub fn interval_sigma(&self) -> f64 {
        todo!("row 7.3: sqrt of the summed stage variances")
    }
}

/// A cell: which tray, which row and column.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CellId {
    pub tray: u8,
    pub r: u8,
    pub c: u8,
}

/// Something that happened to the animal in a cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// The animal left its old shell: a second body appears in the box.
    Molted(CellId),
    /// The soft-shell window is ending: harvest now or miss it.
    WindowClosing(CellId),
    /// The animal ate its shed shell.
    ShellEaten(CellId),
    /// The crane lifted this cell's box.
    BoxLifted(CellId),
    /// The window closed with the animal still in its box.
    WindowMissed(CellId),
}

/// One animal and where it is in its cycle.
#[derive(Debug, Clone, PartialEq)]
pub struct Crawfish {
    pub stage: MoltStage,
    /// Seconds spent in the current stage.
    pub t_in_stage: f64,
    /// Length of the current stage, drawn when the stage began.
    pub next_transition: f64,
}

impl Crawfish {
    /// A new animal at a random point of its intermolt, so a world doesn't
    /// molt in lockstep.
    pub fn new(params: &LifecycleParams, rng: &mut ChaCha8Rng) -> Self {
        let _ = (params, rng);
        todo!("row 7.3: start in Intermolt at a random phase")
    }

    /// Advance by `bio_dt` seconds. Returns the event of a stage change, if any.
    pub fn step(
        &mut self,
        bio_dt: f64,
        params: &LifecycleParams,
        rng: &mut ChaCha8Rng,
    ) -> Option<Event> {
        let _ = (bio_dt, params, rng);
        todo!("row 7.3: advance, transition, draw the next duration")
    }
}

/// A subscriber: called with every published event and its tick.
pub type Subscriber = Box<dyn FnMut(&Event, Tick)>;

/// Publishes events to subscribers and keeps an append-only log (pattern:
/// **Observer**).
#[derive(Default)]
pub struct EventBus {
    subscribers: Vec<Subscriber>,
    log: Vec<(Tick, Event)>,
}

impl std::fmt::Debug for EventBus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EventBus")
            .field("subscribers", &self.subscribers.len())
            .field("log", &self.log.len())
            .finish()
    }
}

impl EventBus {
    pub fn subscribe(&mut self, f: Subscriber) {
        let _ = (f, &mut self.subscribers);
        todo!("row 7.5")
    }

    pub fn publish(&mut self, tick: Tick, e: Event) {
        let _ = (tick, e);
        todo!("row 7.5: log, then call every subscriber")
    }

    pub fn log(&self) -> &[(Tick, Event)] {
        &self.log
    }
}

/// Every animal in the farm, one per cell, and the bus their events go to.
pub struct World {
    pub params: LifecycleParams,
    rng: ChaCha8Rng,
    animals: Vec<(CellId, Crawfish)>,
    pub bus: EventBus,
    steps: Tick,
}

impl World {
    /// One animal per cell, all drawn from `seed`, so the same seed gives
    /// the same history.
    pub fn new(params: LifecycleParams, seed: u64, cells: &[CellId]) -> Self {
        let _ = (params, seed, cells);
        todo!("row 7.3: seed the rng, create one Crawfish per cell")
    }

    /// Advance every animal by `bio_dt` seconds, publish their events and
    /// return them.
    pub fn step(&mut self, bio_dt: f64) -> Vec<Event> {
        let _ = (bio_dt, &mut self.rng, &mut self.animals, self.steps);
        todo!("rows 7.3 + 7.5: step each animal, publish, return")
    }

    /// The animals and their cells.
    pub fn animals(&self) -> &[(CellId, Crawfish)] {
        &self.animals
    }
}

/// What a detector can see of one cell, built by the Integrator from the
/// sim snapshot.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CellObs {
    /// Bodies inside the box: 1 normally, 2 just after a molt (animal + shell).
    pub bodies_in_box: u8,
    pub shell_present: bool,
}

/// A detector's verdict for one cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoltObservation {
    pub cell: CellId,
    /// How sure, 0..=1.
    pub confidence: f64,
}

/// Decides from what it can see whether a cell has just molted (pattern:
/// **Strategy**). The oracle reads `World`; the Phase 11 camera reads depth.
pub trait MoltDetector {
    fn detect(&mut self, cell: CellId, obs: &CellObs, world: &World) -> Option<MoltObservation>;
}
