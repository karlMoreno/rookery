//! `rookery-core`: the farm's domain model.
//!
//! Row 7.1 declared this surface; row 7.3 filled in the state machine and
//! the bus. Rows 7.5 (shells, events to Lua) and 7.6 (biology) fill in the rest without changing a
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

mod bio;
pub use bio::{BioParams, CellBio};

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, LogNormal};

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
        Self::STAGES.iter().map(|&s| self.mean(s)).sum()
    }

    /// Expected standard deviation of the molt-to-molt interval, from the
    /// stage variances (the stages are independent).
    pub fn interval_sigma(&self) -> f64 {
        Self::STAGES
            .iter()
            .map(|&s| self.sigma(s).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    /// Coefficient of variation for every stage but intermolt.
    pub const STAGE_CV: f64 = 0.1;
    const STAGES: [MoltStage; 5] = [
        MoltStage::Intermolt,
        MoltStage::Premolt,
        MoltStage::Molting,
        MoltStage::SoftShell,
        MoltStage::Hardened,
    ];

    /// Mean length of `stage`, seconds.
    pub fn mean(&self, stage: MoltStage) -> f64 {
        match stage {
            MoltStage::Intermolt => self.intermolt_mean,
            MoltStage::Premolt => self.premolt_mean,
            MoltStage::Molting => self.molting_mean,
            MoltStage::SoftShell => self.soft_window,
            MoltStage::Hardened => self.hardened_mean,
        }
    }

    /// Standard deviation of the length of `stage`, seconds.
    pub fn sigma(&self, stage: MoltStage) -> f64 {
        match stage {
            MoltStage::Intermolt => self.intermolt_sigma,
            s => Self::STAGE_CV * self.mean(s),
        }
    }

    /// Draw a length for `stage`: lognormal with the stage's mean and sigma
    /// (so always positive). Parameters: σ_ln² = ln(1 + σ²/m²), μ_ln = ln m − σ_ln²/2.
    pub fn draw(&self, stage: MoltStage, rng: &mut ChaCha8Rng) -> f64 {
        let (m, s) = (self.mean(stage), self.sigma(stage));
        if m <= 0.0 {
            return 0.0;
        }
        let var_ln = (1.0 + (s / m).powi(2)).ln();
        let mu = m.ln() - var_ln / 2.0;
        LogNormal::new(mu, var_ln.sqrt())
            .map(|d| d.sample(rng))
            .unwrap_or(m)
    }
}

/// A cell: which tray, which row and column.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CellId {
    pub tray: u8,
    pub r: u8,
    pub c: u8,
}

impl CellId {
    /// A placeholder cell: `Crawfish::step` doesn't know where it lives, and
    /// `World` fills in the real cell.
    pub const NONE: CellId = CellId {
        tray: u8::MAX,
        r: u8::MAX,
        c: u8::MAX,
    };
}

impl Event {
    /// The same event, for `cell`.
    pub fn at(self, cell: CellId) -> Event {
        match self {
            Event::Molted(_) => Event::Molted(cell),
            Event::WindowClosing(_) => Event::WindowClosing(cell),
            Event::ShellEaten(_) => Event::ShellEaten(cell),
            Event::BoxLifted(_) => Event::BoxLifted(cell),
            Event::WindowMissed(_) => Event::WindowMissed(cell),
        }
    }
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

impl MoltStage {
    /// The stage that follows this one.
    pub fn next(self) -> MoltStage {
        match self {
            MoltStage::Intermolt => MoltStage::Premolt,
            MoltStage::Premolt => MoltStage::Molting,
            MoltStage::Molting => MoltStage::SoftShell,
            MoltStage::SoftShell => MoltStage::Hardened,
            MoltStage::Hardened => MoltStage::Intermolt,
        }
    }
}

impl Crawfish {
    /// A new animal at a random point of its intermolt, so a world doesn't
    /// molt in lockstep.
    pub fn new(params: &LifecycleParams, rng: &mut ChaCha8Rng) -> Self {
        let next_transition = params.draw(MoltStage::Intermolt, rng);
        let t_in_stage = rng.random::<f64>() * next_transition;
        Self {
            stage: MoltStage::Intermolt,
            t_in_stage,
            next_transition,
        }
    }

    /// Advance by `bio_dt` seconds. Returns the event of a stage change, if any.
    pub fn step(
        &mut self,
        bio_dt: f64,
        params: &LifecycleParams,
        rng: &mut ChaCha8Rng,
    ) -> Option<Event> {
        // One step may cross several short stages (molting lasts about an
        // hour). Each crossing may produce an event; `Molted` wins, because
        // the molt count is what the farm (and the gate) is built on.
        self.t_in_stage += bio_dt.max(0.0);
        let mut event = None;
        while self.t_in_stage >= self.next_transition {
            self.t_in_stage -= self.next_transition;
            let from = self.stage;
            self.stage = from.next();
            self.next_transition = params.draw(self.stage, rng).max(f64::MIN_POSITIVE);
            let e = match (from, self.stage) {
                (MoltStage::Premolt, MoltStage::Molting) => Some(Event::Molted(CellId::NONE)),
                (MoltStage::SoftShell, MoltStage::Hardened) => {
                    Some(Event::WindowClosing(CellId::NONE))
                }
                _ => None,
            };
            if !matches!(event, Some(Event::Molted(_))) && e.is_some() {
                event = e;
            }
        }
        event
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
        self.subscribers.push(f);
    }

    pub fn publish(&mut self, tick: Tick, e: Event) {
        self.log.push((tick, e));
        for s in &mut self.subscribers {
            s(&e, tick);
        }
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
        let mut rng = ChaCha8Rng::seed_from_u64(seed);
        let animals = cells
            .iter()
            .map(|&c| (c, Crawfish::new(&params, &mut rng)))
            .collect();
        World {
            params,
            rng,
            animals,
            bus: EventBus::default(),
            steps: 0,
        }
    }

    /// Advance every animal by `bio_dt` seconds, publish their events and
    /// return them.
    pub fn step(&mut self, bio_dt: f64) -> Vec<Event> {
        self.steps += 1;
        let mut out = Vec::new();
        for (cell, animal) in &mut self.animals {
            if let Some(e) = animal.step(bio_dt, &self.params, &mut self.rng) {
                out.push(e.at(*cell));
            }
        }
        for e in &out {
            self.bus.publish(self.steps, *e);
        }
        out
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
