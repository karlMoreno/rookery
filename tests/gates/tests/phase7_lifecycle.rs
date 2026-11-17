//! Phase 7 gate, part A: the molt-interval distribution.
//!
//! 1,000 animals (one per synthetic cell), one seeded `World`, stepped
//! hourly through one simulated year. For every animal, each pair of
//! consecutive `Molted` events gives one interval. Across all intervals:
//!   * the mean is within 5% of `LifecycleParams::interval_mean()`;
//!   * the standard deviation is within 15% of `interval_sigma()`;
//!   * the same seed gives the identical event log twice (determinism).
//!
//! Pure arithmetic, no physics, so it costs CI seconds, not minutes.
//!
//! Part B (every `Molted` has its shell in the box, with physics) is the
//! scenario `lua/scenarios/week.lua`. It needs the fast-forward clock (7.2),
//! so it is committed with `ci = false` and switched on when 7.2/7.5 land.
//!
//! ```text
//!   World::new(default params, seed 7, 1000 cells) ─▶ step(1 h) × 8760 ─▶ Molted log
//!     ─▶ per-cell intervals ─▶ mean ≈ interval_mean ±5%, σ ≈ interval_sigma ±15%
//! ```
//!
//! Red on `main` from row 7.1 (`World::new` is `todo!()`). Green once 7.3 lands.

use rookery_core::{CellId, DAY, Event, HOUR, LifecycleParams, World};

const ANIMALS: usize = 1000;
const SEED: u64 = 7;
const BIO_DT: f64 = HOUR;
const YEAR: f64 = 365.0 * DAY;

fn cells() -> Vec<CellId> {
    (0..ANIMALS)
        .map(|i| CellId {
            tray: (i / 256) as u8,
            r: ((i / 16) % 16) as u8,
            c: (i % 16) as u8,
        })
        .collect()
}

fn run(seed: u64) -> Vec<(usize, CellId)> {
    let mut w = World::new(LifecycleParams::default(), seed, &cells());
    let steps = (YEAR / BIO_DT) as usize;
    let mut molts = Vec::new();
    for step in 0..steps {
        for e in w.step(BIO_DT) {
            if let Event::Molted(c) = e {
                molts.push((step, c));
            }
        }
    }
    molts
}

#[test]
fn molt_intervals_match_the_configured_distribution() {
    let p = LifecycleParams::default();
    let molts = run(SEED);
    let mut last: std::collections::HashMap<CellId, usize> = std::collections::HashMap::new();
    let mut intervals = Vec::new();
    for (step, cell) in &molts {
        if let Some(prev) = last.insert(*cell, *step) {
            intervals.push((step - prev) as f64 * BIO_DT);
        }
    }
    assert!(
        intervals.len() > 5 * ANIMALS,
        "only {} intervals in a year",
        intervals.len()
    );
    let n = intervals.len() as f64;
    let mean = intervals.iter().sum::<f64>() / n;
    let sigma = (intervals.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0)).sqrt();
    let (want_mean, want_sigma) = (p.interval_mean(), p.interval_sigma());
    assert!(
        (mean - want_mean).abs() <= 0.05 * want_mean,
        "interval mean {:.2} d, configured {:.2} d (±5%)",
        mean / DAY,
        want_mean / DAY
    );
    assert!(
        (sigma - want_sigma).abs() <= 0.15 * want_sigma,
        "interval sigma {:.2} d, configured {:.2} d (±15%)",
        sigma / DAY,
        want_sigma / DAY
    );
    assert_eq!(
        run(SEED),
        molts,
        "the same seed must give the same event log"
    );
}
