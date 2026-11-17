//! Per-cell biology scalars (row 7.6): dissolved oxygen, waste and feed.
//!
//! The mesh boxes are permeable, so each tray is one shallow pool and there
//! is no fluid simulation (v5, "Water"). Each cell instead carries three
//! numbers that follow first-order dynamics toward an equilibrium:
//!
//! ```text
//!   o2     d/dt = k_air · (o2_sat − o2) − uptake      → o2_sat − uptake / k_air
//!   waste  d/dt = production − k_flush · waste        → production / k_flush
//!   feed   d/dt = − k_feed · feed   (+ add_feed jumps) → 0
//! ```
//!
//! Each equation is solved exactly over a step, `x' = x_eq + (x − x_eq)·e^(−k·dt)`,
//! rather than with an Euler step. Why: [`crate::World`] may jump days at
//! once in fast-forward (7.2), and an Euler step that long overshoots and
//! oscillates. The exact solution never passes its equilibrium, at any `dt`.
//! The result is then clamped to hard bounds, and anything non-finite becomes
//! the lower bound, so no input can produce a NaN.
//!
//! The rates are placeholders with the right orders of magnitude for a
//! flow-through soft-shell system, not calibrated values. Feeding, and with
//! it the feed → waste coupling, comes with the deferred Phase 9.

/// Rates and bounds for [`CellBio`]. Units: mg/L for oxygen and waste,
/// grams for feed, and per-second rates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BioParams {
    /// Oxygen saturation, also the upper bound on `o2`. About 8 mg/L at 25 °C.
    pub o2_sat: f64,
    /// Re-aeration rate toward `o2_sat` from water exchange, per second.
    pub k_air: f64,
    /// Oxygen taken up by one animal, mg/L per second.
    pub o2_uptake: f64,
    /// Waste (ammonia) produced by one animal, mg/L per second.
    pub waste_production: f64,
    /// Rate at which water exchange flushes waste, per second.
    pub k_flush: f64,
    /// Upper bound on `waste`, mg/L.
    pub waste_max: f64,
    /// Rate at which uneaten feed disappears (eaten or flushed), per second.
    pub k_feed: f64,
    /// Upper bound on `feed`, grams.
    pub feed_max: f64,
}

impl Default for BioParams {
    fn default() -> Self {
        let hour = crate::HOUR;
        BioParams {
            o2_sat: 8.0,
            k_air: 1.0 / hour,
            o2_uptake: 0.5 / hour,
            waste_production: 0.1 / hour,
            k_flush: 1.0 / (4.0 * hour),
            waste_max: 50.0,
            k_feed: 1.0 / (12.0 * hour),
            feed_max: 5.0,
        }
    }
}

impl BioParams {
    /// Problems with these parameters, empty when they are usable: every
    /// value must be finite and non-negative, and the bounds positive.
    pub fn problems(&self) -> Vec<String> {
        let fields = [
            ("o2_sat", self.o2_sat),
            ("k_air", self.k_air),
            ("o2_uptake", self.o2_uptake),
            ("waste_production", self.waste_production),
            ("k_flush", self.k_flush),
            ("waste_max", self.waste_max),
            ("k_feed", self.k_feed),
            ("feed_max", self.feed_max),
        ];
        let mut p: Vec<String> = fields
            .iter()
            .filter(|(_, v)| !(v.is_finite() && *v >= 0.0))
            .map(|(n, v)| format!("{n} must be finite and >= 0, got {v}"))
            .collect();
        for (n, v) in [
            ("o2_sat", self.o2_sat),
            ("waste_max", self.waste_max),
            ("feed_max", self.feed_max),
        ] {
            if v == 0.0 {
                p.push(format!("{n} must be > 0"));
            }
        }
        p
    }
}

/// One cell's water: oxygen and waste in mg/L, uneaten feed in grams.
///
/// The fields are private so every change goes through [`CellBio::step`] or
/// [`CellBio::add_feed`], which keep them inside their bounds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CellBio {
    o2: f64,
    waste: f64,
    feed: f64,
}

impl CellBio {
    /// Fresh water: saturated with oxygen, no waste, no feed.
    pub fn new(p: &BioParams) -> CellBio {
        CellBio {
            o2: bound(p.o2_sat, 0.0, p.o2_sat),
            waste: 0.0,
            feed: 0.0,
        }
    }

    pub fn o2(&self) -> f64 {
        self.o2
    }

    pub fn waste(&self) -> f64 {
        self.waste
    }

    pub fn feed(&self) -> f64 {
        self.feed
    }

    /// Advance by `dt` seconds with `animals` crawfish in the cell (0 or 1
    /// today). A `dt` that is not a positive finite number changes nothing.
    pub fn step(&mut self, dt: f64, animals: u32, p: &BioParams) {
        if !(dt.is_finite() && dt > 0.0) {
            return;
        }
        let n = f64::from(animals);
        self.o2 = bound(
            relax(self.o2, p.k_air * p.o2_sat - n * p.o2_uptake, p.k_air, dt),
            0.0,
            p.o2_sat,
        );
        self.waste = bound(
            relax(self.waste, n * p.waste_production, p.k_flush, dt),
            0.0,
            p.waste_max,
        );
        self.feed = bound(relax(self.feed, 0.0, p.k_feed, dt), 0.0, p.feed_max);
    }

    /// Drop `grams` of feed into the cell (Phase 9's feeder). Bounded like
    /// every other change; a negative or non-finite amount adds nothing.
    pub fn add_feed(&mut self, grams: f64, p: &BioParams) {
        if grams.is_finite() && grams > 0.0 {
            self.feed = bound(self.feed + grams, 0.0, p.feed_max);
        }
    }
}

/// The exact solution of `dx/dt = source − k·x` after `dt`: it decays toward
/// `source / k` and never passes it. With `k = 0` it is a straight line.
fn relax(x: f64, source: f64, k: f64, dt: f64) -> f64 {
    if k > 0.0 {
        let eq = source / k;
        eq + (x - eq) * (-k * dt).exp()
    } else {
        x + source * dt
    }
}

/// `v` clamped to `[lo, hi]`, with NaN mapped to `lo`. Rust's `clamp` keeps
/// NaN, which is exactly what this module promises never to return.
fn bound(v: f64, lo: f64, hi: f64) -> f64 {
    if v.is_nan() { lo } else { v.clamp(lo, hi) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn relax_reaches_the_equilibrium_without_passing_it() {
        // source 2, k 0.5: equilibrium 4.
        let mut x = 0.0;
        for _ in 0..100 {
            let next = relax(x, 2.0, 0.5, 1.0);
            assert!(next >= x && next <= 4.0, "passed the equilibrium: {next}");
            x = next;
        }
        assert!((x - 4.0).abs() < 1e-9);
        assert_eq!(
            relax(1.0, 2.0, 0.5, 1e12),
            4.0,
            "one giant step lands on it"
        );
        assert_eq!(relax(1.0, 2.0, 0.0, 3.0), 7.0, "k = 0 is a straight line");
    }

    #[test]
    fn bound_maps_nan_to_the_lower_bound() {
        assert_eq!(bound(f64::NAN, 0.0, 1.0), 0.0);
        assert_eq!(bound(f64::INFINITY, 0.0, 1.0), 1.0);
        assert_eq!(bound(-3.0, 0.0, 1.0), 0.0);
    }
}
