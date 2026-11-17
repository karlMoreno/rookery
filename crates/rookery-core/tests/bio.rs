//! Row 7.6: per-cell biology stays in bounds, with no NaN, at any `bio_dt`.

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use rookery_core::{BioParams, CellBio, DAY, HOUR};

const YEAR: f64 = 365.0 * DAY;

fn in_bounds(b: &CellBio, p: &BioParams) -> bool {
    (0.0..=p.o2_sat).contains(&b.o2())
        && (0.0..=p.waste_max).contains(&b.waste())
        && (0.0..=p.feed_max).contains(&b.feed())
}

/// Step one cell to `horizon` in steps of `dt`, with an animal coming and
/// going and feed dropped now and then, checking the bounds after every step.
fn run(dt: f64, horizon: f64, p: &BioParams, rng: &mut ChaCha8Rng) -> CellBio {
    let mut b = CellBio::new(p);
    let mut t = 0.0;
    while t < horizon {
        let animals = u32::from(rng.random_bool(0.9));
        if rng.random_bool(0.01) {
            b.add_feed(rng.random_range(0.0..2.0), p);
        }
        b.step(dt, animals, p);
        assert!(in_bounds(&b, p), "dt {dt} at t {t}: {b:?}");
        t += dt;
    }
    b
}

#[test]
fn defaults_are_valid_and_fresh_water_is_saturated() {
    let p = BioParams::default();
    assert!(p.problems().is_empty(), "{:?}", p.problems());
    let b = CellBio::new(&p);
    assert_eq!((b.o2(), b.waste(), b.feed()), (p.o2_sat, 0.0, 0.0));
}

#[test]
fn an_animal_draws_oxygen_down_and_waste_up_to_their_equilibria() {
    let p = BioParams::default();
    let mut b = CellBio::new(&p);
    // Three days: 18 time constants of the 4 h flush, 72 of re-aeration.
    for _ in 0..(3.0 * DAY / 60.0) as usize {
        b.step(60.0, 1, &p);
    }
    let o2_eq = p.o2_sat - p.o2_uptake / p.k_air;
    let waste_eq = p.waste_production / p.k_flush;
    assert!((b.o2() - o2_eq).abs() < 1e-6, "{b:?}");
    assert!((b.waste() - waste_eq).abs() < 1e-6, "{b:?}");
    // The animal leaves (transferred out): the water recovers.
    b.step(3.0 * DAY, 0, &p);
    assert!(
        (b.o2() - p.o2_sat).abs() < 1e-6 && b.waste() < 1e-6,
        "{b:?}"
    );
}

#[test]
fn feed_disappears_over_hours_and_is_capped() {
    let p = BioParams::default();
    let mut b = CellBio::new(&p);
    b.add_feed(1.0, &p);
    b.step(12.0 * HOUR, 1, &p);
    assert!(
        (b.feed() - (-1.0f64).exp()).abs() < 1e-9,
        "one time constant: {b:?}"
    );
    b.add_feed(100.0, &p);
    assert_eq!(b.feed(), p.feed_max);
    for bad in [-1.0, f64::NAN, f64::INFINITY] {
        b.add_feed(bad, &p);
        assert_eq!(b.feed(), p.feed_max);
    }
}

#[test]
fn one_long_step_equals_many_short_ones() {
    // The exact solution: fast-forward (7.2) may jump a day at once and must
    // land where per-minute stepping does.
    let p = BioParams::default();
    let mut fine = CellBio::new(&p);
    fine.add_feed(2.0, &p);
    let mut coarse = fine;
    for _ in 0..(DAY / 60.0) as usize {
        fine.step(60.0, 1, &p);
    }
    coarse.step(DAY, 1, &p);
    assert!((fine.o2() - coarse.o2()).abs() < 1e-9);
    assert!((fine.waste() - coarse.waste()).abs() < 1e-9);
    assert!((fine.feed() - coarse.feed()).abs() < 1e-9);
}

#[test]
fn a_sim_year_stays_in_bounds_at_every_bio_dt() {
    let p = BioParams::default();
    let mut rng = ChaCha8Rng::seed_from_u64(76);
    // Short steps run a shorter horizon so the test stays fast in debug
    // builds: 20 ms (the behaviour-tree tick) for 6 h and 1 s for 2 days,
    // which both pass the slowest time constant (4 h) and so reach every
    // equilibrium. Because each step is exact, a longer run at the same dt
    // only repeats them (see one_long_step_equals_many_short_ones). From
    // 60 s up, each dt runs the whole year.
    run(0.02, 6.0 * HOUR, &p, &mut rng);
    run(1.0, 2.0 * DAY, &p, &mut rng);
    for dt in [60.0, HOUR, DAY, 7.0 * DAY, YEAR, 10.0 * YEAR] {
        let b = run(dt, YEAR, &p, &mut rng);
        assert!(b.o2().is_finite() && b.waste().is_finite() && b.feed().is_finite());
    }
    // Fast-forward's uneven jumps: log-uniform from 1 ms to 10 days.
    let mut b = CellBio::new(&p);
    let mut t = 0.0;
    while t < YEAR {
        let dt = 10f64.powf(rng.random_range(-3.0..6.0));
        b.step(dt, u32::from(rng.random_bool(0.5)), &p);
        assert!(in_bounds(&b, &p), "dt {dt} at t {t}: {b:?}");
        t += dt;
    }
}

#[test]
fn degenerate_steps_change_nothing() {
    let p = BioParams::default();
    let mut b = CellBio::new(&p);
    b.add_feed(1.0, &p);
    b.step(HOUR, 1, &p);
    let before = b;
    for dt in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        b.step(dt, 1, &p);
        assert_eq!(b, before, "dt {dt}");
    }
    b.step(f64::MIN_POSITIVE, 1, &p);
    assert!(in_bounds(&b, &p));
    b.step(f64::MAX, 1, &p);
    assert!(in_bounds(&b, &p), "{b:?}");
}

#[test]
fn extreme_parameters_hit_the_bounds_instead_of_breaking() {
    let base = BioParams::default();
    let cases = [
        // So much uptake the equilibrium is negative: o2 stops at 0.
        BioParams {
            o2_uptake: 1.0,
            ..base
        },
        // No flushing at all: waste climbs in a straight line to its cap.
        BioParams {
            k_flush: 0.0,
            waste_production: 1.0,
            ..base
        },
        // No re-aeration: o2 falls in a straight line to 0.
        BioParams { k_air: 0.0, ..base },
        // Huge rates: e^(-k·dt) underflows to 0.
        BioParams {
            k_air: 1e300,
            k_flush: 1e300,
            k_feed: 1e300,
            ..base
        },
    ];
    let mut rng = ChaCha8Rng::seed_from_u64(7);
    for p in cases {
        assert!(p.problems().is_empty(), "{p:?}");
        let b = run(HOUR, YEAR, &p, &mut rng);
        assert!(in_bounds(&b, &p), "{p:?}: {b:?}");
    }
    // Even unusable parameters never produce a NaN.
    let bad = BioParams {
        k_air: f64::INFINITY,
        o2_uptake: f64::NAN,
        ..base
    };
    assert_eq!(bad.problems().len(), 2, "{:?}", bad.problems());
    let mut b = CellBio::new(&bad);
    b.step(HOUR, 1, &bad);
    assert!(
        !b.o2().is_nan() && !b.waste().is_nan() && !b.feed().is_nan(),
        "{b:?}"
    );
}

#[test]
fn bad_parameters_are_named() {
    let p = BioParams {
        o2_sat: 0.0,
        k_feed: -1.0,
        ..BioParams::default()
    };
    let msgs = p.problems().join("; ");
    assert!(
        msgs.contains("o2_sat must be > 0") && msgs.contains("k_feed must be finite and >= 0"),
        "{msgs}"
    );
}
