//! Phase 8 gate: the autonomy milestone. One tray runs unattended under
//! `lua/trees/autonomy.lua` for a sim-week, on several seeds. Every molt is
//! harvested inside its soft-shell window, quickly, into an exit slot.
//!
//! ```text
//!   for seed in SEEDS (one thread each):
//!     rookery::autonomy::run { tree = autonomy.lua, bio = 1 week + WINDOW_TAIL }
//!       ─▶ events: Molted(cell) … BoxLifted(cell) …   (never WindowMissed)
//!       ─▶ boxes_in_slots ⊇ every cell that molted in the week
//!   across seeds: ≥ MIN_MOLTS molts, mean(BoxLifted − Molted) < LATENCY_BOUND
//! ```
//!
//! The run goes on for `WINDOW_TAIL` past the week, so a molt late in the
//! week still has its whole window. Only molts inside the week count.
//!
//! Red on `main` from row 8.1. Green once 8.2–8.5 land.
//!
//! Phase 10 (row 10.1): `TRAYS=3` runs the same gate on the three-shelf
//! rack (`scene/generated/rack3.xml`). The rack's trays are one box deep
//! (12 cells, decided in #53: under a shelf a box can't be lifted over the
//! handles of the boxes in front of it), so it runs the four seeds for two
//! weeks each (about 20 molts). Unset or `TRAYS=1` is the Phase 8 gate
//! exactly as it was. Red at `TRAYS=3` until 10.3–10.5 land.

use std::path::PathBuf;

use rookery::autonomy::{AutonomyArgs, AutonomyReport, run};
use rookery_core::{CellId, Event};

const DAY: f64 = 86_400.0;
const WEEK: f64 = 7.0 * DAY;
/// A soft-shell window lasts about 7 h from the molt (1 h molting, 6 h soft).
const WINDOW_TAIL: f64 = 8.0 * 3600.0;
/// One seed's week had 3 molts among 16 animals (Phase 7), too few to prove
/// anything, so the gate pools several seeds.
const SEEDS: [u64; 4] = [7, 11, 23, 42];
const MIN_MOLTS: usize = 10;
/// Mean sim time from `Molted` to `BoxLifted`. A transfer takes about 30 s;
/// 10 min leaves room for a queue of molts but not for a stuck tree.
const LATENCY_BOUND: f64 = 600.0;

fn repo(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(rel)
}

/// The rack runs two weeks per seed and must reach this many molts in all
/// (the four seeds give 20 there, counted with rookery-core alone).
const RACK_WEEKS: f64 = 2.0;
const RACK_MIN_MOLTS: usize = 16;

/// What `TRAYS` asks for: the scene, the seeds, the minimum molt count and
/// the counted horizon in seconds.
fn config() -> (&'static str, Vec<u64>, usize, f64) {
    match std::env::var("TRAYS").as_deref() {
        Err(_) | Ok("1") => (
            "scene/generated/rookery.xml",
            SEEDS.to_vec(),
            MIN_MOLTS,
            WEEK,
        ),
        Ok("3") => (
            "scene/generated/rack3.xml",
            SEEDS.to_vec(),
            RACK_MIN_MOLTS,
            RACK_WEEKS * WEEK,
        ),
        Ok(other) => panic!("TRAYS={other}: the gate knows 1 and 3"),
    }
}

fn week(scene: &str, seed: u64, horizon: f64) -> Result<AutonomyReport, String> {
    run(&AutonomyArgs {
        scene: repo(scene),
        tree: repo("lua/trees/autonomy.lua"),
        seed,
        bio_seconds: horizon + WINDOW_TAIL,
        log: None,
    })
}

#[test]
fn every_molt_in_a_week_is_harvested_inside_its_window() {
    let (scene, seeds, min_molts, horizon) = config();
    let handles: Vec<_> = seeds
        .iter()
        .map(|&seed| std::thread::spawn(move || (seed, week(scene, seed, horizon))))
        .collect();
    let mut molts = 0usize;
    let mut latencies = Vec::new();
    let mut physics = 0.0;
    let mut problems = Vec::new();
    for h in handles {
        let (seed, report) = h.join().expect("the run thread finished");
        let r = match report {
            Ok(r) => r,
            Err(e) => {
                problems.push(format!("seed {seed}: {e}"));
                continue;
            }
        };
        physics += r.physics_seconds;
        if !r.errors.is_empty() {
            problems.push(format!("seed {seed}: errors {:?}", r.errors));
        }
        for &(t, e) in &r.events {
            match e {
                Event::WindowMissed(c) => {
                    problems.push(format!("seed {seed}: window missed at {c:?}, t = {t:.0} s"))
                }
                Event::Molted(c) if t <= horizon => {
                    molts += 1;
                    match lifted_after(&r.events, c, t) {
                        Some(t_lift) => latencies.push(t_lift - t),
                        None => problems.push(format!(
                            "seed {seed}: {c:?} molted at t = {t:.0} s and was never lifted"
                        )),
                    }
                    if !r.boxes_in_slots.contains(&c) {
                        problems.push(format!(
                            "seed {seed}: {c:?} molted but its box is not in an exit slot"
                        ));
                    }
                }
                _ => {}
            }
        }
    }
    assert!(problems.is_empty(), "{}", problems.join("\n"));
    assert!(
        molts >= min_molts,
        "only {molts} molts across {} seeds; need {min_molts}",
        seeds.len()
    );
    let mean = latencies.iter().sum::<f64>() / latencies.len() as f64;
    assert!(
        mean < LATENCY_BOUND,
        "mean Molted → BoxLifted {mean:.0} s, bound {LATENCY_BOUND:.0} s ({molts} molts, {physics:.0} s of physics)"
    );
}

/// When the box of `cell` was first lifted at or after `t`.
fn lifted_after(events: &[(f64, Event)], cell: CellId, t: f64) -> Option<f64> {
    events
        .iter()
        .find(|&&(te, e)| te >= t && e == Event::BoxLifted(cell))
        .map(|&(te, _)| te)
}
