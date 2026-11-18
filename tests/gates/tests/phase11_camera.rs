//! Phase 11 gate: the camera detector.
//!
//! **A (fidelity).** On the rack, with no harvesting, every cell is seen on
//! the camera schedule while shells come and go. On at least 500 held-out
//! samples the camera's "two bodies on the box floor" must match the truth
//! "a shed shell is in that box" with precision and recall of at least 0.98.
//! The truth is what a camera can see: it can't tell soft from hard, so the
//! oracle's soft-shell state is not the label (#58).
//!
//! **B (it runs the farm).** The Phase 8 gate on the rack with
//! `DetectorKind::Camera` instead of the oracle, and nothing else changed:
//! every molt harvested, none missed.
//!
//! ```text
//!   A: camera::fidelity(rack3, seed) × HELD_OUT ─▶ samples ─▶ precision, recall ≥ 0.98
//!   B: autonomy::run_with(rack3, autonomy.lua, seed, Camera) × SEEDS ─▶ zero WindowMissed
//! ```
//!
//! Both are physics-heavy, so they run in release in the `gates-rack` CI
//! job (`-- --ignored`), not in the main gates job.
//!
//! Red on `main` from row 11.1. Green once 11.2–11.5 land.

use std::path::PathBuf;

use rookery::autonomy::{AutonomyArgs, run_with};
use rookery::camera::fidelity;
use rookery_core::{DetectorKind, Event};

const DAY: f64 = 86_400.0;
/// Seeds never used while building the detector (held out).
const HELD_OUT: [u64; 3] = [101, 202, 303];
const FIDELITY_DAYS: f64 = 21.0;
const MIN_SAMPLES: usize = 500;
/// Enough shells seen to make the recall mean something.
const MIN_POSITIVES: usize = 40;
const MIN_PRECISION: f64 = 0.98;
const MIN_RECALL: f64 = 0.98;
/// Part B: the Phase 8 rack run (row 10.1's seeds and horizon).
const SEEDS: [u64; 4] = [7, 11, 23, 42];
const RACK_DAYS: f64 = 14.0;
const WINDOW_TAIL: f64 = 8.0 * 3600.0;

fn repo(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(rel)
}

#[test]
#[ignore = "physics-heavy: runs in release in the gates-rack CI job"]
fn the_camera_sees_every_shed_shell_and_nothing_else() {
    let scene = repo("scene/generated/rack3.xml");
    let handles: Vec<_> = HELD_OUT
        .iter()
        .map(|&seed| {
            let scene = scene.clone();
            std::thread::spawn(move || fidelity(&scene, seed, FIDELITY_DAYS * DAY))
        })
        .collect();
    let (mut tp, mut fp, mut fn_, mut n) = (0usize, 0usize, 0usize, 0usize);
    for h in handles {
        for s in h
            .join()
            .expect("the sampling thread finished")
            .expect("the run")
        {
            n += 1;
            match (s.bodies_seen >= 2, s.shell_in_box) {
                (true, true) => tp += 1,
                (true, false) => fp += 1,
                (false, true) => fn_ += 1,
                (false, false) => {}
            }
        }
    }
    assert!(n >= MIN_SAMPLES, "only {n} samples; need {MIN_SAMPLES}");
    assert!(
        tp + fn_ >= MIN_POSITIVES,
        "only {} shells seen; need {MIN_POSITIVES}",
        tp + fn_
    );
    let precision = tp as f64 / (tp + fp).max(1) as f64;
    let recall = tp as f64 / (tp + fn_).max(1) as f64;
    assert!(
        precision >= MIN_PRECISION && recall >= MIN_RECALL,
        "precision {precision:.3}, recall {recall:.3} over {n} samples (tp {tp}, fp {fp}, fn {fn_})"
    );
}

#[test]
#[ignore = "physics-heavy: runs in release in the gates-rack CI job"]
fn the_rack_runs_itself_on_the_camera() {
    let handles: Vec<_> = SEEDS
        .iter()
        .map(|&seed| {
            std::thread::spawn(move || {
                let args = AutonomyArgs {
                    scene: repo("scene/generated/rack3.xml"),
                    tree: repo("lua/trees/autonomy.lua"),
                    seed,
                    bio_seconds: RACK_DAYS * DAY + WINDOW_TAIL,
                    log: None,
                };
                (seed, run_with(&args, DetectorKind::Camera))
            })
        })
        .collect();
    let mut problems = Vec::new();
    let mut molts = 0;
    for h in handles {
        let (seed, report) = h.join().expect("the run thread finished");
        let r = match report {
            Ok(r) => r,
            Err(e) => {
                problems.push(format!("seed {seed}: {e}"));
                continue;
            }
        };
        if !r.errors.is_empty() {
            problems.push(format!("seed {seed}: errors {:?}", r.errors));
        }
        for &(t, e) in &r.events {
            match e {
                Event::WindowMissed(c) => {
                    problems.push(format!("seed {seed}: window missed at {c:?}, t = {t:.0} s"))
                }
                Event::Molted(c) if t <= RACK_DAYS * DAY => {
                    molts += 1;
                    if !r
                        .events
                        .iter()
                        .any(|&(tl, el)| tl >= t && el == Event::BoxLifted(c))
                    {
                        problems.push(format!(
                            "seed {seed}: {c:?} molted at t = {t:.0} s and was never lifted"
                        ));
                    }
                }
                _ => {}
            }
        }
    }
    assert!(problems.is_empty(), "{}", problems.join("\n"));
    assert!(molts >= 16, "only {molts} molts");
}
