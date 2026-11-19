//! Phase 12 gate: benchmark rows are reproducible.
//!
//! Two sweep points, one per kind of farm, each run twice with the same seed,
//! must give byte-identical CSV rows with exactly the header's columns. The
//! sweep driver (`scene/scene/bench.py`) and the figures stand on that.
//!
//! ```text
//!   point A: one tray, oracle, molt rate 3     ─┐
//!   point B: the rack, camera, molt rate 1     ─┴─ bench::row(p) twice ─▶ identical, 15 columns
//! ```
//!
//! Physics-heavy, so it runs in release in the `clean-clone` CI job
//! (`-- --ignored`), which also runs the small sweep end to end.
//!
//! Red on `main` from row 12.1. Green once 12.2 lands.

use std::path::PathBuf;

use rookery::bench::{CSV_HEADER, Point, row};
use rookery_core::DetectorKind;

fn repo(rel: &str) -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../..")).join(rel)
}

#[test]
#[ignore = "physics-heavy: runs in release in the clean-clone CI job"]
fn the_same_point_and_seed_give_the_same_row() {
    let points = [
        Point {
            scene: repo("scene/generated/rookery.xml"),
            molt_rate: 3.0,
            detector: DetectorKind::Oracle,
            seed: 7,
            days: 3.0,
        },
        Point {
            scene: repo("scene/generated/rack3.xml"),
            molt_rate: 1.0,
            detector: DetectorKind::Camera,
            seed: 11,
            days: 3.0,
        },
    ];
    let columns = CSV_HEADER.split(',').count();
    assert_eq!(columns, 15, "{CSV_HEADER}");
    let handles: Vec<_> = points
        .into_iter()
        .map(|p| std::thread::spawn(move || (row(&p), row(&p), p)))
        .collect();
    for h in handles {
        let (a, b, p) = h.join().expect("the bench thread finished");
        let (a, b) = (a.expect("first run"), b.expect("second run"));
        assert_eq!(a, b, "{p:?} gave two different rows");
        assert_eq!(a.split(',').count(), columns, "{a}");
        assert!(!a.contains('\n'), "one line: {a:?}");
    }
}
