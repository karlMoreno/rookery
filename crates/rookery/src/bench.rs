//! Benchmark rows (Phase 12): one sweep point run unattended and boiled down
//! to one CSV row with a fixed column order.
//!
//! Row 12.1 declared this surface; 12.2 (molt rate, metrics) filled it in. `scene/scene/bench.py` (12.3) builds each point's scene in a
//! temp directory and asks `rookery` for the row; the Phase 12 gate checks
//! that the same point and seed give the same row, byte for byte.
//!
//! ```text
//!   Point { scene, molt_rate, detector, seed, days } ─▶ autonomy::run_with ─▶ metrics ─▶ CSV row
//! ```

use std::path::PathBuf;

use mast::SceneMeta;
use rookery_core::{DetectorKind, Event, LifecycleParams};

use crate::autonomy::{AutonomyArgs, run_params};

/// A molt late in the horizon still gets its whole window (as in the gates).
const WINDOW_TAIL: f64 = 8.0 * 3600.0;
const DAY: f64 = 86_400.0;

/// What a run achieved (row 12.2), from its events.
#[derive(Debug, Clone, PartialEq)]
pub struct Metrics {
    /// Molts inside the counted horizon.
    pub molts: usize,
    pub missed_windows: usize,
    /// Seconds from `Molted` to that cell's `BoxLifted`, mean and 95th
    /// percentile (nearest rank), over the harvested molts.
    pub latency_mean_s: f64,
    pub latency_p95_s: f64,
    /// Times a molted animal's box was lifted, per molt in the horizon: 1
    /// when every molt is handled once (a later molt's lift doesn't count).
    pub handlings_per_animal: f64,
}

/// The metrics of `events` for molts at or before `horizon` seconds.
pub fn metrics(events: &[(f64, Event)], horizon: f64) -> Metrics {
    let mut lat = Vec::new();
    let (mut molts, mut missed, mut lifted) = (0, 0, 0);
    for (k, &(t, e)) in events.iter().enumerate() {
        let Event::Molted(c) = e else { continue };
        if t > horizon {
            continue;
        }
        molts += 1;
        // This molt's lifts and miss: from it up to the cell's next molt.
        let rest = &events[k + 1..];
        let until = rest
            .iter()
            .position(|&(_, el)| el == Event::Molted(c))
            .unwrap_or(rest.len());
        let mine = &rest[..until];
        let lifts: Vec<f64> = mine
            .iter()
            .filter(|&&(_, el)| el == Event::BoxLifted(c))
            .map(|&(tl, _)| tl)
            .collect();
        lifted += lifts.len();
        if let Some(&tl) = lifts.first() {
            lat.push(tl - t);
        }
        missed += mine
            .iter()
            .filter(|&&(_, el)| el == Event::WindowMissed(c))
            .count();
    }
    lat.sort_by(f64::total_cmp);
    let mean = if lat.is_empty() {
        0.0
    } else {
        lat.iter().sum::<f64>() / lat.len() as f64
    };
    let p95 = lat
        .get(((lat.len() as f64 * 0.95).ceil() as usize).saturating_sub(1))
        .copied()
        .unwrap_or(0.0);
    Metrics {
        molts,
        missed_windows: missed,
        latency_mean_s: mean,
        latency_p95_s: p95,
        handlings_per_animal: if molts == 0 {
            0.0
        } else {
            lifted as f64 / molts as f64
        },
    }
}

/// Camera precision and recall against the oracle from a confusion count
/// [both, camera only, oracle only, neither]; 1 and 1 when the camera never
/// spoke and the oracle never saw anything (the oracle detector).
pub fn precision_recall(c: [u64; 4]) -> (f64, f64) {
    let [tp, fp, fn_, _] = c.map(|v| v as f64);
    let p = if tp + fp == 0.0 { 1.0 } else { tp / (tp + fp) };
    let r = if tp + fn_ == 0.0 {
        1.0
    } else {
        tp / (tp + fn_)
    };
    (p, r)
}

/// The columns, in order. Changing them changes every results file.
pub const CSV_HEADER: &str = "scene,trays,cells,molt_rate,detector,seed,days,molts,missed_windows,\
latency_mean_s,latency_p95_s,handlings_per_animal,precision,recall,physics_s";

/// One sweep point.
#[derive(Debug, Clone, PartialEq)]
pub struct Point {
    /// The scene (its metadata is the `.json` beside it).
    pub scene: PathBuf,
    /// Multiplier on the molt rate: 1 is the default cycle, 3 is about an
    /// eyestalk-ablated one (every stage a third as long).
    pub molt_rate: f64,
    pub detector: DetectorKind,
    pub seed: u64,
    /// Biological days to run (the counted horizon; an 8 h tail follows).
    pub days: f64,
}

/// Run `point` with `lua/trees/autonomy.lua` and return its CSV row, in
/// [`CSV_HEADER`]'s order.
///
/// # Errors
/// A scene or tree that cannot be loaded.
pub fn row(point: &Point) -> Result<String, String> {
    let meta = SceneMeta::load(point.scene.with_extension("json")).map_err(|e| e.to_string())?;
    let repo = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."));
    let horizon = point.days * DAY;
    let args = AutonomyArgs {
        scene: point.scene.clone(),
        tree: repo.join("lua/trees/autonomy.lua"),
        seed: point.seed,
        bio_seconds: horizon + WINDOW_TAIL,
        log: None,
    };
    let r = run_params(
        &args,
        point.detector,
        LifecycleParams::default().scaled(point.molt_rate),
    )?;
    let m = metrics(&r.events, horizon);
    let (precision, recall) = precision_recall(r.confusion);
    let scene = point
        .scene
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("?");
    let detector = match point.detector {
        DetectorKind::Oracle => "oracle",
        DetectorKind::Camera => "camera",
    };
    Ok(format!(
        "{scene},{},{},{:.3},{detector},{},{:.3},{},{},{:.3},{:.3},{:.6},{:.6},{:.6},{:.3}",
        meta.trays,
        crate::cells::count(&meta),
        point.molt_rate,
        point.seed,
        point.days,
        m.molts,
        m.missed_windows,
        m.latency_mean_s,
        m.latency_p95_s,
        m.handlings_per_animal,
        precision,
        recall,
        r.physics_seconds,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rookery_core::CellId;

    fn cell(c: u8) -> CellId {
        CellId { tray: 0, r: 0, c }
    }

    #[test]
    fn metrics_count_molts_misses_and_latencies() {
        let ev = [
            (10.0, Event::Molted(cell(0))),
            (40.0, Event::BoxLifted(cell(0))),
            (50.0, Event::Molted(cell(1))),
            (60.0, Event::WindowMissed(cell(1))),
            (70.0, Event::Molted(cell(2))),
            (170.0, Event::BoxLifted(cell(2))),
            (900.0, Event::Molted(cell(3))),    // after the horizon
            (950.0, Event::BoxLifted(cell(3))), // its lift doesn't count
        ];
        let m = metrics(&ev, 100.0);
        assert_eq!((m.molts, m.missed_windows), (3, 1));
        assert_eq!((m.latency_mean_s, m.latency_p95_s), (65.0, 100.0));
        assert!((m.handlings_per_animal - 2.0 / 3.0).abs() < 1e-12);
        assert_eq!(precision_recall([0; 4]), (1.0, 1.0));
        assert_eq!(precision_recall([8, 2, 0, 90]), (0.8, 1.0));
    }

    #[test]
    fn the_header_has_one_column_per_row_field() {
        assert_eq!(CSV_HEADER.split(',').count(), 15);
    }
}
