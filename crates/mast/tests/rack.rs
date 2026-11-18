//! Row 10.3: shelf windows from the scene, and routes and transfers that
//! keep the arm inside them on the three-shelf rack.

use mast::kinematics::shelf_windows;
use mast::limits::{ARM_RETRACTED, MAST_Z};
use mast::route::{Pose, RouteError, route};
use mast::{
    Axis, CmdStatus, Command, GantryObs, GantryState, JointTargets, SceneMeta, ShelfWindows,
    Transfer, WindowError, exit_slot_pose, home,
};

/// A copy of `scene/generated/rack3.json`, so the crate tests outside the
/// repo (`scripts/standalone-mast.sh`); the test below catches drift.
const RACK: &str = include_str!("fixtures/rack3.json");
const ONE: &str = include_str!("fixtures/rookery.json");
const DT: f64 = 0.02;

fn rack() -> SceneMeta {
    SceneMeta::from_json(RACK).unwrap()
}

/// Tick to completion with perfect tracking; record every target vector.
fn run(cmd: &mut dyn Command, st: &mut GantryState, max: usize) -> (CmdStatus, Vec<JointTargets>) {
    let mut trace = Vec::new();
    for _ in 0..max {
        let obs = GantryObs {
            q: st.targets.0,
            ee_contact: 0.0,
        };
        let s = cmd.tick(st, &obs, DT);
        trace.push(st.targets);
        if s != CmdStatus::Running {
            return (s, trace);
        }
    }
    (CmdStatus::Running, trace)
}

/// Every pose with the arm out lies inside a window that fits the arm and hook.
fn all_admitted(trace: &[JointTargets], w: &ShelfWindows) -> Result<(), String> {
    for q in trace {
        let [_, z, x, h] = q.0;
        if x > ARM_RETRACTED + 1e-9 && !w.admits(z, h) {
            return Err(format!(
                "arm out to {x:.3} at mast {z:.3} (lift {h:.3}), outside {w:?}"
            ));
        }
    }
    Ok(())
}

#[test]
fn the_rack_fixture_matches_the_generated_scene() {
    let scene = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scene/generated/rack3.json"
    );
    if let Ok(text) = std::fs::read_to_string(scene) {
        assert_eq!(
            SceneMeta::from_json(&text).expect("scene json parses"),
            rack(),
            "tests/fixtures/rack3.json is stale: copy scene/generated/rack3.json over it"
        );
    }
}

#[test]
fn the_rack_has_a_window_per_shelf_and_one_tray_keeps_the_phase_3_window() {
    let m = rack();
    let w = ShelfWindows::from_meta(&m).unwrap();
    assert_eq!(w.0.len(), 3);
    let bases = [0.0, 0.285, 0.57];
    for (i, r) in w.0.iter().enumerate() {
        assert!((r.lo - (bases[i] + 0.085)).abs() < 1e-9, "shelf {i}: {r:?}");
        assert!(r.hi > r.lo + 0.07);
    }
    assert_eq!(w.0[2].hi, MAST_Z.hi);
    let one = SceneMeta::from_json(ONE).unwrap();
    assert_eq!(ShelfWindows::from_meta(&one).unwrap(), shelf_windows(&one));
}

#[test]
fn an_impossible_rack_is_refused_with_a_named_error() {
    let squash = RACK.replace("0.285", "0.2").replace("0.57", "0.4");
    let e = ShelfWindows::from_meta(&SceneMeta::from_json(&squash).unwrap()).unwrap_err();
    assert!(matches!(e, WindowError::TooThin { shelf: 0, .. }), "{e:?}");
    let tall = RACK.replace("0.57", "0.9");
    let e = ShelfWindows::from_meta(&SceneMeta::from_json(&tall).unwrap()).unwrap_err();
    assert!(matches!(e, WindowError::OutsideMast { .. }), "{e:?}");
    assert!(e.to_string().contains("outside the mast"));
}

#[test]
fn routes_reach_every_cell_on_every_shelf_inside_the_windows() {
    let m = rack();
    let w = ShelfWindows::from_meta(&m).unwrap();
    for t in 0..3 {
        for c in 0..4 {
            let mut st = GantryState {
                targets: home(),
                windows: w.clone(),
            };
            let cmds = route(home(), Pose::CellOn(t, 0, c), &m).unwrap();
            let mut trace = Vec::new();
            for mut cmd in cmds {
                let (s, tr) = run(&mut cmd, &mut st, 5_000);
                assert_eq!(s, CmdStatus::Done, "tray {t} cell {c}");
                trace.extend(tr);
            }
            all_admitted(&trace, &w).unwrap();
            let want = mast::kinematics::cell_pose_on(t, 0, c, &m).unwrap();
            assert!(
                st.targets
                    .0
                    .iter()
                    .zip(want.0)
                    .all(|(a, b)| (a - b).abs() < 1e-9),
                "{:?}",
                st.targets
            );
        }
    }
    assert_eq!(
        route(home(), Pose::CellOn(3, 0, 0), &m).unwrap_err(),
        RouteError::NoSuchTray { tray: 3, trays: 3 }
    );
}

#[test]
fn a_transfer_from_every_shelf_into_a_slot_stays_inside_the_windows() {
    let m = rack();
    let w = ShelfWindows::from_meta(&m).unwrap();
    for t in 0..3 {
        for (c, slot) in [(0, 4), (3, 5), (1, 0)] {
            let mut st = GantryState {
                targets: home(),
                windows: w.clone(),
            };
            let mut x = Transfer::build_on(home(), t, 0, c, Some(slot), &m).unwrap();
            let (s, trace) = run(&mut x, &mut st, 30_000);
            assert_eq!(s, CmdStatus::Done, "tray {t} cell {c} slot {slot}");
            all_admitted(&trace, &w).unwrap();
            let at = exit_slot_pose(slot, &m).unwrap();
            assert!(
                trace
                    .iter()
                    .any(|q| (0..3).all(|k| (q.0[k] - at.0[k]).abs() < 1e-6)),
                "tray {t}: never set down at slot {slot}"
            );
            // The lifted box never leaves its shelf's window before the arm is in.
            assert_eq!(st.targets, home());
        }
    }
    let one = SceneMeta::from_json(ONE).unwrap();
    assert!(matches!(
        Transfer::build_on(home(), 1, 0, 0, None, &one),
        Err(RouteError::NoSuchTray { tray: 1, trays: 1 })
    ));
    let _ = Axis::ALL;
}
