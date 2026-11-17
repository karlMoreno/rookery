//! Row 6.2: the Transfer macro-command, with perfect tracking.

use mast::kinematics::shelf_windows;
use mast::limits::ARM_RETRACTED;
use mast::route::PRE;
use mast::{
    Axis, CmdStatus, Command, GantryObs, GantryState, JointTargets, SceneMeta, Transfer, cell_pose,
    exit_pose, home,
};

const META: &str = include_str!("fixtures/rookery.json");
const DT: f64 = 0.02;

fn meta() -> SceneMeta {
    SceneMeta::from_json(META).unwrap()
}

fn pre(r: usize, c: usize) -> JointTargets {
    let mut p = cell_pose(r, c, &meta()).unwrap();
    p.0[Axis::ArmX.index()] -= PRE;
    p
}

/// Tick to completion with q = targets; record every target vector.
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

#[test]
fn a_transfer_passes_through_the_exit_and_ends_home() {
    let m = meta();
    let start = pre(2, 1);
    let mut st = GantryState {
        targets: start,
        windows: shelf_windows(&m),
    };
    let mut t = Transfer::build(start, 2, 1, &m).unwrap();
    let (s, trace) = run(&mut t, &mut st, 100_000);
    assert_eq!(s, CmdStatus::Done);
    assert_eq!(st.targets, home());
    let exit = exit_pose(&m);
    assert!(
        trace
            .iter()
            .any(|t| t.0[0] == exit.0[0] && t.0[2] == exit.0[2] && t.0[1] == exit.0[1]),
        "the crane must reach the exit engage pose"
    );
}

#[test]
fn the_rail_only_moves_with_the_arm_retracted() {
    let m = meta();
    let start = pre(0, 3);
    let mut st = GantryState {
        targets: start,
        windows: shelf_windows(&m),
    };
    let mut t = Transfer::build(start, 0, 3, &m).unwrap();
    let (_, trace) = run(&mut t, &mut st, 100_000);
    for w in trace.windows(2) {
        if w[0].0[0] != w[1].0[0] {
            assert!(
                w[1].0[2] <= ARM_RETRACTED,
                "rail moved with the arm out: {:?}",
                w[1]
            );
        }
    }
}

#[test]
fn undo_after_a_partial_transfer_returns_exactly_to_the_start() {
    let m = meta();
    let start = pre(1, 1);
    for k in [1, 3, 6, 9] {
        let mut st = GantryState {
            targets: start,
            windows: shelf_windows(&m),
        };
        let mut t = Transfer::build(start, 1, 1, &m).unwrap();
        while t.completed() < k {
            let obs = GantryObs {
                q: st.targets.0,
                ee_contact: 0.0,
            };
            assert_ne!(t.tick(&mut st, &obs, DT), CmdStatus::Done);
        }
        let mut back = t.undo();
        assert_eq!(run(back.as_mut(), &mut st, 100_000).0, CmdStatus::Done);
        assert_eq!(st.targets, start, "after {k} steps");
    }
}

#[test]
fn an_off_grid_cell_is_an_error() {
    assert!(Transfer::build(home(), 7, 0, &meta()).is_err());
}
