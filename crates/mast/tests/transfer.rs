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

#[test]
fn the_scene_lists_six_exit_slots_nearest_the_middle_first() {
    // Row 8.4.
    let m = meta();
    let ys: Vec<f64> = m.derived.exit_slots.iter().map(|p| p[1]).collect();
    assert_eq!(ys, [-0.075, 0.075, -0.225, 0.225, -0.375, 0.375]);
    let p = mast::exit_slot_pose(1, &m).unwrap();
    assert_eq!(p.0[Axis::RailY.index()], 0.075);
    assert_eq!(
        p.0[Axis::MastZ.index()],
        exit_pose(&m).0[Axis::MastZ.index()]
    );
    assert!(mast::exit_slot_pose(6, &m).is_none());
    assert_eq!(
        mast::route::route(home(), mast::route::Pose::ExitSlot(6), &m).unwrap_err(),
        mast::route::RouteError::NoSuchSlot { slot: 6, slots: 6 }
    );
}

#[test]
fn a_transfer_into_a_slot_sets_the_box_down_there() {
    let m = meta();
    for slot in 0..m.derived.exit_slots.len() {
        let mut st = GantryState {
            targets: pre(2, 1),
            windows: shelf_windows(&m),
        };
        let mut t = Transfer::build_to(st.targets, 2, 1, Some(slot), &m).unwrap();
        let (status, trace) = run(&mut t, &mut st, 20_000);
        assert_eq!(status, CmdStatus::Done);
        let at = mast::exit_slot_pose(slot, &m).unwrap();
        assert!(
            trace
                .iter()
                .any(|q| (0..3).all(|k| (q.0[k] - at.0[k]).abs() < 1e-6)),
            "slot {slot}: never released at {at:?}"
        );
        assert_eq!(st.targets, home());
    }
    assert!(Transfer::build_to(home(), 0, 0, Some(9), &m).is_err());
}

#[test]
fn the_allocator_never_puts_two_boxes_in_one_slot() {
    // 100 transfers with harvests interleaved (a fixed pseudo-random order):
    // every take is the lowest free slot, and no slot is handed out twice
    // while taken.
    let mut a = mast::SlotAllocator::new(6);
    let mut taken = [false; 6];
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    let mut transfers = 0;
    while transfers < 100 {
        if next() % 3 == 0 {
            // A harvest: a worker takes a box off a random taken slot.
            let full: Vec<usize> = (0..6).filter(|&i| taken[i]).collect();
            if !full.is_empty() {
                let i = full[(next() % full.len() as u64) as usize];
                a.free(i);
                taken[i] = false;
            }
            continue;
        }
        let expect = taken.iter().position(|t| !t);
        let got = a.take();
        assert_eq!(got, expect, "the lowest free slot");
        if let Some(i) = got {
            assert!(!taken[i]);
            taken[i] = true;
            transfers += 1;
        } else {
            // Full: harvest the middle slot so the run continues.
            a.free(0);
            taken[0] = false;
        }
        assert_eq!(a.in_use(), taken.iter().filter(|t| **t).count());
    }
    assert_eq!(a.len(), 6);
}
