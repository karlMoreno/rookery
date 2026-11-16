//! Row 3.3: kinematics for the corner cells and the exit tray, MoveCommand
//! tick/undo, and CommandLog round-tripping through JSON.

use mast::kinematics::{HANDLE_DIAMETER, HOOK_FINGER};
use mast::limits::{MAX_VEL, RANGES};
use mast::{
    Axis, CmdStatus, Command, CommandLog, GantryObs, GantryState, Hook, JointTargets, MastError,
    MoveCommand, PassiveHook, SceneMeta, cell_pose, exit_pose, home,
};

/// A copy of `scene/generated/rookery.json`, kept inside the crate so `mast`
/// still tests standalone (row 3.6). `fixture_matches_the_generated_scene`
/// fails if the two drift apart while the scene is present.
const ROOKERY_JSON: &str = include_str!("fixtures/rookery.json");
const DT: f64 = 0.02; // 50 Hz, the BT tick rate from Phase 4

fn meta() -> SceneMeta {
    SceneMeta::from_json(ROOKERY_JSON).expect("rookery.json parses")
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

#[test]
fn fixture_matches_the_generated_scene() {
    let scene = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scene/generated/rookery.json"
    );
    match std::fs::read_to_string(scene) {
        Ok(text) => assert_eq!(
            SceneMeta::from_json(&text).expect("scene json parses"),
            meta(),
            "tests/fixtures/rookery.json is stale: copy scene/generated/rookery.json over it"
        ),
        Err(_) => eprintln!("standalone: no scene/generated/rookery.json, fixture check skipped"),
    }
}

#[test]
fn corner_cells_put_the_handle_over_mid_finger() {
    let m = meta();
    let rail = m.derived.rail_line_x;
    for (r, c) in [(0, 0), (0, 3), (3, 0), (3, 3)] {
        let p = cell_pose(r, c, &m).expect("corner is in the grid");
        let (x, y) = m.cell_center(r, c);
        assert!(close(p.0[Axis::RailY.index()], y), "({r},{c}) rail");
        // hook body x + half a finger = handle x
        assert!(
            close(rail + p.0[Axis::ArmX.index()] + HOOK_FINGER / 2.0, x),
            "({r},{c}) arm"
        );
        let box_top = m.box_size[2];
        let handle_bottom = box_top + m.handle_height - HANDLE_DIAMETER;
        let z = p.0[Axis::MastZ.index()];
        assert!(
            box_top < z && z < handle_bottom,
            "({r},{c}) z {z} not between box top and handle"
        );
        assert_eq!(p.0[Axis::HookLift.index()], 0.0);
        for (i, v) in p.0.iter().enumerate() {
            assert!(
                RANGES[i].contains(*v),
                "({r},{c}) axis {i} = {v} outside its range"
            );
        }
    }
    // The actual numbers for the default 4x4 scene: rows along X, 0.15 m pitch.
    let p00 = cell_pose(0, 0, &m).unwrap();
    assert!(close(p00.0[0], -0.225) && close(p00.0[1], 0.089));
    let p33 = cell_pose(3, 3, &m).unwrap();
    assert!(close(p33.0[0], 0.225) && close(p33.0[2], 0.225 - rail - 0.02));
}

#[test]
fn outside_the_grid_is_none() {
    let m = meta();
    assert!(cell_pose(4, 0, &m).is_none());
    assert!(cell_pose(0, 4, &m).is_none());
}

#[test]
fn exit_pose_engages_a_box_on_the_exit_tray() {
    let m = meta();
    let p = exit_pose(&m);
    let [x, y, z] = m.derived.exit_pose;
    assert!(close(p.0[0], y));
    assert!(close(m.derived.rail_line_x + p.0[2] + HOOK_FINGER / 2.0, x));
    assert!(close(p.0[1], z + cell_pose(0, 0, &m).unwrap().0[1]));
    for (i, v) in p.0.iter().enumerate() {
        assert!(RANGES[i].contains(*v));
    }
}

#[test]
fn home_is_in_range_and_retracted() {
    let h = home();
    for (i, v) in h.0.iter().enumerate() {
        assert!(RANGES[i].contains(*v));
    }
    assert_eq!(h.0[Axis::ArmX.index()], 0.0);
}

/// Tick `cmd` with perfect tracking (measured q = target) until it stops.
fn run(cmd: &mut dyn Command, st: &mut GantryState) -> (CmdStatus, usize) {
    for n in 1..=10_000 {
        let obs = GantryObs {
            q: st.targets.0,
            ee_contact: 0.0,
        };
        match cmd.tick(st, &obs, DT) {
            CmdStatus::Running => {}
            other => return (other, n),
        }
    }
    panic!("command never finished");
}

#[test]
fn move_then_undo_returns_the_exact_prior_targets() {
    let before = home();
    let mut st = GantryState { targets: before };
    let mut mv = MoveCommand::new(Axis::ArmX, 0.6);
    let (status, ticks) = run(&mut mv, &mut st);
    assert_eq!(status, CmdStatus::Done);
    assert_eq!(st.targets.0[Axis::ArmX.index()], 0.6);
    // 0.6 m at 0.20 m/s and 50 Hz takes 150 ticks, plus one to confirm.
    assert!((150..=152).contains(&ticks), "{ticks} ticks");
    assert_eq!(mv.start(), Some(0.0));

    let mut back = mv.undo();
    assert_eq!(run(back.as_mut(), &mut st).0, CmdStatus::Done);
    assert_eq!(
        st.targets, before,
        "undo must restore the exact prior targets"
    );
}

#[test]
fn a_move_touches_only_its_axis_and_respects_max_vel() {
    let mut st = GantryState { targets: home() };
    let mut mv = MoveCommand::new(Axis::MastZ, 0.1);
    let mut prev = st.targets;
    loop {
        let obs = GantryObs {
            q: st.targets.0,
            ee_contact: 0.0,
        };
        let status = mv.tick(&mut st, &obs, DT);
        for (i, max_vel) in MAX_VEL.iter().enumerate() {
            let step = (st.targets.0[i] - prev.0[i]).abs();
            if i == Axis::MastZ.index() {
                assert!(step <= max_vel * DT + 1e-12);
            } else {
                assert_eq!(step, 0.0, "axis {i} moved");
            }
        }
        prev = st.targets;
        if status != CmdStatus::Running {
            assert_eq!(status, CmdStatus::Done);
            break;
        }
    }
}

#[test]
fn done_waits_for_the_measured_position() {
    let mut st = GantryState {
        targets: JointTargets([0.0, 0.45, 0.0, 0.0]),
    };
    let mut mv = MoveCommand::new(Axis::HookLift, 0.0); // target already at goal
    let lagging = GantryObs {
        q: [0.0, 0.45, 0.0, 0.01],
        ee_contact: 0.0,
    };
    assert_eq!(mv.tick(&mut st, &lagging, DT), CmdStatus::Running);
    let settled = GantryObs {
        q: [0.0, 0.45, 0.0, 0.001],
        ee_contact: 0.0,
    };
    assert_eq!(mv.tick(&mut st, &settled, DT), CmdStatus::Done);
}

#[test]
fn a_joint_that_never_arrives_times_out() {
    let mut st = GantryState { targets: home() };
    let mut mv = MoveCommand::new(Axis::RailY, 0.0);
    let stuck = GantryObs {
        q: [0.3, 0.45, 0.0, 0.0],
        ee_contact: 0.0,
    };
    let mut last = CmdStatus::Running;
    for _ in 0..200 {
        last = mv.tick(&mut st, &stuck, DT);
        if last != CmdStatus::Running {
            break;
        }
    }
    assert_eq!(
        last,
        CmdStatus::Failed(MastError::Timeout { axis: Axis::RailY })
    );
}

#[test]
fn an_out_of_range_goal_fails_without_moving() {
    let mut st = GantryState { targets: home() };
    let mut mv = MoveCommand::new(Axis::MastZ, 2.0);
    let obs = GantryObs {
        q: st.targets.0,
        ee_contact: 0.0,
    };
    assert!(matches!(
        mv.tick(&mut st, &obs, DT),
        CmdStatus::Failed(MastError::OutOfRange { .. })
    ));
    assert_eq!(st.targets, home());
}

#[test]
fn undo_before_any_tick_is_a_noop() {
    let mut st = GantryState { targets: home() };
    let mut u = MoveCommand::new(Axis::ArmX, 0.5).undo();
    let obs = GantryObs {
        q: st.targets.0,
        ee_contact: 0.0,
    };
    assert_eq!(u.tick(&mut st, &obs, DT), CmdStatus::Done);
    assert_eq!(st.targets, home());
}

#[test]
fn command_log_round_trips_through_json() {
    let mut st = GantryState { targets: home() };
    let mut mv = MoveCommand::new(Axis::RailY, 0.2);
    run(&mut mv, &mut st);
    let mut log = CommandLog::default();
    log.push(0, MoveCommand::new(Axis::MastZ, 0.1));
    log.push(42, mv);
    let text = serde_json::to_string(&log).expect("serialises");
    let back: CommandLog = serde_json::from_str(&text).expect("parses");
    assert_eq!(back, log);
    assert!(text.contains("\"RailY\""));
}

#[test]
fn passive_hook_fits_its_stroke() {
    let h = PassiveHook;
    assert!(h.engage_dz() > 0.0 && RANGES[Axis::HookLift.index()].contains(h.engage_dz()));
    assert_eq!(h.reach(), HOOK_FINGER);
}
