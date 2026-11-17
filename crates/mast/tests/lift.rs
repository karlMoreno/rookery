//! Row 5.5: MacroCommand composition and the lift sequence.

use mast::kinematics::shelf_windows;
use mast::lift::ENGAGE_SPEED;
use mast::limits::MAX_VEL;
use mast::{
    Axis, CmdStatus, Command, GantryObs, GantryState, Hook, JointTargets, LIFT_HEIGHT,
    LiftSequence, MacroCommand, MoveCommand, PassiveHook, SceneMeta, cell_pose,
};

const META: &str = include_str!("fixtures/rookery.json");
const DT: f64 = 0.02;

fn meta() -> SceneMeta {
    SceneMeta::from_json(META).unwrap()
}

fn state(t: JointTargets) -> GantryState {
    GantryState {
        targets: t,
        windows: shelf_windows(&meta()),
    }
}

/// Tick with perfect tracking until not Running; return the status and the
/// axis that moved on each tick.
fn run(cmd: &mut dyn Command, st: &mut GantryState, max: usize) -> (CmdStatus, Vec<Option<Axis>>) {
    let mut moved = Vec::new();
    for _ in 0..max {
        let before = st.targets;
        let obs = GantryObs {
            q: st.targets.0,
            ee_contact: 0.0,
        };
        let s = cmd.tick(st, &obs, DT);
        moved.push(
            Axis::ALL
                .into_iter()
                .find(|a| st.targets.0[a.index()] != before.0[a.index()]),
        );
        if s != CmdStatus::Running {
            return (s, moved);
        }
    }
    (CmdStatus::Running, moved)
}

fn engaged() -> JointTargets {
    cell_pose(1, 1, &meta()).unwrap()
}

#[test]
fn the_lift_raises_the_hook_then_the_mast() {
    let start = engaged();
    let mut st = state(start);
    let mut lift = LiftSequence::build(start, LIFT_HEIGHT);
    let (s, moved) = run(&mut lift, &mut st, 10_000);
    assert_eq!(s, CmdStatus::Done);
    let order: Vec<Axis> = moved.into_iter().flatten().fold(Vec::new(), |mut v, a| {
        if v.last() != Some(&a) {
            v.push(a);
        }
        v
    });
    assert_eq!(order, [Axis::HookLift, Axis::MastZ]);
    assert_eq!(
        st.targets.0[Axis::HookLift.index()],
        PassiveHook.engage_dz()
    );
    assert_eq!(
        st.targets.0[Axis::MastZ.index()],
        start.0[Axis::MastZ.index()] + LIFT_HEIGHT
    );
    assert_eq!(lift.completed(), 2);
}

#[test]
fn the_engage_segment_is_slower_than_max_vel() {
    let start = engaged();
    let mut st = state(start);
    let mut lift = LiftSequence::build(start, LIFT_HEIGHT);
    let obs = GantryObs {
        q: st.targets.0,
        ee_contact: 0.0,
    };
    let _ = obs;
    // With the acceleration ramp (6.6) the first steps are smaller; the
    // cruise step during the engage segment is ENGAGE_SPEED × MAX_VEL × dt.
    let limit = MAX_VEL[Axis::HookLift.index()] * DT;
    let mut biggest: f64 = 0.0;
    for _ in 0..200 {
        let before = st.targets.0[Axis::HookLift.index()];
        let obs = GantryObs {
            q: st.targets.0,
            ee_contact: 0.0,
        };
        lift.tick(&mut st, &obs, DT);
        biggest = biggest.max(st.targets.0[Axis::HookLift.index()] - before);
        if lift.completed() >= 1 {
            break;
        }
    }
    assert!(
        (biggest - ENGAGE_SPEED * limit).abs() < 1e-12,
        "cruise step {biggest}, limit {limit}"
    );
}

#[test]
fn undo_after_a_full_lift_lowers_back_exactly() {
    let start = engaged();
    let mut st = state(start);
    let mut lift = LiftSequence::build(start, LIFT_HEIGHT);
    run(&mut lift, &mut st, 10_000);
    let mut down = lift.undo();
    let (s, moved) = run(down.as_mut(), &mut st, 10_000);
    assert_eq!(s, CmdStatus::Done);
    assert_eq!(st.targets, start);
    assert_eq!(
        moved.into_iter().flatten().next(),
        Some(Axis::MastZ),
        "the mast comes down first"
    );
}

#[test]
fn undo_after_a_partial_lift_returns_exactly_to_the_start() {
    let start = engaged();
    let mut st = state(start);
    let mut lift = LiftSequence::build(start, LIFT_HEIGHT);
    // Part-way through the engage segment.
    run(&mut lift, &mut st, 10);
    assert_eq!(lift.completed(), 0);
    assert_ne!(st.targets, start);
    let mut back = lift.undo();
    assert_eq!(run(back.as_mut(), &mut st, 10_000).0, CmdStatus::Done);
    assert_eq!(st.targets, start);
}

#[test]
fn a_failing_step_fails_the_macro_and_stops_there() {
    let start = engaged();
    let mut st = state(start);
    let mut m = MacroCommand::new(vec![
        Box::new(MoveCommand::new(Axis::MastZ, 9.0)), // out of range
        Box::new(MoveCommand::new(Axis::RailY, 0.1)),
    ]);
    let (s, _) = run(&mut m, &mut st, 10);
    assert!(matches!(s, CmdStatus::Failed(_)));
    assert_eq!(st.targets, start);
}

#[test]
fn an_empty_macro_is_done_at_once() {
    let mut st = state(engaged());
    let mut m = MacroCommand::new(Vec::new());
    assert_eq!(run(&mut m, &mut st, 1).0, CmdStatus::Done);
}

#[test]
fn speed_is_clamped_and_kept_by_undo() {
    let start = engaged();
    let mut st = state(start);
    let fast = MoveCommand::new(Axis::MastZ, start.0[1] + 0.05).with_speed(5.0);
    assert_eq!(fast.speed, 1.0, "speed is capped at MAX_VEL");
    let mut slow = MoveCommand::new(Axis::MastZ, start.0[1] + 0.05).with_speed(0.2);
    let (s, moved) = run(&mut slow, &mut st, 10_000);
    assert_eq!(s, CmdStatus::Done);
    // 0.05 m at 0.2 × 0.15 m/s and 50 Hz takes about 84 ticks, not 17.
    assert!(moved.len() > 80, "{} ticks", moved.len());
    let mut back = slow.undo();
    let (_, moved_back) = run(back.as_mut(), &mut st, 10_000);
    assert!(
        moved_back.len() > 80,
        "undo keeps the slow speed: {} ticks",
        moved_back.len()
    );
    assert_eq!(st.targets, start);
}
