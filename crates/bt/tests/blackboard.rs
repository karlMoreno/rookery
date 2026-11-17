//! Row 4.3: the typed Blackboard and the schema.

use bt::{BbError, BbSchema, Blackboard, GantrySnapshot, SCHEMA_VERSION};
use mast::route::Pose;

#[test]
fn set_then_get_returns_the_value() {
    let mut bb = Blackboard::new();
    let k = bb.register::<f64>("x");
    assert_eq!(bb.get(&k), None, "unset reads as None");
    bb.set(&k, 1.5);
    assert_eq!(bb.get(&k), Some(&1.5));
    bb.set(&k, 2.5);
    assert_eq!(bb.get(&k), Some(&2.5));
}

#[test]
fn registering_the_same_name_and_type_returns_the_same_key() {
    let mut bb = Blackboard::new();
    let a = bb.register::<usize>("n");
    let b = bb.register::<usize>("n");
    assert_eq!(a.id(), b.id());
    assert_eq!(bb.names().count(), 1);
}

#[test]
fn a_type_conflict_on_register_is_a_named_error() {
    let mut bb = Blackboard::new();
    bb.register::<f64>("sim_time");
    let err = bb.try_register::<usize>("sim_time").unwrap_err();
    assert!(matches!(err, BbError::WrongType { ref key, .. } if key == "sim_time"));
    assert!(err.to_string().contains("sim_time"));
}

#[test]
fn by_name_access_checks_the_type_at_run_time() {
    let mut bb = Blackboard::new();
    let k = bb.register::<[f64; 4]>("gantry_q");
    bb.set(&k, [0.0, 0.45, 0.0, 0.0]);
    assert_eq!(
        bb.get_named::<[f64; 4]>("gantry_q"),
        Ok(Some(&[0.0, 0.45, 0.0, 0.0]))
    );
    let err = bb.get_named::<f64>("gantry_q").unwrap_err();
    assert_eq!(
        err,
        BbError::WrongType {
            key: "gantry_q".into(),
            expected: "f64",
            found: "[f64; 4]"
        }
    );
    assert_eq!(
        bb.get_named::<f64>("nope").unwrap_err(),
        BbError::UnknownKey { key: "nope".into() }
    );
}

#[test]
fn the_schema_registers_every_key_with_defaults() {
    let mut bb = Blackboard::new();
    let s = BbSchema::register(&mut bb);
    let names: Vec<_> = bb.names().collect();
    assert_eq!(
        names,
        [
            "sim_time",
            "gantry_q",
            "gantry_target",
            "route_len",
            "gantry_goal"
        ]
    );
    assert_eq!(bb.get(&s.gantry_goal), Some(&None));
    assert_eq!(bb.get(&s.route_len), Some(&0));
    assert_eq!(SCHEMA_VERSION, 1);
}

#[test]
fn write_snapshot_fills_snapshot_keys_and_leaves_the_goal_alone() {
    let mut bb = Blackboard::new();
    let s = BbSchema::register(&mut bb);
    bb.set(&s.gantry_goal, Some(Pose::Exit));
    let snap = GantrySnapshot {
        sim_time: 1.25,
        gantry_q: [0.1, 0.2, 0.3, 0.0],
        gantry_target: [0.1, 0.2, 0.31, 0.0],
        route_len: 4,
    };
    s.write_snapshot(&mut bb, &snap);
    assert_eq!(bb.get(&s.sim_time), Some(&1.25));
    assert_eq!(bb.get(&s.gantry_q), Some(&[0.1, 0.2, 0.3, 0.0]));
    assert_eq!(bb.get(&s.gantry_target), Some(&[0.1, 0.2, 0.31, 0.0]));
    assert_eq!(bb.get(&s.route_len), Some(&4));
    assert_eq!(bb.get(&s.gantry_goal), Some(&Some(Pose::Exit)));
}

#[test]
fn a_wrong_type_get_does_not_compile() {
    trybuild::TestCases::new().compile_fail("tests/ui/*.rs");
}
