//! Phase 4 gate 1: a failing leaf fails the root on the first tick, and the
//! crane holds.
//!
//! A tree `Sequence { Leaf("fail.lua") }` whose leaf always returns
//! "failure" is loaded from a temp dir and ticked once with a goal already
//! on the blackboard. The root must return `Failure` on tick 1, and
//! `gantry_goal` must be exactly what it was before the tick. Lua sets
//! goals, never raw targets (#19), so an unchanged goal means the crane holds.
//!
//! ```text
//!   tree.lua: Sequence { Leaf("fail.lua") }     fail.lua: tick → "failure"
//!        │ Tree::load                                 │
//!        ▼                                             ▼
//!   Tree::tick(bb{goal = Cell(1,1)}) ──▶ Failure   and   bb.goal == Cell(1,1)
//! ```
//!
//! Red on `main` from row 4.1. Green once 4.2–4.4 land.

use bt::{BbSchema, Blackboard, Status, Tree};
use mast::route::Pose;
use mlua::Lua;

#[test]
fn failing_leaf_fails_the_root_on_tick_one_and_the_goal_holds() {
    let dir = std::env::temp_dir().join(format!("phase4-failing-leaf-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(
        dir.join("fail.lua"),
        "function tick(bb) return \"failure\" end\n",
    )
    .unwrap();
    std::fs::write(
        dir.join("tree.lua"),
        "return Sequence { Leaf(\"fail.lua\") }\n",
    )
    .unwrap();

    let lua = Lua::new();
    let mut tree = Tree::load(dir.join("tree.lua"), &lua).expect("tree loads");
    let mut bb = Blackboard::new();
    let keys = BbSchema::register(&mut bb);
    let before = Some(Pose::Cell(1, 1));
    bb.set(&keys.gantry_goal, before);

    assert_eq!(
        tree.tick(&mut bb),
        Status::Failure,
        "root must fail on tick 1"
    );
    assert_eq!(
        bb.get(&keys.gantry_goal),
        Some(&before),
        "the goal must not change"
    );
    assert!(
        tree.take_errors().is_empty(),
        "a leaf returning failure is not an error"
    );

    let _ = std::fs::remove_dir_all(&dir);
}
