//! Phase 4 gate 2: a Lua runtime error is contained.
//!
//! A leaf whose `tick` calls `error("boom")` must make the tick return
//! `Failure`, record a `LeafError` naming the leaf (`boom`) and carrying
//! the message, and never panic.
//!
//! [Band 1] On the Air a Rust panic aborts the process (no unwinding), so
//! this gate cannot use `catch_unwind`. Reaching the assertions *is* the
//! proof that nothing panicked. The bridge must contain the error through
//! Lua's own `pcall` / `mlua::Result`. CI on Linux is the verdict.
//!
//! Red on `main` from row 4.1. Green once 4.4 lands.

use bt::{BbSchema, Blackboard, Status, Tree};
use mlua::Lua;

#[test]
fn lua_error_becomes_failure_with_the_leaf_named() {
    let dir = std::env::temp_dir().join(format!("phase4-lua-error-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(
        dir.join("boom.lua"),
        "function tick(bb) error(\"boom\") end\n",
    )
    .unwrap();
    std::fs::write(
        dir.join("tree.lua"),
        "return Sequence { Leaf(\"boom.lua\") }\n",
    )
    .unwrap();

    let lua = Lua::new();
    let mut tree = Tree::load(dir.join("tree.lua"), &lua).expect("tree loads");
    let mut bb = Blackboard::new();
    let _keys = BbSchema::register(&mut bb);

    assert_eq!(tree.tick(&mut bb), Status::Failure);
    let errors = tree.take_errors();
    assert_eq!(errors.len(), 1, "one leaf error: {errors:?}");
    assert_eq!(errors[0].leaf, "boom");
    assert!(
        errors[0].message.contains("boom"),
        "message: {}",
        errors[0].message
    );
    // A second tick still works: the error did not poison the tree.
    assert_eq!(tree.tick(&mut bb), Status::Failure);

    let _ = std::fs::remove_dir_all(&dir);
}
