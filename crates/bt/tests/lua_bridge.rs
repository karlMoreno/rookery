//! Row 4.4: the mlua bridge. Tree files, Lua leaves, the read-only `bb` view,
//! `bb:go_to` and hot reload.

use std::fs;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};

use bt::{BbSchema, Blackboard, GantrySnapshot, RELOAD_CHECK_TICKS, Status, Tree};
use mast::route::Pose;
use mlua::Lua;

/// A fresh temp dir per test.
fn dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("bt-lua-{name}-{}", std::process::id()));
    let _ = fs::remove_dir_all(&d);
    fs::create_dir_all(&d).unwrap();
    d
}

fn write(d: &Path, file: &str, src: &str) -> PathBuf {
    let p = d.join(file);
    fs::write(&p, src).unwrap();
    p
}

fn board() -> (Blackboard, BbSchema) {
    let mut bb = Blackboard::new();
    let s = BbSchema::register(&mut bb);
    s.write_snapshot(
        &mut bb,
        &GantrySnapshot {
            sim_time: 2.5,
            gantry_q: [0.0, 0.45, 0.0, 0.0],
            gantry_target: [0.0, 0.45, 0.0, 0.0],
            route_len: 0,
        },
    );
    (bb, s)
}

fn load(d: &Path, tree: &str) -> Tree {
    let p = write(d, "tree.lua", tree);
    Tree::load(p, &Lua::new()).expect("tree loads")
}

#[test]
fn a_leaf_reads_the_snapshot_and_sets_a_goal() {
    let d = dir("goal");
    write(
        &d,
        "go.lua",
        r#"function tick(bb, r, c)
             assert(bb.sim_time == 2.5 and bb.gantry_q[2] == 0.45 and bb.route_len == 0)
             bb:go_to(Cell(r, c))
             return "running"
           end"#,
    );
    let mut tree = load(&d, r#"return Leaf("go.lua", 2, 3)"#);
    let (mut bb, s) = board();
    assert_eq!(tree.tick(&mut bb), Status::Running);
    assert_eq!(bb.get(&s.gantry_goal), Some(&Some(Pose::Cell(2, 3))));
    assert!(tree.take_errors().is_empty());
}

#[test]
fn the_view_shows_the_current_goal() {
    let d = dir("seegoal");
    write(
        &d,
        "see.lua",
        r#"function tick(bb) if bb.goal and bb.goal.pose == "Exit" then return "success" end return "failure" end"#,
    );
    let mut tree = load(&d, r#"return Leaf("see.lua")"#);
    let (mut bb, s) = board();
    assert_eq!(tree.tick(&mut bb), Status::Failure);
    bb.set(&s.gantry_goal, Some(Pose::Exit));
    assert_eq!(tree.tick(&mut bb), Status::Success);
}

#[test]
fn writing_the_blackboard_directly_is_a_named_error() {
    let d = dir("readonly");
    write(
        &d,
        "w.lua",
        r#"function tick(bb) bb.gantry_target = {1, 2, 3, 4} return "success" end"#,
    );
    let mut tree = load(&d, r#"return Leaf("w.lua")"#);
    let (mut bb, _) = board();
    assert_eq!(tree.tick(&mut bb), Status::Failure);
    let e = tree.take_errors();
    assert_eq!(e[0].leaf, "w");
    assert!(
        e[0].message.contains("read-only") && e[0].message.contains("gantry_target"),
        "{}",
        e[0].message
    );
}

#[test]
fn an_unknown_key_and_a_bad_goto_are_named_errors() {
    let d = dir("named");
    write(
        &d,
        "k.lua",
        r#"function tick(bb) local x = bb.no_such_key return "success" end"#,
    );
    write(
        &d,
        "g.lua",
        r#"function tick(bb) bb:go_to(42) return "success" end"#,
    );
    let mut tree = load(
        &d,
        r#"return Parallel { Leaf("k.lua"), Leaf("g.lua"), success = 2, failure = 2 }"#,
    );
    let (mut bb, s) = board();
    assert_eq!(tree.tick(&mut bb), Status::Failure);
    let msgs: Vec<String> = tree
        .take_errors()
        .into_iter()
        .map(|e| format!("{}: {}", e.leaf, e.message))
        .collect();
    assert!(
        msgs.iter()
            .any(|m| m.starts_with("k:") && m.contains("no key `no_such_key`")),
        "{msgs:?}"
    );
    assert!(
        msgs.iter()
            .any(|m| m.starts_with("g:") && m.contains("bb:go_to expects")),
        "{msgs:?}"
    );
    assert_eq!(
        bb.get(&s.gantry_goal),
        Some(&None),
        "a failed leaf must not set a goal"
    );
}

#[test]
fn a_goal_set_before_an_error_is_dropped() {
    let d = dir("atomic");
    write(
        &d,
        "half.lua",
        r#"function tick(bb) bb:go_to(Home()) error("later") end"#,
    );
    let mut tree = load(&d, r#"return Leaf("half.lua")"#);
    let (mut bb, s) = board();
    assert_eq!(tree.tick(&mut bb), Status::Failure);
    assert_eq!(bb.get(&s.gantry_goal), Some(&None));
}

#[test]
fn a_bad_return_value_is_a_failure_with_a_message() {
    let d = dir("ret");
    write(&d, "r.lua", r#"function tick(bb) return "maybe" end"#);
    let mut tree = load(&d, r#"return Leaf("r.lua")"#);
    let (mut bb, _) = board();
    assert_eq!(tree.tick(&mut bb), Status::Failure);
    assert!(tree.take_errors()[0].message.contains("maybe"));
}

#[test]
fn composites_from_lua_behave_like_their_rust_nodes() {
    let d = dir("comp");
    write(&d, "ok.lua", r#"function tick(bb) return "success" end"#);
    write(&d, "no.lua", r#"function tick(bb) return "failure" end"#);
    let mut tree = load(
        &d,
        r#"return Sequence {
             Fallback { Leaf("no.lua"), Leaf("ok.lua") },
             Inverter(Leaf("no.lua")),
             Repeat(2, Leaf("ok.lua")),
           }"#,
    );
    let (mut bb, _) = board();
    assert_eq!(
        tree.tick(&mut bb),
        Status::Running,
        "Repeat(2) needs two ticks"
    );
    assert_eq!(tree.tick(&mut bb), Status::Success);
}

#[test]
fn load_errors_name_the_file() {
    let d = dir("loaderr");
    let p = write(&d, "tree.lua", "return Sequence { Leaf(\"missing.lua\") }");
    let e = Tree::load(&p, &Lua::new()).unwrap_err().to_string();
    assert!(e.contains("missing.lua"), "{e}");
    let p = write(&d, "tree2.lua", "return 7");
    assert!(
        Tree::load(&p, &Lua::new())
            .unwrap_err()
            .to_string()
            .contains("expected a node")
    );
    write(&d, "notick.lua", "x = 1");
    let p = write(&d, "tree3.lua", "return Leaf(\"notick.lua\")");
    assert!(
        Tree::load(&p, &Lua::new())
            .unwrap_err()
            .to_string()
            .contains("does not define `tick")
    );
}

/// Rewrite `path` and push its mtime forward, so a reload is seen even on
/// filesystems with coarse timestamps.
fn edit(path: &Path, src: &str, bump_s: u64) {
    fs::write(path, src).unwrap();
    let f = fs::File::options().write(true).open(path).unwrap();
    f.set_modified(SystemTime::now() + Duration::from_secs(bump_s))
        .unwrap();
}

fn tick_n(tree: &mut Tree, bb: &mut Blackboard, n: u64) -> Status {
    let mut last = Status::Failure;
    for _ in 0..n {
        last = tree.tick(bb);
    }
    last
}

#[test]
fn hot_reload_runs_the_new_code_after_two_stable_checks() {
    let d = dir("reload");
    let leaf = write(&d, "leaf.lua", r#"function tick(bb) return "failure" end"#);
    let mut tree = load(&d, r#"return Leaf("leaf.lua")"#);
    let (mut bb, _) = board();
    assert_eq!(tree.tick(&mut bb), Status::Failure);

    edit(&leaf, r#"function tick(bb) return "success" end"#, 5);
    // Ticks 2..=10: no check yet. Tick 10 sees the new mtime once (pending).
    assert_eq!(
        tick_n(&mut tree, &mut bb, RELOAD_CHECK_TICKS - 1),
        Status::Failure
    );
    // Tick 20 sees it a second time and reloads; that tick runs the new code.
    assert_eq!(
        tick_n(&mut tree, &mut bb, RELOAD_CHECK_TICKS),
        Status::Success
    );
    assert!(tree.take_errors().is_empty());
}

#[test]
fn a_broken_edit_keeps_the_old_code_and_reports_it() {
    let d = dir("badreload");
    let leaf = write(&d, "leaf.lua", r#"function tick(bb) return "success" end"#);
    let mut tree = load(&d, r#"return Leaf("leaf.lua")"#);
    let (mut bb, _) = board();
    edit(&leaf, "function tick(bb) return  -- syntax error (", 5);
    assert_eq!(
        tick_n(&mut tree, &mut bb, 2 * RELOAD_CHECK_TICKS),
        Status::Success,
        "old code still runs"
    );
    let e = tree.take_errors();
    assert!(
        e.iter().any(|e| e.message.contains("keeping the old code")),
        "{e:?}"
    );
}

#[test]
fn a_leaf_can_ask_for_a_transfer() {
    // Row 6.5: Transfer(r, c) is a goal like any pose.
    let d = dir("transfer");
    write(
        &d,
        "t.lua",
        r#"function tick(bb) bb:go_to(Transfer(1, 2)) return "running" end"#,
    );
    let mut tree = load(&d, r#"return Leaf("t.lua")"#);
    let (mut bb, s) = board();
    assert_eq!(tree.tick(&mut bb), Status::Running);
    assert_eq!(bb.get(&s.gantry_goal), Some(&Some(Pose::Transfer(1, 2))));
}
