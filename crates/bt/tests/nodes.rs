//! Row 4.2: one test per node, including the `Running` cases.

use std::cell::RefCell;
use std::rc::Rc;

use bt::nodes::{Fallback, Inverter, Parallel, Repeat, Scripted, Sequence};
use bt::{Blackboard, Node, Status};

use Status::{Failure as F, Running as R, Success as S};

/// A Scripted leaf shared with the test, so its counters can be read after
/// it has been boxed into a tree.
#[derive(Clone)]
struct Probe(Rc<RefCell<Scripted>>);

impl Probe {
    fn new(name: &str, script: &[Status]) -> (Self, Box<dyn Node>) {
        let p = Probe(Rc::new(RefCell::new(Scripted::new(name, script))));
        (p.clone(), Box::new(p))
    }
    fn ticks(&self) -> usize {
        self.0.borrow().ticks
    }
    fn resets(&self) -> usize {
        self.0.borrow().resets
    }
}

impl Node for Probe {
    fn tick(&mut self, bb: &mut Blackboard) -> Status {
        self.0.borrow_mut().tick(bb)
    }
    fn reset(&mut self) {
        self.0.borrow_mut().reset()
    }
    fn name(&self) -> &str {
        "probe"
    }
}

fn board() -> Blackboard {
    Blackboard::new()
}

#[test]
fn sequence_succeeds_when_all_children_succeed() {
    let (a, na) = Probe::new("a", &[S]);
    let (b, nb) = Probe::new("b", &[S]);
    let mut seq = Sequence::new(vec![na, nb]);
    assert_eq!(seq.tick(&mut board()), S);
    assert_eq!((a.ticks(), b.ticks()), (1, 1));
}

#[test]
fn sequence_fails_at_the_first_failure_and_skips_the_rest() {
    let (_a, na) = Probe::new("a", &[F]);
    let (b, nb) = Probe::new("b", &[S]);
    let mut seq = Sequence::new(vec![na, nb]);
    assert_eq!(seq.tick(&mut board()), F);
    assert_eq!(b.ticks(), 0);
}

#[test]
fn sequence_resumes_at_its_running_child() {
    let (a, na) = Probe::new("a", &[S]);
    let (b, nb) = Probe::new("b", &[R, R, S]);
    let mut seq = Sequence::new(vec![na, nb]);
    let mut bb = board();
    assert_eq!(seq.tick(&mut bb), R);
    assert_eq!(seq.tick(&mut bb), R);
    assert_eq!(seq.tick(&mut bb), S);
    assert_eq!(
        a.ticks(),
        1,
        "the finished child is not re-ticked while b runs"
    );
    assert_eq!(b.ticks(), 3);
}

#[test]
fn fallback_succeeds_at_the_first_success() {
    let (_a, na) = Probe::new("a", &[F]);
    let (_b, nb) = Probe::new("b", &[S]);
    let (c, nc) = Probe::new("c", &[S]);
    let mut fb = Fallback::new(vec![na, nb, nc]);
    assert_eq!(fb.tick(&mut board()), S);
    assert_eq!(c.ticks(), 0);
}

#[test]
fn fallback_fails_when_every_child_fails() {
    let (_a, na) = Probe::new("a", &[F]);
    let (_b, nb) = Probe::new("b", &[F]);
    assert_eq!(Fallback::new(vec![na, nb]).tick(&mut board()), F);
}

#[test]
fn fallback_resumes_at_its_running_child() {
    let (a, na) = Probe::new("a", &[F]);
    let (b, nb) = Probe::new("b", &[R, S]);
    let mut fb = Fallback::new(vec![na, nb]);
    let mut bb = board();
    assert_eq!(fb.tick(&mut bb), R);
    assert_eq!(fb.tick(&mut bb), S);
    assert_eq!((a.ticks(), b.ticks()), (1, 2));
}

#[test]
fn parallel_succeeds_at_its_success_threshold() {
    let (_a, na) = Probe::new("a", &[S]);
    let (_b, nb) = Probe::new("b", &[R, S]);
    let (_c, nc) = Probe::new("c", &[R]);
    let mut p = Parallel::new(vec![na, nb, nc], 2, 2);
    let mut bb = board();
    assert_eq!(p.tick(&mut bb), R);
    assert_eq!(p.tick(&mut bb), S);
}

#[test]
fn parallel_fails_at_its_failure_threshold() {
    let (_a, na) = Probe::new("a", &[F]);
    let (_b, nb) = Probe::new("b", &[R]);
    let mut p = Parallel::new(vec![na, nb], 2, 1);
    assert_eq!(p.tick(&mut board()), F);
}

#[test]
fn parallel_does_not_retick_finished_children_and_resets_on_completion() {
    let (a, na) = Probe::new("a", &[S]);
    let (b, nb) = Probe::new("b", &[R, R, S]);
    let mut p = Parallel::new(vec![na, nb], 2, 1);
    let mut bb = board();
    assert_eq!(p.tick(&mut bb), R);
    assert_eq!(p.tick(&mut bb), R);
    assert_eq!(p.tick(&mut bb), S);
    assert_eq!(a.ticks(), 1);
    assert!(a.resets() >= 1 && b.resets() >= 1);
}

#[test]
fn parallel_with_all_finished_below_threshold_fails() {
    let (_a, na) = Probe::new("a", &[S]);
    let (_b, nb) = Probe::new("b", &[F]);
    assert_eq!(Parallel::new(vec![na, nb], 2, 2).tick(&mut board()), F);
}

#[test]
fn inverter_swaps_success_and_failure_and_passes_running() {
    let (_a, na) = Probe::new("a", &[S, F, R]);
    let mut inv = Inverter::new(na);
    let mut bb = board();
    assert_eq!(inv.tick(&mut bb), F);
    assert_eq!(inv.tick(&mut bb), S);
    assert_eq!(inv.tick(&mut bb), R);
}

#[test]
fn repeat_n_succeeds_after_n_completions_one_per_tick() {
    let (a, na) = Probe::new("a", &[S]);
    let mut rep = Repeat::new(3, na);
    let mut bb = board();
    assert_eq!(rep.tick(&mut bb), R);
    assert_eq!(rep.tick(&mut bb), R);
    assert_eq!(rep.tick(&mut bb), S);
    assert_eq!(a.ticks(), 3);
}

#[test]
fn repeat_forever_never_succeeds() {
    let (_a, na) = Probe::new("a", &[S]);
    let mut rep = Repeat::new(0, na);
    let mut bb = board();
    for _ in 0..100 {
        assert_eq!(rep.tick(&mut bb), R);
    }
}

#[test]
fn repeat_fails_when_its_child_fails() {
    // Scripted restarts its script on reset (which Repeat does after each
    // success), so drive the failure through Running instead.
    let (_a, na) = Probe::new("a", &[R, F]);
    let mut rep = Repeat::new(5, na);
    let mut bb = board();
    assert_eq!(rep.tick(&mut bb), R);
    assert_eq!(rep.tick(&mut bb), F);
    let (_b, nb) = Probe::new("b", &[F]);
    assert_eq!(Repeat::new(0, nb).tick(&mut board()), F);
}

#[test]
fn repeat_waits_while_its_child_runs() {
    let (_a, na) = Probe::new("a", &[R, R, S]);
    let mut rep = Repeat::new(1, na);
    let mut bb = board();
    assert_eq!(rep.tick(&mut bb), R);
    assert_eq!(rep.tick(&mut bb), R);
    assert_eq!(rep.tick(&mut bb), S);
}

#[test]
fn reset_propagates_to_every_child() {
    let (a, na) = Probe::new("a", &[S]);
    let (b, nb) = Probe::new("b", &[R]);
    let inner = Sequence::new(vec![na, Box::new(Inverter::new(nb))]);
    let mut tree = Repeat::new(0, Box::new(inner));
    tree.tick(&mut board());
    tree.reset();
    assert!(a.resets() >= 1, "a not reset");
    assert!(b.resets() >= 1, "b not reset through the Inverter");
}

#[test]
fn a_failing_leaf_fails_the_root_on_the_first_tick() {
    // The pure-Rust half of Phase 4 gate 1.
    let (_a, na) = Probe::new("ok", &[S]);
    let (_b, nb) = Probe::new("fail", &[F]);
    assert_eq!(Sequence::new(vec![na, nb]).tick(&mut board()), F);
}

#[test]
fn scripted_repeats_its_last_status() {
    let mut s = Scripted::new("s", &[R, S]);
    let mut bb = board();
    assert_eq!(s.tick(&mut bb), R);
    assert_eq!(s.tick(&mut bb), S);
    assert_eq!(s.tick(&mut bb), S);
    assert_eq!(s.name(), "s");
}
