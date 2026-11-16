//! The composite and decorator nodes, in pure Rust (row 4.2).
//!
//! Pattern: **Composite**. Every node is a [`Node`], composites own their
//! children as `Box<dyn Node>`, and a whole tree ticks through the root.
//!
//! ```text
//!   Sequence  ticks children in order, stops at the first Failure (fails) or
//!             Running (returns Running, and *resumes at that child* next tick)
//!   Fallback  ticks children in order, stops at the first Success (succeeds)
//!             or Running (resumes there next tick)
//!   Parallel  ticks every unfinished child each tick: Success once
//!             `success_threshold` children succeeded, Failure once
//!             `failure_threshold` failed, otherwise Running
//!   Inverter  Success ↔ Failure, Running passes through
//!   Repeat(n) runs its child to Success n times (one completion per tick at
//!             most); n = 0 repeats forever; a child Failure fails it
//! ```
//!
//! "Memory" means a composite remembers its running child, so a long action
//! (driving to a cell) is not restarted from the first child on every tick.
//! Any completion (Success or Failure) resets the composite and its
//! children, so the next tick starts fresh.

use crate::{Blackboard, Node, Status};

/// Ticks children in order until one fails or is running.
pub struct Sequence {
    children: Vec<Box<dyn Node>>,
    current: usize,
}

impl Sequence {
    pub fn new(children: Vec<Box<dyn Node>>) -> Self {
        Self {
            children,
            current: 0,
        }
    }
}

impl Node for Sequence {
    fn tick(&mut self, bb: &mut Blackboard) -> Status {
        while self.current < self.children.len() {
            match self.children[self.current].tick(bb) {
                Status::Success => self.current += 1,
                Status::Running => return Status::Running,
                Status::Failure => {
                    self.reset();
                    return Status::Failure;
                }
            }
        }
        self.reset();
        Status::Success
    }

    fn reset(&mut self) {
        self.current = 0;
        self.children.iter_mut().for_each(|c| c.reset());
    }

    fn name(&self) -> &str {
        "Sequence"
    }
}

/// Ticks children in order until one succeeds or is running.
pub struct Fallback {
    children: Vec<Box<dyn Node>>,
    current: usize,
}

impl Fallback {
    pub fn new(children: Vec<Box<dyn Node>>) -> Self {
        Self {
            children,
            current: 0,
        }
    }
}

impl Node for Fallback {
    fn tick(&mut self, bb: &mut Blackboard) -> Status {
        while self.current < self.children.len() {
            match self.children[self.current].tick(bb) {
                Status::Failure => self.current += 1,
                Status::Running => return Status::Running,
                Status::Success => {
                    self.reset();
                    return Status::Success;
                }
            }
        }
        self.reset();
        Status::Failure
    }

    fn reset(&mut self) {
        self.current = 0;
        self.children.iter_mut().for_each(|c| c.reset());
    }

    fn name(&self) -> &str {
        "Fallback"
    }
}

/// Ticks all unfinished children each tick and decides by thresholds.
pub struct Parallel {
    children: Vec<Box<dyn Node>>,
    done: Vec<Option<Status>>,
    success_threshold: usize,
    failure_threshold: usize,
}

impl Parallel {
    /// Thresholds are clamped to `1..=children.len()`, so a parallel node
    /// always terminates once every child has.
    pub fn new(
        children: Vec<Box<dyn Node>>,
        success_threshold: usize,
        failure_threshold: usize,
    ) -> Self {
        let n = children.len().max(1);
        Self {
            done: vec![None; children.len()],
            children,
            success_threshold: success_threshold.clamp(1, n),
            failure_threshold: failure_threshold.clamp(1, n),
        }
    }
}

impl Node for Parallel {
    fn tick(&mut self, bb: &mut Blackboard) -> Status {
        if self.children.is_empty() {
            return Status::Success;
        }
        for (child, done) in self.children.iter_mut().zip(self.done.iter_mut()) {
            if done.is_none() {
                match child.tick(bb) {
                    Status::Running => {}
                    finished => *done = Some(finished),
                }
            }
        }
        let count = |s| self.done.iter().filter(|d| **d == Some(s)).count();
        let (ok, failed) = (count(Status::Success), count(Status::Failure));
        let result = if ok >= self.success_threshold {
            Status::Success
        } else if failed >= self.failure_threshold {
            Status::Failure
        } else if ok + failed == self.children.len() {
            // Every child finished but neither threshold was met.
            Status::Failure
        } else {
            return Status::Running;
        };
        self.reset();
        result
    }

    fn reset(&mut self) {
        self.done.iter_mut().for_each(|d| *d = None);
        self.children.iter_mut().for_each(|c| c.reset());
    }

    fn name(&self) -> &str {
        "Parallel"
    }
}

/// Swaps Success and Failure.
pub struct Inverter {
    child: Box<dyn Node>,
}

impl Inverter {
    pub fn new(child: Box<dyn Node>) -> Self {
        Self { child }
    }
}

impl Node for Inverter {
    fn tick(&mut self, bb: &mut Blackboard) -> Status {
        match self.child.tick(bb) {
            Status::Success => Status::Failure,
            Status::Failure => Status::Success,
            Status::Running => Status::Running,
        }
    }

    fn reset(&mut self) {
        self.child.reset();
    }

    fn name(&self) -> &str {
        "Inverter"
    }
}

/// Runs its child to Success `n` times (`n = 0`: forever).
pub struct Repeat {
    child: Box<dyn Node>,
    n: usize,
    done: usize,
}

impl Repeat {
    pub fn new(n: usize, child: Box<dyn Node>) -> Self {
        Self { child, n, done: 0 }
    }
}

impl Node for Repeat {
    fn tick(&mut self, bb: &mut Blackboard) -> Status {
        match self.child.tick(bb) {
            Status::Running => Status::Running,
            Status::Failure => {
                self.reset();
                Status::Failure
            }
            Status::Success => {
                self.done += 1;
                self.child.reset();
                if self.n != 0 && self.done >= self.n {
                    self.done = 0;
                    Status::Success
                } else {
                    // At most one completion per tick, so a repeat of an
                    // instant child cannot spin the tick forever.
                    Status::Running
                }
            }
        }
    }

    fn reset(&mut self) {
        self.done = 0;
        self.child.reset();
    }

    fn name(&self) -> &str {
        "Repeat"
    }
}

/// A test leaf that returns a fixed script of statuses, then repeats the
/// last one. It counts ticks and resets, so tests can check memory.
#[derive(Debug, Clone)]
pub struct Scripted {
    name: String,
    script: Vec<Status>,
    at: usize,
    /// How many times this leaf was ticked.
    pub ticks: usize,
    /// How many times `reset` reached this leaf.
    pub resets: usize,
}

impl Scripted {
    pub fn new(name: &str, script: &[Status]) -> Self {
        Self {
            name: name.to_string(),
            script: script.to_vec(),
            at: 0,
            ticks: 0,
            resets: 0,
        }
    }
}

impl Node for Scripted {
    fn tick(&mut self, _bb: &mut Blackboard) -> Status {
        self.ticks += 1;
        let s = self
            .script
            .get(self.at)
            .or(self.script.last())
            .copied()
            .unwrap_or(Status::Failure);
        self.at += 1;
        s
    }

    fn reset(&mut self) {
        self.at = 0;
        self.resets += 1;
    }

    fn name(&self) -> &str {
        &self.name
    }
}
