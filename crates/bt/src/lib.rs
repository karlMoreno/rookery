//! `bt`: behavior-tree engine, typed Blackboard and the `mlua` bridge.
//!
//! Pattern: **Composite**. Sequence, Fallback, Parallel and decorators are
//! nodes that hold child nodes, so a whole tree ticks through one
//! `Node::tick` call. Why: trees become data that Lua can compose and
//! hot-reload, while the engine that runs them stays small, typed Rust.
//!
//! Empty until Phase 4.
