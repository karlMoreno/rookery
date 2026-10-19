//! `mast`: stacker-crane kinematics, independent of the farm.
//!
//! Patterns (from Phase 3):
//! - **Command**: every gantry move is a `MoveCommand` with `tick`/`undo`,
//!   which gives replay, undo-on-fault and deterministic tests.
//! - **Strategy**: `Hook` (passive J-hook now, active later) is a trait, so
//!   swapping hardware is a new impl, not an edit.
//!
//! Safety constants live in `src/limits.rs` (protected path, rule 5).
//! This crate must never depend on `rookery-core`; CI checks that.
//!
//! Empty until Phase 3.
