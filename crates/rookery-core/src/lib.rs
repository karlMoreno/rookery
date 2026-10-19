//! `rookery-core`: the farm's domain model.
//!
//! Patterns (from Phase 7):
//! - **State**: the crawfish lifecycle
//!   `Intermolt → Premolt → Molting → SoftShell → Hardened`.
//! - **Observer**: an event bus (`Molted`, `BoxLifted`, ...) so detectors
//!   and the behavior tree subscribe instead of polling.
//! - **Strategy**: `MoltDetector` with oracle and camera implementations.
//!
//! Empty until Phase 7.
