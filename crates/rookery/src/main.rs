//! `rookery`: the binary that wires the crates together.
//!
//! Pattern (from 1.7): **Template Method**. A `Runner` fixes the loop
//! skeleton `pre_step → step → post_step → tick` and phases plug in by
//! overriding hooks, so the loop itself is never rewritten.
//!
//! Placeholder until 1.7.

fn main() {
    println!("rookery: workspace skeleton (Phase 1.1)");
}
