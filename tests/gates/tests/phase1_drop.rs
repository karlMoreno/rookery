//! Phase 1 gate: a 0.1 m box dropped onto a plane comes to rest at half
//! its size.
//!
//! Load `examples/drop.xml`, step 2,000 times at 1 ms (2 s of sim time),
//! and assert `|z − 0.05| < 0.001` for body `box`.
//!
//! It talks to MuJoCo only through the `sim` **Facade**, the same surface
//! every other crate uses, so passing it proves the facade (rows 1.5–1.6)
//! rather than some test-only path into the C API.
//!
//! ```text
//!   phase1_drop ──▶ Sim::load ──▶ Sim::step ×2000 ──▶ Sim::snapshot
//!                                                      │
//!                     Sim::body_index("box") ──index──▶ bodies[i].pos[2] ≈ 0.05
//! ```
//!
//! Red on `main` from row 1.4 (the facade bodies are `todo!()`); green once
//! row 1.6 lands. Row 2.7 points it at `scene/generated/drop.xml`.

use sim::Sim;

/// The scene, relative to this crate so `cargo test -p gates` works from any cwd.
const SCENE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/drop.xml");
/// 2,000 steps × the scene's 1 ms timestep = 2 s.
const STEPS: usize = 2_000;
const TIMESTEP: f64 = 0.001;
/// Half of the 0.1 m cube: the resting height of its centre.
const REST_Z: f64 = 0.05;
/// Gate tolerance: 1 mm.
const TOL: f64 = 0.001;

#[test]
fn box_rests_at_half_its_size() {
    let mut sim = Sim::load(SCENE).expect("examples/drop.xml should load");
    let box_idx = sim
        .body_index("box")
        .expect("drop.xml should have a body named `box`");

    for _ in 0..STEPS {
        sim.step();
    }

    let snap = sim.snapshot();
    // Guards the "at 1 ms" half of the gate: if someone changes the scene's
    // timestep, 2,000 steps is no longer 2 s and this fails loudly.
    let expected_t = STEPS as f64 * TIMESTEP;
    assert!(
        (snap.time - expected_t).abs() < 1e-9,
        "sim time {} after {STEPS} steps, expected {expected_t}",
        snap.time
    );

    let z = snap.bodies[box_idx].pos[2];
    assert!(
        (z - REST_Z).abs() < TOL,
        "box z = {z:.6}, expected {REST_Z} ± {TOL}"
    );
}
