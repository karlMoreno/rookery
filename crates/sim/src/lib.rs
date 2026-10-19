//! `sim`: the safe Rust face of the MuJoCo C API.
//!
//! Pattern: **Facade**. Everything else in Rookery talks to `Sim`
//! (`load`, `step`, `snapshot`, `apply`) and never to MuJoCo directly.
//! Why: it confines every `unsafe` block to this crate, where each one
//! carries a `// SAFETY:` invariant and its own tests, so the rest of the
//! workspace stays 100% safe Rust and a MuJoCo upgrade touches one crate.
//!
//! ```text
//!   rookery (binary) ──uses──▶ Sim  (safe Facade)
//!                               │
//!                               ▼ unsafe, private
//!                             ffi.rs (bindgen over mujoco.h)
//! ```
//!
//! Empty until 1.4 declares the surface and 1.5–1.6 fill it in.
