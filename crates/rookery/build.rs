//! Build script for `rookery`: give this crate's binaries an rpath to MuJoCo.
//!
//! `sim` links MuJoCo, but Cargo applies `sim`'s `rustc-link-arg` only to
//! `sim`'s own targets. Without this script the `rookery` binary links but
//! fails at launch on the Air with "Library not loaded: @rpath/mujoco.framework/...".
//!
//! ```text
//!   sim/build.rs ──cargo::metadata=rpath=<dir>──▶ DEP_MUJOCO_RPATH
//!                                                     │ (direct dependents only)
//!   rookery/build.rs ──rustc-link-arg=-Wl,-rpath,<dir>──▶ rookery bin + its tests
//! ```

fn main() {
    println!("cargo:rerun-if-env-changed=DEP_MUJOCO_RPATH");
    // Set whenever `sim`'s build script ran, which it must have for `sim` to
    // build at all. Missing only if `sim` stops publishing it; then the
    // binary still links and only run-time lookup on macOS is affected.
    if let Some(dir) = std::env::var_os("DEP_MUJOCO_RPATH") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.to_string_lossy());
    }
}
