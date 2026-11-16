//! `cargo run -p mast --example dump-limits` prints `limits.rs` as JSON.
//! Row 3.2's pytest compares it with the MJCF joint ranges and fails on drift.
//! Hand-formatted, so `mast` needs no serde for it.

use mast::limits::{ARM_RETRACTED, ARM_THICKNESS, MAX_VEL, RANGES, WORKSPACE};

const JOINTS: [&str; 4] = ["rail_y", "mast_z", "arm_x", "hook_lift"];

fn main() {
    println!("{{");
    println!("  \"ranges\": {{");
    for (i, (name, r)) in JOINTS.iter().zip(RANGES).enumerate() {
        let comma = if i + 1 < JOINTS.len() { "," } else { "" };
        println!("    \"{name}\": [{:.6}, {:.6}]{comma}", r.lo, r.hi);
    }
    println!("  }},");
    let v: Vec<String> = MAX_VEL.iter().map(|x| format!("{x:.6}")).collect();
    println!("  \"max_vel\": [{}],", v.join(", "));
    let fmt3 = |a: [f64; 3]| format!("[{:.6}, {:.6}, {:.6}]", a[0], a[1], a[2]);
    println!(
        "  \"workspace\": {{\"min\": {}, \"max\": {}}},",
        fmt3(WORKSPACE.min),
        fmt3(WORKSPACE.max)
    );
    println!("  \"arm_thickness\": {ARM_THICKNESS:.6},");
    println!("  \"arm_retracted\": {ARM_RETRACTED:.6}");
    println!("}}");
}
