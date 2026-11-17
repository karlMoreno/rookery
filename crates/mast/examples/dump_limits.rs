//! `cargo run -p mast --example dump-limits` prints `limits.rs` as JSON.
//! Row 3.2's pytest compares it with the MJCF joint ranges, and row 4.1's
//! with the builder's hook geometry and home pose; both fail on drift.
//! Hand-formatted, so `mast` needs no serde for it.

use mast::kinematics::{
    HANDLE_DIAMETER, HOME, HOOK_FINGER, HOOK_RADIUS, HOOK_TIP_RISE, HOOK_WIDTH,
};
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
    println!("  \"arm_retracted\": {ARM_RETRACTED:.6},");
    // Hook geometry, which the scene builder duplicates (row 4.1 drift check).
    println!(
        "  \"hook\": {{\"finger\": {HOOK_FINGER:.6}, \"radius\": {HOOK_RADIUS:.6}, \
         \"tip_rise\": {HOOK_TIP_RISE:.6}, \"width\": {HOOK_WIDTH:.6}, \"handle_diameter\": {HANDLE_DIAMETER:.6}}},"
    );
    let h: Vec<String> = HOME.0.iter().map(|x| format!("{x:.6}")).collect();
    println!("  \"home\": [{}]", h.join(", "));
    println!("}}");
}
