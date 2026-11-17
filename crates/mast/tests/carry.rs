//! Row 6.3: the rail moves only with the arm retracted.

use mast::limits::{ARM_RETRACTED, MAST_Z, RANGES, Range};
use mast::{Axis, JointTargets, ShelfWindows, clamp};
use proptest::prelude::*;

fn window() -> ShelfWindows {
    ShelfWindows(vec![Range::new(0.085, MAST_Z.hi)])
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(10_000))]

    #[test]
    fn the_rail_never_moves_with_the_arm_out(
        y in RANGES[0].lo..=RANGES[0].hi,
        z in 0.1..0.8f64,
        x in RANGES[2].lo..=RANGES[2].hi,
        next in prop::array::uniform4(-2.0..2.0f64),
        dt in 0.001..0.1f64,
    ) {
        let prev = JointTargets([y, z, x, 0.0]);
        let out = clamp(&prev, JointTargets(next), dt, &window());
        let (ry, ax) = (Axis::RailY.index(), Axis::ArmX.index());
        if out.0[ry] != prev.0[ry] {
            prop_assert!(prev.0[ax] <= ARM_RETRACTED && out.0[ax] <= ARM_RETRACTED,
                "rail moved {} → {} with the arm at {} → {}", prev.0[ry], out.0[ry], prev.0[ax], out.0[ax]);
        }
    }
}

#[test]
fn with_the_arm_retracted_the_rail_moves() {
    let prev = JointTargets([0.0, 0.45, 0.0, 0.0]);
    let out = clamp(&prev, JointTargets([0.3, 0.45, 0.0, 0.0]), 0.02, &window());
    assert!(out.0[0] > 0.0);
}

#[test]
fn extending_the_arm_and_moving_the_rail_together_holds_the_rail() {
    // The arm at the edge of the retracted band, asked to extend past it
    // while the rail also moves: the arm goes out, the rail waits.
    let prev = JointTargets([0.0, 0.45, ARM_RETRACTED, 0.0]);
    let out = clamp(&prev, JointTargets([0.3, 0.45, 0.6, 0.0]), 0.02, &window());
    assert_eq!(out.0[0], 0.0);
    assert!(out.0[2] > ARM_RETRACTED, "the arm still extends");
}
