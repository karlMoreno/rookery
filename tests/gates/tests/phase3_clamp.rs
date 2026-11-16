//! Phase 3 gate: the clamp keeps every target safe.
//!
//! 10,000 random cases. Each draws a valid previous target `prev`, an
//! arbitrary (often wild) `next`, a tick `dt` and a set of shelf windows, and
//! asserts that `mast::clamp(prev, next, dt, windows)`:
//!   1. keeps every axis inside its range in `limits.rs`;
//!   2. moves no axis more than `MAX_VEL · dt` from `prev`;
//!   3. extends the arm past `ARM_RETRACTED` only while the mast is inside a
//!      window taller than `ARM_THICKNESS + hook_lift` (the arm and the lifted
//!      hook never enter a gap they do not fit).
//!
//! `prev` is always a legal state. From an illegal one, rules 2 and 3 can
//! conflict, and no clamp could satisfy both.
//!
//! It drives `mast` through its public surface only.
//!
//! ```text
//!   proptest ──(prev valid, next wild, dt, windows)──▶ mast::clamp ──▶ out
//!                                                                   │
//!          range ✓   |out − prev| ≤ MAX_VEL·dt ✓   arm ⇒ in window ✓ ◀┘
//! ```
//!
//! Red on `main` from row 3.1 (`clamp` is `todo!()`). Green once row 3.4 lands.

use mast::limits::{ARM_RETRACTED, ARM_THICKNESS, MAST_Z, MAX_VEL, RANGES, Range};
use mast::{Axis, JointTargets, ShelfWindows, clamp};
use proptest::prelude::*;

/// Float slack for comparisons, metres.
const EPS: f64 = 1e-9;

/// 0 to 3 windows inside the mast range, some thinner than the arm.
fn windows() -> impl Strategy<Value = ShelfWindows> {
    prop::collection::vec((MAST_Z.lo..MAST_Z.hi, 0.0..0.15f64), 0..=3).prop_map(|ws| {
        ShelfWindows(
            ws.into_iter()
                .map(|(lo, h)| Range::new(lo, (lo + h).min(MAST_Z.hi)))
                .collect(),
        )
    })
}

/// A target that already satisfies every rule for `w`: in range, and the arm
/// retracted unless the mast sits in a window the arm fits.
fn valid_prev(w: ShelfWindows) -> impl Strategy<Value = (JointTargets, ShelfWindows)> {
    let r = RANGES;
    (
        r[0].lo..=r[0].hi,
        r[1].lo..=r[1].hi,
        r[2].lo..=r[2].hi,
        r[3].lo..=r[3].hi,
    )
        .prop_map(move |(y, z, x, h)| {
            let x = if w.admits(z, h) {
                x
            } else {
                x.min(ARM_RETRACTED)
            };
            (JointTargets([y, z, x, h]), w.clone())
        })
}

/// Anything, including far outside the ranges.
fn wild() -> impl Strategy<Value = JointTargets> {
    prop::array::uniform4(-2.0..2.0f64).prop_map(JointTargets)
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(10_000))]

    #[test]
    fn clamp_keeps_every_target_safe(
        (prev, w) in windows().prop_flat_map(valid_prev),
        next in wild(),
        dt in 0.001..0.1f64,
    ) {
        let out = clamp(&prev, next, dt, &w);

        for axis in Axis::ALL {
            let i = axis.index();
            let (r, v) = (RANGES[i], out.0[i]);
            prop_assert!(r.lo - EPS <= v && v <= r.hi + EPS,
                "{axis:?} = {v} outside {}..{}", r.lo, r.hi);
            let step = (v - prev.0[i]).abs();
            prop_assert!(step <= MAX_VEL[i] * dt + EPS,
                "{axis:?} moved {step} in {dt} s (max {})", MAX_VEL[i] * dt);
        }

        let z = out.0[Axis::MastZ.index()];
        let x = out.0[Axis::ArmX.index()];
        let h = out.0[Axis::HookLift.index()];
        if x > ARM_RETRACTED + EPS {
            prop_assert!(w.admits(z, h),
                "arm out to {x} at mast z {z} with lift {h}, outside every window taller than \
                 {ARM_THICKNESS} + {h}: {w:?}");
        }
    }
}
