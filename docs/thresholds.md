# Thresholds and tolerances

Every number a gate or a command judges against, with where it lives and
where its value came from. The safety limits themselves are in
`crates/mast/src/limits.rs` (protected). They last changed for the telescoping-fork
crane (`7258d8a`): `ARM_X` 0.07..0.85, `MAST_Z` 0.05..0.85, `HOOK_LIFT` locked at 0,
`ARM_RETRACTED` 0.075 and the new `FORK_ABOVE_HOOK` 0.072.

## Phase 5 lift (`tests/gates/tests/phase5_lift.rs`)

| threshold | value | source |
|---|---|---|
| approach offset | uniform ±5 mm in X and Y | v5 plan, Phase 5 gate |
| box tilt (roll, pitch), every tick | < 5° | v5 plan. Measured worst case **0.83°** (100 seeds, 3 cm saddle), and 0.34–0.81° across the sweep at 3–4 cm |
| crawfish touch (`touch_1_1`), every tick | < 3 × weight = **0.883 N** (30 g) | chosen in 5.4: a lift may jostle the crawfish but never press on it. Measured worst case 0.553 N on the gate, 0.755 N across the sweep |
| box underside above the water, at the end | ≥ 1 cm | chosen in 5.4 (MuJoCo has no buoyancy here, so this is geometric). Measured ≥ 2.65 cm |
| still engaged at the end | handle over the finger in X, 0 ≤ Δz ≤ tip rise + 1 cm | 5.4 |
| lift height (mast) | `LIFT_HEIGHT` 0.08 m | 5.4: clears the 0.05 m water by about 3 cm. Inside the shelf window, whose top is now `MAST_Z.hi` 0.85 |
| engage speed / lift speed | `ENGAGE_VEL` 0.01 m/s (the mast lowers the hook onto the handle) / 1.0 × `MAX_VEL` | fork port (D1): the fork has no hook joint, so the mast drop engages. The sweep's speed axis is the lift speed |

### Sweep (`runs/golden/lift_sweep.csv`, row 5.6, rerun for the fork crane)
Grid: `hook_width` {0.02, **0.03**, 0.04} × kp (mast_z; `hook_lift` is locked on the fork) {1000, **2000**, 4000} × lift speed {0.5, **1.0**}, 20 lifts each. **All 18 points pass 20/20.**

| hook_width | worst roll | worst touch | min clearance |
|---|---|---|---|
| 0.02 | 4.33° | 0.755 N | 2.70 cm |
| 0.03 | 0.80° | 0.530 N | 2.70 cm |
| 0.04 | 0.47° | 0.466 N | 2.70 cm |

The defaults (3 cm, kp 2000, speed 1.0) sit in the middle of a region where every neighbour passes. The two problems the 5.5 probe found and fixed: a round finger let ±5 mm offsets roll the box 14–22°, which the flat 30 mm saddle fixed; and kp 300/100 let the loaded mast and hook sag past `DONE_TOL`, which kp 2000 fixed.

## Commands and kinematics (`crates/mast`)

| threshold | value | source |
|---|---|---|
| `DONE_TOL` (measured q within it before `Done`) | 2 mm | settled in 3.1 |
| `SETTLE_TIMEOUT` (target reached, waiting for measured q) | 2 s | 3.3: a critically damped axis settles in under 1 s |
| hook envelope / clearance under a handle | 23 mm / ≥ 4 mm each side (5 mm today) | 3.5 handle-strike bug, enforced in `SceneSpec` (5.1) |
| shelf window floor | box top + 5 mm | 3.4 `shelf_windows` |
| shelf window top | max(handle rule, `FORK_ABOVE_HOOK` 0.072 m) | fork port: the fork's top sits 72 mm above the hook, so a shelf must leave it that much room |
| `ARM_THICKNESS` | 0.04 m | settled in 3.1 |
| route pre-engage back-off `PRE` | finger/2 + 15 mm | 3.5 |
| travel height | tallest handle + 3 cm | 3.5 |

## Scene and scenarios

| threshold | value | source |
|---|---|---|
| box displacement in a scenario | ≤ 1 mm | 4.7 `patrol_4x4` (lesson from the 3.5 handle strike) |
| Phase 1 drop rest height | 0.05 ± 1 mm | v5 Phase 1 gate |
| Phase 3 clamp proptest | 10,000 cases | v5 Phase 3 gate |
