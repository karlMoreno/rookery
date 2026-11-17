# Thresholds and tolerances

Every number a gate or a command judges against, with where it lives and
where its value came from. The safety limits themselves are in
`crates/mast/src/limits.rs` (protected), which hasn't changed since Phase 3
(`fdbf4de`).

## Phase 5 lift (`tests/gates/tests/phase5_lift.rs`)

| threshold | value | source |
|---|---|---|
| approach offset | uniform ±5 mm in X and Y | v5 plan, Phase 5 gate |
| box tilt (roll, pitch), every tick | < 5° | v5 plan. Measured worst case **0.83°** (100 seeds, 3 cm saddle), and 0.34–0.81° across the sweep at 3–4 cm |
| crawfish touch (`touch_1_1`), every tick | < 3 × weight = **0.883 N** (30 g) | chosen in 5.4: a lift may jostle the crawfish but never press on it. Measured worst case 0.553 N on the gate, 0.755 N across the sweep |
| box underside above the water, at the end | ≥ 1 cm | chosen in 5.4 (MuJoCo has no buoyancy here, so this is geometric). Measured ≥ 2.65 cm |
| still engaged at the end | handle over the finger in X, 0 ≤ Δz ≤ tip rise + 1 cm | 5.4 |
| lift height (mast) | `LIFT_HEIGHT` 0.08 m | 5.4: clears the 0.05 m water by about 3 cm. Inside the shelf window [0.085, 0.80] |
| engage speed / lift speed | 0.5 / 1.0 × `MAX_VEL` | 5.5. The sweep shows 0.5 and 1.0 both pass at every width and kp |

### Sweep (`runs/golden/lift_sweep.csv`, row 5.6)
Grid: `hook_width` {0.02, **0.03**, 0.04} × kp (mast_z, hook_lift) {1000, **2000**, 4000} × lift speed {0.5, **1.0**}, 20 lifts each. **All 18 points pass 20/20.**

| hook_width | worst roll | worst touch | min clearance |
|---|---|---|---|
| 0.02 | 4.34° | 0.755 N | 2.66 cm |
| 0.03 | 0.81° | 0.550 N | 2.65 cm |
| 0.04 | 0.47° | 0.486 N | 2.65 cm |

The defaults (3 cm, kp 2000, speed 1.0) sit in the middle of a region where every neighbour passes. The two problems the 5.5 probe found and fixed: a round finger let ±5 mm offsets roll the box 14–22°, which the flat 30 mm saddle fixed; and kp 300/100 let the loaded mast and hook sag past `DONE_TOL`, which kp 2000 fixed.

## Commands and kinematics (`crates/mast`)

| threshold | value | source |
|---|---|---|
| `DONE_TOL` (measured q within it before `Done`) | 2 mm | settled in 3.1 |
| `SETTLE_TIMEOUT` (target reached, waiting for measured q) | 2 s | 3.3: a critically damped axis settles in under 1 s |
| hook envelope / clearance under a handle | 23 mm / ≥ 4 mm each side (5 mm today) | 3.5 handle-strike bug, enforced in `SceneSpec` (5.1) |
| shelf window floor | box top + 5 mm | 3.4 `shelf_windows` |
| `ARM_THICKNESS` + lift in the window rule | 0.04 m + `hook_lift` | settled in 3.1 |
| route pre-engage back-off `PRE` | finger/2 + 15 mm | 3.5 |
| travel height | tallest handle + 3 cm | 3.5 |

## Scene and scenarios

| threshold | value | source |
|---|---|---|
| box displacement in a scenario | ≤ 1 mm | 4.7 `patrol_4x4` (lesson from the 3.5 handle strike) |
| Phase 1 drop rest height | 0.05 ± 1 mm | v5 Phase 1 gate |
| Phase 3 clamp proptest | 10,000 cases | v5 Phase 3 gate |
