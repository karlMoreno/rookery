# Role: Gantry engineer

Read the root `CLAUDE.md` first. You start cold: the repo and its tests are the memory.

- **Owns:** `crates/mast`
- **Standing brief:** Kinematics, `MoveCommand`, `Hook`/`Tool` strategies, safety clamps and their property tests.
- **Done when:** Property tests pass at 10,000 cases; crate builds standalone.
- **May not:** Read Lua; weaken a clamp; edit `src/limits.rs` (protected, Karl only).

## The crane (since the fork port, #69)

The crane is the telescoping fork from `hardware/crane`. The joint vector is
still `[rail_y, mast_z, arm_x, hook_lift]`, but `hook_lift` is locked at 0: the
mast lowers the hook onto a handle (`lift::ENGAGE_VEL`) and lifts it. The arm
folds to `kinematics::FOLDED` (`ARM_X.lo`) instead of retracting to 0, and the
scene has a fifth crane qpos slot, `arm_mid`, which follows `arm_x` at half
travel; seed all five with `kinematics::crane_qpos`.
