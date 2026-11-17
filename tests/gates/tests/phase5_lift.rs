//! Phase 5 gate: 100 lifts from randomised approach offsets.
//!
//! For each seed 0..100: load the committed scene, place the crane at the
//! pre-engage pose for cell (1, 1) offset by (dx, dy), each uniform in
//! ±5 mm (`Sim::set_qpos`, so no trip from home), slide the finger under
//! the handle, then run `LiftSequence::build(engaged, LIFT_HEIGHT)` and let it
//! settle. Every seed must meet all four:
//!   1. the box's underside clears the water plane by at least 1 cm
//!      (MuJoCo has no buoyancy here, so this is geometric);
//!   2. |pitch| and |roll| of `box_1_1` stay under 5° on every tick;
//!   3. the crawfish's touch sensor (`touch_1_1`) stays under
//!      `CRAWFISH_MAX_N` on every tick, so it is never crushed;
//!   4. at the end the handle sits over the finger (between the hook's root
//!      and its tip in X, above the finger in Z by less than the tip rise),
//!      i.e. still engaged.
//!
//! The loop mirrors `rookery`'s: physics at 1 kHz, commands at 50 Hz through
//! `mast::clamp`. Seeds come from a fixed xorshift, so a failure names a seed
//! you can rerun.
//!
//! ```text
//!   seed ─▶ (dx, dy) ─▶ set_qpos(pre-engage + offset) ─▶ ArmX slide ─▶ LiftSequence ─▶ settle
//!                                       every tick: pitch, roll, touch_1_1   end: z, engaged
//! ```
//!
//! Red on `main` from row 5.4 (`LiftSequence::build` is `todo!()`). Green once
//! row 5.5 lands, after 5.6's contact tuning if needed.

use mast::kinematics::shelf_windows;
use mast::route::PRE;
use mast::{
    Axis, CmdStatus, Command, GantryObs, GantryState, JointTargets, LIFT_HEIGHT, LiftSequence,
    MoveCommand, SceneMeta, cell_pose,
};
use sim::{Sim, Targets};

const SCENE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scene/generated/rookery.xml"
);
const META: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../scene/generated/rookery.json"
);
const SEEDS: u64 = 100;
const OFFSET: f64 = 0.005; // ±5 mm
const MAX_TILT_DEG: f64 = 5.0;
/// Three times the 30 g crawfish's weight: a lift may shake it, never press on it.
const CRAWFISH_MAX_N: f64 = 3.0 * 0.03 * 9.81;
const WATER_CLEARANCE: f64 = 0.01;
const TICK_EVERY: usize = 20; // 1 kHz physics, 50 Hz commands
const SETTLE_TICKS: usize = 50; // 1 s after the lift reports Done
const MAX_TICKS: usize = 600; // 12 s per seed: far more than a lift needs

/// xorshift64*: deterministic, no dependency.
struct Rng(u64);
impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn offset(&mut self) -> f64 {
        (2.0 * self.unit() - 1.0) * OFFSET
    }
}

struct Ids {
    truck: usize,
    carriage: usize,
    arm: usize,
    hook: usize,
    box_: usize,
}

fn observe(sim: &Sim, id: &Ids, rail_x: f64) -> GantryObs {
    let b = sim.snapshot().bodies;
    GantryObs {
        q: [
            b[id.truck].pos[1],
            b[id.carriage].pos[2],
            b[id.arm].pos[0] - rail_x,
            b[id.hook].pos[2] - b[id.carriage].pos[2],
        ],
        ee_contact: 0.0,
    }
}

/// Run `cmd` to completion at 50 Hz, calling `check` after every tick.
fn play(
    sim: &mut Sim,
    st: &mut GantryState,
    cmd: &mut dyn Command,
    id: &Ids,
    rail_x: f64,
    mut check: impl FnMut(&Sim) -> Result<(), String>,
) -> Result<(), String> {
    for _ in 0..MAX_TICKS {
        for _ in 0..TICK_EVERY {
            sim.apply(&Targets(st.targets.0.to_vec()))
                .map_err(|e| e.to_string())?;
            sim.step();
        }
        check(sim)?;
        let obs = observe(sim, id, rail_x);
        match cmd.tick(st, &obs, TICK_EVERY as f64 * sim.timestep()) {
            CmdStatus::Running => {}
            CmdStatus::Done => return Ok(()),
            CmdStatus::Failed(e) => return Err(format!("command failed: {e}")),
        }
    }
    Err("command did not finish".into())
}

fn one_lift(seed: u64, meta: &SceneMeta) -> Result<(), String> {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15 ^ (seed + 1));
    let (dx, dy) = (rng.offset(), rng.offset());

    let mut sim = Sim::load(SCENE).map_err(|e| e.to_string())?;
    let idx = |n: &str| sim.body_index(n).ok_or(format!("no body {n}"));
    let id = Ids {
        truck: idx("crane_truck")?,
        carriage: idx("crane_carriage")?,
        arm: idx("crane_arm")?,
        hook: idx("crane_hook")?,
        box_: idx("box_1_1")?,
    };
    let rail_x = meta.derived.rail_line_x;

    let engaged = cell_pose(1, 1, meta).ok_or("cell (1, 1)")?;
    let engaged = JointTargets([engaged.0[0] + dy, engaged.0[1], engaged.0[2] + dx, 0.0]);
    let mut pre = engaged;
    pre.0[Axis::ArmX.index()] -= PRE;

    // The crane's 4 slides are the last 4 qpos entries, and their values are
    // the joint coordinates (ref = home in the scene).
    let mut q = sim.qpos();
    let n = q.len();
    q[n - 4..].copy_from_slice(&pre.0);
    sim.set_qpos(&q).map_err(|e| e.to_string())?;
    let mut st = GantryState {
        targets: pre,
        windows: shelf_windows(meta),
    };

    let box_z0 = sim.snapshot().bodies[id.box_].pos[2];
    let water = meta.water_depth;
    let watch = |sim: &Sim| -> Result<(), String> {
        let b = &sim.snapshot().bodies[id.box_];
        let (r, p) = (b.roll.to_degrees(), b.pitch.to_degrees());
        if r.abs() >= MAX_TILT_DEG || p.abs() >= MAX_TILT_DEG {
            return Err(format!("box tilted: roll {r:.2}°, pitch {p:.2}°"));
        }
        let t = sim.sensor("touch_1_1").ok_or("no touch_1_1 sensor")?;
        if t >= CRAWFISH_MAX_N {
            return Err(format!(
                "crawfish pressed: {t:.3} N (limit {CRAWFISH_MAX_N:.3})"
            ));
        }
        Ok(())
    };

    let mut slide = MoveCommand::new(Axis::ArmX, engaged.0[Axis::ArmX.index()]);
    play(&mut sim, &mut st, &mut slide, &id, rail_x, watch)?;
    let mut lift = LiftSequence::build(engaged, LIFT_HEIGHT);
    play(&mut sim, &mut st, &mut lift, &id, rail_x, watch)?;
    for _ in 0..SETTLE_TICKS * TICK_EVERY {
        sim.apply(&Targets(st.targets.0.to_vec()))
            .map_err(|e| e.to_string())?;
        sim.step();
    }
    watch(&sim)?;

    let s = sim.snapshot();
    let box_bottom = s.bodies[id.box_].pos[2];
    if box_bottom - water < WATER_CLEARANCE {
        return Err(format!(
            "box underside at {box_bottom:.4} m (started {box_z0:.4}), needs {:.3} above water {water:.3}",
            WATER_CLEARANCE
        ));
    }
    // Engaged: the handle (at the box centre in X) lies over the finger.
    let hook = s.bodies[id.hook].pos;
    let handle_x = s.bodies[id.box_].pos[0];
    let handle_z = box_bottom + meta.box_size[2] + meta.handle_height
        - mast::kinematics::HANDLE_DIAMETER / 2.0;
    let over_finger = hook[0] <= handle_x && handle_x <= hook[0] + mast::kinematics::HOOK_FINGER;
    let dz = handle_z - hook[2];
    if !over_finger || !(0.0..=mast::kinematics::HOOK_TIP_RISE + 0.01).contains(&dz) {
        return Err(format!(
            "not engaged: hook at {hook:?}, handle x {handle_x:.4} z {handle_z:.4}"
        ));
    }
    Ok(())
}

#[test]
fn one_hundred_lifts_from_randomised_offsets() {
    let meta = SceneMeta::load(META).expect("rookery.json");
    let failures: Vec<String> = (0..SEEDS)
        .filter_map(|seed| {
            one_lift(seed, &meta)
                .err()
                .map(|e| format!("seed {seed}: {e}"))
        })
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {SEEDS} lifts failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
