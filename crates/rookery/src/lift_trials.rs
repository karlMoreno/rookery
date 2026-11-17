//! `rookery lift`: run N offset lifts on a scene and report margins (row 5.6).
//!
//! The same trial as the Phase 5 gate (`tests/gates/tests/phase5_lift.rs`):
//! a ±5 mm approach offset on cell (1, 1), start at pre-engage, slide under
//! the handle, lift, settle. Instead of pass/fail, each trial reports its
//! worst roll and pitch, peak crawfish touch, water clearance and whether the
//! handle is still on the finger. The lift's mast speed is a parameter, so
//! `scene/scene/sweep.py` can tune it alongside scene parameters.
//!
//! ```text
//!   rookery lift --scene S --seeds 20 --lift-speed 1.0
//!     → {"trials": 20, "passed": 20, "max_roll_deg": …, "max_touch_n": …, "min_clearance_m": …}
//! ```

use std::path::Path;

use mast::kinematics::{HANDLE_DIAMETER, HOOK_FINGER, HOOK_TIP_RISE, shelf_windows};
use mast::lift::ENGAGE_SPEED;
use mast::route::PRE;
use mast::{
    Axis, CmdStatus, Command, GantryObs, GantryState, Hook, JointTargets, LIFT_HEIGHT,
    MacroCommand, MoveCommand, PassiveHook, SceneMeta, cell_pose,
};
use sim::{Sim, Targets};

/// The gate's limits, so a sweep point "passes" exactly when the gate would.
pub const OFFSET: f64 = 0.005;
pub const MAX_TILT_DEG: f64 = 5.0;
pub const CRAWFISH_MAX_N: f64 = 3.0 * 0.03 * 9.81;
pub const WATER_CLEARANCE: f64 = 0.01;
const TICK_EVERY: usize = 20;
const SETTLE_TICKS: usize = 50;
const MAX_TICKS: usize = 600;

/// One trial's margins.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct Trial {
    pub seed: u64,
    pub max_roll_deg: f64,
    pub max_pitch_deg: f64,
    pub max_touch_n: f64,
    pub clearance_m: f64,
    pub engaged: bool,
    pub error: Option<String>,
}

impl Trial {
    pub fn passed(&self) -> bool {
        self.error.is_none()
            && self.engaged
            && self.max_roll_deg < MAX_TILT_DEG
            && self.max_pitch_deg < MAX_TILT_DEG
            && self.max_touch_n < CRAWFISH_MAX_N
            && self.clearance_m >= WATER_CLEARANCE
    }
}

/// xorshift64*, seeded like the gate, so seed n here is seed n there.
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

/// Run one offset lift. Errors (a command failing, a missing body) are
/// recorded in the trial rather than returned.
pub fn trial(scene: &Path, meta: &SceneMeta, seed: u64, lift_speed: f64) -> Trial {
    let mut t = Trial {
        seed,
        ..Trial::default()
    };
    if let Err(e) = run(scene, meta, seed, lift_speed, &mut t) {
        t.error = Some(e);
    }
    t
}

fn run(
    scene: &Path,
    meta: &SceneMeta,
    seed: u64,
    lift_speed: f64,
    t: &mut Trial,
) -> Result<(), String> {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15 ^ (seed + 1));
    let (dx, dy) = (rng.offset(), rng.offset());
    let mut sim = Sim::load(scene).map_err(|e| e.to_string())?;
    let idx = |n: &str| sim.body_index(n).ok_or(format!("no body {n}"));
    let ids = [
        idx("crane_truck")?,
        idx("crane_carriage")?,
        idx("crane_arm")?,
        idx("crane_hook")?,
        idx("box_1_1")?,
    ];
    let rail_x = meta.derived.rail_line_x;

    let e = cell_pose(1, 1, meta).ok_or("cell (1, 1)")?;
    let engaged = JointTargets([e.0[0] + dy, e.0[1], e.0[2] + dx, 0.0]);
    let mut pre = engaged;
    pre.0[Axis::ArmX.index()] -= PRE;
    let mut q = sim.qpos();
    let n = q.len();
    q[n - 4..].copy_from_slice(&pre.0);
    sim.set_qpos(&q).map_err(|e| e.to_string())?;
    let mut st = GantryState {
        targets: pre,
        windows: shelf_windows(meta),
    };

    let observe = |sim: &Sim| {
        let b = sim.snapshot().bodies;
        GantryObs {
            q: [
                b[ids[0]].pos[1],
                b[ids[1]].pos[2],
                b[ids[2]].pos[0] - rail_x,
                b[ids[3]].pos[2] - b[ids[1]].pos[2],
            ],
            ee_contact: 0.0,
        }
    };
    let watch = |sim: &Sim, t: &mut Trial| {
        let b = &sim.snapshot().bodies[ids[4]];
        t.max_roll_deg = t.max_roll_deg.max(b.roll.to_degrees().abs());
        t.max_pitch_deg = t.max_pitch_deg.max(b.pitch.to_degrees().abs());
        t.max_touch_n = t.max_touch_n.max(sim.sensor("touch_1_1").unwrap_or(0.0));
    };
    let dt = TICK_EVERY as f64 * sim.timestep();
    let play = |sim: &mut Sim,
                st: &mut GantryState,
                cmd: &mut dyn Command,
                t: &mut Trial|
     -> Result<(), String> {
        for _ in 0..MAX_TICKS {
            for _ in 0..TICK_EVERY {
                sim.apply(&Targets(st.targets.0.to_vec()))
                    .map_err(|e| e.to_string())?;
                sim.step();
            }
            watch(sim, t);
            match cmd.tick(st, &observe(sim), dt) {
                CmdStatus::Running => {}
                CmdStatus::Done => return Ok(()),
                CmdStatus::Failed(e) => return Err(format!("command failed: {e}")),
            }
        }
        Err("command did not finish".into())
    };

    let mut slide = MoveCommand::new(Axis::ArmX, engaged.0[Axis::ArmX.index()]);
    play(&mut sim, &mut st, &mut slide, t)?;
    let mut lift = MacroCommand::new(vec![
        Box::new(
            MoveCommand::new(Axis::HookLift, PassiveHook.engage_dz()).with_speed(ENGAGE_SPEED),
        ),
        Box::new(
            MoveCommand::new(Axis::MastZ, engaged.0[Axis::MastZ.index()] + LIFT_HEIGHT)
                .with_speed(lift_speed),
        ),
    ]);
    play(&mut sim, &mut st, &mut lift, t)?;
    for _ in 0..SETTLE_TICKS * TICK_EVERY {
        sim.apply(&Targets(st.targets.0.to_vec()))
            .map_err(|e| e.to_string())?;
        sim.step();
    }
    watch(&sim, t);

    let s = sim.snapshot();
    let bottom = s.bodies[ids[4]].pos[2];
    t.clearance_m = bottom - meta.water_depth;
    let hook = s.bodies[ids[3]].pos;
    let handle_x = s.bodies[ids[4]].pos[0];
    let handle_z = bottom + meta.box_size[2] + meta.handle_height - HANDLE_DIAMETER / 2.0;
    let dz = handle_z - hook[2];
    t.engaged = hook[0] <= handle_x
        && handle_x <= hook[0] + HOOK_FINGER
        && (0.0..=HOOK_TIP_RISE + 0.01).contains(&dz);
    Ok(())
}

/// Run `seeds` trials and summarise them as JSON.
pub fn summary(scene: &Path, seeds: u64, lift_speed: f64) -> Result<serde_json::Value, String> {
    let meta = SceneMeta::load(scene.with_extension("json")).map_err(|e| e.to_string())?;
    let trials: Vec<Trial> = (0..seeds)
        .map(|s| trial(scene, &meta, s, lift_speed))
        .collect();
    let fold = |f: fn(&Trial) -> f64, init: f64, pick: fn(f64, f64) -> f64| {
        trials.iter().map(f).fold(init, pick)
    };
    Ok(serde_json::json!({
        "scene": scene.display().to_string(),
        "lift_speed": lift_speed,
        "trials": trials.len(),
        "passed": trials.iter().filter(|t| t.passed()).count(),
        "max_roll_deg": fold(|t| t.max_roll_deg, 0.0, f64::max),
        "max_pitch_deg": fold(|t| t.max_pitch_deg, 0.0, f64::max),
        "max_touch_n": fold(|t| t.max_touch_n, 0.0, f64::max),
        "min_clearance_m": fold(|t| t.clearance_m, f64::INFINITY, f64::min),
        "errors": trials.iter().filter_map(|t| t.error.clone().map(|e| format!("seed {}: {e}", t.seed))).collect::<Vec<_>>(),
    }))
}
