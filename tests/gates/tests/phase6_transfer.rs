//! Phase 6 gate: 50 transfers, and undo after a fault.
//!
//! Seed `s` transfers cell `s mod cells` (row-major), so 50 seeds cover all
//! 16 cells three times or more. (Until #61 it was 100 seeds with an
//! xorshift pick, which repeated cells and left some out; trimmed to save
//! CI time, with Karl's approval.) The crane
//! starts at that cell's pre-engage pose (`Sim::set_qpos`, so no trip from
//! home), runs `Transfer::build(...)`, and settles. Every seed must meet:
//!   1. once the box has been lifted (underside ≥ water + 1 cm), it never
//!      dips below water + 1 cm again;
//!   2. it ends inside the exit-tray footprint, level within 5°;
//!   3. the crane ends at `home()`, every measured joint within `DONE_TOL`.
//!
//! Fault case, on every fourth seed (13 runs): fault `k` uses cell
//! `5k mod cells`, 13 different cells; abort when the transfer has finished a
//! random number of its steps, run its `undo()`, and the box must be back in
//! its own cell within 5 mm, resting on the tray floor.
//!
//! Seeds run on threads, one `Sim` per thread, so the gate fits CI. A
//! failure names its seed.
//!
//! ```text
//!   seed ─▶ (r, c) ─▶ set_qpos(pre-engage) ─▶ Transfer ─▶ settle
//!      every tick: lifted ⇒ above water + 1 cm   end: in footprint, level, crane home
//!   seed % 4 == 0 ─▶ abort after k steps ─▶ undo ─▶ box back in (r, c) ± 5 mm
//! ```
//!
//! Red on `main` from row 6.1 (`Transfer::build` is `todo!()`). Green once
//! rows 6.2–6.6 land.

use mast::kinematics::shelf_windows;
use mast::route::PRE;
use mast::{
    Axis, CmdStatus, Command, DONE_TOL, GantryObs, GantryState, SceneMeta, Transfer, cell_pose,
    home,
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
const SEEDS: u64 = 50;
const FAULT_EVERY: u64 = 4;
const WATER_CLEARANCE: f64 = 0.01;
const MAX_TILT_DEG: f64 = 5.0;
const HOME_TOL: f64 = DONE_TOL;
const BACK_IN_CELL: f64 = 0.005;
const TICK_EVERY: usize = 20; // 1 kHz physics, 50 Hz commands
const SETTLE_TICKS: usize = 50; // 1 s
const MAX_TICKS: usize = 5_000; // 100 s per transfer: far more than needed

/// xorshift64*: deterministic, no dependency.
struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Rng(0xD1B5_4A32_D192_ED03 ^ (seed + 1))
    }
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
}

struct World {
    sim: Sim,
    st: GantryState,
    meta: SceneMeta,
    ids: [usize; 4],
    box_: usize,
}

impl World {
    fn new(meta: &SceneMeta, r: usize, c: usize) -> Result<Self, String> {
        let mut sim = Sim::load(SCENE).map_err(|e| e.to_string())?;
        let idx = |n: &str| sim.body_index(n).ok_or(format!("no body {n}"));
        let ids = [
            idx("crane_truck")?,
            idx("crane_carriage")?,
            idx("crane_arm")?,
            idx("crane_hook")?,
        ];
        let box_ = idx(&format!("box_{r}_{c}"))?;
        let mut pre = cell_pose(r, c, meta).ok_or("cell outside the grid")?;
        pre.0[Axis::ArmX.index()] -= PRE;
        let mut q = sim.qpos();
        let n = q.len();
        q[n - mast::kinematics::CRANE_NQ..].copy_from_slice(&mast::kinematics::crane_qpos(&pre));
        sim.set_qpos(&q).map_err(|e| e.to_string())?;
        let st = GantryState {
            targets: pre,
            windows: shelf_windows(meta),
        };
        Ok(World {
            sim,
            st,
            meta: meta.clone(),
            ids,
            box_,
        })
    }

    fn obs(&self) -> GantryObs {
        let b = self.sim.snapshot().bodies;
        let [t, ca, a, h] = self.ids;
        GantryObs {
            q: [
                b[t].pos[1],
                b[ca].pos[2],
                b[a].pos[0] - self.meta.derived.rail_line_x,
                b[h].pos[2] - b[ca].pos[2],
            ],
            ee_contact: 0.0,
        }
    }

    fn physics_tick(&mut self) -> Result<(), String> {
        for _ in 0..TICK_EVERY {
            self.sim
                .apply(&Targets(self.st.targets.0.to_vec()))
                .map_err(|e| e.to_string())?;
            self.sim.step();
        }
        Ok(())
    }

    fn box_bottom(&self) -> f64 {
        self.sim.snapshot().bodies[self.box_].pos[2]
    }

    /// Tick `cmd` until it finishes or `stop(&cmd)` says to stop early.
    /// `watch` runs after every tick.
    fn play(
        &mut self,
        cmd: &mut dyn Command,
        mut stop: impl FnMut(&dyn Command) -> bool,
        mut watch: impl FnMut(&World) -> Result<(), String>,
    ) -> Result<bool, String> {
        let dt = TICK_EVERY as f64 * self.sim.timestep();
        for _ in 0..MAX_TICKS {
            self.physics_tick()?;
            watch(self)?;
            if stop(cmd) {
                return Ok(false);
            }
            let obs = self.obs();
            match cmd.tick(&mut self.st, &obs, dt) {
                CmdStatus::Running => {}
                CmdStatus::Done => return Ok(true),
                CmdStatus::Failed(e) => return Err(format!("command failed: {e}")),
            }
        }
        Err("command did not finish in 100 s".into())
    }

    fn settle(&mut self) -> Result<(), String> {
        for _ in 0..SETTLE_TICKS {
            self.physics_tick()?;
        }
        Ok(())
    }
}

/// Cell number `k`, row-major, wrapping.
fn cell(k: u64, meta: &SceneMeta) -> (usize, usize) {
    let k = (k % (meta.rows * meta.cols) as u64) as usize;
    (k / meta.cols, k % meta.cols)
}

/// The normal transfer: lift, carry, place, go home.
fn transfer(seed: u64, meta: &SceneMeta) -> Result<(), String> {
    let (r, c) = cell(seed, meta);
    let mut w = World::new(meta, r, c)?;
    let from = w.st.targets;
    let mut t = Transfer::build(from, r, c, meta).map_err(|e| e.to_string())?;
    let water = meta.water_depth;
    let mut lifted = false;
    w.play(
        &mut t,
        |_| false,
        |w| {
            let z = w.box_bottom();
            if z >= water + WATER_CLEARANCE {
                lifted = true;
            } else if lifted {
                return Err(format!(
                    "box {r},{c} dipped to {z:.4} m after the lift (water {water:.3})"
                ));
            }
            Ok(())
        },
    )
    .map_err(|e| format!("cell ({r},{c}): {e}"))?;
    w.settle()?;

    let s = w.sim.snapshot();
    let b = &s.bodies[w.box_];
    let [ex, ey, ez] = meta.derived.exit_pose;
    let room = meta.cell_pitch / 2.0 - meta.box_size[0].max(meta.box_size[1]) / 2.0;
    if (b.pos[0] - ex).abs() > room || (b.pos[1] - ey).abs() > room {
        return Err(format!(
            "cell ({r},{c}): box at ({:.4}, {:.4}), outside the exit footprint ±{room:.3} of ({ex:.3}, {ey:.3})",
            b.pos[0], b.pos[1]
        ));
    }
    if (b.pos[2] - ez).abs() > 0.01 {
        return Err(format!(
            "cell ({r},{c}): box underside at {:.4}, not on the exit tray ({ez:.3})",
            b.pos[2]
        ));
    }
    let (roll, pitch) = (b.roll.to_degrees(), b.pitch.to_degrees());
    if roll.abs() >= MAX_TILT_DEG || pitch.abs() >= MAX_TILT_DEG {
        return Err(format!(
            "cell ({r},{c}): box not level: roll {roll:.2}°, pitch {pitch:.2}°"
        ));
    }
    let q = w.obs().q;
    let h = home().0;
    if q.iter().zip(h).any(|(a, b)| (a - b).abs() > HOME_TOL) {
        return Err(format!("cell ({r},{c}): crane not home: {q:?}"));
    }
    Ok(())
}

/// The fault case: abort after k completed steps, undo, box back in its cell.
fn fault(seed: u64, meta: &SceneMeta) -> Result<(), String> {
    let mut rng = Rng::new(seed);
    // 5 is coprime with the 16 cells, so the 13 faults land on 13 cells.
    let (r, c) = cell(seed / FAULT_EVERY * 5, meta);
    let mut w = World::new(meta, r, c)?;
    let start = w.sim.snapshot().bodies[w.box_].pos;
    let mut t = Transfer::build(w.st.targets, r, c, meta).map_err(|e| e.to_string())?;
    // Abort after k of the n steps, 1 ≤ k < n: always mid-transfer.
    let n = t.steps();
    let k = 1 + rng.below(n.saturating_sub(1) as u64) as usize;
    let dt = TICK_EVERY as f64 * w.sim.timestep();
    let mut ticks = 0;
    while t.completed() < k {
        if ticks == MAX_TICKS {
            return Err(format!("cell ({r},{c}): did not reach step {k} of {n}"));
        }
        ticks += 1;
        w.physics_tick()?;
        let obs = w.obs();
        if let CmdStatus::Failed(e) = t.tick(&mut w.st, &obs, dt) {
            return Err(format!(
                "cell ({r},{c}): failed before the fault at step {k}: {e}"
            ));
        }
    }
    let mut back = t.undo();
    w.play(back.as_mut(), |_| false, |_| Ok(()))
        .map_err(|e| format!("cell ({r},{c}): undo after step {k} of {n}: {e}"))?;
    w.settle()?;
    let p = w.sim.snapshot().bodies[w.box_].pos;
    let off = ((p[0] - start[0]).powi(2) + (p[1] - start[1]).powi(2)).sqrt();
    if off > BACK_IN_CELL || (p[2] - start[2]).abs() > BACK_IN_CELL {
        return Err(format!(
            "cell ({r},{c}): after a fault at step {k} of {n} and undo, the box is {:.1} mm from its cell (z {:.4} vs {:.4})",
            off * 1000.0,
            p[2],
            start[2]
        ));
    }
    Ok(())
}

#[test]
fn one_hundred_transfers_and_undo_after_a_fault() {
    let meta = SceneMeta::load(META).expect("rookery.json");
    let threads = std::thread::available_parallelism()
        .map_or(2, |n| n.get())
        .min(8) as u64;
    let failures: Vec<String> = std::thread::scope(|s| {
        let handles: Vec<_> = (0..threads)
            .map(|k| {
                let meta = &meta;
                s.spawn(move || {
                    let mut out = Vec::new();
                    for seed in (k..SEEDS).step_by(threads as usize) {
                        if let Err(e) = transfer(seed, meta) {
                            out.push(format!("seed {seed}: {e}"));
                        }
                        if seed % FAULT_EVERY == 0
                            && let Err(e) = fault(seed, meta)
                        {
                            out.push(format!("seed {seed} (fault): {e}"));
                        }
                    }
                    out
                })
            })
            .collect();
        handles
            .into_iter()
            .flat_map(|h| {
                h.join()
                    .unwrap_or_else(|_| vec!["a worker thread panicked".into()])
            })
            .collect()
    });
    assert!(
        failures.is_empty(),
        "{} failure(s):\n{}",
        failures.len(),
        failures.join("\n")
    );
}
