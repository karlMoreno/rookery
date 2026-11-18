//! Run logs that can be replayed (row 8.6).
//!
//! A live run records, per 20 ms block of physics, the fast-forward jump
//! before it and the actuator targets held through it (`targets.csv`), plus
//! what it ran (`meta.json`: seed, scene and its hash, tree, git sha, step
//! counts). That is enough to rebuild the whole run without the behavior
//! tree: the world is seeded, the physics is deterministic, and the tree
//! only ever reached the sim through the targets.
//!
//! ```text
//!   live:   tree ─▶ goals ─▶ crane ─▶ targets ─┐
//!                                              ├─▶ physics ─▶ world events ─▶ summary.json "events"
//!   replay: targets.csv ───────────────────────┘                   │
//!                                                diff, event by event ◀──┘
//! ```
//!
//! `rookery replay <run dir>` re-applies the targets step by step, steps the
//! world with the same jumps, and diffs the event log against the recorded
//! one: same kinds, cells and biological times, in the same order.

use std::fs;
use std::path::{Path, PathBuf};

use mast::SceneMeta;
use rookery_core::{CellId, LifecycleParams, World};
use serde::{Deserialize, Serialize};
use sim::{Sim, Targets};

use crate::bt_hooks::{Block, describe, harvest_lifted};
use crate::shells::ShellMover;

/// What a run was, as `meta.json` records it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RunMeta {
    pub seed: u64,
    pub scene: PathBuf,
    /// FNV-1a 64 of the scene file, so a replay against a changed scene is
    /// refused instead of silently diverging.
    pub scene_hash: String,
    pub tree: PathBuf,
    /// `git rev-parse HEAD` when the run was made, or `unknown`.
    pub git_sha: String,
    /// Physics steps per block, and the physics timestep in seconds.
    pub every: usize,
    pub dt_phys: f64,
}

impl RunMeta {
    pub fn new(
        seed: u64,
        scene: &Path,
        tree: &Path,
        every: usize,
        dt_phys: f64,
    ) -> Result<Self, String> {
        Ok(RunMeta {
            seed,
            scene: absolute(scene),
            scene_hash: hash_file(scene)?,
            tree: absolute(tree),
            git_sha: git_sha(),
            every,
            dt_phys,
        })
    }
}

/// Write `targets.csv` and `meta.json` into `dir`.
pub fn write_log(dir: &Path, meta: &RunMeta, blocks: &[Block]) -> Result<(), String> {
    let mut csv = String::from("ff,rail_y,mast_z,arm_x,hook_lift\n");
    for b in blocks {
        // `{}` prints the shortest text that parses back to the same f64.
        let [y, z, x, h] = b.targets;
        csv.push_str(&format!("{},{y},{z},{x},{h}\n", b.ff));
    }
    fs::write(dir.join("targets.csv"), csv).map_err(|e| e.to_string())?;
    let json = serde_json::to_string_pretty(meta).map_err(|e| e.to_string())? + "\n";
    fs::write(dir.join("meta.json"), json).map_err(|e| e.to_string())
}

/// What a replay found.
#[derive(Debug, Clone, PartialEq)]
pub struct ReplayReport {
    pub blocks: usize,
    pub recorded: usize,
    pub replayed: usize,
    /// The first event where the two logs part, or `None` when they agree.
    pub first_difference: Option<String>,
}

/// Replay the run logged in `dir` and diff its events against `summary.json`.
///
/// # Errors
/// Missing or malformed log files, or a scene whose hash no longer matches.
pub fn replay(dir: &Path) -> Result<ReplayReport, String> {
    let read = |f: &str| {
        fs::read_to_string(dir.join(f)).map_err(|e| format!("{}: {e}", dir.join(f).display()))
    };
    let meta: RunMeta =
        serde_json::from_str(&read("meta.json")?).map_err(|e| format!("meta.json: {e}"))?;
    if hash_file(&meta.scene)? != meta.scene_hash {
        return Err(format!(
            "{} changed since the run; replay needs the same scene",
            meta.scene.display()
        ));
    }
    let blocks = parse_targets(&read("targets.csv")?)?;
    let recorded = recorded_events(&read("summary.json")?)?;

    let scene_meta =
        SceneMeta::load(meta.scene.with_extension("json")).map_err(|e| e.to_string())?;
    let mut sim = Sim::load(&meta.scene).map_err(|e| e.to_string())?;
    sim.apply(&Targets(vec![0.0; 4]))
        .map_err(|e| e.to_string())?;
    let cells: Vec<CellId> = (0..scene_meta.rows * scene_meta.cols)
        .map(|i| CellId {
            tray: 0,
            r: (i / scene_meta.cols) as u8,
            c: (i % scene_meta.cols) as u8,
        })
        .collect();
    let mut world = World::new(LifecycleParams::default(), meta.seed, &cells);
    let mut mover = ShellMover::subscribe(
        &sim,
        &meta.scene.with_extension("json"),
        &scene_meta,
        &mut world,
    )?;

    // The same sequence of operations as BtHooks: jump, block, then the
    // tick's world step, shell moves and harvest check.
    let tick_dt = meta.every as f64 * meta.dt_phys;
    let (mut bio, mut events) = (0.0, Vec::new());
    for b in &blocks {
        if b.ff > 0.0 {
            let evs = world.step(b.ff);
            bio += b.ff;
            events.extend(evs.into_iter().map(|e| (bio, e)));
        }
        for _ in 0..meta.every {
            sim.apply(&Targets(b.targets.to_vec()))
                .map_err(|e| e.to_string())?;
            sim.step();
        }
        bio += tick_dt;
        let evs = world.step(tick_dt);
        events.extend(evs.into_iter().map(|e| (bio, e)));
        if let Some(m) = mover.as_mut() {
            m.apply(&mut sim, bio);
        }
        let lifted = harvest_lifted(&mut world, &sim.qpos());
        events.extend(lifted.into_iter().map(|e| (bio, e)));
    }

    let replayed: Vec<(f64, String, u8, u8)> = events
        .iter()
        .map(|&(t, e)| {
            let (k, c) = describe(e);
            (t, k.to_string(), c.r, c.c)
        })
        .collect();
    let first_difference = (0..recorded.len().max(replayed.len())).find_map(|i| {
        let (a, b) = (recorded.get(i), replayed.get(i));
        let same = match (a, b) {
            (Some(a), Some(b)) => {
                a.1 == b.1
                    && a.2 == b.2
                    && a.3 == b.3
                    && (a.0 - b.0).abs() <= 1e-6 * a.0.abs().max(1.0)
            }
            _ => false,
        };
        (!same).then(|| format!("event {i}: recorded {a:?}, replayed {b:?}"))
    });
    Ok(ReplayReport {
        blocks: blocks.len(),
        recorded: recorded.len(),
        replayed: replayed.len(),
        first_difference,
    })
}

fn parse_targets(text: &str) -> Result<Vec<Block>, String> {
    text.lines()
        .skip(1)
        .enumerate()
        .map(|(i, line)| {
            let v: Vec<f64> = line
                .split(',')
                .map(|s| {
                    s.parse::<f64>()
                        .map_err(|e| format!("targets.csv line {}: {e}", i + 2))
                })
                .collect::<Result<_, _>>()?;
            match v[..] {
                [ff, y, z, x, h] => Ok(Block {
                    ff,
                    targets: [y, z, x, h],
                }),
                _ => Err(format!("targets.csv line {}: expected 5 numbers", i + 2)),
            }
        })
        .collect()
}

fn recorded_events(summary: &str) -> Result<Vec<(f64, String, u8, u8)>, String> {
    let v: serde_json::Value =
        serde_json::from_str(summary).map_err(|e| format!("summary.json: {e}"))?;
    let events = v["events"].as_array().ok_or("summary.json has no events")?;
    events
        .iter()
        .map(|e| {
            Some((
                e["t"].as_f64()?,
                e["kind"].as_str()?.to_string(),
                u8::try_from(e["r"].as_u64()?).ok()?,
                u8::try_from(e["c"].as_u64()?).ok()?,
            ))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| "summary.json: a malformed event".to_string())
}

fn absolute(p: &Path) -> PathBuf {
    std::path::absolute(p).unwrap_or_else(|_| p.to_path_buf())
}

/// FNV-1a 64-bit of a file's bytes, as hex. Not cryptographic: it only has
/// to notice that a regenerated scene differs.
fn hash_file(p: &Path) -> Result<String, String> {
    let bytes = fs::read(p).map_err(|e| format!("{}: {e}", p.display()))?;
    let h = bytes.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3)
    });
    Ok(format!("{h:016x}"))
}

fn git_sha() -> String {
    std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".into())
}
