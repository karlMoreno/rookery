//! The shell mover (row 7.5): it puts a shed shell in its box on `Molted`
//! and parks it again on `ShellEaten`.
//!
//! Pattern: **Observer**. The mover subscribes to the World's
//! [`EventBus`](rookery_core::EventBus) instead of the hooks calling it, so
//! adding a listener (the run log, the tree's `events` key) never touches
//! the World. The bus calls its subscribers synchronously from
//! `World::step`, which has no `Sim`, so the subscriber only posts the event
//! to the mover's inbox. [`ShellMover::apply`] then moves the bodies at the
//! start of the next tick, before physics or the tree see the snapshot.
//!
//! ```text
//!   World::step ─▶ bus.publish(e) ─▶ subscriber: inbox.push(e)
//!                                          │ (shared Rc<RefCell<…>>)
//!   BtHooks::tick ─▶ mover.apply(sim) ◀────┘
//!        Molted(r, c)      gravity on,  set_body_pose(shell_r_c, in box r_c)
//!        ShellEaten(r, c)  gravity off, set_body_pose(shell_r_c, parked)
//!   next tick ─▶ mover.apply(sim) checks each new shell lies inside its box
//! ```
//!
//! Where things go comes from the scene itself: the parked pose is the
//! shell's pose when the scene loads, and the in-box pose is the box's
//! current pose plus the offset `rookery.json` gives (`shell_offset_y`
//! beside the crawfish, resting on the box floor).

use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;

use mast::SceneMeta;
use rookery_core::{CellId, Event, World};
use serde::Deserialize;
use sim::Sim;

/// Box floor thickness, metres. Copied from `scene/scene/builder.py`
/// (`BOX_FLOOR`); the test `a_moved_shell_rests_where_it_was_put` fails if
/// the two drift, because the shell would drop or sink when placed.
const BOX_FLOOR: f64 = 0.004;
/// Box wall thickness, metres, copied from the builder's `BOX_WALL`.
const BOX_WALL: f64 = 0.003;

/// The shell fields of `rookery.json` (row 7.4).
#[derive(Debug, Clone, Deserialize)]
struct ShellSpec {
    #[serde(default)]
    shells: bool,
    #[serde(default)]
    shell_length: f64,
    #[serde(default)]
    shell_radius: f64,
    #[serde(default)]
    shell_offset_y: f64,
}

/// The result of placing a shell: where it lay, a tick later, in its box's
/// frame, and whether that is inside the box.
#[derive(Debug, Clone, PartialEq)]
pub struct ShellCheck {
    pub cell: CellId,
    /// Biological time of the molt, seconds.
    pub t: f64,
    /// Shell centre minus box origin, in the box's frame, metres.
    pub offset: [f64; 3],
    pub inside: bool,
}

/// Moves shed shells between their park spots and their boxes.
pub struct ShellMover {
    inbox: Rc<RefCell<Vec<Event>>>,
    cols: usize,
    /// Each cell's parked pose (position, quaternion), row-major.
    parked: Vec<([f64; 3], [f64; 4])>,
    /// The shell's rest position in its box's frame.
    in_box: [f64; 3],
    /// Half the box's inside, along X and Y, and its height.
    inner_half: f64,
    box_height: f64,
    half_length: f64,
    radius: f64,
    /// Shells placed last tick, still to be checked: (cell, molt time).
    to_check: Vec<(CellId, f64)>,
    /// Every placement check so far, in order (the run log's `shells`).
    pub checks: Vec<ShellCheck>,
}

impl ShellMover {
    /// A mover for the scene loaded in `sim`, whose metadata is at
    /// `meta_path` (`rookery.json`), subscribed to `world`'s bus. `None` for
    /// a scene without shells (Phase 6 and earlier).
    pub fn subscribe(
        sim: &Sim,
        meta_path: &Path,
        meta: &SceneMeta,
        world: &mut World,
    ) -> Result<Option<ShellMover>, String> {
        let text = std::fs::read_to_string(meta_path)
            .map_err(|e| format!("{}: {e}", meta_path.display()))?;
        let spec: ShellSpec =
            serde_json::from_str(&text).map_err(|e| format!("{}: {e}", meta_path.display()))?;
        if !spec.shells {
            return Ok(None);
        }
        // The builder emits the free joints as boxes, crawfish, then shells,
        // each block row-major with 7 values per body, and the crane last.
        let q = sim.qpos();
        let cells = meta.rows * meta.cols;
        let mut parked = Vec::with_capacity(cells);
        for k in 0..cells {
            let name = shell_name(k / meta.cols, k % meta.cols);
            let i = 7 * (2 * cells + k);
            let b = q.get(i..i + 7).filter(|_| sim.body_index(&name).is_some());
            // A parked shell is below the ground; anything else means the
            // layout is not what this code assumes.
            let Some(b) = b.filter(|b| b[2] < -0.05) else {
                return Err(format!(
                    "{}: cannot find {name}'s parked pose in qpos",
                    meta_path.display()
                ));
            };
            parked.push(([b[0], b[1], b[2]], [b[3], b[4], b[5], b[6]]));
        }
        let inbox: Rc<RefCell<Vec<Event>>> = Rc::default();
        let tx = Rc::clone(&inbox);
        world.bus.subscribe(Box::new(move |e, _tick| {
            if matches!(e, Event::Molted(_) | Event::ShellEaten(_)) {
                tx.borrow_mut().push(*e);
            }
        }));
        Ok(Some(ShellMover {
            inbox,
            cols: meta.cols,
            parked,
            in_box: [0.0, spec.shell_offset_y, BOX_FLOOR + spec.shell_radius],
            inner_half: meta.box_size[0].min(meta.box_size[1]) / 2.0 - BOX_WALL,
            box_height: meta.box_size[2],
            half_length: spec.shell_length / 2.0,
            radius: spec.shell_radius,
            to_check: Vec::new(),
            checks: Vec::new(),
        }))
    }

    /// Check last tick's placements, then act on every event posted since.
    /// `bio_time` stamps the checks. Returns errors as `shells: …` strings.
    pub fn apply(&mut self, sim: &mut Sim, bio_time: f64) -> Vec<String> {
        let mut errors = Vec::new();
        let snap = sim.snapshot();
        for (cell, t) in std::mem::take(&mut self.to_check) {
            let (Some(s), Some(b)) = (
                sim.body_index(&shell_name(cell.r.into(), cell.c.into())),
                sim.body_index(&box_name(cell)),
            ) else {
                continue;
            };
            let (s, b) = (&snap.bodies[s], &snap.bodies[b]);
            let d = rotate_inv(b.quat, sub(s.pos, b.pos));
            let tol = 1e-3;
            let inside = d[0].abs() + self.half_length <= self.inner_half + tol
                && d[1].abs() + self.radius <= self.inner_half + tol
                && d[2] >= -tol
                && d[2] <= self.box_height + tol;
            self.checks.push(ShellCheck {
                cell,
                t,
                offset: d,
                inside,
            });
        }

        let events: Vec<Event> = std::mem::take(&mut *self.inbox.borrow_mut());
        for e in events {
            let (cell, into_box) = match e {
                Event::Molted(c) => (c, true),
                Event::ShellEaten(c) => (c, false),
                _ => continue,
            };
            let (r, c) = (usize::from(cell.r), usize::from(cell.c));
            let name = shell_name(r, c);
            let result = if into_box {
                // Pose from the box's current pose: snapshot xpos lags qpos by
                // a step, so read the box's free joint from qpos.
                match sim
                    .body_index(&box_name(cell))
                    .and_then(|_| self.box_pose(sim, cell))
                {
                    Some((p, q)) => {
                        let pos = add(p, rotate(q, self.in_box));
                        sim.set_gravcomp(&name, 0.0)
                            .and_then(|()| sim.set_body_pose(&name, pos, q))
                            .map(|()| self.to_check.push((cell, bio_time)))
                    }
                    None => {
                        errors.push(format!("shells: no box for {name}"));
                        continue;
                    }
                }
            } else {
                let (pos, q) = self.parked[r * self.cols + c];
                sim.set_gravcomp(&name, 1.0)
                    .and_then(|()| sim.set_body_pose(&name, pos, q))
            };
            if let Err(err) = result {
                errors.push(format!("shells: {name}: {err}"));
            }
        }
        errors
    }

    /// The box's position and orientation, read from its free joint.
    fn box_pose(&self, sim: &Sim, cell: CellId) -> Option<([f64; 3], [f64; 4])> {
        // Boxes come first in qpos, row-major, 7 values each.
        let i = 7 * (usize::from(cell.r) * self.cols + usize::from(cell.c));
        let q = sim.qpos();
        let b = q.get(i..i + 7)?;
        Some(([b[0], b[1], b[2]], [b[3], b[4], b[5], b[6]]))
    }
}

fn shell_name(r: usize, c: usize) -> String {
    format!("shell_{r}_{c}")
}

fn box_name(cell: CellId) -> String {
    format!("box_{}_{}", cell.r, cell.c)
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// Rotate `v` by the unit quaternion `q = [w, x, y, z]`.
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    // v' = v + 2w (u × v) + 2 u × (u × v), with u = (x, y, z).
    let u = [x, y, z];
    let t = cross(u, v).map(|k| 2.0 * k);
    let ut = cross(u, t);
    [
        v[0] + w * t[0] + ut[0],
        v[1] + w * t[1] + ut[1],
        v[2] + w * t[2] + ut[2],
    ]
}

/// Rotate `v` by the inverse of the unit quaternion `q`.
fn rotate_inv(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    rotate([q[0], -q[1], -q[2], -q[3]], v)
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn scene() -> PathBuf {
        PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../scene/generated/rookery.xml"
        ))
    }

    fn setup() -> (Sim, World, ShellMover) {
        let path = scene();
        let meta = SceneMeta::load(path.with_extension("json")).expect("meta");
        let sim = Sim::load(&path).expect("scene loads");
        let cells: Vec<CellId> = (0..16)
            .map(|i| CellId {
                tray: 0,
                r: i / 4,
                c: i % 4,
            })
            .collect();
        let mut world = World::new(rookery_core::LifecycleParams::default(), 1, &cells);
        let mover = ShellMover::subscribe(&sim, &path.with_extension("json"), &meta, &mut world)
            .expect("reads the scene")
            .expect("the scene has shells");
        (sim, world, mover)
    }

    fn run(sim: &mut Sim, steps: usize) {
        for _ in 0..steps {
            sim.step();
        }
    }

    fn pos(sim: &Sim, name: &str) -> [f64; 3] {
        sim.snapshot().bodies[sim.body_index(name).unwrap()].pos
    }

    #[test]
    fn a_moved_shell_rests_where_it_was_put_and_parks_again() {
        let (mut sim, mut world, mut mover) = setup();
        run(&mut sim, 200);
        let parked = pos(&sim, "shell_1_2");
        let cell = CellId {
            tray: 0,
            r: 1,
            c: 2,
        };
        world.bus.publish(1, Event::Molted(cell));
        world.bus.publish(1, Event::WindowClosing(cell)); // ignored
        assert!(mover.apply(&mut sim, 100.0).is_empty());
        run(&mut sim, 2);
        let placed = pos(&sim, "shell_1_2");
        run(&mut sim, 1_000);
        let rest = pos(&sim, "shell_1_2");
        // BOX_FLOOR matches the builder: the shell neither drops nor sinks.
        for k in 0..3 {
            assert!(
                (rest[k] - placed[k]).abs() < 1e-3,
                "settled from {placed:?} to {rest:?}"
            );
        }
        assert!(mover.apply(&mut sim, 100.02).is_empty());
        let [check] = &mover.checks[..] else {
            panic!("{:?}", mover.checks)
        };
        assert!(
            check.inside && check.cell == cell && check.t == 100.0,
            "{check:?}"
        );
        assert!(
            (check.offset[1] - 0.03).abs() < 1e-3,
            "beside the crawfish: {check:?}"
        );

        world.bus.publish(2, Event::ShellEaten(cell));
        assert!(mover.apply(&mut sim, 200.0).is_empty());
        run(&mut sim, 1_000);
        let back = pos(&sim, "shell_1_2");
        for k in 0..3 {
            assert!(
                (back[k] - parked[k]).abs() < 3e-3,
                "parked at {back:?}, started at {parked:?}"
            );
        }
    }

    #[test]
    fn a_shell_follows_a_box_that_has_moved() {
        let (mut sim, mut world, mut mover) = setup();
        run(&mut sim, 20);
        // Slide box (0, 0) 2 cm along +X and turn it a quarter turn.
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let b = pos(&sim, "box_0_0");
        sim.set_body_pose("box_0_0", [b[0] + 0.02, b[1], b[2]], [h, 0.0, 0.0, h])
            .unwrap();
        world.bus.publish(
            1,
            Event::Molted(CellId {
                tray: 0,
                r: 0,
                c: 0,
            }),
        );
        mover.apply(&mut sim, 0.0);
        run(&mut sim, 20);
        mover.apply(&mut sim, 0.0);
        assert!(mover.checks[0].inside, "{:?}", mover.checks[0]);
        let s = pos(&sim, "shell_0_0");
        // The +Y offset turned with the box: it now points along -X.
        assert!(
            (s[0] - (b[0] + 0.02 - 0.03)).abs() < 2e-3,
            "shell at {s:?}, box at {b:?}"
        );
    }

    #[test]
    fn rotate_turns_x_into_y_about_z_and_back() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let q = [h, 0.0, 0.0, h]; // a quarter turn about Z
        let v = rotate(q, [1.0, 0.0, 0.0]);
        assert!((v[0]).abs() < 1e-12 && (v[1] - 1.0).abs() < 1e-12, "{v:?}");
        let back = rotate_inv(q, v);
        assert!(
            (back[0] - 1.0).abs() < 1e-12 && back[1].abs() < 1e-12,
            "{back:?}"
        );
    }
}
