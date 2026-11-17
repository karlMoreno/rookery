//! The safe [`Sim`] facade over the raw bindings in [`crate::ffi`] (row 1.6).
//!
//! Every `unsafe` block in Rookery lives in this file. Each one carries a
//! `// SAFETY:` comment naming the invariant it relies on. All of them reduce
//! to the three invariants that [`Sim::load`] establishes and that nothing
//! after it can break:
//!
//! 1. **Non-null.** `model` and `data` are non-null from a successful
//!    `load` until `Drop`. `load` returns `Err` rather than build a `Sim`
//!    around a null pointer.
//! 2. **Paired.** `data` was made by `mj_makeData(model)`, so every array in
//!    `data` is sized from `model` (`xpos` is `nbody * 3`, `ctrl` is `nu`).
//! 3. **Exclusive.** The pointers are private and never handed out, so `&self`
//!    and `&mut self` borrows of `Sim` are the only access MuJoCo's memory
//!    gets. `Sim` is neither `Send` nor `Sync` (raw pointers), so it never
//!    crosses threads.
//!
//! Patterns:
//! - **Facade**: callers see `load / step / snapshot / apply / body_index`
//!   and plain Rust values. They never see a MuJoCo type.
//! - **RAII** (Resource Acquisition Is Initialisation): `load` acquires
//!   `mjModel` and `mjData`, and `Drop` releases them in reverse order.
//!   Why: no caller can leak or double-free them, and `?` early returns stay
//!   correct for free.
//!
//! ```text
//!   ┌─────────────────────────────────────┐
//!   │ «facade» Sim                        │
//!   ├─────────────────────────────────────┤
//!   │ - model: *mut mjModel   (owned)     │──mj_loadXML──▶ C heap
//!   │ - data:  *mut mjData    (owned)     │──mj_makeData─▶ C heap
//!   │ - body_names: Vec<String>           │  copied once at load
//!   ├─────────────────────────────────────┤
//!   │ + load(path) -> Result<Sim>         │
//!   │ + step(&mut self)                   │──▶ mj_step
//!   │ + snapshot(&self) -> Snapshot       │──▶ reads time, xpos, xquat
//!   │ + apply(&mut self, &Targets)        │──▶ writes ctrl
//!   │ + body_index(&self, name)           │    (no FFI)
//!   │ + timestep(&self) -> f64            │──▶ reads opt.timestep
//!   │ + qpos(&self) -> Vec<f64>           │──▶ copies qpos
//!   │ + set_body_pose(&mut self, name, …) │──▶ writes one free joint, mj_forward
//!   │ + set_gravcomp(&mut self, name, g)  │──▶ writes body_gravcomp (model)
//!   │ ~ drop: mj_deleteData, mj_deleteModel
//!   └─────────────────────────────────────┘
//! ```

use std::ffi::{CStr, CString, c_char, c_int};
use std::path::Path;
use std::ptr;

use crate::ffi;
use crate::{BodyState, SimError, Snapshot, Targets};

/// Size of the buffer `mj_loadXML` writes its error message into. MuJoCo's
/// own samples use 1000 bytes.
const ERROR_BUF_LEN: usize = 1000;

/// `mjJNT_FREE` from MuJoCo's `mjtJoint` enum (`mjmodel.h`): a free joint,
/// 7 qpos values (position, then quaternion `[w, x, y, z]`) and 6 dofs.
/// The enum itself is outside the bindgen allowlist, so its one value
/// Rookery needs is copied here.
const MJ_JNT_FREE: c_int = 0;

/// The Facade itself: one loaded MuJoCo model plus its simulation data.
///
/// Fields are private so no caller can reach the raw pointers (invariant 3
/// in the module docs).
#[derive(Debug)]
pub struct Sim {
    model: *mut ffi::mjModel,
    data: *mut ffi::mjData,
    /// `body_names[id]` is the name of MuJoCo body `id` (`""` if unnamed).
    /// Copied once at load so `body_index` and `snapshot` need no FFI call
    /// to find names.
    body_names: Vec<String>,
}

impl Sim {
    /// Load an MJCF file and allocate its data (`mj_loadXML` + `mj_makeData`).
    ///
    /// # Errors
    /// [`SimError::Load`] when the path is not valid UTF-8 or contains a NUL,
    /// when the file is missing, or when MuJoCo rejects it. The message is
    /// MuJoCo's own error text where there is one.
    pub fn load(path: impl AsRef<Path>) -> Result<Sim, SimError> {
        let path = path.as_ref();
        let c_path = path
            .to_str()
            .and_then(|s| CString::new(s).ok())
            .ok_or_else(|| {
                SimError::Load(format!(
                    "{}: path must be UTF-8 without NUL bytes",
                    path.display()
                ))
            })?;

        let mut error = [0 as c_char; ERROR_BUF_LEN];
        // SAFETY: `c_path` is a valid NUL-terminated string that outlives
        // the call. A null `vfs` means "read from the file system", which the
        // API allows. `error` is a writable buffer of exactly `error_sz` bytes
        // (ERROR_BUF_LEN fits a c_int), and MuJoCo writes at most that many,
        // NUL included.
        let model = unsafe {
            ffi::mj_loadXML(
                c_path.as_ptr(),
                ptr::null(),
                error.as_mut_ptr(),
                ERROR_BUF_LEN as c_int,
            )
        };
        if model.is_null() {
            // SAFETY: `error` was zero-filled above and MuJoCo NUL-terminates
            // whatever it writes inside the buffer, so a NUL exists within
            // `error`'s bounds.
            let msg = unsafe { CStr::from_ptr(error.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            return Err(SimError::Load(format!("{}: {msg}", path.display())));
        }

        // SAFETY: `model` is non-null and was just returned by `mj_loadXML`,
        // so it is a complete, valid `mjModel`.
        let data = unsafe { ffi::mj_makeData(model) };
        if data.is_null() {
            // SAFETY: `model` is non-null, came from `mj_loadXML`, and no
            // `Sim` owns it yet, so this is its only release.
            unsafe { ffi::mj_deleteModel(model) };
            return Err(SimError::Load(format!(
                "{}: mj_makeData could not allocate",
                path.display()
            )));
        }

        // From here on `Drop` owns the cleanup, including the case where
        // reading names below goes wrong.
        let mut sim = Sim {
            model,
            data,
            body_names: Vec::new(),
        };
        sim.body_names = (0..sim.nbody()).map(|id| sim.read_body_name(id)).collect();
        Ok(sim)
    }

    /// Advance the physics by one `timestep` (`mj_step`).
    pub fn step(&mut self) {
        // SAFETY: invariants 1 and 2. `model` and `data` are non-null and
        // paired. `&mut self` gives exclusive access to `data`, which both
        // calls mutate. `mj_rnePostConstraint` fills `cfrc_ext` (the external
        // contact forces) from the state `mj_step` just computed. MuJoCo
        // skips it by default, and the snapshot's `contact_force` needs it
        // (row 5.3).
        unsafe {
            ffi::mj_step(self.model, self.data);
            ffi::mj_rnePostConstraint(self.model, self.data);
        }
    }

    /// The first value of the sensor called `name` (`sensordata` at its
    /// `sensor_adr`), or `None` if the model has no such sensor.
    pub fn sensor(&self, name: &str) -> Option<f64> {
        let c_name = CString::new(name).ok()?;
        // SAFETY: invariant 1, so `model` is non-null. `c_name` is
        // NUL-terminated and outlives the call. `mjOBJ_SENSOR` is a valid
        // object type. The result is -1 or a valid sensor id.
        let id = unsafe {
            ffi::mj_name2id(
                self.model,
                ffi::mjtObj_mjOBJ_SENSOR as c_int,
                c_name.as_ptr(),
            )
        };
        let id = usize::try_from(id).ok()?;
        // SAFETY: invariants 1 and 2. `id < nsensor` (mj_name2id returned it),
        // so `sensor_adr[id]` is in bounds. Its value indexes `sensordata`,
        // which holds `nsensordata` values, and every sensor's address is
        // below that by MuJoCo's construction. Both are read once and copied.
        unsafe {
            let m = &*self.model;
            let adr = usize::try_from(*m.sensor_adr.add(id)).ok()?;
            let n = usize::try_from(m.nsensordata).ok()?;
            if adr >= n {
                return None;
            }
            Some(*(*self.data).sensordata.add(adr))
        }
    }

    /// Copy `time`, `xpos` and `xquat` for every body into a [`Snapshot`].
    pub fn snapshot(&self) -> Snapshot {
        let nbody = self.body_names.len();
        // SAFETY: invariant 1, so `data` is non-null and readable. Invariant
        // 2 means `xpos` holds `nbody * 3` and `xquat` holds `nbody * 4`
        // `mjtNum`s (MuJoCo allocates them that size in `mj_makeData`).
        // `body_names.len() == nbody`, set at load. `&self` means no `step`
        // runs while these borrowed slices are alive, and they are copied out
        // before this function returns.
        // `cfrc_ext` holds `nbody * 6` values (torque, then force) by the same
        // invariant 2.
        let (time, xpos, xquat, cfrc) = unsafe {
            let d = &*self.data;
            (
                d.time,
                std::slice::from_raw_parts(d.xpos, nbody * 3),
                std::slice::from_raw_parts(d.xquat, nbody * 4),
                std::slice::from_raw_parts(d.cfrc_ext, nbody * 6),
            )
        };

        let bodies = self
            .body_names
            .iter()
            .enumerate()
            .map(|(i, name)| {
                let quat = [
                    xquat[4 * i],
                    xquat[4 * i + 1],
                    xquat[4 * i + 2],
                    xquat[4 * i + 3],
                ];
                let f = &cfrc[6 * i + 3..6 * i + 6];
                let (roll, pitch) = crate::roll_pitch(quat);
                BodyState {
                    name: name.clone(),
                    pos: [xpos[3 * i], xpos[3 * i + 1], xpos[3 * i + 2]],
                    quat,
                    contact_force: (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt(),
                    roll,
                    pitch,
                }
            })
            .collect();

        Snapshot { time, bodies }
    }

    /// Write `targets` to `mjData.ctrl` for the next step.
    ///
    /// # Errors
    /// [`SimError::TargetLen`] when `targets.0.len() != nu`. `ctrl` is left
    /// unchanged in that case.
    pub fn apply(&mut self, targets: &Targets) -> Result<(), SimError> {
        let nu = self.nu();
        if targets.0.len() != nu {
            return Err(SimError::TargetLen {
                expected: nu,
                got: targets.0.len(),
            });
        }
        if nu == 0 {
            // `ctrl` may be null when the model has no actuators, and
            // `from_raw_parts_mut` needs a non-null pointer even for length 0.
            return Ok(());
        }
        // SAFETY: invariants 1 and 2. `ctrl` is a non-null array of exactly
        // `nu` `mjtNum`s when `nu > 0`, checked just above. `&mut self` means
        // nothing else reads or writes `data` while the slice is alive.
        let ctrl = unsafe { std::slice::from_raw_parts_mut((*self.data).ctrl, nu) };
        ctrl.copy_from_slice(&targets.0);
        Ok(())
    }

    /// Overwrite the generalized positions `qpos` and recompute every derived
    /// quantity (`mj_forward`), without stepping time. Gates use it to start
    /// a trial from a chosen pose (row 5.4).
    ///
    /// # Errors
    /// [`SimError::QposLen`] when `qpos.len() != nq`, and nothing changes.
    pub fn set_qpos(&mut self, qpos: &[f64]) -> Result<(), SimError> {
        // SAFETY: invariant 1, so `model` is non-null and valid.
        let nq = usize::try_from(unsafe { (*self.model).nq }).unwrap_or(0);
        if qpos.len() != nq {
            return Err(SimError::QposLen {
                expected: nq,
                got: qpos.len(),
            });
        }
        if nq > 0 {
            // SAFETY: invariants 1 and 2. `qpos` is a non-null array of
            // exactly `nq` mjtNums when nq > 0, the length was checked above,
            // and `&mut self` is exclusive while the slice is alive.
            let q = unsafe { std::slice::from_raw_parts_mut((*self.data).qpos, nq) };
            q.copy_from_slice(qpos);
        }
        // SAFETY: invariants 1 and 2 (non-null, paired). `&mut self` is
        // exclusive, and mj_forward only writes `data`.
        unsafe { ffi::mj_forward(self.model, self.data) };
        Ok(())
    }

    /// Set how much of the body called `name`'s weight is cancelled:
    /// `body_gravcomp`, 0 for normal gravity and 1 to float. Row 7.5: a parked
    /// shell floats so it touches nothing, and gets its weight back when the
    /// shell mover puts it in a box. Takes effect from the next step.
    ///
    /// This is the one place `Sim` writes the model rather than the data.
    /// MuJoCo reads `body_gravcomp` on every step, but only computes gravity
    /// compensation at all when the model was *compiled* with some body
    /// using it (`ngravcomp > 0`): changing that count at runtime is not
    /// enough (found writing this). Rookery's scene always has some (the
    /// crane, the parked shells), and a model without any is refused.
    ///
    /// # Errors
    /// [`SimError::UnknownBody`] for a name the model lacks,
    /// [`SimError::BadGravcomp`] for a non-finite value, and
    /// [`SimError::NoGravcomp`] for a model compiled without gravity
    /// compensation. Nothing changes then.
    pub fn set_gravcomp(&mut self, name: &str, gravcomp: f64) -> Result<(), SimError> {
        if !gravcomp.is_finite() {
            return Err(SimError::BadGravcomp(gravcomp));
        }
        let id = self
            .body_index(name)
            .ok_or_else(|| SimError::UnknownBody(name.to_owned()))?;
        // SAFETY: invariant 1, so `model` is non-null and valid. Reading one
        // field does not mutate it.
        if unsafe { (*self.model).ngravcomp } == 0 {
            return Err(SimError::NoGravcomp);
        }
        // SAFETY: invariants 1 and 3. `model` is non-null and valid, and
        // `&mut self` makes this the only access to it. `body_gravcomp` holds
        // `nbody` mjtNums and `id < nbody`, because `body_index` searched
        // `body_names`, which has exactly `nbody` entries (set at load).
        unsafe { *(*self.model).body_gravcomp.add(id) = gravcomp };
        Ok(())
    }

    /// Place the body called `name` at world position `pos` with orientation
    /// `quat` (`[w, x, y, z]`, normalised here) and stop it: its free
    /// joint's 7 `qpos` values are overwritten, its 6 `qvel` values zeroed,
    /// then `mj_forward` recomputes everything derived. Time does not move.
    ///
    /// Row 7.4: the lifecycle moves a shed shell into its box after a molt
    /// and parks it again once it is eaten. A free joint's qpos is in world
    /// coordinates because MuJoCo only allows one on a top-level body.
    ///
    /// # Errors
    /// [`SimError::UnknownBody`] for a name the model lacks,
    /// [`SimError::NotFree`] unless the body has exactly one joint and it is
    /// free, and [`SimError::BadQuat`] for a zero or non-finite quaternion.
    /// On any error nothing changes.
    pub fn set_body_pose(
        &mut self,
        name: &str,
        pos: [f64; 3],
        quat: [f64; 4],
    ) -> Result<(), SimError> {
        let norm = quat.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !(norm.is_finite() && norm > 0.0) || pos.iter().any(|v| !v.is_finite()) {
            return Err(SimError::BadQuat(quat));
        }
        let id = self
            .body_index(name)
            .ok_or_else(|| SimError::UnknownBody(name.to_owned()))?;
        // SAFETY: invariant 1, so `model` is non-null and valid. `id < nbody`
        // because `body_index` searched `body_names`, which has exactly
        // `nbody` entries (set at load), and `body_jntnum` / `body_jntadr`
        // are `nbody` ints each. Reading does not mutate the model.
        let (jntnum, jnt) = unsafe {
            (
                *(*self.model).body_jntnum.add(id),
                *(*self.model).body_jntadr.add(id),
            )
        };
        if jntnum != 1 || jnt < 0 {
            return Err(SimError::NotFree(name.to_owned()));
        }
        let jnt = jnt as usize; // checked non-negative above
        // SAFETY: invariant 1. `jnt` is the body's first joint, so `jnt < njnt`
        // (MuJoCo's compiler guarantees `body_jntadr` points into the joint
        // arrays when `body_jntnum > 0`), and `jnt_type`, `jnt_qposadr` and
        // `jnt_dofadr` are `njnt` ints each.
        let (kind, qadr, dadr) = unsafe {
            let m = &*self.model;
            (
                *m.jnt_type.add(jnt),
                *m.jnt_qposadr.add(jnt),
                *m.jnt_dofadr.add(jnt),
            )
        };
        if kind != MJ_JNT_FREE || qadr < 0 || dadr < 0 {
            return Err(SimError::NotFree(name.to_owned()));
        }
        let (qadr, dadr) = (qadr as usize, dadr as usize);
        let q = [
            pos[0],
            pos[1],
            pos[2],
            quat[0] / norm,
            quat[1] / norm,
            quat[2] / norm,
            quat[3] / norm,
        ];
        // SAFETY: invariants 1, 2 and 3. A free joint owns qpos[qadr..qadr + 7]
        // and qvel[dadr..dadr + 6], which MuJoCo's compiler lays out inside
        // `nq` and `nv` (the arrays `mj_makeData(model)` sized, invariant 2).
        // `&mut self` is exclusive while the two slices live, and they are
        // different arrays, so they do not alias. mj_forward only writes `data`.
        unsafe {
            let d = &mut *self.data;
            std::slice::from_raw_parts_mut(d.qpos.add(qadr), 7).copy_from_slice(&q);
            std::slice::from_raw_parts_mut(d.qvel.add(dadr), 6).fill(0.0);
            ffi::mj_forward(self.model, self.data);
        }
        Ok(())
    }

    /// Index of the body called `name` in [`Snapshot::bodies`] (equal to its
    /// MuJoCo body id), or `None` if the model has no such body.
    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.body_names.iter().position(|n| n == name)
    }

    /// The model's physics timestep `opt.timestep`, in seconds. Callers use
    /// it to turn a duration into a step count.
    pub fn timestep(&self) -> f64 {
        // SAFETY: invariant 1, so `model` is non-null and points to a valid
        // `mjModel`. Reading one field does not mutate it.
        unsafe { (*self.model).opt.timestep }
    }

    /// A copy of the generalized positions `mjData.qpos` (`nq` values, in
    /// MuJoCo joint order). The run log records it every tick, so a run can
    /// be replayed exactly (rule 6).
    pub fn qpos(&self) -> Vec<f64> {
        // SAFETY: invariant 1, so `model` is non-null and valid.
        let nq = usize::try_from(unsafe { (*self.model).nq }).unwrap_or(0);
        if nq == 0 {
            return Vec::new();
        }
        // SAFETY: invariants 1 and 2. `qpos` is a non-null array of exactly
        // `nq` mjtNums when nq > 0 (sized from `model` by mj_makeData).
        // `&self` means no step runs while the slice is alive, and it is
        // copied out before returning.
        unsafe { std::slice::from_raw_parts((*self.data).qpos, nq) }.to_vec()
    }

    /// Number of bodies in the model, the world body included.
    fn nbody(&self) -> usize {
        // SAFETY: invariant 1, so `model` is non-null and points to a valid
        // `mjModel`. Reading one field does not mutate it.
        let n = unsafe { (*self.model).nbody };
        usize::try_from(n).unwrap_or(0)
    }

    /// Number of actuators, the required length of [`Targets`].
    fn nu(&self) -> usize {
        // SAFETY: invariant 1, as in `nbody`.
        let n = unsafe { (*self.model).nu };
        usize::try_from(n).unwrap_or(0)
    }

    /// Name of body `id`, or `""` if it has none. Called only from `load`
    /// with `id < nbody`.
    fn read_body_name(&self, id: usize) -> String {
        let Ok(id) = c_int::try_from(id) else {
            return String::new();
        };
        // SAFETY: invariant 1 (`model` is non-null). `id` is in
        // `0..nbody`, because the caller iterates that range. `mjOBJ_BODY` is
        // a valid object type. MuJoCo returns either null or a NUL-terminated
        // string inside `model`'s name buffer, which lives as long as `model`.
        // The string is copied out before this function returns.
        unsafe {
            let name = ffi::mj_id2name(self.model, ffi::mjtObj_mjOBJ_BODY as c_int, id);
            if name.is_null() {
                String::new()
            } else {
                CStr::from_ptr(name).to_string_lossy().into_owned()
            }
        }
    }
}

impl Drop for Sim {
    /// Free `data`, then `model`. `data` is sized from `model`, so it goes
    /// first. This is the reverse of acquisition (RAII).
    fn drop(&mut self) {
        // SAFETY: invariant 1. Both pointers are non-null and owned only by
        // this `Sim`. `drop` runs once, so each is released exactly once and
        // is never used again.
        unsafe {
            ffi::mj_deleteData(self.data);
            ffi::mj_deleteModel(self.model);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DROP_XML: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scene/generated/drop.xml"
    );

    /// Write `xml` to a temp file and load it.
    fn load_str(name: &str, xml: &str) -> Sim {
        let p = std::env::temp_dir().join(format!("sim-test-{name}-{}.xml", std::process::id()));
        std::fs::write(&p, xml).expect("write temp scene");
        Sim::load(&p).expect("temp scene loads")
    }

    #[test]
    fn a_settled_box_reports_its_weight_as_contact() {
        let mut sim = Sim::load(DROP_XML).expect("drop.xml loads");
        let b = sim.body_index("box").unwrap();
        assert_eq!(
            sim.snapshot().bodies[b].contact_force,
            0.0,
            "in the air before any step"
        );
        for _ in 0..2_000 {
            sim.step();
        }
        let f = sim.snapshot().bodies[b].contact_force;
        assert!(
            (f - 9.81).abs() < 0.5,
            "1 kg at rest should feel ~9.81 N, got {f}"
        );
    }

    #[test]
    fn a_tilted_body_reports_roll_and_pitch_within_a_tenth_of_a_degree() {
        let mut sim = load_str(
            "tilt",
            r#"<mujoco><worldbody>
                 <body name="r" euler="10 0 0"><geom type="box" size=".1 .1 .1"/></body>
                 <body name="p" euler="0 10 0" pos="1 0 0"><geom type="box" size=".1 .1 .1"/></body>
               </worldbody></mujoco>"#,
        );
        // xquat is filled by the first step's forward pass; these bodies are
        // welded to the world, so a step does not move them.
        sim.step();
        let s = sim.snapshot();
        let deg = |x: f64| x.to_degrees();
        let r = &s.bodies[sim.body_index("r").unwrap()];
        let p = &s.bodies[sim.body_index("p").unwrap()];
        assert!(
            (deg(r.roll) - 10.0).abs() < 0.1 && deg(r.pitch).abs() < 0.1,
            "{r:?}"
        );
        assert!(
            (deg(p.pitch) - 10.0).abs() < 0.1 && deg(p.roll).abs() < 0.1,
            "{p:?}"
        );
    }

    #[test]
    fn sensor_reads_by_name_and_unknown_is_none() {
        let mut sim = load_str(
            "touch",
            r#"<mujoco><option timestep="0.001"/><worldbody>
                 <geom type="plane" size="1 1 .1"/>
                 <body name="b" pos="0 0 .1"><freejoint/>
                   <geom type="box" size=".05 .05 .05" mass="2"/>
                   <site name="s" type="box" size=".06 .06 .06"/></body>
               </worldbody><sensor><touch name="t" site="s"/></sensor></mujoco>"#,
        );
        assert_eq!(sim.sensor("nope"), None);
        for _ in 0..1_000 {
            sim.step();
        }
        let t = sim.sensor("t").expect("touch sensor exists");
        assert!((t - 2.0 * 9.81).abs() < 1.0, "touch {t}");
    }

    #[test]
    fn set_qpos_moves_the_body_without_stepping_time() {
        let mut sim = Sim::load(DROP_XML).expect("drop.xml loads");
        let b = sim.body_index("box").unwrap();
        let mut q = sim.qpos();
        q[2] = 0.3;
        sim.set_qpos(&q).expect("same length");
        let s = sim.snapshot();
        assert_eq!(s.time, 0.0);
        assert!((s.bodies[b].pos[2] - 0.3).abs() < 1e-12);
        assert!(matches!(
            sim.set_qpos(&[0.0]),
            Err(SimError::QposLen {
                expected: 7,
                got: 1
            })
        ));
    }

    #[test]
    fn set_body_pose_places_a_free_body_and_stops_it() {
        let mut sim = load_str(
            "pose",
            r#"<mujoco><option timestep="0.001"/><worldbody>
                 <geom type="plane" size="1 1 .1"/>
                 <body name="a" pos="0 0 .5"><freejoint/><geom type="sphere" size=".02"/></body>
                 <body name="b" pos=".5 0 .5"><freejoint/><geom type="sphere" size=".02"/></body>
                 <body name="hinged" pos=".3 0 .5"><joint type="hinge"/><geom type="sphere" size=".02"/></body>
               </worldbody></mujoco>"#,
        );
        for _ in 0..200 {
            sim.step(); // both spheres are falling now
        }
        // b's free joint is qpos[7..14]. (Compare qpos, not xpos: a snapshot
        // taken after a step shows xpos from that step's start.)
        let b_before = sim.qpos()[7..14].to_vec();
        let t = sim.snapshot().time;
        let half = std::f64::consts::FRAC_1_SQRT_2;
        // An unnormalised quaternion is fine: a quarter turn about Z.
        sim.set_body_pose("a", [0.2, -0.1, 0.3], [2.0, 0.0, 0.0, 2.0])
            .expect("a is free");
        let s = sim.snapshot();
        let a = &s.bodies[sim.body_index("a").unwrap()];
        assert_eq!(s.time, t, "no time passes");
        assert_eq!(a.pos, [0.2, -0.1, 0.3]);
        for (got, want) in a.quat.iter().zip([half, 0.0, 0.0, half]) {
            assert!((got - want).abs() < 1e-12, "{:?}", a.quat);
        }
        assert_eq!(sim.qpos()[7..14].to_vec(), b_before, "only a moved");
        // Stopped: after one step it has dropped by about g·dt² (10 µm), not
        // the 2 mm a step its old 2 m/s speed would take. The second step's
        // snapshot is the one that shows where the first step left it.
        sim.step();
        sim.step();
        let z = sim.snapshot().bodies[sim.body_index("a").unwrap()].pos[2];
        assert!((0.3 - z) < 2e-5, "a kept falling at its old speed: z = {z}");

        assert!(matches!(
            sim.set_body_pose("nope", [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            Err(SimError::UnknownBody(_))
        ));
        assert!(matches!(
            sim.set_body_pose("hinged", [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            Err(SimError::NotFree(_))
        ));
        assert!(matches!(
            sim.set_body_pose("world", [0.0; 3], [1.0, 0.0, 0.0, 0.0]),
            Err(SimError::NotFree(_))
        ));
        assert!(matches!(
            sim.set_body_pose("a", [0.0; 3], [0.0; 4]),
            Err(SimError::BadQuat(_))
        ));
        assert_eq!(
            sim.snapshot().bodies[sim.body_index("a").unwrap()].pos[2],
            z,
            "errors change nothing"
        );
    }

    #[test]
    fn set_gravcomp_floats_a_body_and_gives_its_weight_back() {
        let mut sim = load_str(
            "gravcomp",
            r#"<mujoco><option timestep="0.001"/><worldbody>
                 <body name="a" pos="0 0 .5"><freejoint/><geom type="sphere" size=".02"/></body>
                 <body name="crane" pos="1 0 .5" gravcomp="1"><freejoint/><geom type="sphere" size=".02"/></body>
               </worldbody></mujoco>"#,
        );
        let z = |sim: &Sim| sim.qpos()[2];
        sim.set_gravcomp("a", 1.0).expect("a exists");
        for _ in 0..1_000 {
            sim.step();
        }
        assert!((z(&sim) - 0.5).abs() < 1e-9, "floats: z = {}", z(&sim));
        sim.set_gravcomp("a", 0.0).expect("a exists");
        for _ in 0..100 {
            sim.step();
        }
        assert!(z(&sim) < 0.5 - 0.04, "falls again: z = {}", z(&sim)); // ½ g (0.1 s)² ≈ 0.049
        assert!(matches!(
            sim.set_gravcomp("nope", 1.0),
            Err(SimError::UnknownBody(_))
        ));
        assert!(matches!(
            sim.set_gravcomp("a", f64::NAN),
            Err(SimError::BadGravcomp(_))
        ));
    }

    #[test]
    fn roll_pitch_of_identity_is_zero() {
        assert_eq!(crate::roll_pitch([1.0, 0.0, 0.0, 0.0]), (0.0, 0.0));
    }

    #[test]
    fn bad_path_is_an_error() {
        let err = Sim::load("/no/such/scene.xml").expect_err("missing file must not load");
        assert!(matches!(err, SimError::Load(_)), "got {err:?}");
    }

    #[test]
    fn drop_xml_has_world_and_box() {
        let sim = Sim::load(DROP_XML).expect("drop.xml loads");
        assert_eq!(sim.nbody(), 2);
        assert_eq!(sim.body_index("world"), Some(0));
        assert_eq!(sim.body_index("box"), Some(1));
        assert_eq!(sim.body_index("nope"), None);
        assert_eq!(sim.timestep(), 0.001);
        // One free joint: 3 position + 4 quaternion values, starting at z = 0.5.
        let q = sim.qpos();
        assert_eq!(q.len(), 7);
        assert_eq!(q[2], 0.5);
    }

    #[test]
    fn thousand_steps_lower_the_box() {
        let mut sim = Sim::load(DROP_XML).expect("drop.xml loads");
        let b = sim.body_index("box").expect("box exists");
        // `xpos` is filled by the first step's forward pass, so measure the
        // start after one step rather than reading the zeroed initial state.
        sim.step();
        let z0 = sim.snapshot().bodies[b].pos[2];
        for _ in 0..1_000 {
            sim.step();
        }
        let z1 = sim.snapshot().bodies[b].pos[2];
        assert!(z1 < z0, "box did not fall: z0 = {z0}, z1 = {z1}");
    }

    #[test]
    fn wrong_target_length_is_an_error() {
        let mut sim = Sim::load(DROP_XML).expect("drop.xml loads");
        // drop.xml has no actuators, so nu == 0.
        assert!(sim.apply(&Targets(vec![])).is_ok());
        let err = sim
            .apply(&Targets(vec![1.0]))
            .expect_err("length 1 against nu = 0");
        assert!(
            matches!(
                err,
                SimError::TargetLen {
                    expected: 0,
                    got: 1
                }
            ),
            "got {err:?}"
        );
    }
}
