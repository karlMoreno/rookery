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
        // paired. `&mut self` gives exclusive access to `data`, which
        // `mj_step` mutates.
        unsafe { ffi::mj_step(self.model, self.data) };
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
        let (time, xpos, xquat) = unsafe {
            let d = &*self.data;
            (
                d.time,
                std::slice::from_raw_parts(d.xpos, nbody * 3),
                std::slice::from_raw_parts(d.xquat, nbody * 4),
            )
        };

        let bodies = self
            .body_names
            .iter()
            .enumerate()
            .map(|(i, name)| BodyState {
                name: name.clone(),
                pos: [xpos[3 * i], xpos[3 * i + 1], xpos[3 * i + 2]],
                quat: [
                    xquat[4 * i],
                    xquat[4 * i + 1],
                    xquat[4 * i + 2],
                    xquat[4 * i + 3],
                ],
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

    /// Index of the body called `name` in [`Snapshot::bodies`] (equal to its
    /// MuJoCo body id), or `None` if the model has no such body.
    pub fn body_index(&self, name: &str) -> Option<usize> {
        self.body_names.iter().position(|n| n == name)
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

    const DROP_XML: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/drop.xml");

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
