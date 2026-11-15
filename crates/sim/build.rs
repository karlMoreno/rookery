//! Build script for `sim`: find the pinned MuJoCo (3.16.0), link it, and run
//! bindgen over `mujoco.h` into `$OUT_DIR/bindings.rs` for `src/ffi.rs`.
//!
//! `MUJOCO_DIR` has a different shape per platform, and both must keep
//! building: development happens on the Air, CI runs on Linux.
//!
//! ```text
//!   macOS (the Air)                          Linux (CI)
//!   $MUJOCO_DIR/                             $MUJOCO_DIR/
//!   └── mujoco.framework/                    ├── include/mujoco/mujoco.h
//!       ├── Headers/mujoco.h                 └── lib/libmujoco.so
//!       ├── mujoco.tbd                           (bin/ on releases that
//!       └── Versions/A/libmujoco.3.16.0.dylib     put it there)
//! ```
//!
//! Flow: read `MUJOCO_DIR` -> pick the [`Layout`] for the target OS -> emit
//! the link lines -> generate the bindings. Every failure prints a message
//! naming both layouts and exits non-zero, so a bad `MUJOCO_DIR` is one line
//! of `cargo build` output rather than a linker error.

use std::env;
use std::path::{Path, PathBuf};
use std::process;

/// Pattern: **Strategy**. One variant per `MUJOCO_DIR` layout; `main` runs the
/// same flow on every platform and asks the chosen layout for the three things
/// that differ: link lines, header path and clang arguments.
/// Why: the OS branch lives in one place ([`Layout::detect`]) instead of being
/// repeated at every step, and a third layout (a static build, say) is one
/// more variant rather than another round of `if os ==` edits.
///
/// ```text
///   ┌──────────────────────────┐
///   │ Layout                   │
///   ├──────────────────────────┤
///   │ Framework { dir }        │  macOS: dir holds mujoco.framework
///   │ Release { dir, lib_dir } │  Linux: extracted release tarball
///   ├──────────────────────────┤
///   │ + detect(os, dir)        │
///   │ + emit_link_lines()      │
///   │ + header() -> PathBuf    │
///   │ + clang_args() -> Vec    │
///   └──────────────────────────┘
/// ```
enum Layout {
    /// macOS: `dir` holds `mujoco.framework`.
    Framework { dir: PathBuf },
    /// Linux: `dir` is the extracted release; `lib_dir` holds `libmujoco.so`.
    Release { dir: PathBuf, lib_dir: PathBuf },
}

impl Layout {
    /// Pick the layout for `target_os` and check that `dir` has that shape.
    fn detect(target_os: &str, dir: &Path) -> Result<Self, String> {
        match target_os {
            "macos" => {
                let header = dir.join("mujoco.framework/Headers/mujoco.h");
                if !header.is_file() {
                    return Err(format!(
                        "MUJOCO_DIR={} does not hold mujoco.framework (no {}).",
                        dir.display(),
                        header.display()
                    ));
                }
                Ok(Self::Framework {
                    dir: dir.to_path_buf(),
                })
            }
            "linux" => {
                let header = dir.join("include/mujoco/mujoco.h");
                if !header.is_file() {
                    return Err(format!(
                        "MUJOCO_DIR={} is not an extracted MuJoCo release (no {}).",
                        dir.display(),
                        header.display()
                    ));
                }
                // `-lmujoco` needs the unversioned name; release tarballs ship
                // it as a symlink to libmujoco.so.3.16.0, and `is_file` follows it.
                let lib_dir = ["lib", "bin"]
                    .into_iter()
                    .map(|sub| dir.join(sub))
                    .find(|d| d.join("libmujoco.so").is_file())
                    .ok_or_else(|| {
                        format!(
                            "MUJOCO_DIR={}: no libmujoco.so under lib/ or bin/.",
                            dir.display()
                        )
                    })?;
                Ok(Self::Release {
                    dir: dir.to_path_buf(),
                    lib_dir,
                })
            }
            other => Err(format!(
                "target OS `{other}` is not supported; sim links MuJoCo on macOS and Linux only."
            )),
        }
    }

    /// Tell rustc where MuJoCo is and how to link it. The rpath makes the
    /// binary find the shared library at run time without DYLD_*/LD_* help.
    fn emit_link_lines(&self) {
        match self {
            Self::Framework { dir } => {
                println!("cargo:rustc-link-search=framework={}", dir.display());
                println!("cargo:rustc-link-lib=framework=mujoco");
                println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.display());
            }
            Self::Release { lib_dir, .. } => {
                println!("cargo:rustc-link-search=native={}", lib_dir.display());
                println!("cargo:rustc-link-lib=dylib=mujoco");
                println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir.display());
            }
        }
    }

    /// The umbrella header bindgen parses.
    fn header(&self) -> PathBuf {
        match self {
            Self::Framework { dir } => dir.join("mujoco.framework/Headers/mujoco.h"),
            Self::Release { dir, .. } => dir.join("include/mujoco/mujoco.h"),
        }
    }

    /// How `#include <mujoco/mjdata.h>` inside `mujoco.h` resolves: framework
    /// search (`-F`) on macOS, a plain include directory on Linux.
    fn clang_args(&self) -> Vec<String> {
        match self {
            Self::Framework { dir } => vec![format!("-F{}", dir.display())],
            Self::Release { dir, .. } => vec![format!("-I{}", dir.join("include").display())],
        }
    }
}

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DIR");

    let dir = match env::var_os("MUJOCO_DIR") {
        Some(dir) if !dir.is_empty() => PathBuf::from(dir),
        _ => fail("MUJOCO_DIR is not set."),
    };
    let target_os = env::var("CARGO_CFG_TARGET_OS").expect("cargo sets CARGO_CFG_TARGET_OS");
    let layout = Layout::detect(&target_os, &dir).unwrap_or_else(|why| fail(&why));

    layout.emit_link_lines();
    generate_bindings(&layout);
}

/// Run bindgen over `mujoco.h` and write `$OUT_DIR/bindings.rs`.
///
/// The allowlist *is* the raw surface of this crate. Widen it here, in one
/// place, only when a phase needs a symbol (standing brief: minimal bridge).
/// Types that the allowlisted items mention (`mjOption`, `mjvScene`, ...) come
/// along automatically; nothing else from the 3.16 headers is emitted.
fn generate_bindings(layout: &Layout) {
    let header = layout.header();
    let header = header
        .to_str()
        .unwrap_or_else(|| fail(&format!("header path {} is not UTF-8", header.display())));

    let bindings = bindgen::Builder::default()
        .header(header)
        .clang_args(layout.clang_args())
        .allowlist_function("mj_.*")
        .allowlist_function("mjv_.*")
        .allowlist_function("mjr_.*")
        .allowlist_type("mjModel")
        .allowlist_type("mjData")
        .allowlist_type("mjtNum")
        .allowlist_var("mjVERSION_HEADER")
        // Match the workspace edition so the output uses `unsafe extern`.
        .rust_edition(bindgen::RustEdition::Edition2024)
        // Emits rerun-if-changed for every header bindgen reads.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .unwrap_or_else(|err| fail(&format!("bindgen failed over {header}: {err}")));

    let out =
        PathBuf::from(env::var_os("OUT_DIR").expect("cargo sets OUT_DIR")).join("bindings.rs");
    bindings
        .write_to_file(&out)
        .unwrap_or_else(|err| fail(&format!("cannot write {}: {err}", out.display())));
}

/// Print `why`, then the two layouts `MUJOCO_DIR` may have, and stop the build.
fn fail(why: &str) -> ! {
    eprintln!(
        "\nsim/build.rs: {why}\n\
         \n\
         MUJOCO_DIR must point at the pinned MuJoCo 3.16.0 in one of two layouts:\n\
         \x20 macOS: the directory holding mujoco.framework\n\
         \x20        (e.g. ~/.mujoco, so that mujoco.framework/Headers/mujoco.h exists under it)\n\
         \x20 Linux: the extracted release tarball with lib/ and include/\n\
         \x20        (e.g. ~/.mujoco/mujoco-3.16.0, with lib/libmujoco.so and include/mujoco/mujoco.h)\n\
         scripts/worktree-setup.sh writes MUJOCO_DIR (and LIBCLANG_PATH on macOS) into .cargo/config.toml.\n"
    );
    process::exit(1);
}
