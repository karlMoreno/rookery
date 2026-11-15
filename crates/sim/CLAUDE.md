# Role: FFI engineer

Read the root `CLAUDE.md` first. You start cold: the repo and its tests are the memory.

- **Owns:** `crates/sim`
- **Standing brief:** Keep the MuJoCo bridge safe and minimal; add bindings only when a phase needs them.
- **Done when:** `cargo test -p sim` green; every `unsafe` block has a comment stating the invariant.
- **May not:** Touch any other crate.

## Shape of the crate

```text
  build.rs ── reads MUJOCO_DIR, links MuJoCo, runs bindgen ──▶ $OUT_DIR/bindings.rs
                                                                        │ include!
  src/ffi.rs   pub(crate) raw surface, generated, C names   ◀───────────┘
       ▲ the only caller
  src/sim.rs   Sim: the Facade, every unsafe block in Rookery (1.6)
  src/lib.rs   public vocabulary (Snapshot, Targets, SimError) and `pub use Sim`
```

## Finding MuJoCo: the two `MUJOCO_DIR` layouts

`build.rs` reads `MUJOCO_DIR` and branches on `CARGO_CFG_TARGET_OS`. **Both
branches must build on every PR**: development is on the Air (macOS), CI is
Linux, and neither machine can compile the other's branch. A change to one
branch is not done until the other still compiles in CI.

| | macOS (the Air) | Linux (CI) |
| --- | --- | --- |
| `MUJOCO_DIR` | the directory holding `mujoco.framework` (`~/.mujoco`) | the extracted release tarball (`~/.mujoco/mujoco-3.16.0`) |
| shape check | `mujoco.framework/Headers/mujoco.h` exists | `include/mujoco/mujoco.h` and `lib/libmujoco.so` (or `bin/libmujoco.so`) exist |
| bindgen input | `mujoco.framework/Headers/mujoco.h` with clang `-F$MUJOCO_DIR` | `include/mujoco/mujoco.h` with clang `-I$MUJOCO_DIR/include` |
| link lines | `rustc-link-search=framework=$MUJOCO_DIR` `rustc-link-lib=framework=mujoco` `rustc-link-arg=-Wl,-rpath,$MUJOCO_DIR` | `rustc-link-search=native=$MUJOCO_DIR/lib` `rustc-link-lib=dylib=mujoco` `rustc-link-arg=-Wl,-rpath,$MUJOCO_DIR/lib` |

An unset or wrong-shaped `MUJOCO_DIR` stops the build with a message naming
both layouts; `cargo:rerun-if-env-changed=MUJOCO_DIR` re-runs the script when
it changes.

`LIBCLANG_PATH` (macOS): bindgen loads `libclang` at run time and this
variable says which one. On the Air it is Homebrew LLVM,
`/opt/homebrew/opt/llvm/lib`; the Command Line Tools copy under
`/Library/Developer/CommandLineTools/usr/lib` also works. Both variables come
from `.cargo/config.toml`, written by `scripts/worktree-setup.sh`; run that
script if a build cannot find either. In CI the "Export MuJoCo paths" step of
`.github/workflows/ci.yml` sets `MUJOCO_DIR` and `LD_LIBRARY_PATH`, and
bindgen uses the runner image's libclang.

### Run-time lookup: the rpath reaches only this crate's binaries

Cargo applies `cargo:rustc-link-arg` to the emitting package's own targets
only, so the rpath above lands in `sim`'s test binaries and nowhere else
(verified on the Air with a scratch dependent: no `LC_RPATH`). The `-L`/`-l`
lines do propagate, so a binary in another crate (`rookery`, the `gates`
tests) links MuJoCo but carries no rpath. On the Air the framework's install
name is `@rpath/mujoco.framework/Versions/A/libmujoco.3.16.0.dylib`, so such
a binary fails at launch with "Library not loaded" until one of these exists;
Linux CI is covered by `LD_LIBRARY_PATH`.

- **Done (row 1.7) for direct dependents:** `build.rs` here publishes the
  directory as `cargo::metadata=rpath=<dir>`. A crate that depends on `sim`
  reads `DEP_MUJOCO_RPATH` in its own `build.rs` and emits
  `cargo:rustc-link-arg=-Wl,-rpath,$DEP_MUJOCO_RPATH`, as
  `crates/rookery/build.rs` does.
- Still open on the Air: the `gates` tests (protected path). Run them
  locally with `RUSTFLAGS="-C link-arg=-Wl,-rpath,$MUJOCO_DIR"`. Another
  option is rustflags in the generated `.cargo/config.toml`, e.g.
  `[target.'cfg(target_os = "macos")'] rustflags = ["-C", "link-arg=-Wl,-rpath,<MUJOCO_DIR>"]`.

## The raw surface (`src/ffi.rs`)

The bindgen allowlist in `build.rs` **is** the raw surface: functions
`mj_.*`, `mjv_.*`, `mjr_.*`; types `mjModel`, `mjData`, `mjtNum`; the
constant `mjVERSION_HEADER`. Types those items mention (`mjOption`,
`mjvScene`, ...) come along automatically. Widen the allowlist only when a
phase needs a symbol, in `build.rs`, in one place.

`src/ffi.rs` is `pub(crate)`. Nothing outside this crate may name a MuJoCo
type or call a MuJoCo function; `Sim` in `src/sim.rs` is the only caller, and
every `unsafe` block there carries a `// SAFETY:` comment.

The tests in `src/ffi.rs` pin the install: `mjVERSION_HEADER == 3016000`,
`mj_version() == mjVERSION_HEADER` and `mj_versionString() == "3.16.0"`.
Bumping MuJoCo means changing the root `CLAUDE.md` pin, `MUJOCO_VERSION` in
`ci.yml`, the framework in `~/.mujoco` and these constants together.

## Known noise on the Air

- The stale Command Line Tools linker prints `ld: ignoring duplicate library
  '-lc'` / `'-lm'` when linking the build script (bindgen's dependencies pull
  in the `libc` crate). rustc reports it through the `linker_messages` lint,
  which ignores `-D warnings`, so clippy stays green. Linux CI does not emit it.
- A panicking test aborts the whole test binary (`fatal runtime error: failed
  to initiate panic`) until Command Line Tools 27.0 are installed; see the
  Phase 1 environment note. CI is the verdict for a red test.
