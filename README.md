# Rookery

A MuJoCo simulation of a soft-shell crawfish shedding facility, served by a
stacker-crane gantry (`mast`). Rust (sim bridge, BT engine, kinematics, safety),
Lua (behaviors), Python (scene generation, video).

## Setup (macOS)

- MuJoCo 3.16.0: `mujoco.framework` in `~/.mujoco`
- `brew install rustup llvm uv gh`, then `rustup default stable`
- `export LIBCLANG_PATH="$(brew --prefix llvm)/lib"` in your shell profile
- `scripts/worktree-setup.sh` once per checkout, then `cargo build --workspace`
