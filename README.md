# Rookery

A MuJoCo simulation of a soft-shell crawfish shedding facility, served by a
stacker-crane gantry (`mast`). Rust (sim bridge, BT engine, kinematics, safety),
Lua (behaviors), Python (scene generation, video).

## Setup (macOS)

- MuJoCo 3.16.0: `mujoco.framework` in `~/.mujoco`
- `brew install rustup llvm uv gh`, then `rustup default stable`
- `export LIBCLANG_PATH="$(brew --prefix llvm)/lib"` in your shell profile
- `scripts/worktree-setup.sh` once per checkout, then `cargo build --workspace`

## Run

`rookery <scene.xml>` prints the body count and names, then runs 2 s headless.

Phase 1 demo. This drops a 0.1 m box onto a plane, and the last line is the box's final height (`box z 0.049892`):

```bash
cargo run -p rookery -- scene/generated/drop.xml
```

Phase 2 demo. This builds the farm scene (one tray, 4x4 mesh boxes with hook handles, an exit tray, the crane rails and, since Phase 3, the crane), then loads it (`nbody 38`: world, tray_0, 16 boxes, 16 crawfish and the four crane bodies):

```bash
(cd scene && uv run python -m scene.cli --trays 1 --cells 4x4 --shelf-gap 0.08)
```

```bash
cargo run -p rookery -- scene/generated/rookery.xml
```

Phase 6 demo. This moves box (1, 2) to the exit tray as one undoable transfer, then idles at home:

![transfer](docs/transfer.gif)

```bash
cargo run --release -p rookery -- run --tree lua/trees/transfer.lua --seconds 40 --log runs/ --id demo
```

```bash
(cd scene && uv run python -m scene.render --run demo --out transfer.mp4)
```
