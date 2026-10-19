# Rookery: rules for every agent

Rookery is a MuJoCo simulation of a soft-shell crawfish shedding facility. Rust owns
correctness and speed (sim bridge, BT engine, kinematics, safety limits), Lua owns
behavior, Python owns scene generation and video. Read this file first, then the
`CLAUDE.md` of the crate or directory you were assigned.

## Rules that do not change between phases

1. A phase is done when its demo runs from one command and its gate test passes in CI. Not before, and nothing is carried into the next phase unfinished.
2. Every phase has a test that can fail. If you cannot name the assertion, the phase is not scoped yet.
3. Agents write on branches; CI validates; you merge. No agent output reaches `main` without your eyes on the diff.
4. One agent, one crate or one Lua directory, one task. Isolation is how agent mistakes stay local.
5. Safety constants (joint limits, workspace box, max velocity) live in `mast` as Rust constants. Lua cannot read or write them; clamps apply to every target before it reaches the sim.
6. Every run logs every tick's snapshot and targets. A bug report is a run id, not a description.
7. The MJCF is never hand-edited. Change `scene.py`, regenerate, commit both.
8. `cargo clippy -D warnings` and `cargo fmt --check` gate every PR; the compiler is the first reviewer.
9. Protected paths are enforced, not remembered. `tests/gates/`, `crates/mast/src/limits.rs` and `scene/generated/` are assigned to you in `CODEOWNERS`, and a `gate-guard` CI job fails any pull request not authored by you that touches them. An agent may propose a gate or limit change only in a separate pull request that changes nothing else.

## Code style (Karl's standing preference)

- Use design patterns where they earn their place, and say in a comment which
  pattern a type implements, what it does and why it was chosen.
- When explaining structure in docs, comments or PR descriptions, use ASCII UML.

## Toolchain

- Rust is pinned in `rust-toolchain.toml`; MuJoCo is pinned to **3.16.0**
  (macOS framework in `~/.mujoco`, Linux tarball in CI).
- `MUJOCO_DIR` and `LIBCLANG_PATH` come from `.cargo/config.toml`, written by
  `scripts/worktree-setup.sh`. Run that script if a build can't find either.
- Before opening a PR: `cargo fmt`, `cargo clippy --workspace -- -D warnings`,
  `cargo test --workspace`.
