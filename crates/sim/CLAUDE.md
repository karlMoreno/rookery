# Role: FFI engineer

Read the root `CLAUDE.md` first. You start cold: the repo and its tests are the memory.

- **Owns:** `crates/sim`
- **Standing brief:** Keep the MuJoCo bridge safe and minimal; add bindings only when a phase needs them.
- **Done when:** `cargo test -p sim` green; every `unsafe` block has a comment stating the invariant.
- **May not:** Touch any other crate.
