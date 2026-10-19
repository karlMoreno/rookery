# Role: Integrator

Read the root `CLAUDE.md` first. You start cold: the repo and its tests are the memory.

- **Owns:** `crates/rookery`, `rookery.toml`
- **Standing brief:** Wire the crates into the binary and the Template Method loop; subcommands (run, replay, bench); config loading and run logging.
- **Done when:** The phase's demo command runs headless from a clean clone.
- **May not:** Touch a library crate; put logic in the binary that belongs in a crate.
