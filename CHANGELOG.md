# Changelog

Each phase of the build proposal ended with a tag; `v0.1.0` gathers them.

## [Unreleased]

- **Telescoping-fork crane** (#67, #68, #69): the `hardware/crane` package (CAD, `crane.toml`, BOM, MuJoCo pick test) and its Rookery presets; the sim's crane is now that fork. The mast lowers the hook onto a handle (`hook_lift` is locked at 0), the arm folds instead of retracting, the fork's middle stage follows the arm, and there is a hook camera (`cam_hook`). New limits: `ARM_X` 0.07..0.85, `MAST_Z` 0.05..0.85, rail 0.17 m back from the exit tray, rack3 shelf gap 0.18 m. The old crane is kept on `backup/old-crane`.

## [0.1.0] - unreleased

The twelve-phase build (Phase 9, feeding, deferred to Phase 13+).

- **Phase 12** (#61): `scene.bench` sweeps (small in CI, large overnight), `rookery bench-row`, molt rate, run metrics, benchmark figures, the clean-clone CI job, this changelog.
- **Phase 11** (`phase-11`, #58): the camera detector. Depth by ray casting (`Sim::ray`), body counting, the camera runs the rack with no window missed.
- **Phase 10** (`phase-10`, #53): three stacked trays, one box deep; shelf windows from the scene; level-changing routes; `rookery.toml`.
- **Phase 8** (`phase-8`, #45): autonomy. Oracle detector, `WindowMissed`/`BoxLifted`, exit slots, `autonomy.lua`, run logs and `rookery replay`, the exit-tray worker, a month-long soak.
- **Phase 7** (`phase-7`, #38): the molt lifecycle, the event bus, the fast-forward clock, shed shells, per-cell biology.
- **Phase 6** (`phase-6`, #32): the full transfer to the exit tray as one undoable command.
- **Phase 5** (`phase-5`, #25): the J-hook lift with the crawfish inside.
- **Phase 4** (`phase-4`, #19): behavior trees in Lua (mlua) driving the crane through goals.
- **Phase 3** (`phase-3`, #13): the crane: kinematics, the safety clamp, commands.
- **Phase 2** (`phase-2`, #6): the generated farm scene.
- **Phase 1** (`phase-1`, #1): the MuJoCo bridge.
