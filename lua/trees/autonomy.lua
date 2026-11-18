-- Phase 8: the farm runs itself (row 8.5). Each tick:
--
--   Fallback
--   ├── Sequence                  a molt to harvest?
--   │   ├── molt_pending          bb.detected is not empty
--   │   └── pick_soonest          Transfer(the soonest-closing window)
--   └── idle_home                 nothing to do: succeed, start over
--
--   rookery run --scenario lua/scenarios/week_1tray.lua --log runs/
--
-- Every leaf returns "success" or "failure" when there is nothing to wait
-- for, so the tree goes back to the top and re-reads bb.detected every tick.
return Fallback {
  Sequence {
    Leaf("../leaves/molt_pending.lua"),
    Leaf("../leaves/pick_soonest.lua"),
  },
  Leaf("../leaves/idle_home.lua"),
}
