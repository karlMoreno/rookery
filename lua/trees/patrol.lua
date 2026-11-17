-- Phase 4 demo: patrol every cell of the tray, forever.
--
--   rookery run --tree lua/trees/patrol.lua --seconds 200 --log runs/
--
-- The order is ORDER in ../leaves/patrol_next.lua. Edit it while this runs
-- and the patrol changes within about 20 ticks, with no restart.
return Repeat(0, Leaf("../leaves/patrol_next.lua"))
