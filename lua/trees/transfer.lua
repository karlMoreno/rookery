-- Phase 6 demo: move box (1, 2) to the exit tray, then idle at home.
--
--   rookery run --tree lua/trees/transfer.lua --seconds 60 --log runs/
--
-- Change the numbers to move another box.
return Sequence {
  Leaf("../leaves/transfer.lua", 1, 2),
  Leaf("../leaves/wait.lua", 1000000), -- stay put once done
}
