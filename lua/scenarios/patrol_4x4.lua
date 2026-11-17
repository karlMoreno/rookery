-- Phase 4 scenario: the patrol visits every cell of the 4x4 tray, with no
-- errors, and no box moves more than 1 mm (the 3.5 handle-strike lesson).
--
--   rookery run --scenario lua/scenarios/patrol_4x4.lua --log runs/
--
-- 10,000 ticks at 50 Hz is 200 s of sim time. One full patrol takes about 170 s.

ci = true
tree = "../trees/patrol.lua"
ticks = 10000

local want = 16

function setup(world)
  want = world.rows * world.cols
end

function check(log)
  if #log.errors > 0 then
    return false, "errors: " .. table.concat(log.errors, "; ")
  end
  if log.visited_cells ~= want then
    return false, string.format("visited %d of %d cells", log.visited_cells, want)
  end
  if log.max_box_displacement > 0.001 then
    return false, string.format("a box moved %.4f m (limit 0.001)", log.max_box_displacement)
  end
  return true, string.format("all %d cells visited in %.1f s, boxes still (max %.5f m)",
    want, log.sim_time, log.max_box_displacement)
end
