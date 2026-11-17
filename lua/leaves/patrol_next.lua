-- patrol_next(): visit the cells in ORDER, one per success, wrapping round.
--
-- The patrol order lives here, in a leaf, so that editing it takes effect
-- without a restart: leaves hot-reload (checked every 10 ticks, reloaded
-- after two stable checks), but tree files are only read at start-up.
-- After a reload the patrol starts again from the first entry of the new
-- ORDER.
--
-- Rows run along +X and columns along +Y, counted from 0. This is the
-- default 4x4 grid, row by row.
ORDER = {
  {0, 0}, {0, 1}, {0, 2}, {0, 3},
  {1, 3}, {1, 2}, {1, 1}, {1, 0},
  {2, 0}, {2, 1}, {2, 2}, {2, 3},
  {3, 3}, {3, 2}, {3, 1}, {3, 0},
}

local TOL = 0.002 -- metres, mast::DONE_TOL
local i = 1

local function settled(bb)
  if bb.route_len ~= 0 then return false end
  for k = 1, 4 do
    if math.abs(bb.gantry_q[k] - bb.gantry_target[k]) > TOL then return false end
  end
  return true
end

function tick(bb)
  local r, c = ORDER[i][1], ORDER[i][2]
  local g = bb.goal
  if g == nil or g.pose ~= "Cell" or g.r ~= r or g.c ~= c then
    bb:go_to(Cell(r, c))
    return "running"
  end
  if not settled(bb) then return "running" end
  print(string.format("patrol: visited %d,%d at t=%.2f", r, c, bb.sim_time))
  i = i % #ORDER + 1
  return "success"
end
