-- goto_exit(): drive the crane to the engage pose over the exit tray.
-- Returns "running" until it has arrived (route finished, joints within
-- 2 mm), then "success". Same rules as goto_cell.

local TOL = 0.002 -- metres, mast::DONE_TOL

local function settled(bb)
  if bb.route_len ~= 0 then return false end
  for i = 1, 4 do
    if math.abs(bb.gantry_q[i] - bb.gantry_target[i]) > TOL then return false end
  end
  return true
end

function tick(bb)
  local g = bb.goal
  if g == nil or g.pose ~= "Exit" then
    bb:go_to(Exit())
    return "running"
  end
  if settled(bb) then return "success" end
  return "running"
end
