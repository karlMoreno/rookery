-- goto_cell(r, c): drive the crane to the engage pose under the handle of
-- the box in cell (r, c). Returns "running" until it has arrived, then
-- "success".
--
-- It only sets a goal (bb:go_to). Rust turns the goal into the safe route
-- and plays it through mast::clamp, so this leaf can never sweep the hook
-- through a handle. "Arrived" means the route is finished *and* the
-- measured joints are within 2 mm of their targets, the same rule as
-- mast's DONE_TOL.

local TOL = 0.002 -- metres, mast::DONE_TOL

local function settled(bb)
  if bb.route_len ~= 0 then return false end
  for i = 1, 4 do
    if math.abs(bb.gantry_q[i] - bb.gantry_target[i]) > TOL then return false end
  end
  return true
end

local function is_goal(bb, r, c)
  local g = bb.goal
  return g ~= nil and g.pose == "Cell" and g.r == r and g.c == c
end

function tick(bb, r, c)
  if not is_goal(bb, r, c) then
    bb:go_to(Cell(r, c))
    return "running"
  end
  if settled(bb) then return "success" end
  return "running"
end
