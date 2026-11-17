-- transfer(r, c): move the box in cell (r, c) to the exit tray and bring
-- the crane home. Returns "running" until the whole job is done (the
-- crane's queue is empty and its joints have settled within 2 mm), then
-- "success".
--
-- Like goto_cell, it only sets a goal: Transfer(r, c) is a job, and Rust
-- runs it as one Transfer macro (engage, lift, carry, set down, release,
-- home), so a fault can undo the whole thing.

local TOL = 0.002 -- metres, mast::DONE_TOL

local function settled(bb)
  if bb.route_len ~= 0 then return false end
  for i = 1, 4 do
    if math.abs(bb.gantry_q[i] - bb.gantry_target[i]) > TOL then return false end
  end
  return true
end

function tick(bb, r, c)
  local g = bb.goal
  if g == nil or g.pose ~= "Transfer" or g.r ~= r or g.c ~= c then
    bb:go_to(Transfer(r, c))
    return "running"
  end
  if settled(bb) then return "success" end
  return "running"
end
