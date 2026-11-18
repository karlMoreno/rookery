-- pick_soonest: harvest the detected cell whose soft-shell window closes
-- first (row 8.5).
--
--   crane busy (route_len > 0)  → "running": let the current transfer finish
--   a cell detected             → bb:go_to(Transfer(r, c)), "running"
--   nothing detected            → "success": every molt is harvested
--
-- It only sets a goal. Rust turns Transfer(r, c) into one undoable
-- transfer into the lowest free exit slot, and reports BoxLifted once the
-- box leaves its cell, which takes the cell off bb.detected.

function tick(bb)
  if bb.route_len > 0 then
    return "running"
  end
  local d = bb.detected[1]
  if d == nil then
    return "success"
  end
  bb:go_to(Transfer(d.r, d.c))
  return "running"
end
