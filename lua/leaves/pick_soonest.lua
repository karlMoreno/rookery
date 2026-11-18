-- pick_soonest: harvest the detected cell with the least slack (row 8.5,
-- weighed against travel time in 10.5).
--
--   crane busy (route_len > 0)     → "running": let the current transfer finish
--   cells the crane can reach now  → the one with the smallest
--                                    closes_in - eta: bb:go_to(Transfer(r, c, tray)), "running"
--   only blocked cells             → "running": the exit worker frees their row soon
--   nothing detected               → "success": every molt is harvested
--
-- eta (seconds to get there) and blocked come from Rust, so Lua never reads
-- the crane's limits. It only sets a goal; Rust plans the transfer.

function tick(bb)
  if bb.route_len > 0 then
    return "running"
  end
  local best, slack = nil, math.huge
  for _, d in ipairs(bb.detected) do
    if not d.blocked and d.closes_in - d.eta < slack then
      best, slack = d, d.closes_in - d.eta
    end
  end
  if best == nil then
    if #bb.detected > 0 then
      return "running"
    end
    return "success"
  end
  bb:go_to(Transfer(best.r, best.c, best.tray))
  return "running"
end
