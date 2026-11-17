-- Phase 6 scenario: the transfer tree moves box (1, 2) to the exit tray.
-- Passes when the transfer was completed (the crane arrived home after
-- it), no errors were logged, and box (1, 2) is the only box that moved.
--
--   rookery run --scenario lua/scenarios/transfer_1_2.lua --log runs/

ci = true
tree = "../trees/transfer.lua"
ticks = 3000 -- 60 s; one transfer takes about 35 s

function check(log)
  if #log.errors > 0 then
    return false, "errors: " .. table.concat(log.errors, "; ")
  end
  local done = false
  for _, v in ipairs(log.visited) do
    if type(v) == "table" and v.transfer and v.transfer[1] == 1 and v.transfer[2] == 2 then
      done = true
    end
  end
  if not done then
    return false, "the transfer of (1, 2) never completed"
  end
  if #log.moved_boxes ~= 1 or log.moved_boxes[1][1] ~= 1 or log.moved_boxes[1][2] ~= 2 then
    local m = {}
    for _, b in ipairs(log.moved_boxes) do m[#m + 1] = b[1] .. "," .. b[2] end
    return false, "boxes moved: " .. table.concat(m, " ") .. " (only 1,2 should)"
  end
  return true, string.format("box (1, 2) transferred in %.1f s, no other box moved", log.sim_time)
end
