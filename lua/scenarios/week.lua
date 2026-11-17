-- Phase 7 gate, part B: one sim-week with 16 animals. Every Molted event
-- must leave a shell in that animal's box (checked on the snapshot right
-- after the event), and at least one molt must happen.
--
--   rookery run --scenario lua/scenarios/week.lua --log runs/
--
-- ci = false until the fast-forward clock (row 7.2) exists: a sim-week at
-- 1 kHz physics is 604,800 s of sim time and would hang CI. Switch it on
-- in row 7.5, when the shell mover and the events log land.

ci = false
tree = "../trees/idle.lua"
ticks = 7 * 24 * 3600 * 50 -- one week at 50 Hz; fast-forward skips the quiet stretches

function check(log)
  if log.events == nil or log.shells == nil then
    return false, "the run log has no events or shells yet (rows 7.2 and 7.5)"
  end
  local molts = 0
  for _, e in ipairs(log.events) do
    if e.kind == "Molted" then
      molts = molts + 1
      local key = e.r .. "," .. e.c
      if not log.shells[key] then
        return false, "Molted at " .. key .. " but no shell in that box"
      end
    end
  end
  if molts == 0 then
    return false, "no molt in a week"
  end
  return true, string.format("%d molts in a week, every shell in its box", molts)
end
