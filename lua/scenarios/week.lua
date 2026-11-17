-- Phase 7 gate, part B: one sim-week with 16 animals. Every Molted event
-- must leave a shell in that animal's box (checked on the snapshot right
-- after the event), and at least one molt must happen.
--
--   rookery run --scenario lua/scenarios/week.lua --log runs/
--
-- It runs in CI because the clock fast-forwards (row 7.2): physics only
-- runs for a tick after each stage change, so a week takes about a second.
-- The shell mover and the events log came in row 7.5.

ci = true
tree = "../trees/idle.lua"
ticks = 7 * 24 * 3600 * 50 -- one week at 50 Hz; fast-forward skips the quiet stretches

function check(log)
  if log.events == nil or log.shells == nil then
    return false, "the run log has no events or shells"
  end
  local molts, eaten = 0, 0
  for _, e in ipairs(log.events) do
    local key = e.r .. "," .. e.c
    if e.kind == "Molted" then
      molts = molts + 1
      local s = log.shells[key]
      if not (s and s.inside) then
        return false, "Molted at " .. key .. " but no shell in that box"
      end
    elseif e.kind == "ShellEaten" then
      eaten = eaten + 1
    end
  end
  if molts == 0 then
    return false, "no molt in a week"
  end
  if log.max_box_displacement > 0.002 then
    return false, string.format("a box or crawfish moved %.4f m", log.max_box_displacement)
  end
  return true, string.format("%d molts in a week, every shell in its box (%d eaten since)", molts, eaten)
end
