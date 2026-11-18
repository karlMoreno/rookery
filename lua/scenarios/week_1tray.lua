-- Phase 8 demo: one tray runs unattended for a sim-week (row 8.5). Every
-- molt must be harvested (BoxLifted) inside its soft-shell window: no
-- WindowMissed. The Rust gate (tests/gates/tests/phase8_autonomy.rs) runs
-- the same tree on four seeds.
--
--   rookery run --scenario lua/scenarios/week_1tray.lua --log runs/

ci = true
tree = "../trees/autonomy.lua"
ticks = 7 * 24 * 3600 * 50 -- one week at 50 Hz; fast-forward skips the quiet stretches

function check(log)
  local molted, lifted, missed = {}, {}, 0
  local molts, worst = 0, 0
  for _, e in ipairs(log.events) do
    local key = e.r .. "," .. e.c
    if e.kind == "Molted" then
      molts = molts + 1
      molted[key] = e.t
    elseif e.kind == "BoxLifted" and molted[key] then
      lifted[key] = true
      worst = math.max(worst, e.t - molted[key])
    elseif e.kind == "WindowMissed" then
      missed = missed + 1
    end
  end
  if missed > 0 then
    return false, string.format("%d soft-shell window(s) missed", missed)
  end
  for key, _ in pairs(molted) do
    if not lifted[key] then
      return false, "molted at " .. key .. " but never harvested"
    end
  end
  if #log.errors > 0 then
    return false, "errors: " .. log.errors[1]
  end
  if molts == 0 then
    return false, "no molt in a week"
  end
  return true, string.format("%d molts, all harvested, none missed (slowest molt → lift %.0f s)", molts, worst)
end
