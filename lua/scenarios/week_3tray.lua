-- Phase 10 demo: the three-shelf rack runs unattended for two sim-weeks
-- (row 10.5). Every molt on every shelf is harvested inside its soft-shell
-- window: no WindowMissed. The Rust gate runs the same tree on four seeds
-- (TRAYS=3 cargo test -p gates --test phase8_autonomy).
--
--   rookery run --scenario lua/scenarios/week_3tray.lua --log runs/

ci = true
scene = "../../scene/generated/rack3.xml"
tree = "../trees/autonomy.lua"
ticks = 14 * 24 * 3600 * 50 -- two weeks at 50 Hz: 12 cells molt less often than 16

function check(log)
  local molted, lifted, missed, molts, trays = {}, {}, 0, 0, {}
  for _, e in ipairs(log.events) do
    local key = e.tray .. "," .. e.r .. "," .. e.c
    if e.kind == "Molted" then
      molts = molts + 1
      molted[key] = true
      trays[e.tray] = true
    elseif e.kind == "BoxLifted" then
      lifted[key] = true
    elseif e.kind == "WindowMissed" then
      missed = missed + 1
    end
  end
  if #log.errors > 0 then
    return false, "errors: " .. log.errors[1]
  end
  if missed > 0 then
    return false, string.format("%d soft-shell window(s) missed", missed)
  end
  for key, _ in pairs(molted) do
    if not lifted[key] then
      return false, "molted at tray,r,c " .. key .. " but never harvested"
    end
  end
  if molts == 0 then
    return false, "no molt in two weeks"
  end
  local n = 0
  for _ in pairs(trays) do n = n + 1 end
  return true, string.format("%d molts on %d shelves, all harvested, none missed", molts, n)
end
