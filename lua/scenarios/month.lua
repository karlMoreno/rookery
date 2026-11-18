-- Phase 8 soak (row 8.7): one tray runs unattended for a sim-month. Every
-- molt is harvested inside its window, the worker clears every box off the
-- exit tray two hours later, and nothing errors. Not run in CI (it takes
-- several minutes); run it overnight on the Air:
--
--   caffeinate -i cargo run --release -p rookery -- run --scenario lua/scenarios/month.lua --log runs/ --id soak
--
-- A Rust panic aborts the process on the Air, so a clean exit code is the
-- "no panics" check; watch memory with `ps -o rss= -p <pid>` while it runs.

ci = false
tree = "../trees/autonomy.lua"
ticks = 30 * 24 * 3600 * 50 -- 30 days at 50 Hz; fast-forward skips the quiet stretches

function check(log)
  local molted, lifted, harvested = {}, 0, 0
  local molts, missed = 0, 0
  for _, e in ipairs(log.events) do
    local key = e.r .. "," .. e.c
    if e.kind == "Molted" then
      molts = molts + 1
      molted[key] = true
    elseif e.kind == "BoxLifted" then
      lifted = lifted + 1
    elseif e.kind == "Harvested" then
      harvested = harvested + 1
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
  if lifted < molts then
    return false, string.format("%d molts but only %d boxes lifted", molts, lifted)
  end
  if harvested < lifted - 1 then
    return false, string.format("%d boxes lifted but only %d cleared off the exit tray", lifted, harvested)
  end
  return true, string.format("%d molts, %d lifted, %d cleared off the exit tray, none missed", molts, lifted, harvested)
end
