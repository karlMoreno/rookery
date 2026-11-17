-- wait(ticks): "running" for `ticks` ticks (20 ms each at 50 Hz), then
-- "success" once, then starts counting again.

local count = 0

function tick(bb, ticks)
  count = count + 1
  if count >= ticks then
    count = 0
    return "success"
  end
  return "running"
end
