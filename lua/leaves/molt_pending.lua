-- molt_pending: "success" while the molt detector sees any cell (row 8.5),
-- "failure" otherwise. bb.detected is rebuilt by Rust every tick, soonest-
-- closing window first, so this only reads it.

function tick(bb)
  if #bb.detected > 0 then
    return "success"
  end
  return "failure"
end
