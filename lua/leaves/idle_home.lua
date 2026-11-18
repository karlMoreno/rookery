-- idle_home: nothing to harvest. Every transfer already ends at home, so
-- there is nothing to do but report "success", which lets the tree start
-- again from the top on the next tick (row 8.5). With the crane idle the
-- clock fast-forwards to the next molt.

function tick(bb)
  return "success"
end
