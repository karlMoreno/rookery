-- at_target(tol): "success" when the crane has nothing left to do: the
-- route is empty and every measured joint is within `tol` metres of its
-- target (default 0.002, mast::DONE_TOL). Otherwise "failure". It is a
-- condition, so it never returns "running".

function tick(bb, tol)
  tol = tol or 0.002
  if bb.route_len ~= 0 then return "failure" end
  for i = 1, 4 do
    if math.abs(bb.gantry_q[i] - bb.gantry_target[i]) > tol then return "failure" end
  end
  return "success"
end
