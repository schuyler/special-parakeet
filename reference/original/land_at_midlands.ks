set target to vessel("Midlands Base Ship").
set tgt to target:geoposition.
set offset_east to 0.004.
set offset_north to -0.004.
set h_gate to 1200.
run plan_doi(tgt:lat+offset_north, tgt:lng+offset_east,h_gate).
run next.
if hasnode {
  remove nextnode.
}
run powered_descent(tgt:lat+offset_north, tgt:lng+offset_east, h_gate).
