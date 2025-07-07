@lazyGlobal off.

cd("/landing").

run "calculate_deorbit_burn".
run "predict_terrain_impact".
run "../common".

function create_deorbit_burn {
  deletepath("deorbit_log.txt").
  clearscreen.
  local tgt is body:geopositionlatlng(0,0).
  local burn is calculate_deorbit_burn(tgt).

  print "Deorbit approach: ".
  print "  Time: " + burn:time.
  print "  Distance: " + burn:distance.
  print "  Vector: " + burn:vector.
  print "  Geo: " + burn:geo.

  add node_from_velocity(burn:vector, burn:time).
  run "../next".
  remove nextnode.

  local prediction to predict_terrain_impact().
  local info to prediction:geo + " at " + prediction:eta + "s".
  print info.
  log info to "deorbit_log.txt".

  set warp to 5.
  when alt:radar < 2000 then {
    set warp to 1.
  }
  until false {
      local info to "Δp: "+ round(surface_distance(tgt, ship:geoposition)):tostring:padleft(8) + 
            " h: " + round(alt:radar):tostring:padleft(8) +
            " p: " + ship:geoposition.
            // " Surface velocity: " + ship:velocity:surface. 
            //+ " Descent angle: " + flight_path_angle(ship:orbit, prediction:geo:terrainheight):angle.
      print info.
      log info to "deorbit_log.txt".
      wait alt:radar/1000.
  }
}

create_deorbit_burn.