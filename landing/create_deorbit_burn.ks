@lazyGlobal off.

run "calculate_deorbit_burn".
run "../common".

function create_deorbit_burn {
  clearscreen.
  local tgt is body:geopositionlatlng(0,0).
  local burn is calculate_deorbit_burn(tgt).

  add node_from_velocity(burn:vector, burn:time).
  run "../next".

  until false {
      print "Ground error: "+surface_distance(tgt, ship:geoposition) + 
            " Surface velocity: " + ship:velocity:surface. 
            //+ " Descent angle: " + flight_path_angle(ship:orbit, prediction:geo:terrainheight):angle.
      wait alt:radar/1000.
  }
}

create_deorbit_burn.