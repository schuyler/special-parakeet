// === SAFE ALTITUDES ===
//
// The lowest altitude worth flying over each body, as policy in one
// place. Kerbin's bound is its atmosphere, which ends sharply at 70 km —
// one kilometre of clearance is real clearance, and the low parking
// orbits we actually launch to (72-80 km) must pass. The airless moons'
// bounds are their highest terrain plus room to be wrong. Bodies not in
// the table get a deliberately conservative fallback so nothing errors
// out (or aerobrakes by surprise) in an unplanned SOI.
//
// Callers: detune.ks (floors the phasing orbit's dip), transfer.ks
// (won't aim at a target apsis below the floor), refine.ks (won't tune a
// node into the ground), meet.ks (refuses to loiter on an unsafe orbit).

global safe_alt_table is lexicon(
  "Kerbin", 71000,  // atmosphere ends at 70 km sharp
  "Mun",     9000,  // highest peak ~7.1 km
  "Minmus",  7000   // highest peak ~5.7 km
).

// Minimum safe altitude over a body, m.
function safe_alt {
  parameter b is body.
  if safe_alt_table:haskey(b:name) {
    return safe_alt_table[b:name].
  }
  if b:atm:exists {
    return b:atm:height + 10000.
  }
  return 10000.
}

// The same bound as a body-center radius, m.
function safe_radius {
  parameter b is body.
  return b:radius + safe_alt(b).
}
