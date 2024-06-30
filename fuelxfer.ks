@lazyglobal off.

parameter retain_lf is 100.
parameter retain_ox is round(retain_lf * 11 / 9).

local primary_lf is 0.
local primary_ox is 0.


local src to ship:partstagged("Source").

for part in src {
  for resource in part:resources {
    if resource:name = "liquidfuel" {
      set primary_lf to primary_lf + resource:amount.
    } else if resource:name = "oxidizer" {
      set primary_ox to primary_ox + resource:amount.
    }
  }
}

print "Primary LF: " + round(primary_lf).
print "Primary Ox: " + round(primary_ox).

local dest to ship:partstagged("Target").
local lf to transferall("liquidfuel", src, dest).
local ox to transferall("oxidizer", src, dest).
set lf:active to true.
set ox:active to true.

wait until lf:status = "Finished" and ox:status = "Finished".

local resv to ship:partstagged("Reserve").
local surplus_lf to 0.
local surplus_ox to 0.

for resource in resv[0]:resources {
  if resource:name = "liquidfuel" {
    set surplus_lf to max(resource:amount - retain_lf, 0).
  } else if resource:name = "oxidizer" {
    set surplus_ox to max(resource:amount - retain_ox, 0).
  }
}

print "Surplus LF: " + round(surplus_lf).
print "Surplus Ox: " + round(surplus_ox).

set lf to transfer("liquidfuel", resv, dest, surplus_lf).
set ox to transfer("oxidizer", resv, dest, surplus_ox).
set lf:active to true.
set ox:active to true.

wait until lf:status = "Finished" and ox:status = "Finished".
