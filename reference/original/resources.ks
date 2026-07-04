@lazyglobal off.

local lf to 0.
local ox to 0.
local lf_to_ox to 9/11.
local reslist to list().

list resources in reslist.
for res in reslist {
  if res:name = "liquidfuel" {
    set lf to res:amount.
  }
  if res:name = "oxidizer" {
    set ox to res:amount.
  }
}

print "Excess LF: " + round(lf - ox * lf_to_ox) + " units.".
print "Excess Ox: " + round(ox - lf / lf_to_ox) + " units.".
