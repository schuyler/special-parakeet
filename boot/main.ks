wait until ship:unpacked.
clearscreen.
switch to 0.
print "Ready.".

if ship:status = "PRELAUNCH" {
  CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
}

on ag10 {
  CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
  return true.
}
