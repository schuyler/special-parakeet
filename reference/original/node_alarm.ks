// plan_alarm_lead — how early a stock alarm fires before its event —
// comes from core/planning.ks, located relative to this file (common.ks's
// idiom) so the caller's current directory is not disturbed.
runoncepath(scriptPath():parent:parent:combine("core", "planning.ks")).

print "".
print "== NODE ALARM ==".

if not hasnode {
  print "No maneuver node to alarm on.".
} else if not addons:alarmclock:available {
  print "Stock alarm clock addon not found.".
} else {
  set nd to nextnode.
  set alarm to addmaneuveralarm(nd, plan_alarm_lead).

  // title the alarm with the ship's name so it's identifiable in the
  // stock Alarm Clock app, where several vessels' alarms share one list
  set alarm:name to ship:name.

  print alarm:name + ": fires in " + round(alarm:remaining) + "s".
}
