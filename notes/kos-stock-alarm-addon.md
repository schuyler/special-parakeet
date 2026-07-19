# A kOS addon for the stock alarm clock

*Feasibility study for a kOS addon that reads and writes the stock KSP alarm clock — the
app KSP 1.10 shipped and 1.12 finalized — so a KerboScript program can, for example, set
a maneuver-node alarm the way the in-game UI does. Not the Kerbal Alarm Clock mod (kOS
already binds that as `ADDONS:KAC`); the stock system in `Assembly-CSharp.dll`. Every API
claim below was read out of the installed binary (`KSP_x64_Data/Managed/Assembly-CSharp.dll`,
build 03190 = KSP 1.12.5) with `monodis`, not from memory or docs — the stock alarm API is
undocumented, so the binary is the only authority.*

## Prior art: none

kOS has an open feature request — [issue #2983, "Support 1.12 stock alarm clock
app"](https://github.com/KSP-KOS/KOS/issues/2983) — with no assignee, no branch, and no PR.
kOS's only alarm binding is the Kerbal Alarm Clock mod. kRPC, the other scripting bridge,
is the same: KAC only. So this is greenfield, and #2983 is a natural home for a PR.

## The two interfaces

### kOS side — the addon pattern

kOS discovers addons by reflection over `[kOSAddon]`-tagged classes. The KAC binding under
`src/kOS/AddOns/KerbalAlarmClock/` is the template to copy. Three pieces:

- **The addon object.** A class extending `kOS.Suffixed.Addon`, tagged
  `[kOSAddon("ALARMCLOCK")]` and `[KOSNomenclature("StockAlarmAddon")]`, reached from script
  as `ADDONS:ALARMCLOCK`. It overrides `Available()` and registers members with
  `AddSuffix("ALARMS", …)`.
- **Global functions.** `ADDALARM` / `LISTALARMS` / `DELETEALARM` are separate classes, each
  tagged `[Function("addalarm")]` and extending `FunctionBase`.
- **A wrapper structure** per alarm, extending `kOS.Suffixed.Structure`, exposing the alarm's
  fields as suffixes (`:NAME`, `:UT`, `:REMAINING`, …).

### Game side — the stock API (verified present in the binary)

- `AlarmClockScenario` — a `ScenarioModule` singleton (`.Instance`), holding the master list
  in a **public** field `alarms`, typed `DictionaryValueList<uint, AlarmTypeBase>` (usable as
  both dict-by-id and list). Public methods `AddAlarm` and `DeleteAlarm`.
- `AlarmTypeBase` — abstract base, with **public** fields: `title`, `description`,
  `ut` (double, absolute trigger UT), `eventOffset`, `vesselId` (uint), `vesselName`,
  `actions`; and properties `TimeToAlarm` / `TimeToEvent`.
- `AlarmActions` — public fields `warp`, `message`, `deleteWhenDone`, `playSound` (the first
  two are enums controlling warp-stop and the message dialog).
- Concrete types present: `AlarmTypeRaw`, `AlarmTypeManeuver`, `AlarmTypeApoapsis`,
  `AlarmTypePeriapsis`, `AlarmTypeSOI`, `AlarmTypeTransferWindow`.

## Why this is simpler than the KAC binding

KAC is an external DLL that may be absent, so kOS wraps it in `KACWrapper.cs` — ~26 KB of
hand-written reflection glue, and `Available()` reports whether the mod loaded. The stock
alarm clock lives in `Assembly-CSharp.dll`, which kOS already compiles against. You
reference `AlarmClockScenario` and `AlarmTypeBase` as ordinary types — no reflection wrapper
— and `Available()` returns a constant `true` (the app is stock in every 1.12 install). Half
the KAC binding's bulk simply doesn't exist here.

## The maneuver-node case, settled

The worry was that `AlarmTypeManeuver` might only be constructible through its own UI. The
binary says otherwise. Its only constructor is **parameterless** (it news-up empty
`List<ManeuverNode>` fields); the node reference lives in a private field `maneuver`. But
there is a **public property setter** `Maneuver { get; set; }` (`set_Maneuver(ManeuverNode)`)
and a public `UpdateAlarmUT()`. So a faithful maneuver alarm is reachable with public members
only — no reflection into private fields:

    var a = new AlarmTypeManeuver();
    a.Maneuver = node;        // public setter, stores the private field
    a.UpdateAlarmUT();        // derives ut from the node's burn time
    AlarmClockScenario.Instance.AddAlarm(a);

Other public members on the type worth knowing when we implement: `RequiresVessel()`,
`CanSetAlarm(displayMode)`, `CannotSetAlarmText()`, and `useBurnTimeMargin` / `marginEntry`
fields (the "alarm N seconds before the burn" offset the UI exposes). These are the guards
the UI runs before it lets you place the alarm; the addon should call them too and surface a
clean error rather than push an alarm the game would reject.

### The easy fallback

If linking the live `ManeuverNode` proves fussy in flight (node objects can go stale across
warp and patch recomputation), the certain path is an `AlarmTypeRaw` dropped at
`nextnode:time` minus a lead. It loses only the maneuver label and icon, and the burn-time
math the script already does stands in for `UpdateAlarmUT()`. Worth having as the v1 target,
with the faithful `AlarmTypeManeuver` as v2.

## Open questions before building

1. **`AddAlarm`'s exact signature.** Confirmed to exist and be public; the parameter list
   wasn't cleanly extracted from the IL. Almost certainly `AddAlarm(AlarmTypeBase)`, but read
   it before writing the call.
2. **Packaging.** The clean home is a PR into kOS against #2983, compiled into `kOS.dll`.
   Whether a *standalone* DLL that merely references `kOS.dll` gets its `[kOSAddon]` scanned
   at load is unconfirmed — kOS's addon discovery walk needs checking before betting on that
   path.
3. **Save/load.** Stock alarms persist through `AlarmClockScenario` (the type has
   `OnAlarmSave`/`OnAlarmLoad`). An alarm the addon creates should survive a save/reload; a
   test flight has to confirm it, since that's the whole point over a bare in-script
   `WAIT UNTIL`.

## Verdict

Feasible, low-to-moderate effort. The kOS half is a well-worn pattern; the game half is
present, mostly public, and simpler to bind than the KAC mod already bound. The maneuver-node
goal that motivated this is reachable through public members. Building it is a Standard-tier
task; the first uncertainty to close is the `AddAlarm` signature, then decide v1-Raw vs
v1-Maneuver.
