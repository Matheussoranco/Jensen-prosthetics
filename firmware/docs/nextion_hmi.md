# Nextion HMI specification

The Nextion 2.4" panel runs its own UI; the firmware pushes values into named
components and interprets the touch events that come back. The panel project
(`.HMI`) is built in the Nextion Editor and flashed to the display separately
from the ESP32 firmware, so it is not source-controlled here — this document
is what you need to rebuild it from scratch.

**The display is optional.** The hand works identically with it unplugged.
`src/ui/nextion.cpp` never blocks on it, treats every received frame as
untrusted, and reports `connected()` only once a well-formed frame has
arrived.

## Link

| Setting | Value |
|:--------|:------|
| Baud | 9600 |
| ESP32 RX | GPIO 16 |
| ESP32 TX | GPIO 17 |
| Instruction terminator | `0xFF 0xFF 0xFF` |
| `bkcmd` | set to 0 by the firmware — no acknowledgements are parsed |

9600 baud is slow enough that a full refresh is ~40 ms of link time, so the
driver only sends fields that changed. A panel design that needs everything
redrawn every frame will crowd out touch events.

## Page 0 — main

Page id **0**. This is the only page the firmware talks to; extra pages are
fine as long as they do not send touch events with page id 0.

### Components the firmware writes

| Name | Type | Content |
|:-----|:-----|:--------|
| `tMode` | text | operating mode: `active`, `idle`, `hold`, `calibrating`, `fault`, `safe_torque_off` |
| `tGrip` | text | grasp pattern name: `power`, `pinch`, `tripod`, `key`, `point`, `hook`, `rest` |
| `jAperture` | progress bar | 0–100, how closed the hand is |
| `jBattery` | progress bar | 0–100, state of charge |
| `tBattery` | text | pack voltage, e.g. `7.84V` |
| `tForce` | text | peak fingertip force, e.g. `11.3N` |
| `tFault` | text | worst active fault name, or empty |
| `tPrompt` | text | calibration prompt, or empty |
| `jCal` | progress bar | 0–100, calibration progress (only written while calibrating) |

Names are the component's **objname** in the Nextion Editor, not its caption.
Missing components are harmless: the panel ignores instructions addressed to
objects it does not have.

### Components the firmware reads

Touch events arrive as `0x65 <page> <component> <event> FF FF FF`. Only
**press** events (`0x01`) on page 0 are acted on, so set each button to send
its component id on press. In the Nextion Editor this is the *Send Component
ID* checkbox on the Touch Press Event.

| Component id | Action |
|-------------:|:-------|
| 1 | next grasp pattern |
| 2 | previous grasp pattern |
| 3 | toggle hold |
| 4 | start electrode calibration |
| 5 | toggle park (safe torque off) |
| 6 | open |
| 7 | close |
| 8 | request a full refresh |
| 20 + *n* | select grasp pattern *n* directly |

Direct selection maps `20 + GripId`:

| Component id | Grasp pattern |
|-------------:|:--------------|
| 20 | rest |
| 21 | power |
| 22 | pinch |
| 23 | tripod |
| 24 | key |
| 25 | point |
| 26 | hook |

A selection for a pattern the user has disabled is ignored — `GripSelector`
never lands on a disabled pattern.

## Suggested layout

Nothing here is enforced; it is what the field set above was designed around.

```
┌──────────────────────────────────────┐
│  tMode              tBattery  ▓▓▓░░  │   mode + charge
│                               jBattery│
│  ┌────────────────────────────────┐  │
│  │ tGrip                          │  │   current pattern, large
│  └────────────────────────────────┘  │
│  aperture ▓▓▓▓▓▓▓▓░░░░░░  jAperture  │
│  force    tForce                     │
│                                      │
│  tPrompt                             │   calibration prompt
│  ▓▓▓▓▓░░░░░░░░░░░░░░░░░░  jCal       │
│                                      │
│  [ ◀ 2 ] [ grip 1 ▶ ] [ hold 3 ]     │
│  [ open 6 ] [ close 7 ] [ park 5 ]   │
│  [ calibrate 4 ]          tFault     │
└──────────────────────────────────────┘
```

Put `tPrompt` and `jCal` somewhere prominent: guiding electrode calibration is
the one job the display genuinely does better than the haptic cues alone.

## Adding fields

`NextionDisplay::update()` sends a field only when its value has changed since
the last refresh, and the "last value" cache lives in the class. To add a
field, add a cache member, compare, and call `sendText()` / `sendValue()`.
Keep the comparison coarse — `jAperture` compares at integer-percent
resolution precisely so a hand creeping by 0.1% does not saturate the link.
