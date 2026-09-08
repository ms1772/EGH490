# Standard Callouts

Short, unambiguous, always acknowledged. If a call is not acknowledged, it was
not heard — repeat it.

The point of fixed phrasing is that under stress people hear the *shape* of a
call before they hear its content. Improvised wording costs a second, and in an
8 x 6 m volume a second is the whole margin.

---

## Sequence — every flight card

| # | Caller | Call | Acknowledged by | Meaning |
|---|---|---|---|---|
| 1 | GCS | `PREFLIGHT GREEN` | all | `preflight_check.py` returned all-green |
| 2 | PIC | `KILL CHECKED` | SAFETY | Kill switch ground-tested on this airframe, this battery |
| 3 | DATA | `RECORDING` | GCS | Bag is running and growing |
| 4 | SAFETY | `VOLUME CLEAR` | PIC | No people, no loose equipment inside the net |
| 5 | PIC | `ARMING` | all | About to arm |
| 6 | PIC | `ARMED` | SAFETY | Motors live |
| 7 | GCS | `OFFBOARD` | PIC | Offboard engaged, AS2 has the aircraft |
| 8 | GCS | `<card ID> RUNNING` | DATA | Mission command sent |
| 9 | SAFETY | `<position>` | — | Continuous position commentary while airborne |
| 10 | GCS | `<card ID> COMPLETE` | DATA | Behavior reported success |
| 11 | PIC | `LANDING` | all | Descent commanded or commencing |
| 12 | PIC | `DISARMED` | all | Motors stopped, confirmed visually |
| 13 | PIC | `CLEAR` | all | Safe to approach the aircraft |

Nobody enters the netted volume before `CLEAR`. `DISARMED` is not `CLEAR`.

---

## Safety observer commentary

While the aircraft is airborne, SAFETY talks continuously. Silence from SAFETY
is itself a warning — if SAFETY goes quiet, PIC asks "SAFETY?" and prepares to
take control.

Useful phrasing, in rough order of urgency:

- `HOLDING` — stable, where expected
- `TRACKING` — moving as commanded
- `DRIFTING <direction>` — moving, not as commanded
- `LOW` / `HIGH` — approaching floor or ceiling limit
- `NEAR NET <direction>` — inside one metre of the net
- `KNOCK IT OFF` — stop now

`DRIFTING` is the important one. It is the first observable symptom of a
localisation problem, and it will be visible to SAFETY several seconds before it
is obvious on any screen. Call it early and call it often; a false `DRIFTING`
costs nothing.

---

## Readback discipline

GCS reads back every mission command **before** pressing enter, and PIC confirms:

> GCS: "Ready to send I-09, DECK-GA baseline, drone A, path key deckga_paths."
> PIC: "Confirmed, I-09, baseline, drone A."
> GCS: "Sending." … "I-09 RUNNING."

The card ID and the `--path_key` value are read aloud every time. I-09 and I-10
differ **only** in that argument, and confusing them silently invalidates both
results.

---

## What not to say

- Never `stop` — ambiguous between "stop the mission" and "hold position".
  Use `KNOCK IT OFF` or `HOLD`.
- Never `down` — ambiguous between "descend" and "it's crashed". Use `LANDING`
  or `AIRCRAFT DOWN`.
- Never `okay` as an acknowledgement. Repeat the substance of the call.
- Never narrate a diagnosis while airborne. Fly first, diagnose on the ground.
