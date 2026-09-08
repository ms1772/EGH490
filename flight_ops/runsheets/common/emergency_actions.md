# Emergency Actions

**Print this. Keep it visible at the GCS position and at the PIC position.**

Actions are listed in the order they are performed. Read them before the
session; do not read them during one.

---

## The abort call

> ### `KNOCK IT OFF`

Any crew member. Any time. No justification required, at the time or afterwards.
Nobody is ever questioned for calling it.

**On hearing it, immediately:**

| Role | Action |
|---|---|
| PIC | Take manual control. Position mode. Hold. If already unsafe, go to E-1. |
| GCS | Stop sending commands. Hands off the keyboard. Announce "commands stopped". |
| SAFETY | Keep eyes on the aircraft. Call its position continuously. |
| DATA | Note the time and what triggered it, verbatim. Keep the bag running. |

Nobody moves toward the aircraft until the PIC calls `CLEAR`.

---

## E-1 — Uncommanded motion or flyaway (indoor)

The netted volume is 8 x 6 m. There is no time to diagnose. Prefer a controlled
crash to a net strike at speed.

1. PIC: **mode switch to Position**. If the aircraft does not respond within
   one second —
2. PIC: **kill switch**. Do not hesitate, do not attempt to fly it out.
3. All: stay outside the net until rotors have visibly stopped.
4. GCS: stop all mission commands. Do not relaunch anything.
5. DATA: mark the bag timestamp.
6. Do not re-fly the airframe this session. The card has failed twice by
   definition.

> Killing over a net at 1 m costs a broken prop. Not killing costs the airframe,
> the net, and the booking.

## E-2 — Motion capture loss in flight

Symptoms: `volume_guard` alarms, `/mocap_health` goes untracked, the alphanumeric
viewer freezes, or the aircraft begins drifting steadily in one direction.

1. `volume_guard` (S2) should command land automatically. **Announce whether it
   did.** If it did not, that is a finding — record it.
2. PIC: take manual control immediately. **Position mode will not help** — PX4's
   estimate is derived from the same lost source. Go to **Altitude** or
   **Stabilized** and hand-fly it down.
3. If the aircraft is drifting toward the net: **kill**.
4. Do not wait to see whether tracking recovers. A recovered pose after a gap
   can produce a large jump correction, which the controller will chase.
5. After recovery: do not re-fly until the dropout is understood. Check the
   coverage map — was the aircraft in a known dead zone?

## E-3 — Loss of offboard control or GCS link

1. PIC: take manual control. Position mode. Land.
2. GCS: announce "offboard lost". Do not attempt to re-establish while airborne.
3. Expect PX4 to run its offboard-loss failsafe: **Land** (not RTL — RTL in this
   room is a manoeuvre, not a return). If it does anything else, that is a
   finding and the session ends.

## E-4 — Battery emergency

1. PIC: land immediately at the nearest clear point. Do not return to the pad.
2. If the aircraft is descending on its own, let it.
3. On the ground: disconnect, isolate the pack, do not re-use it.
4. If the pack is hot, swollen, venting, or smoking, go to **E-6**.

## E-5 — Aircraft in the net or on the floor

1. Confirm rotors stopped. **Watch, do not assume.**
2. PIC disarms and calls `CLEAR` before anyone approaches.
3. Approach from the side, never over the top of the aircraft.
4. Disconnect the battery before untangling anything.
5. Photograph the position before moving it — it is evidence for the write-up.
6. Inspect: arms, props, GPS mast, markers, battery, frame cracks.
7. Any airframe that has struck the net or floor is **grounded for the session**.

## E-6 — LiPo fire or venting

1. `KNOCK IT OFF`. Everyone away from the pack.
2. Do not use water. Do not pick it up.
3. Use the fire containment provided; if it is not immediately controllable,
   evacuate and raise the alarm.
4. Ventilate. LiPo smoke is toxic.
5. Session is over. Do not resume.

## E-7 — Person enters the netted volume

1. `KNOCK IT OFF` — anyone who sees it.
2. PIC: hold position, do not manoeuvre toward or away — hold.
3. SAFETY: verbally direct the person out, loudly and by name if known.
4. Land only once the volume is clear, or kill if the aircraft is close to them.
5. Restart the card from the beginning after the volume is confirmed clear.

---

## Two failure modes worth knowing before they happen

**Mocap loss does not look like mocap loss.** Reading the AS2 source, the
Pixhawk platform republishes the last received pose into PX4's EKF2 on a 100 Hz
timer with no staleness check. Unless the S3 staleness gate is running, PX4 will
see a *healthy, stationary* vision source and fire **no failsafe at all**. The
aircraft will look calm and then drive itself into the net correcting an error
that does not exist. This is why `volume_guard` runs on the GCS, independently,
and why E-2 says take manual control rather than wait.

**A wrong rigid-body name looks like a working system.** The stock `mocap_pose`
plugin publishes a valid-looking pose at the origin with identity orientation
when the named body is absent, at full rate, with no warning. The guarded plugin
(S1) drops the message instead. If you ever see the aircraft convinced it is at
(0, 0, 0) while sitting on the pad, stop — you are running the stock plugin.
