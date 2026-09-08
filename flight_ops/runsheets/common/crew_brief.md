# Crew Brief

**Read aloud before the first card of every session. Every crew member present.**
No card is flown until this brief is complete and every role is filled and
acknowledged out loud.

---

## 1. Roles

| Role | Call | Responsibility |
|---|---|---|
| **Pilot in Command** | "PIC" | On the sticks, transmitter in hand, thumbs on the mode switch. Final authority on the aircraft. Never leaves the transmitter while a battery is connected. |
| **GCS Operator** | "GCS" | Terminals, launches, mission commands. Reads back every command before executing. Announces every state change. |
| **Safety Observer** | "SAFETY" | Eyes on the aircraft, nothing else. No screen, no laptop, no phone. Calls the abort. |
| **Data Recorder** | "DATA" | Fills the cards, times each run, notes anomalies verbatim, confirms bags are recording. |

**Doubling up:** PIC and Safety Observer may **never** be the same person.
GCS and Data Recorder may be doubled if crew is short. With three people:
PIC, GCS+Data, Safety.

Minimum crew to fly: **three**. Minimum crew for a props-off card: **two**.

---

## 2. Brief content — say all of it

1. **Objective of this session.** Which cards, in what order, and where we stop.
2. **The envelope.** Volume dimensions, ceiling, geofence limits, maximum
   commanded speed. State the numbers, do not gesture at them.
3. **The abort word.** `KNOCK IT OFF`. Anyone may call it. No justification
   required, at the time or afterwards.
4. **Emergency actions.** Walk the `emergency_actions.md` cards. Confirm each
   crew member knows their action for a flyaway, a mocap loss, and a LiPo event.
5. **Kill switch.** PIC states which switch, and demonstrates the motion.
   Kill switch is ground-tested before every flight card, no exceptions.
6. **Who is inside the netted volume, and when.** Default: nobody, while a
   battery is connected.
7. **Anything different today.** New airframe, new config, a card that failed
   last session, a crew member who wasn't at the last session.
8. **Go / no-go poll.** Round the crew by name. Each answers "go" or "no-go"
   out loud. **Any single no-go stops the session** until it is resolved.

---

## 3. Session-level rules

**Build-up discipline.** Cards run in order. Each card changes exactly **one**
variable from the card before it. No card is skipped to "save time" — the
skipped card is the one that would have caught the problem.

**Two-failure rule.** Any card that fails twice **ends the session**. We do not
debug in O-134. The failure is reproduced off-site on R1/R2/R3, fixed, re-proven,
and the card returns next session. Sessions are lost to open-ended debugging,
not to failed cards.

**No configuration is edited in the lab.** Not a YAML file, not a parameter, not
a launch argument. Everything comes from `flight_ops/lab_config/` under version
control, launched by `session_bringup.sh`. If something needs changing, the card
fails and we pack up.

**`Proven on:` is binding.** A card whose `Proven on:` field is empty has not
been rehearsed and is not flown. This is not negotiable under time pressure —
time pressure is exactly the condition it exists to protect against.

**Preflight gates props-on.** `preflight_check.py` must return all-green before
any propeller is fitted. A single amber line stops the card.

---

## 4. Before the first card

- [ ] Snapshot taken: `snapshot.py --label <session>-pre --check` returned 0
- [ ] Rollback artifacts present in the snapshot directory and readable
- [ ] Crew brief read aloud, all roles acknowledged
- [ ] Go / no-go poll complete, all "go"
- [ ] Netted volume clear of people and loose equipment
- [ ] Batteries charged, logged, and voltage-checked
- [ ] Kill switch ground-tested on the airframe being flown
- [ ] Fire containment for LiPo present and accessible
- [ ] Bag recording confirmed running
- [ ] Session stop time agreed and written on the first card

## 5. After the last card

- [ ] All batteries disconnected and logged
- [ ] Bags offloaded and checksummed
- [ ] `collision_audit.py` run on the session's bags
- [ ] Cards collected, signed, and scanned
- [ ] Post-session snapshot taken with `--baseline` pointing at the pre-session one
- [ ] Any drift reported by the snapshot reviewed and explained in writing
- [ ] Rollback applied if the lab hardware is being handed back
- [ ] Anomalies written up while memory is fresh, not later
