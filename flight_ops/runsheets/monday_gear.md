# Monday O-134 — Friday evening prep and packing

One aircraft (drone0), 2.5 hours, four short flights at most. Tick as packed.

## Power

    [ ] 4x 4S flight packs, fully charged Friday night, voltage written on tape on each (≥16.6 V)
    [ ] 1 more pack at storage charge if there is one
    [ ] Charger + mains lead, to top up between flights in the lab
    [ ] LiPo fire bag (fire containment is a go/no-go item)
    [ ] Whatever powers the Jetson on the bench (its bench supply / USB-C PD bank + cable)
    [ ] Laptop charger. Phone charger.

## Airframe

    [ ] Props: 2 CW + 2 CCW fitted-ready, plus one full spare set (4). Prop wrench.
    [ ] Reflective markers: at least 5 on the airframe, ASYMMETRIC pattern (no
        square, no line), on the top plate / arm tops where the cameras see them
        through spinning props. Spares: 4 markers + bases. Double-sided tape / Velcro.
    [ ] Nose marked (tape arrow) so "nose along +X" in Motive is unambiguous
    [ ] Hex keys 2 / 2.5 / 3 mm, small Phillips, zip ties, electrical tape, scissors, spare M3s
    [ ] A stand or box to sit the aircraft on for the 10-minute mocap noise watch
    [ ] A cloth big enough to cover every marker at once (G2 / G3 step 9)

## Radio

    [ ] Transmitter charged, bound to this receiver, tested Friday (bars move in QGC Radio)
    [ ] Mode switch: Stabilized / Altitude / Position on a 3-pos; kill on a 2-pos.
        Do the QGC mapping Friday if the FC is at hand (`param show RC_MAP_KILL_SW` non-zero)

## Cables

    [ ] USB cable FC ↔ laptop (QGC). Check it is a data cable, not charge-only.
    [ ] Ethernet cable, 5 m, spare: GS ↔ Jetson direct if the lab wifi is hopeless
    [ ] Spare short Ethernet for the Jetson ↔ FC link, same type as fitted

## Paper

    [ ] Printed: runsheets/monday_o134.md, common/emergency_actions.md,
        common/callouts.md, common/crew_brief.md (two copies of the first two)
    [ ] Pen, marker, tape measure (frame check, marker offsets), clipboard

## Friday software prep (needs the Jetson powered and on wifi; 15 minutes)

    [ ] From the GS, copy the launcher trio and the mocap script to the Jetson — the
        bundle does not ship them and tmuxinator is not in packages.txt:
        scp "$R"/flight_ops/{session_bringup.sh,session_stop.sh,preflight_check.py} jetson@<jetson-ip>:~/as2_o134_ws/flight_ops/
        scp "$R"/flight_ops/deploy/mocap_bringup.sh jetson@<jetson-ip>:~/as2_o134_ws/flight_ops/deploy/
        ssh jetson@<jetson-ip> 'chmod +x ~/as2_o134_ws/flight_ops/*.sh ~/as2_o134_ws/flight_ops/deploy/*.sh; sudo apt-get install -y tmuxinator'
    [ ] On the Jetson: `source ~/as2_o134_ws/setup_env.sh; bash ~/as2_o134_ws/flight_ops/session_bringup.sh --hardware --role drone --drones drone0 --print-plan`
        prints a plan and exits 0
    [ ] On the Jetson: `bash ~/as2_o134_ws/flight_ops/deploy/mocap_bringup.sh --print-plan --server 1.2.3.4 --drone drone0` exits 0
    [ ] On the GS, WSL interactive terminal: `rviz2` opens a window (WSLg). If not,
        Monday runs with `--no-rviz` and the topic echoes.
    [ ] On the GS, dry-run the sheet's G2 commands against the fake VRPN server built
        11 Sept (WSL home, not in the repo):
        ~/o134_mocap_test/fake_vrpn_server 3883 drone0 100 &
        bash "$R/flight_ops/deploy/mocap_bringup.sh" --server 127.0.0.1 --gs      → MOCAP OK
        bash "$R/flight_ops/deploy/mocap_bringup.sh" --stop --gs; pkill -f 'fake_vrpn_server 3883'
    [ ] Write the Jetson's current wifi IP on the run-sheet (ssh needs it; DHCP)
    [ ] QGC on the laptop opens and talks to the FC over USB; params read back as listed on the sheet
    [ ] Phone: photo of the Motive Streaming pane from EGH450 if anyone still has one
