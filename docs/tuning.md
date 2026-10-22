# Tuning reference (v71j)

Every board constant in `src/main.py`, with the comment next to it in the source. Only `HW_*` values
affect the real mouse; the simulator profile is set separately and stays at 98.6.

| Constant | Value | Notes |
|---|---|---|
| `HW_MOTOR_POL_L` | `1` | flipped for the Oct-2026 MicroPython firmware |
| `HW_MOTOR_POL_R` | `1` |  |
| `HW_ENC_POL_L` | `1` | clean MicroPython (diag66): left counts + forwards. The first drive still checks it |
| `HW_ENC_POL_R` | `1` |  |
| `HW_LEFT_ENC_OFF` | `1` | v71f: never use the left encoder: its travel always from the right wheel + gyro (it failed 3 ways in 4 days) |
| `HW_LREV_SUB` | `1` | v71d: left wheel commanded backwards: its travel from the right wheel + gyro (spincheck: the left encoder loses most counts backwards) |
| `HW_SPIN_TRIM0_CW` | `5.0` | v71i: right spins (right wheel backwards is the faster one: demo right turns came out -11) |
| `HW_SPIN_TRIM0` | `8.0` | v71h: spincheck - left wheel ~20-25% stronger: the right needs ~+8 to match in spins (+6 left every turn 9-24 mm left) |
| `HW_ENC_FIX` | `1` | v65: undo wrong-way encoder counts with the gyro (v64: left encoder counted one way only) |
| `HW_LEFT_DEADBAND` | `20.0` | diag.py sweep 20 Oct: rolling dead-band (61 over-drove every command by ~40 PWM) |
| `HW_RIGHT_DEADBAND` | `15.0` | diag.py sweep: 15.2 |
| `HW_TICKS_PER_M` | `5709.0` | back to measured: the overshoot came from the wrong dead-band, not the encoders |
| `HW_GYRO_GAIN` | `1.0` | measured: 360 deg by hand reads +360 (0 = auto-measure at start) |
| `HW_CAL_SAMPLES` | `150` | start-up gyro samples (mouse still) |
| `HW_VMAX` | `498.0` | mm/s at PWM 100, LEFT wheel (diag: 6.23 mm/s per PWM x (100-20)) |
| `HW_TAU` | `0.06` | s, motor lag used for accel feed-forward |
| `HW_MAZE_ROWS` | `4` | lab maze: 4 cells to the side (0 = ask uct_mouse.get_maze_dimensions(); the board says 6x6) |
| `HW_MAZE_COLS` | `6` | ... and 6 along the start heading (East). Known outer walls: a misread there cannot send it out |
| `HW_ARENA_SIDE` | `0` | 0 = detect from the first side opening (+1 maze on the LEFT of the start heading, -1 RIGHT) |
| `HW_SIDE_X` | `50.0` | mm side ToF ahead of the axle |
| `HW_FRONT_OFF` | `57.0` | mm front ToF ahead of the axle (info only: the code uses CELL/2 - HW_FRONT_CENTER) |
| `HW_WALL_K` | `0.0` | deg/s of steering per mm off-centre, straight from side ToF |
| `HW_GYRO_TRIM` | `0.0` | deg/s added to gyro bias. Drifts RIGHT -> make it NEGATIVE (see straight test) |
| `HW_TEST_STRAIGHT` | `0` | >0: just drive this many cells straight and stop (calibration) |
| `HW_TEST_TURN` | `0` | >0: in the start cell do this many 90 deg turns left, then right, print each |
| `HW_CALIBRATE` | `0` | 1 = calibration run, prints the values to paste below; 2 = also side-sensor offset |
| `HW_RIGHT_SCALE` | `1.00` | straight driving: right wheel power x this, as v53. Ends up left -> 0.97 |
| `HW_KPSI` | `8.0` | heading loop (1/s), as v53 (the only settings that completed mazes). Wobbles -> 6 |
| `HW_VMAX_R` | `412.0` | RIGHT wheel, as v53 (completed mazes; v55 tried 424) |
| `HW_TURN_LATFIX` | `1` | 1 = after every turn, set the sideways position from still side readings before driving |
| `HW_PSI_MAX` | `0.10` | rad (~6 deg): max angle it steers to recentre. v69: 4 deg could not beat a 5-8 deg heading error (v67) |
| `HW_RETURN_STOPS` | `1` | v71: kept ON - continuous returns caused 4 of 9 emulator failures; v70's stopped return had the best corners on the real mouse |
| `HW_STOP_LAT` | `1` | v69: at every new-cell stop, set the sideways position from the still side medians and correct the gyro heading from the walls |
| `HW_KLAT` | `0.010` | rad heading per mm off-centre (0.007 tested worse) |
| `HW_SIDE_OFFSET` | `2.0` | v71h: sidecal #2 = +2, hand placements after the tie ~+3 (the +10 reading was skewed by spin drift) |
| `HW_STOP_EACH_CELL` | `1` | 1 = STABLE: stop on every cell centre and sense walls standing still (v53 finished mazes this way) |
| `HW_SIDE_GUARD_L` | `40.0` | v71b: steer away from the left wall from 40 mm (was 35) |
| `HW_SIDE_GUARD` | `40.0` | v71b: same on the right |
| `HW_FRONT_STOP` | `30.0` | mm: front reflex - blocked below this + speed * HW_FRONT_REACT (centre reads 37 now) |
| `HW_FRONT_REACT` | `0.15` | s: ToF latency + braking time, so faster = stop earlier |
| `HW_FRONT_CHECK` | `40` | v67: longer still look after each turn (v66: 30) |
| `HW_TURN_LEAD` | `0.06` | s: brake on the predicted stopping heading (motor lag), stops overshoot |
| `HW_TURN_KLIN` | `10.0` | 1/s: gentle linear approach for the last ~20 deg (no chatter at the end) |
| `HW_TURN_WMIN` | `20.0` | deg/s: never command less while outside tolerance (beats stiction) |
| `HW_ALIGN_MOVE` | `1` | 1 = align to the front wall with ONE encoder move from a median reading |
| `HW_ARCS` | `0` | 0 = return/sprint stop and turn on the spot at corners (safest) |
| `HW_KEEP_IW` | `1` | 1 = keep the learned L/R motor-imbalance trim across stops (anti-veer) |
| `HW_STOP_FIX` | `1` | 1 = at each cell stop, a wall seen ahead (<500 mm) fixes the along position |
| `HW_PIVOT_BACK` | `0.0` | mm short of centre before turning (backing off put the axle off-centre after the turn: left wheel caught the corner) |
| `HW_REVERSE_OUT` | `0` | 0 = 180 spin in dead ends (reversing is blind: drifted right into walls behind) |
| `HW_REV_HOME` | `0` | 0 = drive into the start forwards, then spin to face the start heading |
| `HW_SPIN_SHIFT` | `8.0` | mm step off a wall before a risky spin (15 left it off-centre into the next corridor) |
| `HW_V_REVERSE` | `100.0` | mm/s top speed backwards (slow: more side-wall fixes per mm) |
| `HW_SENSE_STEPS` | `80` | 10 ms steps of standing-still wall sensing per NEW cell (v53: 40, v64: 60) |
| `HW_SKIP_SEEN` | `1` | 1 = cells already mapped: no wall sensing, just a quick front fix |
| `HW_WALL_STOP` | `0` | 0 = stop on the position estimate (the near front wall corrects it while approaching) |
| `HW_ZUPT_STOP` | `1` | 1 = re-estimate the gyro bias while standing still (both wheels dead still) |
| `HW_TURN_BB` | `1` | 1 = spin turns at a fixed PWM, cut early, coast onto the heading |
| `HW_TURN_PWM` | `48` | v67: slower spins, less overshoot/slide (v66: 52; wheels start ~42-46) |
| `HW_TURN_COAST` | `0.045` | s: coast per deg/s after cutting (diag: ~7 deg at 200 deg/s); learned each turn |
| `HW_KICK_L` | `0` | start kick off: per-wheel speed control starts both wheels together (diag.py had none) |
| `HW_KICK_R` | `0` | HW_STOP_CREEP     = 45.0        # mm/s: slowest speed into a stop (70 arrived too hot); stop swing is fixed by HW_STRAIGHTEN |
| `HW_BRAKE_HOLD` | `0` | 1 = gyro-steered roll-out at stops (tested worse in emulation: off) |
| `HW_STRAIGHTEN` | `0.0` | deg: 0 = off (correction spins after stops looked like wiggling) |
| `HW_EXPLORE_ALL` | `0` | 0 = brief: pirouette as soon as it is in the confirmed room (1 = map the whole maze first) |
| `HW_EXPLORE_TMAX` | `200.0` | s: ... but stop exploring after this long once the room is known |
| `HW_STOP_EXTRA` | `0.0` | mm past the cell centre at every stop (8 was making up for the bad far-wall fixes) |
| `HW_WHEEL_PI` | `1` | 1 = each wheel on its OWN encoder speed loop (diag.py's controller: straight, square stops) |
| `HW_WHEEL_KP` | `0.12` | PWM per mm/s of wheel speed error (diag.py) |
| `HW_WHEEL_KI` | `0.8` | PWM per mm of accumulated wheel speed error (diag.py) |
| `HW_LOOP_FIXED` | `1` | 1 = fixed 10 ms loop period (wait only the rest of the period) |
| `HW_FRONT_CENTER` | `37.0` | mm front reading with the axle on a cell centre (MEASURED: 155 readings, median 37) |
| `HW_FRONT_NEAR` | `120.0` | mm: front readings set the position only below this (longer ones read 15-30 short) |
| `HW_FRONT_GAIN` | `0.12` | share of each near front reading's error applied while driving towards the wall |
| `HW_POS_MODEL` | `1` | 1 = keep an along/sideways uncertainty; every position fix is weighted by it |
| `HW_EDGE_FIX` | `1` | 1 = a side wall ending/starting (cell boundary) fixes the along position on the move |
| `HW_EDGE_OFF` | `18.0` | mm: start value - a wall END is seen this far past the boundary, a START this far before (learned: "p" in edge lines; v46 run: 17-19) |
| `HW_SIDE_NOM` | `52.0` | mm side reading with the axle centred (measured L 50 / R 53); used until a cell with walls on both sides re-measures it |
| `HW_NOM_L` | `0.0` | mm LEFT reading at the TRUE centre, from workspace/centre_test.py (0 = measure at start, re-learn) |
| `HW_NOM_R` | `0.0` | mm RIGHT reading at the TRUE centre (set both, or neither) |
| `HW_TOF_LAG` | `0.02` | s: age of a ToF reading (the mouse has moved speed x lag since) |
| `HW_ALIGN_TOL` | `3.0` | mm: before a turn, move to the cell centre if further off than this |
| `HW_ON_CONTACT` | `0` | v71j: 0 = back off and drive on after a bump (the demo allows recovering); at most 6 per run |
| `HW_POST_BLANK` | `30.0` | mm: ignore side readings while the side sensor is this close to a cell boundary (posts, wall ends) |
| `HW_SPIN_MATCH` | `1` | 1 = spin with EQUAL wheel speeds (right motor ~25% weaker: equal PWM shifted the axle left on every turn) |
| `HW_SPIN_TRIM_MAX` | `12.0` | v71d: 12 (learned only from right spins now; start +6) |
| `HW_TURN_PWM2` | `48` | v67: correction spins as slow (v66: 52) |
| `HW_PIVOT_LEARN` | `1` | v66: learn per spin direction where to line up so the axle ENDS on the centre (v65: 3-17 mm left after every left turn) |
| `HW_PIVOT_MAX` | `4.0` | v71e: 4 (a -6 line-up put a right spin ~5 mm past the centre before a sprint crash) |
| `HW_PSI_TURN` | `0.105` | rad (~6 deg): max recentring steer for the first 150 mm after a turn (off tested worse) |
| `HW_TILT_LIM` | `0.25` | climb detector: accel tilt as a fraction of g (0 = off) |
| `HW_SIDE_WALL` | `100.0` | mm: walls read 20-70, open sides 140+ (measured) |
| `HW_FRONT_FIX_MAX` | `150.0` | mm: trust front ToF for distance only below this |
| `HW_FRONT_SNAP` | `30.0` | mm: front fix only if within this of a wall line |
| `HW_TOF_SIGMA` | `20.0` | mm side ToF noise (tof_check: spread ~+-20 standing still) |
| `HW_V_EXPLORE` | `75.0` | v71b: back to v70's 75 for the demo runs |
| `HW_V_RETURN` | `90.0` | v67: slower for rigour (v66: 120) |
| `HW_V_SPRINT` | `130.0` | v71: slower (v70: 160); every v70 failure was in the sprint, still 1.5x explore |
| `HW_V_TURN` | `170.0` | mm/s through smooth 90 deg arcs |
| `HW_ACCEL` | `300.0` | v67: gentler starts/stops (v66: 400) |
| `HW_W_TURN` | `180.0` | deg/s in-place turn ceiling |
| `HW_ALPHA_TURN` | `1000.0` | deg/s^2 |
| `HW_TURN_TOL` | `3.0` | deg: accept the spin within this (1.5 caused extra back-and-forth corrections) |
| `HW_ROOM_DOORS` | `8` | max openings in the target room's rim (8 = any: the brief allows arbitrary openings) |
| `HW_KP_V` | `0.10` | effort per mm/s (common mode) |
| `HW_KI_V` | `0.6` |  |
| `HW_KP_W` | `0.08` | effort per deg/s (gyro rate loop; was 0.06 - snappier heading hold) |
| `HW_KI_W` | `0.3` | was 0.5: with the big right dead-band it over-corrected |
| `HW_WAIT_BUTTON` | `1` | 1 = press SW1 to start (board only) |
| `HW_DEBUG` | `1` | 1 = print sensing/decisions (run tethered via mpremote) |
| `HW_TPMM_AUTO` | `0` | 1 = measure encoder ticks/mm at start (spin vs gyro) |
| `HW_KF_HEADING` | `0` | 0 = heading from the gyro only (diag: gyro-only straight was perfect; walls caused bends) |
| `HW_SYSID` | `0` | 1 = measure dead-bands/ticks per mm with a start spin |

The full history of the lab sheet used during development is in [lab-sheet.md](lab-sheet.md).
