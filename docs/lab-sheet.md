> Historical lab sheet, kept as it was during development. Paths refer to the old
> course-framework layout (`workspace/...`): the solution is now `src/main.py`.

# Lab tuning sheet (build v71j CENTRE)

## v71j (main): bad side readings no longer steer it; recovers from bumps
- Both demo failures (v71h, v71i) happened right after the stop at 1 2, where L+R read 121 and 145 instead of
  ~104. That isn't a clean corridor (wall ends or posts in the beams).
- Now, when the two walls don't add up (more than 12 mm off), the stop does no position or heading fix, and
  the post-turn fix uses only the side nearer its normal reading.
- The side reference can only move 6 mm at a time (was 20; it jumped 52 -> 61). Heading fix at most 2.5° per
  stop.
- `HW_ON_CONTACT` 0: after a bump it backs off, marks the wall and replans, up to 6 times, then stops.

## v71i (main): gyro re-zeroing fixed for the left-encoder-off builds
- In v71f/g/h, the gyro zero was never re-measured at stops: that needs both wheels reading dead still, and
  the computed left count never is. The v71h demo started with a wrong zero (-0.08 deg/s, others -0.5..-1.7)
  and drifted left all run.
- Now the stillness check uses the raw encoder readings. Emulator with a 1 deg/s zero error: v71h 0 of 10
  finished, v71i 18 of 20.
- Spin trims: +8 for left spins, +5 for right spins (`HW_SPIN_TRIM0`, `HW_SPIN_TRIM0_CW`).
- Side offset +2 (from sidecal).

## v71f LEFTOFF (workspace/stable_v71f): never uses the left encoder
After the wires were tied (31 Oct), the left encoder counted the wrong way round and lost most forward counts:
spincheck air +63/+67 forwards, diag -280 forwards. The left wheel was also slower than the right, and the turns
took 3.6-8.6 s. v71e/v71d with that fault (emulator): 0 of 10 finished. v71f takes the left wheel's travel from
the right wheel + gyro all the time.
- Emulator: 16 of 20 finished with the fault, 9 of 10 healthy. Simulator 98.6.
- The spin trim is fixed at +6 (no learning: it needs the left encoder).
- Hardware first: both wheels must spin freely by hand, and nothing may pull on the left motor/encoder cable.

## v71e: far-wall limits (v71d: 2 of 4 runs passed)
- A far front reading (a wall 2 cells away) corrects the position by at most 30 mm. v71d: a 269 mm reading
  corrected it by -52 and the line-up drove 41 mm into a corner.
- With no wall close ahead, a line-up moves at most 15 mm.
- `HW_PIVOT_MAX` 4 (was 6).
- Fallback: `workspace/stable_v71d/board` (deploy with --src-dir), plus copy its source_main.py into
  final_task if you film with it.

## v71d: left encoder fails running backwards (spincheck)
In the air, the left counted 584/826/1010 forwards but only -185/-120/-56 backwards at PWM 40/60/80. The
right counted -439/-700/-873 backwards.
- `HW_LREV_SUB` 1: while the left wheel is commanded backwards, its travel comes from the right wheel and
  the gyro.
- Spin trim is learned only in right spins (both encoders valid), starting from `HW_SPIN_TRIM0` = +6.
- Real fix: reseat or replace the left encoder connector, then re-run `board_tools/spincheck.py`. The
  backward counts should then match the forward ones.

## v71c: left-spin fix
In the v71b logs every left (CCW) spin left the mouse 12-17 mm left. The left-spin trim ran away from -6 to -35,
while the left wheel's wrong-way repairs went up during those spins: the left encoder under-counts running
backwards.
- Trim limit 8 (was 35); trim learning slower.
- The position is frozen during every spin, not only 180s.
- Data script: `workspace/board_tools/spincheck.py` (floor spins plus in-air wheel runs).

## v71b: demo patch
- sidecal (`workspace/board_tools/sidecal.py`): sensors unchanged (offset -4). The CCW spin trim hit its 20
  limit in one spin.
- A negative spin trim now speeds the left wheel up, instead of slowing the right below its breakaway; trim
  limit 35.
- Side reflex at 40 mm (was 35).
- Explore 75 again.
- The board folder only holds `main.py` + `s4.mpy`. The tools are in `workspace/board_tools`; run them from
  Thonny.

## v71 FINAL (see workspace/stable_v71/README.md)
- Odometry learner removed: back to 5709 ticks/m. v70's learner caused the overshoot.
- One spin trim per direction; log shows `trim A B`.
- Sprint 130, explore 85, return 90. The return still stops on every cell (`HW_RETURN_STOPS` 1).
- The line-up repeats when one 30 mm move isn't enough.
- Weak wall votes are re-checked if no room is found.
- The explorer skips cells no room could contain.
- `TURN FAILED` stops the run if a spin ends >45° off.
- Charge the battery: 5 of 6 v70 logs said LOW.

## v70 LANE: v69 plus odometry learning, return stops, sprint 160 (see workspace/stable_v70/README.md)
- `HW_ODO_LEARN` 1: ticks/mm learned from still front-wall fixes (v68: 59 of 60 arrived short). The log's
  `tpm` value is the learned ticks per metre.
- `HW_RETURN_STOPS` 1: the return stops and senses on every cell.
- `HW_V_SPRINT` 160 (was 200).

## v69 LANE: walls correct the heading at every stop (see workspace/stable_v69/README.md)
- At each new-cell stop the still side readings set the sideways position. With walls on both sides, they
  also correct the gyro heading (by up to 4° per stop).
- `HW_STOP_LAT` 1 (0 = off).
- `HW_PSI_MAX` 0.10 rad (6°, was 4°). If it wobbles too much: 0.08.
- Log: `stop lat Z corr C hdg fix H`. Z = mm off centre (+ = left); it should stay within about ±6.

## v68: v67 without false encoder repairs (see workspace/stable_v68/README.md)
- `HW_ENC_POL_L` 1 for the clean MicroPython.
- The encoder repair no longer uses "which wheel was wrong before". With the lagging gyro, that flipped the
  healthy left encoder 177 times in the v67 run.
- In the log, `enc wrong-way steps` should stay in single digits.

## v67 SLOW: v66, slower (see workspace/stable_v67/README.md)
- Speeds: explore 75 mm/s, return 90, accel 300; sprint unchanged at 200.
- Spins: PWM 48 (was 52). If `turn blocked` appears, set both back to 52.
- Still look after each turn: 40 steps (was 30).
- One-wall latfix range: ±25 mm (was ±15).

## v66 TURNFIX: centred after turns (see workspace/stable_v66/README.md)

v65 run: encoders fine, but every turn left the mouse 3-17 mm LEFT of centre, and a crooked U-turn made it
over-steer. v66:
- learns per spin direction where to line up before a turn (`HW_PIVOT_LEARN` 1, `HW_PIVOT_MAX` 6 mm);
- corrects latfix for the heading error;
- re-learns the side reference up to 10° off heading;
- logs `TILT` and more sensor data.

Knobs:
- `HW_PIVOT_LEARN` 0 turns the learning off (as v65).
- `HW_PIVOT_MAX` limits it: lower it if the nose gets close to the wall ahead while spinning.

Data script: `diag66.py` (encoders, ToF offsets, gyro drift, turn displacement), ~4 min.

## v65 ENCFIX: copes with the faulty left encoder (see workspace/stable_v65/README.md)

The left encoder counts one way whichever way its wheel turns (its direction channel fails). That broke v53
and v64: every left spin looked like a ~40 mm forward move, and reversing looked like standing still. v65
checks both counts against the gyro at every step and reverses a count that went the wrong way. At the first
drive it also reverses an encoder that counts backwards, without moving the position estimate.
- New knob: `HW_ENC_FIX` 1 (0 = off, as v64).
- Log: `encoder check ... counted, ... after the gyro check`, `LEFT encoder counts backwards - reversed`, and
  `enc wrong-way steps L n R n` (grows on left turns while the fault is there).
- `HW_SENSE_STEPS` 80 (was 60): more still readings per new cell.
- Hand check: `import hw_cal`, push forward then back. `encL` must go up then down.

## v60 STABLE: rigour first (see workspace/stable_v60/README.md)

Golden v53's behaviour and motor settings (the only ones that completed mazes), slower, with more sensing:
- explore 100 mm/s, stopping on every cell for 0.6 s of still sensing;
- return 120 mm/s;
- sprint 200 mm/s;
- lines up to within 3 mm before every turn.

Only once runs are clean, one step at a time:
- `HW_V_SPRINT` +20 (faster sprint);
- if it wobbles, `HW_KPSI` 6;
- if it ends up left on straights, `HW_RIGHT_SCALE` 0.97.

## v58: your two straight-line knobs (top of main.py)

| Knob | Now | What it does | You see | Change |
|---|---|---|---|---|
| `HW_RIGHT_SCALE` | 1.00 | right wheel power when driving straight | leans / ends up LEFT | 0.97, then 0.95 |
| | | | leans RIGHT | 1.03 |
| `HW_KPSI` | 8.0 | how hard it steers back onto its heading | wobbles / over-corrects | 6, then 5 |
| | | | slow to straighten | 10 |

Change one at a time by a small step, rebuild with `zsh devtools/build_deploy.sh`, and watch a long straight.
Also in v58: pivot learning capped at 6 mm (v56 reached 10 and stopped noticeably short), and `HW_SIDE_OFFSET`
is back to 0 so the scale knob is the only thing moving the line.

> **GOLDEN BUILD: v53** completed the maze on 2 of 3 runs. It is saved unchanged in `workspace/golden_v53/`
> (source, the exact `s4.mpy`, the successful log, and how to put it back:
> `python3 tools/deploy.py --engine micropython --src-dir workspace/golden_v53/board`).

All knobs are the `HW_...` lines at the top of `workspace/final_task/main.py`. They only affect the
real mouse; the Gradescope simulator uses its own values, so tuning cannot lower the sim score.

**Deploying a change (important):** the mouse runs `workspace/final_board/s4.mpy`, a *compiled* copy of
`main.py`. After every edit run, from the repo root:

    zsh devtools/build_deploy.sh

It rebuilds `s4.mpy` and then deploys. Running `tools/deploy.py` alone sends the OLD compiled file, so
your edits would not reach the mouse. Check the first log line shows the new `S4 BUILD`.

**Method:** change ONE value, run, look at the log line named below, keep or revert. Write down what
you changed. The first log line `S4 BUILD vNN` tells you which file actually ran.

## Start position and mission (follows the brief since v48)

- **Start:** cell (0,0) is the corner cell with walls on three sides, open only to the East. Put the
  axle on its centre, facing the opening. The log must show `start walls L F R 1 0 1`; anything else
  means it was placed in the wrong cell or facing the wrong way. It returns to exactly where it started.
- **Room:** it stops and does the clockwise pirouette **as soon as** it is in the confirmed 2x2 room
  (`HW_EXPLORE_ALL = 0`), instead of mapping the whole maze first. The room may have any number of
  openings in its rim (`HW_ROOM_DOORS = 8`). Expect `ROOM found at x y doors N`, then `PIROUETTE at x y`
  within a cell or two.
- It then returns by the shortest path on the map it has, faces East, stops, waits 3.0 s and sprints.
- Film the run with the exact `main.py` you submit: the log's code checksum must match the code.
- **Maze size:** `HW_MAZE_ROWS = 4`, `HW_MAZE_COLS = 6` (4 cells to the side, 6 along the start heading).
  The board's `get_maze_dimensions()` says 6x6. With the real size the outer walls are known in advance,
  so a misread there cannot send it out of the maze. Set both to 0 to ask the board again.

## v57: v56 that fits in the board's memory

v56 froze right after SW1. The compiled program had grown with every build, from 32.7 KB (v45) to 40.3 KB, and
the board keeps all of it in its small RAM, so the most likely cause is running out of memory as the run
started. v57 removes code that no setting uses any more and is 34.4 KB. Behaviour is identical to v56. Removed:
- the reversing paths (`HW_REVERSE_OUT`, `HW_REV_HOME`);
- the start-up auto-calibrations (gyro / ticks);
- the old `HW_CALIBRATE` run (use `workspace/final_board/hw_cal.py`);
- the pre-v46 line-up and front braking (`HW_WALL_STOP`);
- `HW_STRAIGHTEN`, `HW_WALL_K`.

The second log line is now `mem free N` (bytes free after loading). If anything goes wrong, the OLED shows
`ERR <type>` and `runlog.txt` ends with `ERROR <type> <message>` instead of the mouse just freezing.

## v56: centred after every turn (keeps the v55 right-wheel fix)

In the v55 log, straight after every turn the mouse was 12-15 mm too close to the wall that had been in front
of it before the turn: on the right after left turns, on the left after right turns. Two reasons:
1. **Centred side reading never re-measured.** In continuous mode it stayed at the start cell's value all
   run (57 in v55, normally ~51), so every one-wall correction was off by ~6 mm. v56 re-measures it on the
   move at every cell with walls on both sides (`side nom N` lines) and at every still look, as v53 did.
2. **Spins end ~7 mm forward of the cell centre.** After each 90° turn lined up on a wall, the side reading
   of that wall shows how far forward the spin ended. v56 learns it (`piv N` in `align` and `latfix` lines)
   and lines up that much short before the next turn at a wall. Expect `piv` to settle at a steady value
   within ~5 turns, and `latfix` right after turns to shrink towards 0.

`HW_VMAX_R = 424`, the capped spin trim and the gentler retry spins from v55 are unchanged. `HW_PIVOT_LEARN = 0`
turns the pivot learning off.

**Which mode for the demo** (v56, 40 emulator runs each):

| Mode | Crashes / 40 | Mission time |
|---|---|---|
| continuous (default) | 10 | 111 s |
| stop on every cell: `HW_STOP_EACH_CELL = 1`, `HW_V_EXPLORE = 120` | 6 | 124 s |
| golden v53 | 4 | 121 s |

Stopping on every cell has been safer in every comparison. If a contact fails the run, the stop-each setting is
worth the extra 13 s.

## v55: right wheel toned down

v54 had one clean run, then drifted LEFT and clipped on the left. Two right-wheel boosts were stacked up:
- `HW_VMAX_R` was 412, below the 424 diag.py measured, so the right wheel got extra drive when driving
  straight (set that way back when it drifted right). Now 424: that is the slight right-wheel cut.
- In spins, the right wheel gets extra PWM so both wheels turn equally, plus a trim learned from the
  encoders. That trim may now only add or take up to 8 PWM (`HW_SPIN_TRIM_MAX`, was 20) and learns half as
  fast. Every `turn` line now ends with `trim N`, the value it is using.
- The 2nd/3rd correction tries of a turn spin at PWM 46 (`HW_TURN_PWM2`) instead of 52: less overshoot (v54
  had a 12° overshoot with 3 tries right before the crash), and each extra spin shifts the axle less.

What to look for: `latfix` right after turns averaging about 0, with no long run of the same sign. `trim` settling
to a steady value rather than sitting at +8 or -8. Still drifting left: `HW_VMAX_R` +6 (less right drive).
Drifting right: -6.

## v54: continuous exploration (no stop on every cell)

`HW_STOP_EACH_CELL = 0`: walls are read on the move. It decides each cell 60 mm before its centre, and
stops only where it turns (lines up and spins as before), or for the room. Cells it has already mapped
keep their map (`pass x y next N` lines). The `cell` lines now come from those moving votes, e.g.
`cell 1 0 d 0 walls L F R 0 1 1 votes L 1 / 16 F 14 / 16 R 19 / 20 next 1`. Votes are clear when one side of
each `a / b` is near 0 or near b.

Speeds, as tested in emulation (40 runs per setting):

| Setting (explore continuous / return / sprint, mm/s) | Crashes in 40 | Mission time |
|---|---|---|
| v53: 120 stop-each / 160 / 200 | 4 | 121 s |
| **v54: 150 / 160 / 200** | 7 | 107 s |
| 150 / 180 / 240 | 10 | 101 s |
| 200 / 200 / 260 | 9 | 92 s |
| 200 / 260 / 330 | 13 | 89 s |

Faster return and sprint crashed clearly more, so v54 keeps v53's 160 / 200 there. Map errors from reading
on the move stayed tiny (0.1-0.2% of walls). If the real mouse is clean at v54, raise
`HW_V_EXPLORE` +20 at a time first, then `HW_V_SPRINT`.

**Back to v53 behaviour in one line:** `HW_STOP_EACH_CELL = 1` and `HW_V_EXPLORE = 120`, or deploy the golden folder.

## v53: no more ending up against the left wall after turns

**Why it ended up left after every turn.** Spinning on the spot used equal PWM on both motors, but
the right motor is ~25% weaker (diag.py: 5.0 against 6.23 mm/s per PWM). The left wheel ran faster,
so the mouse rotated about a point a few mm right of the axle. After either a left or a right 90°
turn that leaves the axle ~4 mm to the LEFT of where it started; after a U-turn ~8 mm left. The side
walls measured exactly that after U-turns in the v49/v51 logs (+7..+10 mm).

v53 spins with **equal wheel speeds** (`HW_SPIN_MATCH = 1`). The right motor gets the extra PWM its
measured model needs, plus a trim learned from the encoders during each spin and kept for the next.
It also trusts the still side readings after each turn more, so `latfix` `g` is ~70-90 instead of
~50. And for the first 150 mm after a turn it may steer up to 6° (`HW_PSI_TURN`) instead of 4° to get
back to the centre line quickly.

**More room on the left, if you still want it:** `HW_SIDE_OFFSET` = +4 moves the centre line 2 mm
to the right (+ = right, by half the value).

**Log fix:** v52 emptied `runlog.txt` as soon as it powered up, including when you plugged in USB
to copy the log, so the log was gone. v53 only empties it after SW1 starts a run.

## v52: straight and centred, and any contact ends the run

**Contact = stop.** In the demo any contact fails, so v52 never backs up and carries on. On `BUMP`
(tilt), `STALL`, `PINNED`, `SLIP` or `turn blocked` it stops dead, logs
`CONTACT - run stopped (any contact fails the demo)` and ends the run (`HW_ON_CONTACT = 1`; set it to
0 to only log it and drive on). The back-up / slow-retry logic from v51 below is gone.

**Why it drifted right.** Right after every turn, `latfix` compares the mouse's estimate of its
sideways position with the still side-wall readings. After every 180 in the v49 and v51 logs the
estimate sat 7-10 mm LEFT of what the walls showed. Spinning on the spot, the wheels turn at slightly
different speeds and the odometry read that as the axle sliding sideways. So after each dead end it
set off steering right. v52 keeps the position fixed during 180° spins and the pirouette. 90° turns
showed no clear bias (typically +2 mm), so they keep normal odometry.

**Posts.** Level with a cell boundary the side sensor sees the post/prong (~10 mm closer) or half a
wall end. Next to a single wall those readings pushed the estimate the wrong way. v52 ignores side
readings while the side sensor is within `HW_POST_BLANK` = 30 mm of a boundary.

What to look for: `latfix` after turns should now show small `corr` values (the estimate already
agrees with the walls), and `th` in `cell` lines should no longer lean the same way every time.

**Measured side centre (step A).** Run `workspace/centre_test.py` (instructions at its top). Paste
its `HW_NOM_L` / `HW_NOM_R` lines into `main.py`, and they are used as they are, never re-learned. With
both at 0 it measures at the start and re-learns at cells with two walls, as before.

## v51: what it did after touching something (replaced in v52: it now stops)

In the v50 run it touched something going north from (0,2) into (0,3). It then backed off only 21 mm,
retried the same move at full speed twice more, slipped, lost its position, and the map fell apart.
After any contact (`BUMP`, `STALL`, `PINNED`, `SLIP`) v51:
1. backs off ~40 mm, straightens, re-centres sideways (`latfix`);
2. treats its position along the corridor as unknown, and re-anchors it on the wall ahead: near
   (normal `fix`) or, if only a mapped wall further away is visible, `far fix C front F`;
3. goes back to the centre of that cell and retries **once, slowly** (60 mm/s):
   `hit at a known opening - map kept, one slow retry`;
4. a second contact at the same place closes it on the map and it goes another way:
   `hit 2 times at x y side d - closed, going round`.

A between-cells stop only re-centres if the creep really reached the wall. Otherwise it logs
`re-centre failed: front still F`, or `re-centre rejected: would jump to x y` if it would move the
cell by more than one.

**If you see `hit ... at` lines at the same spot in every run, look at that spot**: a post or wall
piece sticking out, a floor seam the wheels catch, or a gap narrower than the mouse.

## v50: the log is only written standing still

Saving `runlog.txt` freezes the board for about a second while the motors keep their last power,
and the encoder counts from that second are lost. Until v49 it saved every 25 lines *whenever* the
25th line came. In the v49 run that was an `edge` line in the middle of the drive from (1,1) to (1,2):
- it drove blind for ~1.2 s and the encoders missed ~140 mm;
- it stopped between (1,2) and (1,3) and mapped a wall that is not there;
- the rest of the map broke from there.

v50 queues the lines and appends them to the file only after a stop, once the wheels are still. In the
log there is no sign of this; you may just see the mouse sit for about a second at a stop now and then.
The sprint never pauses: its lines are written at the end.

If a stop ever does land between two cells (front reading 82-147 mm, which no cell centre gives),
it logs `front F - between cells: creeping to the wall`, drives up to the wall ahead, re-centres
(`re-centred in x y front F`) and senses the cell again, instead of mapping a wall at the wrong edge.

## v49: centre line

`HW_SIDE_OFFSET` is now 0 (was 9). In the v47 log, right after each turn where the front wall becomes a
side wall, `latfix` read +4.5 mm on average. It was the same after left turns (old front wall on the
right, +4.8) and right turns (old front wall on the left, +4.2). That means the front line-up is
accurate and both side sensors read the same, while the 9 mm offset made it drive 4.5 mm right of
centre all run. Both crashes were posts on the right. In the next log, `latfix` should average about 0.

## v46: how the mouse knows where it is in a cell (read this first)

Every turn is a spin on the spot. Spinning, the front corners sweep ~86 mm round the axle and the
walls are ~94 mm away, so **the axle must be within ~5 mm of the cell centre before it spins**. v46
gets the position from three absolute references and weights each one by how much it is trusted.
`sa` in the log is the mouse's own along-track uncertainty in mm.

1. **A wall close ahead** (front reading below `HW_FRONT_NEAR` = 120). While approaching, every reading
   pulls the position a little. Before a turn it takes up to 25 readings standing still and uses the
   median. Calibrated by `HW_FRONT_CENTER = 37` (your measurement with the axle on a cell centre).
   *Longer front readings are no longer used for position.* In the v45 log, readings of a wall one cell
   further on (~200 mm) were 15-30 mm short (the beam catches the floor). That made stops land short,
   and it is why the line-up used to back the mouse up at "far" walls.
2. **A side wall ending or starting** as the side sensor passes a cell boundary (`edge` lines). The
   distance between where the reading switches and the actual boundary is learned during the run
   (`p` in the edge lines).
3. **The side walls right after a turn** (`latfix`), weighted by how many readings went into each median.
   Centring needs the side reading the mouse sees when centred. v47 re-measures it at every new cell with
   walls on both sides (`side nom N` lines, usually 48-58). Until then it uses `HW_SIDE_NOM` = 52, your
   measurement. *The v46 crash:* the start cell had one side wall, so this stayed at 60. With one wall the
   mouse then sat ~9 mm towards the open side and clipped the post where the next wall began.

Before every turn it then makes one short encoder move to the cell centre if it is more than
`HW_ALIGN_TOL` = 4 mm off. Stops no longer aim past the centre (`HW_STOP_EXTRA` 8 → 0): the 8 mm only
made up for the bad far-wall fixes.

### What the next runlog.txt should show

| Line | Good | Trouble |
|---|---|---|
| `side nom N` (after the first cell with walls on both sides) | 48-58, later lines within a few mm of it | missing for the whole run (only one-wall cells) |
| `cell … front F fix E … sa S` at a cell with a wall ahead | `front` ~30-45 (v45: 48-67), `fix` within ±8, `sa` 3-6 | `front` 55+ nearly every time = still stopping short |
| `align front F fix E move M sa S` before a turn | `move` within ±6 | `move` the same sign and 10+ on most turns |
| `edge L wo e E fix C new p P` | `e` mostly within ±15; `p` settles somewhere 3-15 | lots of `edge undo`; `p` stuck at 0 or 20 |
| `latfix Z mm corr C g G` after a turn | `Z` within ±8 on most turns | `Z` ±15-25 on most turns |

`wo` = a wall ended (wall → open), `ow` = a wall started. `new` = the cell ahead was not mapped yet
(checked once its walls are sensed, `edge undo` if they disagree); `map` = checked against the map.
v47 only accepts an edge once the new reading has lasted 40 mm. All five `edge undo` lines in the v46
run were lone posts between two open sides, so there should now be almost none.

Protection lines: `PINNED - heading stuck at A` (heading held more than 12° off for 0.3 s) and
`SLIP - wheels W gyro G deg/s` (the wheels say it is turning but the gyro disagrees for 0.2 s, i.e. a
wheel is held or slipping). Both mean a corner caught something: it backs off, straightens and
re-centres, and treats its position as unknown until the next wall fix. `side hit - map kept` and
`hit at a known opening - map kept` mean a bump did not close the passage on the map. In v46 that
false wall is why it ended with `no room found`.

### The two knobs you may touch

| What you see | Change | Step |
|---|---|---|
| Nose gets close to the wall **ahead** while spinning at a corner, or `front` in `cell` lines at wall-ahead cells is mostly below 30 | `HW_FRONT_CENTER` (now 37) up | +3 |
| It visibly stops **short** of the centre at a wall ahead (`front` mostly above 45) | `HW_FRONT_CENTER` down | -3 |
| After a run, the last `p` in the edge lines | copy it into `HW_EDGE_OFF` (now 18, from the v46 run) | it then starts the next run already calibrated |

## Calibration run (measures your mouse, prints values to paste)

1. Set `HW_CALIBRATE = 2`. Put the mouse **centred in the start cell, facing along the start corridor**
   (it needs walls on both sides and **2 clear cells ahead**). Press SW1.
2. It stands still (gyro drift), turns 180° and back (side-sensor offset), finds the PWM where spins
   start, does four 90° turns, drives one cell with each wheel speed-matched on its own encoder
   (per-wheel dead-band, gain and start PWM), then one cell on the gyro only with the walls ignored.
3. Copy the lines under `=== CALIBRATION RESULT ===` over the same names at the top of `main.py`.
   Set `HW_CALIBRATE = 0`.

`HW_CALIBRATE = 1` does the same without the 180° side-sensor step. To re-measure `HW_FRONT_CENTER`,
run `workspace/front_center.py` with the axle exactly on a cell centre facing a real maze wall.

## Two quick tests (much faster than a full maze run)

| Set | What the mouse does | Read this | Good looks like |
|---|---|---|---|
| `HW_TEST_TURN = 4` | 4 left + 4 right 90° spins in the start cell | `turn 90 err E ovs O ms T tries N coast C` per turn, then `TURN TEST done: heading X` | `err` within ±3, `ovs` < 3, `tries 1`, final heading near 0 |
| `HW_TEST_STRAIGHT = 4` | drives 4 cells straight and stops | `along … lat … head …` lines, then `STRAIGHT TEST done` | `head` stays within ±2°; it ends centred |

In the straight test, `head` going **negative** means it is turning **right**. Set both back to `0` for a
real run.

## Going straight

Each wheel runs on its own encoder speed loop (`HW_WHEEL_PI = 1`) and the heading is held on the gyro,
so a lean that survives is usually the side walls' centre line or gyro drift, not the motors.

| What you see | Change | Step |
|---|---|---|
| Drives along the corridor consistently off to one side (`latfix` averages well away from 0) | `HW_SIDE_OFFSET` (now 0): + moves the centre line RIGHT by half of it | ±3 |
| Steady pull to the right | `HW_VMAX_R` (now 412) down | -10 |
| Heading drifts while **sitting still** | `HW_GYRO_TRIM` (now 0) | drifts right → -0.2; drifts left → +0.2 |
| Weaves left-right in corridors | `HW_KLAT` (now 0.010) down | -0.002 |

## Turns

| What you see | Change | Step |
|---|---|---|
| Turn stalls / wheels buzz without turning / `tries 3` with a big `err` | `HW_TURN_PWM` (now 52) up | +4 |
| Turns are violent, `ovs` above 6 on most turns | `HW_TURN_PWM` down | -3 |
| Almost every turn needs `tries 2-3` for a 2-3° error | `HW_TURN_TOL` (now 3.0) | 4.0 |
| Touches walls when spinning in a dead end, the start cell or the pirouette | `HW_SPIN_SHIFT` (now 8 mm) | +4 |

## Bumping walls

| What you see | Change |
|---|---|
| Brushes side walls while driving | `HW_SIDE_GUARD` (now 35 mm) up to 40 |
| Calls an open side a wall, or the reverse (`votes` in `cell` lines close to 50/50) | `HW_SIDE_WALL` (now 100 mm): walls read 20-70, open sides 140+ |
| Every `fix` at wall-ahead cells is big and the same sign, and the `align … move` values too | `HW_TICKS_PER_M` (now 5709): negative fixes → up 2 %, positive → down 2 % |

## Speed (only once it is clean)

| Knob | Now | Note |
|---|---|---|
| `HW_V_EXPLORE` | 120 mm/s | +20 at a time |
| `HW_V_RETURN` | 160 mm/s | |
| `HW_V_SPRINT` | 200 mm/s | first to come down if the sprint wanders |
| `HW_ACCEL` | 400 mm/s² | lower if wheels slip when starting |
| `HW_SENSE_STEPS` | 40 | time standing still in each NEW cell; lower = faster, more wall misreads |
| `HW_FRONT_CHECK` | 30 | look-ahead after each turn (its side readings also set `latfix`); lower = faster, weaker latfix |

If something gets worse after a speed change, go back to the last value that was clean.

## Leave these alone (they are logic, not tuning)

`HW_POS_MODEL`, `HW_EDGE_FIX`, `HW_WALL_STOP`, `HW_FRONT_NEAR`, `HW_TOF_LAG`, `HW_FRONT_GAIN`, `HW_SIDE_NOM`,
`HW_TURN_BB`, `HW_STOP_EACH_CELL`, `HW_REVERSE_OUT`, `HW_REV_HOME`, `HW_KF_HEADING`, `HW_WALL_K`,
`HW_ARENA_SIDE`, the motor/encoder polarities, `HW_LOOP_FIXED`, `HW_SKIP_SEEN`, `HW_ZUPT_STOP`.
To go back to v45's positioning in one step, set `HW_POS_MODEL = 0`, `HW_EDGE_FIX = 0`, `HW_WALL_STOP = 1`.

## When to send it to Claude

Send the whole `runlog.txt` plus one sentence of what you saw. It is especially useful when there are
wrong walls on the map, `no room found`, going round in circles, `FRONT GUARD` / `HEADING LOST` /
`align skipped` lines, `edge undo` lines, a Python error, or anything these knobs do not fix.

## Log lines, short version

- `cell x y d D walls L F R … votes L a/b F c/d R e/f front N fix E th T sa S`: one new cell. `fix` = how much
  the wall ahead corrected the position (mm). `sa` = along uncertainty (mm) after it. `th` should be near a
  multiple of 90.
- `seen x y …` / `pass x y`: a cell it already knew (quick look / drove straight through).
- `at x y d D next N`: next direction (0-3); `-2` = in the room; `-1` = nothing left to explore.
- `edge L|R wo|ow e E fix C new|map p P`: a side wall ended/started at a cell boundary; `e` = position error
  it found (mm), `fix` = how much of it was applied, `p` = learned edge offset. `edge undo C`: taken back.
- `align front F fix E move M sa S`: lined up before turning (`F` = median front reading, `M` = encoder move).
- `turn D err E ovs O ms T tries N coast C`: one spin turn (see Turns above).
- `latfix Z mm corr C g G`: sideways position from still side readings after a turn; `Z` = offset the walls
  measured, `corr` = correction applied, `g` = weight in %.
- `spin spot …`, `safe turn via …`: stepped away from walls before a risky spin.
- `ROOM found`, `PIROUETTE at`, `return [...]`, `sprint [...]`, `done t …`: the mission stages.
