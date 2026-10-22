# The journey: v15 → v71j

The git history has one commit per version. This page tells the story: what each phase tried, what the run
logs showed, and what was learned. Every real-mouse run log is in [`logs/board`](../logs/board).

---

## Phase 1: simulator baseline (v15)

The solver was written against the course physics simulator first:
- floodfill exploration;
- 2×2 room detection;
- a 360° pirouette on the gyro;
- turn-weighted Dijkstra home;
- a sprint with merged straights.

Corridor centring used a 2-state Kalman filter. **98.6 / 100** on the graded seeds.

## Phase 2: first contact with the real mouse (v16–v27)

The simulator's 70 mm collision circle hid the real footprint:
- Spinning on the spot, the front corners sweep about 86 mm round the axle, against about 94 mm to a wall.
- **v16** clipped corners on every turn.
- **v17** added safe turns: back off a wall before spinning, 180s towards the open side, step away from walls
  before the pirouette.
- **v18** spins at fixed PWM and cuts early on a learned coast, for motors that stalled below about 75 PWM
  from rest.
- **v27:** the flight recorder showed the motor model was wrong. The rolling dead-band was **20/15**, not
  61/64, so every command had been over-driving by about 40 PWM.

## Phase 3: tuning on hardware (v28–v45)

- Heading from the gyro only (v28). Wall-based heading caused bends in straight tests.
- A sideways fix from still side readings after every turn (v33).
- Each wheel on its own speed loop (v38).
- Measured the front-sensor geometry (v39).
- Slowed everything down (v41): explore 120, return 160, sprint 200 mm/s.
- About a dozen front-centre and side-offset tweaks (v30–v45). Each moved the problem around rather than
  fixing it.

## Phase 4: a real position model (v46–v52)

**v46** replaced the guesswork with an along-track uncertainty `sa`:
- every position fix is weighted by it;
- side-wall edges (where a wall starts or ends) are used as cell-boundary landmarks;
- the line-up before turns must be within 4 mm.

Then:
- **v47** re-learns the side reference and adds a PINNED detector.
- **v48** follows the brief: pirouette as soon as the room is confirmed.
- **v52:** any contact ends the run (a touch fails the demo anyway). Before this, v38–v41 completed
missions only after 1–8 bumps that were backed off and ignored.

## ⭐ Phase 5: golden v53

The right motor is about 25% weaker. At equal PWM the mouse rotated about a point right of the axle, and
every left turn shifted it about 4 mm left. **v53 spins with equal wheel *speeds***.

**First complete mission under the no-contact rule** (since v52 any touch ends the run): explore,
room, pirouette, return, sprint, done in 123 s. Earlier builds (v38–v42) finished missions too, but
while bumps were still backed off and ignored.
Completed 2 of 3 runs. (`logs/board/v53-golden-success.txt`)

## Phase 6: speed attempts (v54–v59)

- **v54** explored continuously, without stopping on every cell.
- **v55–v59** re-tuned the motors and steering.

Each was worse: 10–12 crashes in 40 emulator runs, against 4 for v53's settings. **v56** froze on the board:
the 40 KB build didn't fit in RAM. **v57** trimmed dead code.

## Phase 7: STABLE (v60–v64)

Back to v53's motor settings, slower (explore 100), stopping and sensing on every cell:
- **v61:** a truly-still gyro calibration.
- **v63:** the encoders are checked on the first drive.
- **v64:** independent of where the mouse is placed at the start.

## Phase 8: the left encoder (v65–v68)

**v64's log was strange:** `encoder check L -110 R 123`, then 33–78 mm sideways corrections after every
*left* turn.

**The cause:** the left encoder counted the same way whichever way the wheel turned. In a left spin the left
wheel runs backwards, so it was counted as a fake forward move of about 40 mm. Golden v53 had stopped working
for the same reason.

- **v65** checks both counts against the gyro at every step and reverses a count that went the wrong way.
  The emulator with this exact fault: v64 crashed 5 of 5 runs, v65 0 of 10.
- **v66** learns, per spin direction, where to line up so the spin *ends* centred.
- **v67** slows everything down: explore 75 mm/s.
- **v68** fixed a self-inflicted bug. The repair's "which wheel was wrong before" tie-break, together with gyro
  lag, flipped the *healthy* left encoder 177 times in one run. **v68 completed 3 of 6 maze runs,
  including Hard and Very Hard.**

## Phase 9: lane keeping (v69–v70)

The still side readings showed the mouse drifting 11–12 mm per cell while the gyro said it pointed the other
way. The gyro heading was several degrees off, and a 4° steering limit couldn't win.

- **v69:** at every stop the walls set the sideways position and correct the heading.
- **v70** learned an odometry scale from front-wall fixes. It was **wrong**: the stop bias wasn't a scale
  error, and the learner made the mouse overshoot. Removed in v71.

## Phase 10: final (v71–v71j)

**v71:**
- one spin trim per direction (a shared trim swung between −19 and +10);
- an explorer that skips cells no room can contain (21–27% faster, same simulator score);
- doubtful walls re-checked when no room is found;
- repeating line-ups;
- a guard against runaway spins.

The autograder replica scored **59.16 / 60**.

Then the hardware fought back:

| Build | Problem found | Fix |
|---|---|---|
| v71b–c | the spin trim ran away from −6 to −35 | trim capped, position frozen in spins |
| v71d | `spincheck`: left encoder backwards −185 / −120 / −56 at PWM 40 / 60 / 80 (right −439 / −700 / −873) | left travel from the right wheel + gyro while reversing. **2 of 4 demo runs passed** |
| v71e | far-wall readings moved the estimate 52 mm and the line-up drove 41 mm into a corner | far fixes capped at 30 mm, line-ups without a near wall at 15 mm |
| v71f | after re-tying the wiring, the left encoder was erratic in *both* directions | the left encoder is never used |
| v71g–h | `sidecal`: the side-sensor offset moved | offset re-measured, spin trim +8 |
| v71i | the gyro zero wasn't re-measured at stops (the computed left count is never "still") | stillness from the raw readings |
| v71j | side readings at one cell added to 121–145 mm instead of ~104, and a "fix" from them crashed both demo runs | readings that don't form a corridor are ignored; bump recovery |

**v71j** was the demo build.

---

### What I'd keep from all this

1. **Log everything, standing still.** Nearly every fix above came from one log line.
2. **Write the diagnostic before the fix.** `spincheck` found the encoder fault in one minute, after a day
   of guessing.
3. **The emulator helps only for faults you inject into it.** It found the self-inflicted bugs only once the
   real faults (encoder, gyro drift) were modelled.
4. **Every correction needs a sanity gate.** A clever fix that trusts a bad reading is worse than no fix.
