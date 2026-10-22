# Architecture

`src/main.py` is a single file (~3,400 lines) that runs unchanged in the course simulator and on the
STM32L476 MicroPython board.

## One file, two profiles

`on_simulator()` decides at start-up which profile is active:

| | Simulator profile | Board profile (`HW_*` constants) |
|---|---|---|
| Exploration | rolling (classifies walls on the move) | stops on every new cell and senses standing still |
| Speeds | explore 260, sprint 360 mm/s | explore 75, return 90, sprint 130 mm/s |
| Chassis model | identified at start (`sysid` spin) | measured constants (dead-band, VMAX, ticks/m) |
| Position | Kalman corridor filter | plus the uncertainty model, edge fixes, stop fixes |

`select_profile()` copies every `HW_*` constant into the runtime globals on the board only. That's how the
simulator score stays at 98.6 while the board path changes freely.

On the board, the source is compiled with `mpy-cross` to `s4.mpy`. Compiling the source on the board itself
runs out of RAM. A two-line launcher (`src/board/main.py`) imports it.

## The map: `MazeSolver`

- **Wall map:** a bitmask per cell (`wall`) plus a "seen" mask (`known`). Outer walls are known from the
  start, and a seen wall is never erased.
- **Room test (`check_room`):** a 2×2 block whose 4 interior walls are seen open and whose 8 rim walls are
  all seen, with at most `ROOM_MAX_DOORS` openings.
- **Exploration (`next_dir`):** a multi-source floodfill (Dial's algorithm on unit edges). The seeds are
  unvisited cells:
  - cost 0 if they're in a block with 3 or more interior walls seen open;
  - cost 1 with 2 open;
  - cost 2 for any other block that could still be the room;
  - **not seeded at all** if no possible room can contain them (every 2×2 block around them has an interior
    wall seen closed, or too many openings).

  The mouse then takes the neighbour with the lowest flood value, preferring straight on, then left/right,
  then back.
- **Doubtful walls:** side walls classified by a weak vote (under 85% of the valid readings) are remembered.
  If exploration ends without a room, they become unknown again. The explorer goes back, faces each one and
  checks it with the front sensor before driving through.
- **Planning (`plan`):** Dijkstra over states (cell, heading), with costs 4 per cell, +2 per 90° turn and
  +7 for a 180°. It uses known-open walls only. The return and the sprint both use it, and the sprint
  merges straights into one accelerate/cruise/decelerate profile.

## Sensing: `Mouse.sense()`

Every 10–20 ms control step:

1. **Encoders → wheel travel.**
   - Each step is checked against the gyro: right − left = yaw × track.
   - A count that went the wrong way is reversed. The gyro decides when that's clear; for a straight, it's
     the wheel counting against its own motor command.
   - With `HW_LEFT_ENC_OFF` (final build) the left encoder isn't used at all: its travel is the right wheel's
     minus the gyro yaw, and exactly 0 when the mouse stands still.
2. **Gyro → heading** (CCW positive, map frame).
3. **ToF:** five slots, three live (left, front, right).
   - About 50% of samples drop out (`8190` = no reading, not "open").
   - The last valid readings are kept with their age.
   - Side readings near a cell boundary (posts, wall ends) are blanked.

## Position estimation

- **Odometry:** X, Y from mid-step heading integration.
- **Corridor Kalman filter:** two states, sideways offset and heading. The measurement is the side ToF
  `z = y + SIDE_X·ψ`, because the side sensors sit 50 mm ahead of the axle.
- **Along-track uncertainty (`sa`):** every absolute fix is weighted by it:
  - the front wall (near readings while approaching, the median when stopped);
  - side-wall edges (a wall starting or ending marks a cell boundary; the offset is learned online);
  - far walls when lost (capped at 30 mm).
- **Still fixes (`stop_lat`):** at every new cell, 80 standing-still samples per side set the sideways
  position. If the walls had to move the estimate by X mm over the D mm since the last still fix, the gyro
  heading was off by X/D. It's corrected, at most 2.5° per stop.
- **Plausibility:** if the two side readings don't add up to a corridor (more than 12 mm off), nothing is
  corrected at that stop. A single outlier pair once "fixed" the heading 4° the wrong way.
- **After every turn (`lat_fix`):** the still side readings measure where the spin really ended, corrected
  for heading (sensor ahead of the axle).

## Motion

- **Per-wheel speed PI:** each wheel follows its own speed target. Feed-forward comes from the measured
  dead-band and gain; acceleration feed-forward covers the motor lag.
- **Heading loop:** steers to `ψ_ref = −K·lateral` (max 6°), with a stronger limit for the first 150 mm
  after a turn. A side reflex turns away when a wall is closer than 40 mm.
- **Spins (`turn_bb`):**
  - fixed PWM, with the right wheel matched to the left by feed-forward plus a per-direction trim;
  - cut early using a learned coast time, then a short brake pulse, at most 3 tries;
  - the position is frozen during the spin: the axle doesn't move, and odometry creep would be wrong.
- **Line-up before turns:**
  - one median front reading fixes the along position, then one encoder move to the line-up point;
  - the line-up point is learned per spin direction, so the axle *ends* on the cell centre;
  - moves without a near wall are capped at 15 mm.

## Safety

| Detector | Trigger |
|---|---|
| TILT | accelerometer: a wheel riding up a wall |
| STALL | told to drive, but the wheels hardly move for 0.4 s |
| PINNED | heading held more than 12° off for 0.3 s |
| SLIP | wheels and gyro disagree on the turn rate |
| TURN FAILED | a spin ended more than 45° off |
| FRONT GUARD | a wall where the map says open |

On contact, the final build backs off, marks the wall, replans and drives on, up to 6 times per run.

## Frames

The map frame has x along the start heading and y towards the maze side. `SIDE = +1` when the maze extends
to the left of the start heading. It's detected from the first side opening, and `mirror()` flips the frame
if needed. L/R sensor readings and rate commands are swapped when `SIDE = −1`.

## Logging

`log()` queues lines and writes them only while the mouse stands still. Flash writes during motion upset
the control loop. The first line is always `S4 BUILD <version>`. See [tuning.md](tuning.md) for what each
log line means.
