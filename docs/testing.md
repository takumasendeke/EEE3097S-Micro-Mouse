# Testing

Three levels of testing, from fastest to most realistic.

## 1. Course simulator (`tools/sim/harness.py`)

The harness runs `src/main.py` in the course's lock-step physics simulator and scores each run with the
official test suite. Scores: room 30, pirouette 20, return 20, sprint 20, bonus 10.

```bash
python3 tools/sim/harness.py src/main.py                                   # the 5 graded seeds
python3 tools/sim/harness.py src/main.py --seeds 101,102,103,104,105 -j 5 -v
python3 tools/sim/harness.py src/main.py --rows 8 --cols 8 --cfg tools/sim/cfg_nomaze.json --max-time 240
```

> These tools need the course framework ([UCT-Micromouse](https://github.com/nicollsf/UCT-Micromouse):
> `tools/physics_sim.py`, the autograder test suite and the mock `python/uct_mouse.py`). It isn't part of this
> repository. Run them from a checkout of that framework.

| Build | Graded seeds | 10 extra 4×6 | True 8×8 | Autograder replica |
|---|---|---|---|---|
| v15 | 98.6 | | | |
| v71 | 98.6 | 96.3 (21% faster than v70) | 99.0 (27% faster) | 59.16 / 60 |
| v71j | 98.6 | | | |

## 2. Board emulator (`tools/sim/board_emu.py`)

This makes a copy of `main.py` that runs the **real board profile** (`HW_*` constants, board code paths)
inside the simulator:
- the board's ToF noise and dropouts, motor stiction and compute delay;
- a gyro bias error;
- a mirrored maze.

It can also **inject encoder faults**, modelled on what the real left encoder did:

| `--enc-fault` | Models |
|---|---|
| `lstuck-` / `lstuck+` | counts one way whatever the wheel does (v64) |
| `lrev` | counts reversed |
| `lweakrev` | loses 85% of its backward counts (v71c spincheck) |
| `lrevweak` | reversed, and loses most forward counts (after the wire re-tie) |

```bash
python3 tools/sim/board_emu.py src/main.py /tmp/emu --hw --stiction 44,46 --offset 0 --enc-fault lrevweak --gyro-drift 1.0
python3 tools/sim/harness.py /tmp/emu/main.py --cfg tools/sim/cfg_board.json --max-time 400 --seeds 101,102,103 -j 5
```

Some results it produced:

| Fault | Old build | Fixed build |
|---|---|---|
| left encoder stuck one way | v64: 5 of 5 crashed | v65: 0 of 10 |
| left encoder weak backwards | v71c: 2 of 10 crashed | v71d: 0 of 10 |
| reversed + weak forwards | v71e: 0 of 10 finished | v71f: 16 of 20 finished |
| gyro zero off by 1 °/s | v71h: 0 of 10 finished | v71i: 18 of 20 finished |

**Caveat:** the emulator only finds problems that are modelled. It has no real tyre scrub, and its motors
are symmetric, so it can't judge the real mouse's left/right balance.

## 3. The real mouse

Every run writes `runlog.txt` on the board (copy it from the USB drive), and the firmware keeps a 25 Hz
flight recorder (`dump_logs.py` from the course tools). The logs from every run are in
[`logs/`](../logs). The diagnostic scripts in [`tools/board`](../tools/board) measure the hardware directly:
see [hardware-notes.md](hardware-notes.md).

### Reading a run log

```
S4 BUILD v71j CENTRE 31-Oct                     <- check the build first
gyro zero -0.08 deg/s spread 0.09               <- start calibration (mouse must be still)
encoder check L -95 R 123 ticks counted ...     <- first drive
cell 1 1 d 1 walls L F R 0 1 1 votes ... side 0 61    <- still look on a new cell
stop lat 13 corr 2 hdg fix 0.0                  <- mm off centre (+ = left), heading corrected
align front 44 fix 0 move 6 ...                 <- line-up on the wall ahead before a turn
turn 92 err 0.4 ovs 5.0 ms 1170 tries 2 ... trim 8 5
latfix 25 mm corr 16 ...                        <- where the spin really ended (+ = left)
```
