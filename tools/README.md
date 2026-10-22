# Tools

## `build_deploy.sh`

Compiles `src/main.py` to `build/s4.mpy` with `mpy-cross` and copies it, plus the launcher, to the mouse.
It uses the mounted `UCT_MMOUSE` USB drive if it's there, and `mpremote` otherwise.

```bash
pip install mpy-cross mpremote
./tools/build_deploy.sh
```

## `board/`: scripts that run on the mouse

Open one in Thonny and press Run, or run `mpremote run tools/board/<script>.py`. The battery must be on.

| Script | What it measures |
|---|---|
| `sidecal.py` | Side-sensor offset: measures, spins 180° (each sensor sees the other wall), measures again |
| `spincheck.py` | Floor spins against the gyro, and each wheel in the air at 3 speeds. Shows encoder faults and the spin trim needed |
| `diag.py` | Encoders (motor-driven), ToF at a known spot (3 placements), gyro drift over 20 s, turn displacement |
| `motion_diag.py` | Straight-line and turn behaviour with a log of everything |
| `hw_cal.py` | Gyro gain from a hand-turned 360° |
| `hw_check.py` | Motor and encoder polarity |
| `tof_check.py` | Prints all five ToF slots: which ones respond |
| `centre_test.py` | Side readings at measured sideways offsets |
| `front_center.py` | Front reading with the axle on a cell centre (`HW_FRONT_CENTER`) |

## `sim/`: PC-side testing

These need the course framework (physics simulator, autograder, mock `uct_mouse`). See
[../docs/testing.md](../docs/testing.md).

| File | What it does |
|---|---|
| `harness.py` | Runs a `main.py` in the simulator on chosen seeds and scores it with the official test suite |
| `board_emu.py` | Makes a copy of `main.py` that behaves like the real board, with optional encoder faults and gyro drift |
| `flight_report.py` | Readable report of a flight-recorder dump: path, wheel speeds, ToF, stalls |
| `footprint.py` | Checks a simulated run against the real chassis footprint (the sim uses a 70 mm circle) |
| `motion_stats.py` | Turn and straight-line statistics from harness logs |
| `cfg_*.json` | Simulator configs: board-like chassis, true 8×8 (no fixed maze) |
