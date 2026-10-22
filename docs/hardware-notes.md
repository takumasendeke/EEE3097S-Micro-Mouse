# Hardware notes

Facts about this particular mouse (STM32L476, MicroPython firmware, LSM6DS3 IMU, VL53L0X ToF sensors),
learned the hard way.

## Firmware and memory

- The 38 KB source raises `MemoryError` if the board compiles it, so always deploy the `mpy-cross` build
  (`s4.mpy`) plus the 2-line launcher.
- Keep `s4.mpy` under about 38 KB. A 40.3 KB build (v56) froze after SW1.
- `mpremote run/exec` fails ("could not enter raw repl"). Run tethered with `mpremote repl`, then
  `import s4; s4.mission()`.
- macOS can silently save a re-download as `s4 (1).mpy`, and the board then keeps running the old file.
  **Always check the first log line, `S4 BUILD vNN`.**

## Motors

| | Left | Right |
|---|---|---|
| Rolling dead-band (PWM) | 20 | 15 |
| Speed per PWM above dead-band | 6.23 mm/s | 5.00 mm/s |
| Start from rest | ~42–46 PWM | ~42–46 PWM |

- Motor polarity +1 / +1 on the current firmware.
- The right motor is weaker going forwards. The right wheel is about 14% faster backwards than forwards at
  the same PWM, so the two spin directions need different trims (`HW_SPIN_TRIM0`, `HW_SPIN_TRIM0_CW`).
- In spins the tyres scrub: on the floor the wheels turn about 2× what the measured rotation implies.

## Encoders

- About 5,709 ticks/m.
- The encoders take their power from the motor driver, which is only enabled while a motor command is
  non-zero. **Hand-pushing the mouse reads 0.**
- The **left encoder's direction decoding is faulty**, and the fault changed from day to day:

| When | Left, forwards | Left, backwards | Right |
|---|---|---|---|
| v64 run | counted backwards | counted forwards | fine |
| v71c spincheck (air, PWM 40/60/80) | 584 / 826 / 1010 | −185 / −120 / −56 | ±440 / ±700 / ±860 |
| after re-tying the wires (v71f) | −2 / 63 / 67 or −280, erratic | +158 / +410 / +610 (sign reversed) | fine |

  A counter that loses *more* counts at *higher* speed points to a direction-channel / decode fault, not the
  motor. The final build (`HW_LEFT_ENC_OFF = 1`) doesn't use the left encoder. Its travel is computed from
  the right wheel and the gyro (`right − left = yaw × track`).

## Gyro

- True deg/s, CCW positive. A hand-turned 360° reads 360–370°, so the gain is 1.0.
- The zero varies between power-ups, from −0.08 to −1.7 deg/s. It's calibrated at start (the mouse must be
  still: up to 4 tries) and re-measured at every stop while the wheels are dead still. In v71f–h a bug
  stopped the re-measuring, and a wrong start zero then made the mouse drift about 1°/s.

## ToF sensors

- Slots (L, FL, C, FR, R). Only L, C and R are live; FL/FR always read 8190.
- About 50% of samples drop out: `8190` means no reading, not "open". Every wall decision is a majority vote
  of the valid readings.
- Walls read 20–70 mm when centred; open sides 140–260 mm. Occasional stray short echoes, and the front
  sometimes reads 30–60 in an open corridor.
- With the axle on a cell centre: front **37 mm**; sides L 50 / R 53 (before the wiring change).
- The **side offset** (how much more the left reads than the right) moved from −4 to about +2 when the wiring
  was re-tied. Measure it with `tools/board/sidecal.py`.
- Readings near cell boundaries see posts and wall ends. A pair that doesn't add up to a corridor
  (L+R ≠ ~104 mm) is ignored.

## Battery

- Under about 3.7 V the motors get weaker, and every speed-based setting shifts. One run reset mid-return on a
  low battery (the log simply ends, with no error line).
- The start of every log shows `battery x.xx V`.

## Calibration scripts (`tools/board`)

| Script | Takes | Gives |
|---|---|---|
| `sidecal.py` | 1 min | side-sensor offset (`HW_SIDE_OFFSET`): measure, spin 180°, measure |
| `spincheck.py` | 1 min | floor spins vs gyro, and each wheel in the air at 3 speeds: finds encoder faults and spin trims |
| `diag.py` (diag66) | 4 min | encoders (motor-driven), ToF offsets at a known spot, gyro drift, turn displacement |
| `hw_cal.py` | 1 min | gyro gain from a hand-turned 360° (encoders read 0 by hand) |
| `centre_test.py` / `front_center.py` | 2 min | side and front readings at measured offsets |
| `motion_diag.py` | 3 min | straight-line and turn behaviour, logged |
