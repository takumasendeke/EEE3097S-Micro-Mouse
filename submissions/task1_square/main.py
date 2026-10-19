# =========================================================================
# EEE3097S Micromouse - Milestone 1: 1 m x 1 m square, anticlockwise
#
# Author : Takudzwa Masendeke
# Date   : 5 September 2026
#
# AI assistance: developed with help from Gemini Pro 3.1 and Claude Opus 5.
# Every HW_ constant below was measured on this particular mouse.
#
# Four 1 m legs and four 90 degree turns. Distance closes on the encoders,
# heading on the gyro. One file serves both the co-simulator and the board -
# on_simulator() picks the profile at start-up - so the FNV-1a code hash
# stays valid against the physical run log.
#
# Only the HW_ constants affect the mouse; the simulator uses the plain
# runtime defaults further down. Each knob carries a one-line note.
# =========================================================================
import uct_mouse

SIDE_LENGTH_M   = 1.00
NUM_SIDES       = 4
GYRO_TARGET_DEG = 90.0

HW_MOTOR_POL_L    = -1
HW_MOTOR_POL_R    = -1
HW_ENC_POL_L      = -1
HW_ENC_POL_R      = 1
HW_LEFT_DEADBAND  = 61.0        # ROLLING threshold from pwm_sweep.py
HW_RIGHT_DEADBAND = 62.0        # ditto. NOT the breakaway value.
HW_CAL_SAMPLES    = 120         # startup gyro calibration samples
HW_TICKS_PER_M    = 5709        # DISTANCE. = ticks / metres measured.
HW_SIDE_MS        = 4300        # 4300 ms was 1.00 m at HW_FWD_SPEED 46
HW_ZUPT_N         = 0           # gyro re-zero. 0 = off (gates never passed).
HW_VOLT_COMP      = 1
HW_TURN_ONE_WAY   = 1
HW_TURN_STALL     = 1
HW_NUDGE_EFFORT   = 10.0
HW_SETTLE_MS      = 60          # pause between phases
HW_FINAL_HOLD     = 600         # dead time at the end of the demo
HW_LAUNCH_SYNC    = 1           # encoder-gated simultaneous break-away
HW_LAUNCH_TICKS   = 40          # encoder counts that mean "this wheel moved"
HW_REAL_TIME      = 1           # time every step from the hardware clock
HW_DECEL_FROM     = 0.78        # ease off over the last 18% of the leg
HW_DECEL_TO       = 0.55

HW_GYRO_TRIM      = 0.0         # + steers LEFT. 0.6 x cm of drift per 1 m.
HW_GAIN_L         = 1.0         # wheel speed ratio, from wheel_cal.py
HW_GAIN_R         = 0.987       # ditto
HW_KH_HEADING     = 1.1         # steer effort per degree of error
HW_STEER_LIM      = 22.0        # cap on the differential
HW_DIST_TRIM      = 1.05        # only used when HW_TICKS_PER_M = 0
HW_KP_BALANCE     = 0.15        # rescale if HW_TICKS_PER_M changes
HW_KC_CROSS       = 0.4         # returns to the line. Up if offset.
HW_LAUNCH_START   = 60.0        # effort a leg starts at
HW_HEADING_RAMP_MS = 120.0      # fade-in for the heading term
HW_TURN_EFFORT    = 55.0        # TURN SPEED. Down if the corner is messy.
HW_TURN_KP        = 0.55        # effort per degree remaining
HW_TURN_KD        = 0.60        # how hard the rate profile is enforced
HW_TURN_RATE_K    = 5.0         # lower = longer slow crawl at the end
HW_TURN_RATE_MAX  = 330.0       # spin rate ceiling (clip is 500)
HW_TURN_FLOOR     = 5.0         # lowest effort it will command
HW_TURN_TOL       = 2.5         # deg; how close is done
HW_TURN_RATE_TOL  = 25.0        # deg/s the body may still be turning
HW_TURN_SETTLE    = 4           # steps inside the window before it stops
HW_TURN_RETRY     = 0           # 0 = no second nudge after a turn
# TURN ANGLE: HW_GYRO_GAIN = 90 / angle you get (scale). The kernel
# already multiplies the gyro by 2.0, which is why this is near 1.8.
# HW_TURN_BIAS_DEG is subtracted from the target: under-turn -> NEGATIVE.
HW_GYRO_GAIN      = 1.94        # TURN ANGLE. = 90 / angle you get.
HW_TURN_BIAS_DEG  = 0.0         # deg off every turn. Under-turn -> NEGATIVE.
HW_HEADROOM       = 1           # keep the differential, drop common speed
HW_FWD_SPEED      = 92.0        # SPEED. Down if it wanders.

KP_BALANCE      = 0.004
KI_HEADING      = 0.15

CROSS_NORM      = 100000.0
CROSS_LIM       = 25.0
MAX_SLEW_PER_STEP = 2.0

ZUPT_SKIP       = 8
ZUPT_BLEND      = 0.35
ZUPT_MAX_JUMP   = 6.0
ZUPT_SPREAD     = 1.0
ZUPT_QUIET_DPS  = 2.0
ZUPT_QUIET_N    = 6
ZUPT_WAIT_MS    = 400
ZUPT_CLAMP      = 2.0

VOLT_COMP_MIN   = 0.95
VOLT_COMP_MAX   = 1.14
VOLT_MIN_SANE   = 2.5
DEADBAND_CEIL   = 94.0

LAUNCH_BOOST      = 100.0
LAUNCH_SYNC_MS    = 400
LAUNCH_DECAY      = 3.0

TURN_OVER_STEPS = 30
TURN_STALL_DPS  = 4.0           # below this counts as "not moving"
TURN_STALL_STEPS = 12
TURN_BOOST_STEP = 3.0
TURN_BOOST_MAX  = 22.0
TURN_TIMEOUT_MS = 6000
TURN_RETRY_TOL  = 1.8
NUDGE_ON        = 2
NUDGE_OFF       = 7
NUDGE_MAX       = 6

DT_MS           = 10
DT_S            = 0.010
STEP_MAX_MS     = 250
SETTLE_MS       = 150
SIDE_TIMEOUT_MS = 15000
FINAL_HOLD_MS   = 3200
PARK_M          = 0.035
PARK_COMP       = 1
PC_TICKS_MIN    = 100000000

GYRO_BIAS       = 0.0
GYRO_BIAS_RAW   = 0.0
GYRO_BIAS0      = 0.0
IS_SIM          = 1
TICKS_PER_M     = 5730
SIDE_MS         = 0
LEFT_DEADBAND   = 58.0
RIGHT_DEADBAND  = 62.0
DB_L_REF        = 58.0
DB_R_REF        = 62.0
FWD_SPEED       = 95.0
TURN_EFFORT     = 55.0
ENC_POL_L       = 1
ENC_POL_R       = 1
CAL_SAMPLES     = 50
ZUPT_N          = 0
VOLT_COMP       = 0
TURN_ONE_WAY    = 0
TURN_STALL      = 0
GYRO_TRIM       = 0.0
TURN_KP         = 0.9
TURN_KD         = 0.30
TURN_MIN_EFFORT = 8.0
NUDGE_EFFORT    = 8.0
TURN_RATE_K     = 4.0
TURN_RATE_MAX   = 200.0
DECEL_FROM      = 1.0
DECEL_TO        = 1.0
GYRO_GAIN       = 1.0
TURN_BIAS_DEG   = 0.0
TURN_TOL_DEG    = 1.5
TURN_RATE_TOL   = 12.0
TURN_SETTLE     = 8
TURN_MAX_RETRY  = 2
KH_HEADING      = 0.8
STEER_LIM       = 12.0
VOLT_REF        = 0.0
STEP_MS         = 10.0
last_ms         = 0
USE_REAL_TIME   = 0
DT_REAL_S       = 0.010
LAUNCH_SYNC     = 0
LAUNCH_TICKS    = 3
LAUNCH_START    = 100.0
HEADING_RAMP_MS = 0.0
KC_CROSS        = 0.0
HEADROOM        = 0
GAIN_L          = 1.0
GAIN_R          = 1.0

current_l       = 0.0
current_r       = 0.0
peak_rate       = 0.0
heading         = 0.0
target_heading  = 0.0

# Encoders and gyro, polarity-corrected and bias-removed.
def _sensors():
    e = uct_mouse.get_encoders()
    gyro = uct_mouse.get_gyro()
    return e[0] * ENC_POL_L, e[1] * ENC_POL_R, gyro - GYRO_BIAS

# One control step: sample, integrate heading over the REAL elapsed time, delay.
def tick():
    global heading, peak_rate, last_ms, STEP_MS
    lenc, renc, rate = _sensors()
    dt = DT_REAL_S
    if USE_REAL_TIME == 1:
        t = uct_mouse.get_ticks_ms()
        d = t - last_ms
        last_ms = t
        if d > 0:
            if d < STEP_MAX_MS:
                dt = d * 0.001
                STEP_MS = STEP_MS + (d - STEP_MS) * 0.05
    heading = heading + rate * dt
    a = fabs(rate)
    if a > peak_rate:
        peak_rate = a
    uct_mouse.delay_ms(DT_MS)
    return lenc, renc, rate

# Constrain a value to a range.
def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x

# Absolute value (no math module on the board).
def fabs(x):
    if x < 0.0:
        return 0.0 - x
    return x

# 1 in co-simulation, 0 on the board - CPython has os.environ, MicroPython does not.
def on_simulator():
    try:
        import os
        os.environ
        return 1
    except AttributeError:
        return 0
    except Exception:
        pass
    try:
        t = uct_mouse.get_ticks_ms()
    except Exception:
        return 1
    if t > PC_TICKS_MIN:
        return 1
    return 0

# Millisecond clock that returns 0 rather than raising if the firmware lacks it.
def now_ms():
    try:
        return uct_mouse.get_ticks_ms()
    except Exception:
        return 0

# Load the constant set for whichever plant we are actually running on.
def select_profile():
    global IS_SIM, TICKS_PER_M, SIDE_MS, LEFT_DEADBAND, RIGHT_DEADBAND
    global FWD_SPEED, TURN_EFFORT, ENC_POL_L, ENC_POL_R, CAL_SAMPLES
    global DB_L_REF, DB_R_REF, ZUPT_N, VOLT_COMP, TURN_ONE_WAY, TURN_STALL
    global GYRO_TRIM, TURN_KP, TURN_KD, TURN_MIN_EFFORT
    global NUDGE_EFFORT, DECEL_FROM, DECEL_TO, GYRO_GAIN, TURN_TOL_DEG
    global TURN_RATE_TOL, TURN_SETTLE, TURN_MAX_RETRY, KH_HEADING, STEER_LIM
    global TURN_RATE_K, TURN_RATE_MAX, LAUNCH_SYNC, LAUNCH_TICKS
    global USE_REAL_TIME, LAUNCH_START, HEADING_RAMP_MS
    global TURN_BIAS_DEG, GAIN_L, GAIN_R, HEADROOM
    global KC_CROSS, KP_BALANCE, SETTLE_MS, FINAL_HOLD_MS
    IS_SIM = on_simulator()
    if IS_SIM == 1:
        return 1, 1
    TICKS_PER_M    = HW_TICKS_PER_M
    SIDE_MS        = int(HW_SIDE_MS * HW_DIST_TRIM)
    LEFT_DEADBAND  = HW_LEFT_DEADBAND
    RIGHT_DEADBAND = HW_RIGHT_DEADBAND
    DB_L_REF       = HW_LEFT_DEADBAND
    DB_R_REF       = HW_RIGHT_DEADBAND
    FWD_SPEED      = HW_FWD_SPEED
    TURN_EFFORT    = HW_TURN_EFFORT
    ENC_POL_L      = HW_ENC_POL_L
    ENC_POL_R      = HW_ENC_POL_R
    CAL_SAMPLES    = HW_CAL_SAMPLES
    ZUPT_N         = HW_ZUPT_N
    VOLT_COMP      = HW_VOLT_COMP
    TURN_ONE_WAY   = HW_TURN_ONE_WAY
    TURN_STALL     = HW_TURN_STALL
    GYRO_TRIM      = HW_GYRO_TRIM
    TURN_KP        = HW_TURN_KP
    TURN_KD        = HW_TURN_KD
    TURN_MIN_EFFORT = HW_TURN_FLOOR
    NUDGE_EFFORT   = HW_NUDGE_EFFORT
    LAUNCH_SYNC    = HW_LAUNCH_SYNC
    LAUNCH_TICKS   = HW_LAUNCH_TICKS
    USE_REAL_TIME  = HW_REAL_TIME
    LAUNCH_START   = HW_LAUNCH_START
    HEADING_RAMP_MS = HW_HEADING_RAMP_MS
    KC_CROSS       = HW_KC_CROSS
    HEADROOM       = HW_HEADROOM
    KP_BALANCE     = HW_KP_BALANCE
    SETTLE_MS      = HW_SETTLE_MS
    FINAL_HOLD_MS  = HW_FINAL_HOLD
    GAIN_L         = HW_GAIN_L
    GAIN_R         = HW_GAIN_R
    TURN_RATE_K    = HW_TURN_RATE_K
    TURN_RATE_MAX  = HW_TURN_RATE_MAX
    DECEL_FROM     = HW_DECEL_FROM
    DECEL_TO       = HW_DECEL_TO
    GYRO_GAIN      = HW_GYRO_GAIN
    TURN_BIAS_DEG  = HW_TURN_BIAS_DEG
    TURN_TOL_DEG   = HW_TURN_TOL
    TURN_RATE_TOL  = HW_TURN_RATE_TOL
    TURN_SETTLE    = HW_TURN_SETTLE
    TURN_MAX_RETRY = HW_TURN_RETRY
    KH_HEADING     = HW_KH_HEADING
    STEER_LIM      = HW_STEER_LIM
    return HW_MOTOR_POL_L, HW_MOTOR_POL_R

# Measure the gyro zero while the mouse is stationary at power-on.
def calibrate_gyro():
    global GYRO_BIAS, GYRO_BIAS_RAW, GYRO_BIAS0, VOLT_REF
    uct_mouse.set_motors(0, 0)

    warm(20)

    GYRO_BIAS = 0.0
    total = sample_gyro(CAL_SAMPLES)
    GYRO_BIAS_RAW = total / CAL_SAMPLES
    GYRO_BIAS0 = GYRO_BIAS_RAW
    GYRO_BIAS = GYRO_BIAS_RAW + GYRO_TRIM

    VOLT_REF = uct_mouse.get_vbatt()

# Convert a real duration in ms into control steps, using the measured step time.
def steps_for(ms):
    n = int(ms / STEP_MS)
    if n < 1:
        n = 1
    return n

# Break BOTH wheels loose in the same control step so the body does not pivot.
def sync_launch(sgn_l, sgn_r):
    if LAUNCH_SYNC <= 0:
        return
    l0, r0, _ = _sensors()
    n = 0
    limit = steps_for(LAUNCH_SYNC_MS)
    while n < limit:
        lenc, renc, _ = _sensors()
        cl = 0.0
        cr = 0.0
        if (lenc - l0) * sgn_l < LAUNCH_TICKS:
            cl = LAUNCH_BOOST * sgn_l
        if (renc - r0) * sgn_r < LAUNCH_TICKS:
            cr = LAUNCH_BOOST * sgn_r
        if cl == 0.0:
            if cr == 0.0:
                n = limit
        if n < limit:
            pwm_l, pwm_r = apply_deadband(cl, cr)
            uct_mouse.set_motors(pwm_l, pwm_r)
            tick()
            n = n + 1

# Idle for a number of steps without touching the heading estimate.
def warm(steps):
    n = 0
    while n < steps:
        uct_mouse.delay_ms(DT_MS)
        n = n + 1

# Sum raw gyro over a known-stationary window.
def sample_gyro(steps):
    total = 0.0
    n = 0
    while n < steps:
        uct_mouse.delay_ms(DT_MS)
        total = total + uct_mouse.get_gyro()
        n = n + 1
    return total

# Wait until the body has genuinely stopped turning; 1 if it did.
def wait_quiet():
    quiet = 0
    n = 0
    limit = steps_for(ZUPT_WAIT_MS)
    while n < limit:
        _, _, rate = tick()
        if fabs(rate) < ZUPT_QUIET_DPS:
            quiet = quiet + 1
            if quiet >= ZUPT_QUIET_N:
                n = limit
        else:
            quiet = 0
        n = n + 1
    if quiet >= ZUPT_QUIET_N:
        return 1
    return 0

# Re-zero the gyro bias against a stationary window.
def zupt():
    global GYRO_BIAS, GYRO_BIAS_RAW
    if ZUPT_N <= 0:
        return
    if wait_quiet() == 0:
        return
    half = int(ZUPT_N / 2)
    a = sample_gyro(half) / half
    b = sample_gyro(half) / half
    if fabs(a - b) > ZUPT_SPREAD:
        return
    est = (a + b) / 2.0
    d = est - GYRO_BIAS_RAW
    if fabs(d) > ZUPT_MAX_JUMP:
        return
    v = GYRO_BIAS_RAW + d * ZUPT_BLEND
    GYRO_BIAS_RAW = clamp(v, GYRO_BIAS0 - ZUPT_CLAMP, GYRO_BIAS0 + ZUPT_CLAMP)
    GYRO_BIAS = GYRO_BIAS_RAW + GYRO_TRIM

# Raise the dead-bands as the pack sags so a given effort keeps its meaning.
def update_deadbands():
    global LEFT_DEADBAND, RIGHT_DEADBAND
    if VOLT_COMP <= 0:
        return
    if VOLT_REF < VOLT_MIN_SANE:
        return
    v = uct_mouse.get_vbatt()
    if v < VOLT_MIN_SANE:
        return
    k = clamp(VOLT_REF / v, VOLT_COMP_MIN, VOLT_COMP_MAX)
    LEFT_DEADBAND = clamp(DB_L_REF * k, 0.0, DEADBAND_CEIL)
    RIGHT_DEADBAND = clamp(DB_R_REF * k, 0.0, DEADBAND_CEIL)

# Map an effort command (0..100) onto each wheel's own usable PWM range.
def apply_deadband(cmd_l0, cmd_r0):
    cmd_l = cmd_l0 * GAIN_L
    cmd_r = cmd_r0 * GAIN_R
    if cmd_l > 0.0:
        pwm_l = LEFT_DEADBAND + cmd_l * (100.0 - LEFT_DEADBAND) / 100.0
    else:
        if cmd_l < 0.0:
            pwm_l = 0.0 - LEFT_DEADBAND + cmd_l * (100.0 - LEFT_DEADBAND) / 100.0
        else:
            pwm_l = 0.0

    if cmd_r > 0.0:
        pwm_r = RIGHT_DEADBAND + cmd_r * (100.0 - RIGHT_DEADBAND) / 100.0
    else:
        if cmd_r < 0.0:
            pwm_r = 0.0 - RIGHT_DEADBAND + cmd_r * (100.0 - RIGHT_DEADBAND) / 100.0
        else:
            pwm_r = 0.0

    return int(pwm_l), int(pwm_r)

# Slew-rate limit the effort commands to prevent wheel slip.
def set_motors_ramped(target_l, target_r, slew_rate):
    global current_l, current_r
    current_l = current_l + clamp(target_l - current_l, 0.0 - slew_rate, slew_rate)
    current_r = current_r + clamp(target_r - current_r, 0.0 - slew_rate, slew_rate)
    pwm_l, pwm_r = apply_deadband(current_l, current_r)
    uct_mouse.set_motors(int(clamp(pwm_l, -100, 100)), int(clamp(pwm_r, -100, 100)))

# Immediate stop, and reset the ramp state.
def stop_motors():
    global current_l, current_r
    current_l = 0.0
    current_r = 0.0
    uct_mouse.set_motors(0, 0)

# Run the control loop for n steps with whatever is already commanded.
def hold_ticks(steps):
    n = 0
    while n < steps:
        tick()
        n = n + 1

# Motors off: coast, then re-zero the gyro and re-read the pack voltage.
def settle(ms):
    stop_motors()
    hold_ticks(steps_for(ms))
    zupt()
    update_deadbands()

# Drive one side under fused encoder-balance, gyro-heading and cross-track control.
def drive_straight(side_num):
    global current_l, current_r
    update_deadbands()
    sync_launch(1.0, 1.0)
    lenc0, renc0, _ = _sensors()
    integ = 0.0
    cross = 0.0
    last_dist = 0.0
    speed = 0.0
    base = FWD_SPEED
    speed = FWD_SPEED
    if LAUNCH_START > FWD_SPEED:
        speed = LAUNCH_START
        base = LAUNCH_START
    current_l = base
    current_r = base

    use_ticks = 0
    target = 0
    if TICKS_PER_M > 0:
        use_ticks = 1
        side_m = SIDE_LENGTH_M
        if PARK_COMP > 0:
            if PARK_M > 0.0:
                if side_num == 1:
                    side_m = SIDE_LENGTH_M - PARK_M / 2.0
                if side_num == 3:
                    side_m = SIDE_LENGTH_M + PARK_M / 2.0
        target = int(side_m * TICKS_PER_M)

    n = 0
    t_start = now_ms()
    limit = steps_for(SIDE_TIMEOUT_MS)
    if use_ticks == 0:
        limit = steps_for(SIDE_MS)

    while n < limit:
        lenc, renc, gyro_dps = tick()
        dl = lenc - lenc0
        dr = renc - renc0
        elapsed = n * STEP_MS
        if USE_REAL_TIME == 1:
            elapsed = uct_mouse.get_ticks_ms() - t_start

        if use_ticks == 1:
            if (dl + dr) / 2.0 >= target:
                n = limit
        else:
            if USE_REAL_TIME == 1:
                if uct_mouse.get_ticks_ms() - t_start >= SIDE_MS:
                    n = limit

        err = target_heading - heading
        integ = clamp(integ + err * DT_REAL_S, -20.0, 20.0)

        ramp = 1.0
        if HEADING_RAMP_MS > 0.0:
            ramp = elapsed * 1.0 / HEADING_RAMP_MS
            if ramp > 1.0:
                ramp = 1.0
        step_ticks = (dl + dr) / 2.0 - last_dist
        last_dist = (dl + dr) / 2.0
        cross = clamp(cross - err * step_ticks / CROSS_NORM,
                      0.0 - CROSS_LIM, CROSS_LIM)
        correction = clamp(KP_BALANCE * (dl - dr)
                           + ramp * (KH_HEADING * err + KI_HEADING * integ)
                           - KC_CROSS * cross,
                           0.0 - STEER_LIM, STEER_LIM)

        frac = n * 1.0 / limit
        if use_ticks == 1:
            if target > 0:
                frac = (dl + dr) / 2.0 / target
        else:
            if USE_REAL_TIME == 1:
                if SIDE_MS > 0:
                    frac = (uct_mouse.get_ticks_ms() - t_start) * 1.0 / SIDE_MS

        cruise = FWD_SPEED
        if DECEL_FROM < 1.0:
            if frac > DECEL_FROM:
                k = (frac - DECEL_FROM) / (1.0 - DECEL_FROM)
                if k > 1.0:
                    k = 1.0
                cruise = FWD_SPEED * (1.0 - (1.0 - DECEL_TO) * k)

        if speed > cruise:
            speed = speed - LAUNCH_DECAY
            if speed < cruise:
                speed = cruise
        else:
            speed = cruise

        base = base + clamp(speed - base,
                            0.0 - MAX_SLEW_PER_STEP, MAX_SLEW_PER_STEP)
        l_cmd = base - correction
        r_cmd = base + correction
        if HEADROOM == 1:
            over = 0.0
            if r_cmd > 100.0:
                over = r_cmd - 100.0
            if l_cmd - 100.0 > over:
                over = l_cmd - 100.0
            l_cmd = l_cmd - over
            r_cmd = r_cmd - over
        current_l = clamp(l_cmd, 0.0, 100.0)
        current_r = clamp(r_cmd, 0.0, 100.0)
        pwm_l, pwm_r = apply_deadband(current_l, current_r)
        uct_mouse.set_motors(pwm_l, pwm_r)
        n = n + 1

    stop_motors()
    lenc_f, renc_f, _ = _sensors()
    dl_f = lenc_f - lenc0
    dr_f = renc_f - renc0
    mean = (dl_f + dr_f) / 2.0
    settle(SETTLE_MS)
    return mean

# One short drive pulse followed by a coast, used by the post-turn re-trim.
def nudge(effort):
    pwm_l, pwm_r = apply_deadband(0.0 - effort, effort)
    uct_mouse.set_motors(pwm_l, pwm_r)
    hold_ticks(NUDGE_ON)
    uct_mouse.set_motors(0, 0)
    hold_ticks(NUDGE_OFF)

# Bounded open-loop nudge back onto the turn target; stops the moment the error flips sign.
def retrim():
    k = 0
    first = 0
    while k < NUDGE_MAX:
        err = target_heading - heading
        s = 0
        if err > 0.0:
            s = 1
        if err < 0.0:
            s = 0 - 1
        if fabs(err) <= TURN_TOL_DEG * 0.6:
            k = NUDGE_MAX
        else:
            if first == 0:
                first = s
            if s != first:
                k = NUDGE_MAX
            else:
                nudge(NUDGE_EFFORT * s)
                k = k + 1
    stop_motors()

# Rotate 90 degrees anticlockwise on a gyro-closed deceleration profile.
def turn_left_90(corner_num):
    global peak_rate, target_heading
    update_deadbands()
    peak_rate = 0.0
    target_heading = target_heading + GYRO_TARGET_DEG * GYRO_GAIN - TURN_BIAS_DEG
    inside = 0
    over = 0
    stalled = 0
    boost = 0.0
    n = 0
    limit = steps_for(TURN_TIMEOUT_MS)

    sync_launch(0.0 - 1.0, 1.0)
    rate_now = 0.0
    while n < limit:
        remaining = target_heading - heading
        settled = 0
        if fabs(remaining) <= TURN_TOL_DEG:
            if fabs(rate_now) <= TURN_RATE_TOL:
                settled = 1

        coast = 0
        if TURN_ONE_WAY == 1:
            if remaining <= 0.0:
                coast = 1

        if settled == 1:
            inside = inside + 1
            if inside >= TURN_SETTLE:
                n = limit
            else:
                uct_mouse.set_motors(0, 0)
                _, _, rate_now = tick()
                n = n + 1
        else:
            inside = 0
            if coast == 1:
                over = over + 1
                uct_mouse.set_motors(0, 0)
                _, _, rate_now = tick()
                n = n + 1
                if over > TURN_OVER_STEPS:
                    n = limit
            else:
                floor = TURN_MIN_EFFORT + boost
                sgn = 1.0
                if remaining < 0.0:
                    sgn = 0.0 - 1.0
                mag = fabs(remaining)
                e = TURN_KP * mag
                if e > TURN_EFFORT:
                    e = TURN_EFFORT
                want = TURN_RATE_K * mag
                if want > TURN_RATE_MAX:
                    want = TURN_RATE_MAX
                spin = rate_now * sgn
                if spin > want:
                    e = e - TURN_KD * (spin - want)
                if e < floor:
                    e = floor
                effort = e * sgn
                moving = 1
                if fabs(rate_now) < TURN_STALL_DPS:
                    moving = 0
                if TURN_STALL == 0:
                    moving = 1
                if moving == 0:
                    stalled = stalled + 1
                    if stalled >= TURN_STALL_STEPS:
                        boost = boost + TURN_BOOST_STEP
                        if boost > TURN_BOOST_MAX:
                            boost = TURN_BOOST_MAX
                        stalled = 0
                else:
                    stalled = 0
                pwm_l, pwm_r = apply_deadband(0.0 - effort, effort)
                uct_mouse.set_motors(pwm_l, pwm_r)
                _, _, rate_now = tick()
                n = n + 1

    stop_motors()
    settle(SETTLE_MS)

    tries = 0
    while tries < TURN_MAX_RETRY:
        err = target_heading - heading
        if fabs(err) <= TURN_RETRY_TOL:
            tries = TURN_MAX_RETRY
        else:
            retrim()
            settle(SETTLE_MS)
            tries = tries + 1

# Short forward creep so the harness can measure the final corner.
def park_and_finish():
    if TICKS_PER_M <= 0:
        return
    lenc0, renc0, _ = _sensors()
    target = int(PARK_M * TICKS_PER_M)
    n = 0
    limit = steps_for(2000)
    while n < limit:
        lenc, renc, _ = _sensors()
        if (lenc - lenc0 + renc - renc0) / 2.0 >= target:
            n = limit
        else:
            set_motors_ramped(FWD_SPEED * 0.5, FWD_SPEED * 0.5, MAX_SLEW_PER_STEP)
            tick()
            n = n + 1
    stop_motors()

# Run the whole mission: calibrate, then four legs and four corners.
def run_square():
    ok = uct_mouse.init()
    if ok == 0:
        return

    mot_l, mot_r = select_profile()

    uct_mouse.set_polarity(mot_l, mot_r)
    try:
        uct_mouse.set_encoder_polarity(1, 1)
    except Exception:
        pass

    calibrate_gyro()

    i = 0
    while i < NUM_SIDES:
        drive_straight(i + 1)
        turn_left_90(i + 1)
        i = i + 1

    park_and_finish()

    stop_motors()
    n = 0
    steps = steps_for(FINAL_HOLD_MS)
    while n < steps:
        uct_mouse.set_motors(0, 0)
        uct_mouse.delay_ms(DT_MS)
        n = n + 1

try:
    __name__
except NameError:
    __name__ = "__main__"

if __name__ == "__main__":
    run_square()