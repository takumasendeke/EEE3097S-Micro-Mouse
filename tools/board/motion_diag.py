# =========================================================================
# EEE3097S Micromouse - motion diagnostic (run from Thonny on the board)
#
# Phases (mouse on the floor, cable slack, ~1.3 m of straight floor ahead):
#   0  STATIC   2 s still: gyro bias/noise, ToF valid %, mean, spread, battery
#   1  SWEEP    spin on the spot at rising PWM: each wheel's real dead-band
#               and speed-per-PWM (used as feed-forward for phases 2-5)
#   2  STRAIGHT 1000 mm closed loop (wheel speed PI + gyro heading hold)
#   3  TURN     90 deg left on the spot (gyro)
#   4  SHORT    150 mm forward
#   5  REVERSE  150 mm backward
# Every 50 ms it prints a CSV row:  D,phase,t_ms,dist_mm,head_deg,rate_dps,
#   vL,vR (mm/s),pwmL,pwmR,tofL,tofC,tofR
# Each phase ends with a "#" summary line. Copy the WHOLE output.
# Then measure with a tape: how far it really went, how far left/right of
# the start line it ended, and its final angle. Those vs the numbers here
# tell us exactly which part (motors, encoders, gyro, controller) is off.
# =========================================================================
import uct_mouse as m

MOT_POL_L = 1        # same as HW_ values in main.py (v15)
MOT_POL_R = 1
ENC_L = -1
ENC_R = 1
TPMM = 5.709         # encoder ticks per mm (HW_TICKS_PER_M / 1000)
GYRO_GAIN = 1.0
HALF_AXLE = 54.0     # mm (wheel to centre)

V_MAX = 200.0        # mm/s straight speed
ACC = 400.0          # mm/s^2
W_MAX = 180.0        # deg/s turn rate
ALPHA = 600.0        # deg/s^2
KP_V = 0.12          # pwm per mm/s of wheel speed error
KI_V = 0.8
KH = 6.0             # mm/s wheel difference per deg of heading error
KHD = 0.15           # mm/s per deg/s of yaw rate (damping)
DT = 10

DB_L = 61.0          # replaced by the sweep
DB_R = 62.0
KV_L = 6.0           # mm/s per PWM above dead-band, replaced by the sweep
KV_R = 6.0

st = {"el": 0, "er": 0, "h": 0.0, "rate": 0.0, "vl": 0.0, "vr": 0.0,
      "dist": 0.0, "bias": 0.0, "t": 0, "il": 0.0, "ir": 0.0, "pl": 0, "pr": 0,
      "tl": 8190, "tc": 8190, "tr": 8190}


def clamp(x, a, b):
    return a if x < a else (b if x > b else x)


def sense():
    t = m.get_ticks_ms()
    dt = (t - st["t"]) * 0.001
    if dt <= 0 or dt > 0.2:
        dt = DT * 0.001
    st["t"] = t
    e = m.get_encoders()
    l = e[0] * ENC_L
    r = e[1] * ENC_R
    dl = (l - st["el"]) / TPMM
    dr = (r - st["er"]) / TPMM
    st["el"] = l
    st["er"] = r
    st["vl"] += 0.35 * (dl / dt - st["vl"])
    st["vr"] += 0.35 * (dr / dt - st["vr"])
    st["dist"] += 0.5 * (dl + dr)
    g = (m.get_gyro() - st["bias"]) / GYRO_GAIN
    st["rate"] = g
    st["h"] += g * dt
    tof = m.get_tof()
    st["tl"] = tof[0]
    st["tc"] = tof[2]
    st["tr"] = tof[4]
    return dt


def pwm_for(v, db, kv):
    if v > 0.5:
        return db + v / kv
    if v < -0.5:
        return -db + v / kv
    return 0.0


# One control step: wheel speed targets (mm/s) -> PWM via feed-forward + PI.
def wheels(tl, tr, dt):
    if tl == 0.0 and tr == 0.0:
        st["il"] = 0.0
        st["ir"] = 0.0
        st["pl"] = 0
        st["pr"] = 0
        m.set_motors(0, 0)
        return
    el = tl - st["vl"]
    er = tr - st["vr"]
    st["il"] = clamp(st["il"] + KI_V * el * dt, -15.0, 15.0)
    st["ir"] = clamp(st["ir"] + KI_V * er * dt, -15.0, 15.0)
    pl = pwm_for(tl, DB_L, KV_L) + KP_V * el + st["il"]
    pr = pwm_for(tr, DB_R, KV_R) + KP_V * er + st["ir"]
    st["pl"] = int(clamp(pl, -100, 100))
    st["pr"] = int(clamp(pr, -100, 100))
    m.set_motors(st["pl"], st["pr"])


def row(ph, t0):
    print("D,%s,%d,%.1f,%.2f,%.1f,%.0f,%.0f,%d,%d,%d,%d,%d" % (
        ph, m.get_ticks_ms() - t0, st["dist"], st["h"], st["rate"], st["vl"], st["vr"],
        st["pl"], st["pr"], st["tl"], st["tc"], st["tr"]))


def still(ms):
    n = 0
    while n < ms // DT:
        m.set_motors(0, 0)
        m.delay_ms(DT)
        sense()
        n += 1


# ---------------- phase 0: static ----------------
def static():
    m.set_motors(0, 0)
    m.delay_ms(500)
    n = 200
    gs = 0.0
    gq = 0.0
    cnt = [0, 0, 0]
    sm = [0.0, 0.0, 0.0]
    sq = [0.0, 0.0, 0.0]
    i = 0
    while i < n:
        m.delay_ms(DT)
        g = m.get_gyro()
        gs += g
        gq += g * g
        tof = m.get_tof()
        k = 0
        for v in (tof[0], tof[2], tof[4]):
            if 0 < v < 8190:
                cnt[k] += 1
                sm[k] += v
                sq[k] += v * v
            k += 1
        i += 1
    b = gs / n
    st["bias"] = b
    sd = (gq / n - b * b) ** 0.5 if gq / n > b * b else 0.0
    print("# STATIC gyro_bias %.3f dps  gyro_noise_sd %.3f dps  vbatt %.2f V" % (b, sd, m.get_vbatt()))
    names = ("L", "C", "R")
    k = 0
    while k < 3:
        if cnt[k]:
            mu = sm[k] / cnt[k]
            v2 = sq[k] / cnt[k] - mu * mu
            print("# STATIC tof_%s valid %d%%  mean %.0f mm  sd %.1f mm" % (
                names[k], 100 * cnt[k] // n, mu, v2 ** 0.5 if v2 > 0 else 0.0))
        else:
            print("# STATIC tof_%s valid 0%% (no readings)" % names[k])
        k += 1
    e = m.get_encoders()
    st["el"] = e[0] * ENC_L
    st["er"] = e[1] * ENC_R
    st["t"] = m.get_ticks_ms()


# ---------------- phase 1: spin sweep ----------------
def fit(xs, ys):
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    k = sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / sxx
    return k, mx - my / k if k != 0 else None


def sweep():
    global DB_L, DB_R, KV_L, KV_R
    print("# SWEEP pwm, wheel speed L, R (mm/s) - spinning on the spot")
    xs = []
    yl = []
    yr = []
    p = 30
    while p <= 90:
        n = 0
        al = 0.0
        ar = 0.0
        k = 0
        while n < 30:
            m.set_motors(-p, p)
            m.delay_ms(DT)
            sense()
            if n >= 15:
                al += -st["vl"]
                ar += st["vr"]
                k += 1
            n += 1
        al /= k
        ar /= k
        print("# SWEEP %d %.0f %.0f" % (p, al, ar))
        if al > 15.0 and ar > 15.0:
            xs.append(p)
            yl.append(al)
            yr.append(ar)
        p += 6
    m.set_motors(0, 0)
    still(600)
    fl = fit(xs, yl)
    fr = fit(xs, yr)
    if fl and fr and fl[1] is not None and fr[1] is not None and fl[0] > 0 and fr[0] > 0:
        KV_L, DB_L = fl[0], clamp(fl[1], 10.0, 90.0)
        KV_R, DB_R = fr[0], clamp(fr[1], 10.0, 90.0)
    print("# SWEEP deadband L %.1f R %.1f   mm/s per pwm L %.2f R %.2f   (L/R gain ratio %.3f)" % (
        DB_L, DB_R, KV_L, KV_R, KV_L / KV_R))
    still(300)


# ---------------- phase 2/4/5: straight ----------------
def straight(ph, target):
    sg = 1.0 if target > 0 else -1.0
    goal = abs(target)
    st["dist"] = 0.0
    h0 = st["h"]
    v = 0.0
    t0 = m.get_ticks_ms()
    maxlat = 0.0
    lat = 0.0
    n = 0
    while n < 2000:
        dt = sense()
        done = abs(st["dist"])
        rem = goal - done
        if rem <= 1.0:
            break
        vt = min(V_MAX, (2.0 * ACC * rem) ** 0.5)
        v = min(vt, v + ACC * dt)
        if v < 35.0:
            v = 35.0
        herr = h0 - st["h"]
        dv = KH * herr - KHD * st["rate"]
        wheels(sg * v - dv, sg * v + dv, dt)
        lat += 0.5 * (st["vl"] + st["vr"]) * dt * (st["h"] - h0) * 0.01745
        if abs(lat) > maxlat:
            maxlat = abs(lat)
        if n % 5 == 0:
            row(ph, t0)
        m.delay_ms(DT)
        n += 1
    stop_t = m.get_ticks_ms() - t0
    d_stop = abs(st["dist"])
    wheels(0.0, 0.0, 0.01)
    k = 0
    while k < 60:
        m.delay_ms(DT)
        sense()
        if k % 5 == 0:
            row(ph + "_stop", t0)
        k += 1
    print("# %s target %d mm  at_stop %.1f  final %.1f mm (coast %.1f)  heading err %.2f deg  "
          "est lateral %.1f mm (max %.1f)  time %d ms" % (
              ph, target, d_stop, abs(st["dist"]), abs(st["dist"]) - d_stop,
              st["h"] - h0, lat, maxlat, stop_t))


# ---------------- phase 3: turn ----------------
def turn(ph, angle):
    h0 = st["h"]
    tgt = h0 + angle
    sg = 1.0 if angle > 0 else -1.0
    t0 = m.get_ticks_ms()
    peak = 0.0
    settled = -1
    good = 0
    n = 0
    while n < 400:
        dt = sense()
        rem = tgt - st["h"]
        over = -rem * sg
        if over > peak:
            peak = over
        if abs(rem) < 1.5 and abs(st["rate"]) < 15.0:
            good += 1
            if good >= 5:
                settled = m.get_ticks_ms() - t0
                break
        else:
            good = 0
        w = min(W_MAX, (2.0 * ALPHA * abs(rem)) ** 0.5 + 2.0 * abs(rem))
        if abs(rem) < 1.0:
            w = 0.0
        w = w if rem > 0 else -w
        vw = w * 0.017453 * HALF_AXLE
        wheels(-vw, vw, dt)
        if n % 5 == 0:
            row(ph, t0)
        m.delay_ms(DT)
        n += 1
    wheels(0.0, 0.0, 0.01)
    still(400)
    print("# %s target %.0f deg  final %.2f deg (err %.2f)  overshoot %.2f deg  settle %d ms" % (
        ph, angle, st["h"] - h0, st["h"] - tgt, peak, settled))


def main():
    m.init()
    m.set_polarity(MOT_POL_L, MOT_POL_R)
    try:
        m.set_encoder_polarity(1, 1)
    except Exception:
        pass
    print("# DIAG start - press SW1, then take your hand away")
    while m.get_button() == 0:
        m.delay_ms(50)
    m.delay_ms(1000)
    static()
    sweep()
    print("# straightening up after sweep")
    a = st["h"] % 360.0
    if a > 180.0:
        a -= 360.0
    turn("REALIGN", -a)
    st["h"] = 0.0
    straight("STRAIGHT", 1000)
    turn("TURN", 90.0)
    straight("SHORT", 150)
    straight("REVERSE", -150)
    m.set_motors(0, 0)
    print("# DIAG done")


try:
    main()
except Exception as e:
    m.set_motors(0, 0)
    raise e
