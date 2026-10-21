# v66 diagnostics: encoders, side/front ToF offsets, gyro drift, turn displacement.
# ~4 minutes. BATTERY SWITCH ON (the encoders read 0 without it).
# Run it in Thonny (open this file, press Run) or in the REPL:  import diag66
# Follow the prompts; each step starts when you press SW1 and take your hands away.
# At the end, copy EVERYTHING it printed (from "DIAG v66") and send it to Claude.
import uct_mouse

uct_mouse.init()
try:
    uct_mouse.set_polarity(1, 1)            # as s4 (HW_MOTOR_POL_L/R = 1)
except Exception:
    pass
try:
    uct_mouse.set_encoder_polarity(1, 1)    # raw counts, as s4 reads them
except Exception:
    pass
uct_mouse.set_motors(0, 0)


def show(t):
    try:
        uct_mouse.display_text(1, t)
    except Exception:
        pass


def sw1(msg):
    print("")
    print(">>> " + msg)
    print("    then press SW1 and take your hands away")
    show("press SW1")
    while uct_mouse.get_button() == 0:
        uct_mouse.delay_ms(20)
    while uct_mouse.get_button() != 0:
        uct_mouse.delay_ms(20)
    show("measuring")
    uct_mouse.delay_ms(800)


def med(v):
    if len(v) < 3:
        return -1
    v = sorted(v)
    return v[len(v) // 2]


def tof(ms):
    # medians of the live slots (left 0, front 2, right 4) over ms; 8190 = no reading
    l = []
    f = []
    r = []
    n = 0
    while n < ms // 20:
        t = uct_mouse.get_tof()
        if t[0] < 8190:
            l.append(t[0])
        if t[2] < 8190:
            f.append(t[2])
        if t[4] < 8190:
            r.append(t[4])
        uct_mouse.delay_ms(20)
        n += 1
    return med(l), med(f), med(r), len(l), len(f), len(r)


def tofline(tag, ms=1500):
    a = tof(ms)
    print(tag, "L", a[0], "F", a[1], "R", a[2], "  valid", a[3], a[4], a[5])
    return a


print("DIAG v66")
try:
    print("battery", uct_mouse.get_vbatt(), "V")
except Exception:
    pass

# ---- 1. encoders, driven by the motors, wheels in the air ----------------
sw1("1/4 ENCODERS: hold the mouse up in the air so BOTH wheels spin freely")
k = 0
while k < 3:
    for name, pl, pr in (("LEFT fwd ", 50, 0), ("LEFT back", -50, 0), ("RIGHT fwd ", 0, 50), ("RIGHT back", 0, -50)):
        e0 = uct_mouse.get_encoders()
        uct_mouse.set_motors(pl, pr)
        uct_mouse.delay_ms(400)
        uct_mouse.set_motors(0, 0)
        uct_mouse.delay_ms(400)
        e1 = uct_mouse.get_encoders()
        print("enc", name, "L", e1[0] - e0[0], "R", e1[1] - e0[1])
    k += 1
print("(expected: each wheel counts one way forwards and the OTHER way backwards, similar size)")

# ---- 2. ToF at a known spot ------------------------------------------------
sw1("2/4 TOF: put it in the START cell FACING THE CLOSED END (walls left, right, ahead),\n"
    "    axle on the cell centre, equal gaps both sides (use a ruler if you can)")
tofline("tof place 1")
sw1("2/4 TOF: pick it up and put it back the same way (centred, facing the closed end)")
tofline("tof place 2")
sw1("2/4 TOF: once more - pick it up, put it back centred")
a = tofline("tof place 3")
print("(earlier measurement with the axle exactly centred: L 50  R 53  F 37)")

# ---- 3. gyro drift ---------------------------------------------------------
sw1("3/4 GYRO: leave it there, untouched, for 20 s")
w = 0
while w < 4:
    s = 0.0
    n = 0
    while n < 250:
        s += uct_mouse.get_gyro()
        uct_mouse.delay_ms(20)
        n += 1
    print("gyro bias, 5 s window", w + 1, ":", int(1000 * s / n) / 1000.0, "deg/s")
    w += 1

# ---- 4. turns with the real s4 turn code (optional) ----------------------
sw1("4/4 TURNS: still in the start cell facing the closed end, centred.\n"
    "    It will turn left 90 and back 3 times, then right 90 and back 3 times.\n"
    "    (Ctrl+C now to skip this part)")
try:
    import gc
    gc.collect()
    import s4
    s4.select_profile()
    uct_mouse.set_polarity(s4.MOT_POL_L, s4.MOT_POL_R)
    m = s4.Mouse()
    m.calibrate()
    tofline("turn start      ")
    for sg, name in ((1.0, "LEFT "), (-1.0, "RIGHT")):
        k = 0
        while k < 3:
            m.turn_by(90.0 * sg)
            m.hold(30)
            tofline("after " + name + " 90  ")
            m.turn_by(-90.0 * sg)
            m.hold(30)
            tofline("back from " + name + "   ")
            k += 1
except Exception as e:
    uct_mouse.set_motors(0, 0)
    print("turn test stopped:", e)
uct_mouse.set_motors(0, 0)
show("diag done")
print("")
print("DIAG DONE - copy everything from 'DIAG v66' and send it to Claude")
