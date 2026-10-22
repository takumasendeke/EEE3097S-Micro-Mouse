# Side-sensor self-calibration (~1 minute). BATTERY ON.
# Put the mouse in the START cell (walls left and right), facing the open
# corridor, roughly centred (by eye is fine). Open this file in Thonny and
# press Run, then press SW1 and take your hands away.
# It measures both side sensors, spins 180 deg (each sensor now sees the other
# wall), measures again and spins back. The difference between the two sensors
# comes out exactly, whatever the placement. Send Claude everything it prints.
import uct_mouse
import gc

uct_mouse.init()
gc.collect()
import s4
s4.select_profile()
uct_mouse.set_polarity(s4.MOT_POL_L, s4.MOT_POL_R)
try:
    uct_mouse.set_encoder_polarity(1, 1)
except Exception:
    pass
uct_mouse.set_motors(0, 0)


def med(v):
    if len(v) < 5:
        return -1
    v = sorted(v)
    return v[len(v) // 2]


def look(m, tag):
    l = []
    f = []
    r = []
    n = 0
    while n < 150:
        m.step(0.0, 0.0)
        t = uct_mouse.get_tof()
        if t[0] < 100:
            l.append(t[0])
        if t[2] < 8190:
            f.append(t[2])
        if t[4] < 100:
            r.append(t[4])
        n += 1
    a = (med(l), med(f), med(r))
    print(tag, "L", a[0], "F", a[1], "R", a[2], "  valid", len(l), len(f), len(r))
    return a


print("SIDECAL - press SW1, hands off")
try:
    uct_mouse.display_text(1, "press SW1")
except Exception:
    pass
while uct_mouse.get_button() == 0:
    uct_mouse.delay_ms(20)
while uct_mouse.get_button() != 0:
    uct_mouse.delay_ms(20)
uct_mouse.delay_ms(1000)
m = s4.Mouse()
m.calibrate()
try:
    a = look(m, "facing out  ")
    m.turn_by(180.0)
    m.hold(30)
    b = look(m, "turned 180  ")
    m.turn_by(-180.0)
    m.hold(30)
    c = look(m, "back again  ")
    if min(a[0], a[2], b[0], b[2]) > 0:
        off = 0.5 * ((a[0] - b[2]) + (b[0] - a[2]))
        print("")
        print("left sensor reads", int(off), "mm more than the right one at the same distance")
        print("sum L+R before / after the spin:", a[0] + a[2], b[0] + b[2], "(should match within ~6)")
        print(">>> HW_SIDE_OFFSET =", int(round(off)), "   (was -4; earlier measured -3)")
    else:
        print("not enough side readings - put it between two walls and run again")
except Exception as e:
    uct_mouse.set_motors(0, 0)
    print("stopped:", e)
uct_mouse.set_motors(0, 0)
