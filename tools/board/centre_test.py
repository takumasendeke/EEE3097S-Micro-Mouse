# Step A centring test: how each side ToF reading changes with the mouse's
# TRUE sideways offset (slope, bias, noise per sensor).
#
# Setup: a straight corridor with walls on BOTH sides (2 cells long is best).
#   Mark the corridor centre line. Put the mouse straight, axle (wheel
#   centres) on a cell centre, exactly centred: equal gap to both walls,
#   measured with a ruler.
# Run from Thonny. At each "#" prompt place the mouse as asked, keep it
#   straight, press SW1 and take your hands away (it waits 0.8 s, then
#   samples ~4.5 s):
#   1. centred   2. 10 mm LEFT   3. 10 mm RIGHT   4. centred again
# Paste everything it prints (from "pos_mm,sensor" to the last "#" line).
import uct_mouse

uct_mouse.init()
POS = (("centred", 0), ("10 mm LEFT of centre", -10),
       ("10 mm RIGHT of centre", 10), ("centred again", 0))
N = 300


def wait_sw1():
    while uct_mouse.get_button() == 0:
        uct_mouse.delay_ms(20)
    while uct_mouse.get_button() != 0:
        uct_mouse.delay_ms(20)
    uct_mouse.delay_ms(800)


def stats(v):
    # (median, p10, p90, count) of the valid readings, or None
    if len(v) < 10:
        return None
    v.sort()
    n = len(v)
    return (v[n // 2], v[n // 10], v[(n * 9) // 10], n)


print("pos_mm,sensor,valid,median,p10,p90")
res = []
for name, off in POS:
    print("# place the mouse", name, "- then press SW1")
    try:
        uct_mouse.display_text(1, "SW1: " + name[:14])
    except Exception:
        pass
    wait_sw1()
    L = []
    C = []
    R = []
    k = 0
    while k < N:
        t = uct_mouse.get_tof()
        if 0 < t[0] < 8190:
            L.append(t[0])
        if 0 < t[2] < 8190:
            C.append(t[2])
        if 0 < t[4] < 8190:
            R.append(t[4])
        uct_mouse.delay_ms(15)
        k += 1
    sl = stats(L)
    sc = stats(C)
    sr = stats(R)
    for nm, s in (("L", sl), ("C", sc), ("R", sr)):
        if s is None:
            print(off, nm, 0, "", "", "", sep=",")
        else:
            print(off, nm, s[3], s[0], s[1], s[2], sep=",")
    res.append((off, sl, sr, sc))


def med(i, side):
    s = res[i][side]
    return None if s is None else s[0]


print("#")
ok = 1
for nm, side, ideal in (("LEFT ", 1, 1.0), ("RIGHT", 2, -1.0)):
    c0 = med(0, side)
    c3 = med(3, side)
    ml = med(1, side)
    mr = med(2, side)
    if c0 is None or c3 is None or ml is None or mr is None:
        print("#", nm, ": not enough valid readings - is there a wall on that side?")
        ok = 0
        continue
    s0 = res[0][side]
    slope = (mr - ml) / 20.0
    noise = (s0[2] - s0[1]) / 2.56
    print("#", nm, ": centred median", c0, "(repeat", str(c3) + ")",
          " slope %+.2f mm/mm (ideal %+.2f)" % (slope, ideal),
          " noise +-%.0f mm" % noise, " valid %d%%" % (100 * s0[3] // N))
    if slope * ideal < 0.8 or slope * ideal > 1.25:
        print("#   ", nm.strip(), "slope is off by more than 20%: the sensor is angled or sees something else")
    if abs(c0 - c3) > 4:
        print("#   ", nm.strip(), "centred readings differ by", abs(c0 - c3), "mm: placement or noise - repeat the test")
if ok:
    nl = 0.5 * (med(0, 1) + med(3, 1))
    nr = 0.5 * (med(0, 2) + med(3, 2))
    print("# -> HW_NOM_L = %.1f" % nl)
    print("# -> HW_NOM_R = %.1f" % nr)
    print("#    (R - L) / 2 = %+.1f mm: sensor asymmetry at the true centre" % (0.5 * (nr - nl)))
c = med(0, 3)
if c is not None and c < 150:
    print("# front median", c, "mm (axle on a cell centre with a wall ahead -> HW_FRONT_CENTER)")
print("# done")
