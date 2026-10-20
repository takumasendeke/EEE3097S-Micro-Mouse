# Measure HW_FRONT_CENTER (and the side readings) precisely.
# Put the mouse with its AXLE (wheel centres) exactly on a marked cell centre,
# straight, facing a real maze wall, with walls on both sides if possible.
# Run it from Thonny (or: python3 -m mpremote run workspace/front_center.py).
# It samples for ~10 s, ignores dropouts (8190), and prints the medians.
import uct_mouse
uct_mouse.init()
uct_mouse.delay_ms(500)


def med(v):
    if not v:
        return 0
    v.sort()
    return v[len(v) // 2]


def spread(v):
    if len(v) < 10:
        return 0, 0
    v.sort()
    return v[len(v) // 10], v[(len(v) * 9) // 10]


L = []
C = []
R = []
n = 0
while n < 500:
    t = uct_mouse.get_tof()
    if 0 < t[0] < 8190:
        L.append(t[0])
    if 0 < t[2] < 8190:
        C.append(t[2])
    if 0 < t[4] < 8190:
        R.append(t[4])
    uct_mouse.delay_ms(20)
    n += 1

for name, v in (("FRONT", C), ("LEFT ", L), ("RIGHT", R)):
    lo, hi = spread(list(v))
    print(name, "valid", len(v), "of", n, " median", med(list(v)), " mm   (80% of readings between", lo, "and", hi, ")")
print("-> HW_FRONT_CENTER =", med(list(C)), " (front median with the axle on the cell centre)")
