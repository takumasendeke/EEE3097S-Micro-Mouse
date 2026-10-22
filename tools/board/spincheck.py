# Spin / encoder check (~1 minute). BATTERY ON. Run from Thonny (Run).
# Part 1: on the FLOOR in an open area (outer corridor end or a room), nothing
#   within ~10 c. It spins left 1.5 s, stops, spins right 1.5 s, three times,
#   at the mission's spin power, and compares the wheel counts with the gyro.
# Part 2: hold it in the AIR. Each wheel runs backwards and forwards at 3
#   speeds: does the left encoder lose counts going backwards when fast?
# Send Claude everything it prints.
import uct_mouse

uct_mouse.init()
try:
    uct_mouse.set_polarity(1, 1)
except Exception:
    pass
try:
    uct_mouse.set_encoder_polarity(1, 1)
except Exception:
    pass
uct_mouse.set_motors(0, 0)


def sw1(msg):
    print("")
    print(">>> " + msg + " - then press SW1, hands off")
    while uct_mouse.get_button() == 0:
        uct_mouse.delay_ms(20)
    while uct_mouse.get_button() != 0:
        uct_mouse.delay_ms(20)
    uct_mouse.delay_ms(800)


print("SPINCHECK")
try:
    print("battery", uct_mouse.get_vbatt(), "V")
except Exception:
    pass
sw1("1/2 FLOOR, open area")
b = 0.0
n = 0
while n < 100:
    b += uct_mouse.get_gyro()
    uct_mouse.delay_ms(10)
    n += 1
b = b / 100
k = 0
while k < 3:
    for name, pl, pr in (("LEFT spin ", -48, 51), ("RIGHT spin", 48, -51)):
        e0 = uct_mouse.get_encoders()
        t = uct_mouse.get_ticks_ms()
        g = 0.0
        gmax = 0.0
        uct_mouse.set_motors(pl, pr)
        while uct_mouse.get_ticks_ms() - t < 1500:
            uct_mouse.delay_ms(10)
            r = uct_mouse.get_gyro() - b
            g += r * 0.01
            if abs(r) > gmax:
                gmax = abs(r)
        uct_mouse.set_motors(0, 0)
        uct_mouse.delay_ms(600)
        e1 = uct_mouse.get_encoders()
        dl = e1[0] - e0[0]
        dr = e1[1] - e0[1]
        # wheel travel the gyro implies for each wheel (54 mm half-track, 5.709 ticks/mm)
        gt = abs(g) * 0.017453 * 54.0 * 5.709
        print(name, "ticks L", dl, "R", dr, "| gyro", int(g), "deg, peak", int(gmax),
              "deg/s | each wheel should be ~", int(gt), "ticks")
    k += 1
sw1("2/2 AIR: hold it up, both wheels free")
for pwm in (40, 60, 80):
    for name, pl, pr in (("LEFT ", pwm, 0), ("LEFT ", -pwm, 0), ("RIGHT", 0, pwm), ("RIGHT", 0, -pwm)):
        e0 = uct_mouse.get_encoders()
        uct_mouse.set_motors(pl, pr)
        uct_mouse.delay_ms(500)
        uct_mouse.set_motors(0, 0)
        uct_mouse.delay_ms(400)
        e1 = uct_mouse.get_encoders()
        print("air", name, "pwm", pl + pr, "ticks L", e1[0] - e0[0], "R", e1[1] - e0[1])
uct_mouse.set_motors(0, 0)
print("")
print("SPINCHECK DONE - send everything from 'SPINCHECK'")
