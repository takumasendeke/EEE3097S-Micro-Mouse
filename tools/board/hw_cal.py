# Hand calibration: encoder ticks per metre and gyro scale.
# In the REPL:  import hw_cal
# Keep the mouse still for the first 2 s, then follow the prompts.
import uct_mouse
uct_mouse.init()
uct_mouse.set_motors(0, 0)
uct_mouse.delay_ms(500)
print("Keep still - measuring gyro bias...")
b = 0.0
i = 0
while i < 200:
    uct_mouse.delay_ms(10)
    b += uct_mouse.get_gyro()
    i += 1
b = b / 200
e0 = uct_mouse.get_encoders()
g = 0.0
t = uct_mouse.get_ticks_ms()
print("GO. 1) Push it straight forward by hand exactly 2 cells (36 cm if 18 cm cells - measure!)")
print("    2) Then rotate it on the spot by hand exactly one full turn ANTICLOCKWISE (360 deg)")
print("Printing for 40 s:  encoder L, encoder R, gyro integral")
i = 0
while i < 4000:
    uct_mouse.delay_ms(10)
    t2 = uct_mouse.get_ticks_ms()
    g += (uct_mouse.get_gyro() - b) * (t2 - t) * 0.001
    t = t2
    if i % 25 == 0:
        e = uct_mouse.get_encoders()
        print("encL", e[0] - e0[0], "  encR", e[1] - e0[1], "  gyro", int(g))
    i += 1
