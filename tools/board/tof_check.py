# ToF sensor check. Run:  python3 -m mpremote run workspace/tof_check.py
# Prints all five ToF slots twice a second for 20 s. Move your hand / put
# the mouse next to walls and watch which numbers respond.
import uct_mouse
uct_mouse.init()
uct_mouse.delay_ms(500)
print("slots:      L   FL    C   FR    R      (8190 = nothing seen)")
n = 0
while n < 40:
    t = uct_mouse.get_tof()
    r = uct_mouse.get_tof_raw()
    print("filtered", t, "  raw", r)
    uct_mouse.delay_ms(500)
    n += 1
