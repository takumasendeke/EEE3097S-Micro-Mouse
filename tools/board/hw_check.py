# Polarity check for the Submission 4 HW_ constants.
# Run with:  python3 -m mpremote run hw_check.py
# Put the mouse on the floor with ~30 cm clear in front of it.
import uct_mouse

MOT_L = -1      # current HW_MOTOR_POL_L
MOT_R = -1      # current HW_MOTOR_POL_R

uct_mouse.init()
uct_mouse.set_polarity(MOT_L, MOT_R)
uct_mouse.set_encoder_polarity(1, 1)
uct_mouse.delay_ms(1000)

# gyro bias while still
b = 0.0
i = 0
while i < 100:
    uct_mouse.delay_ms(10)
    b += uct_mouse.get_gyro()
    i += 1
b = b / 100

# 1) both wheels "forward" for 0.4 s
e0 = uct_mouse.get_encoders()
uct_mouse.set_motors(75, 75)
uct_mouse.delay_ms(400)
uct_mouse.set_motors(0, 0)
uct_mouse.delay_ms(400)
e1 = uct_mouse.get_encoders()
print("TEST 1 - did it move FORWARD or BACKWARD?")
print("  raw encoder change  L =", e1[0] - e0[0], "  R =", e1[1] - e0[1])

# 2) right wheel only "forward" for 0.3 s
g = 0.0
e0 = uct_mouse.get_encoders()
uct_mouse.set_motors(0, 75)
i = 0
while i < 30:
    uct_mouse.delay_ms(10)
    g += (uct_mouse.get_gyro() - b) * 0.01
    i += 1
uct_mouse.set_motors(0, 0)
uct_mouse.delay_ms(400)
e1 = uct_mouse.get_encoders()
print("TEST 2 - did it turn LEFT (anticlockwise) or RIGHT?")
print("  raw encoder change  L =", e1[0] - e0[0], "  R =", e1[1] - e0[1])
print("  integrated gyro =", g)
