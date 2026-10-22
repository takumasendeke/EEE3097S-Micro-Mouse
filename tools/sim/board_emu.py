#!/usr/bin/env python3
"""Make a copy of main.py that behaves like Takudzwa's physical board in the simulator.

The simulator's mouse is too perfect (clean ToF, sim profile constants). This writes
<out>/main.py = main.py with:
  * the HW_ (board) tuning copied into the simulator profile (stop-each-cell, auto
    maze side, 100 mm wall threshold, slower speeds, calmer turns, ToF sigma 15 ...)
  * uct_mouse wrapped to emulate the real sensors: ~50% ToF dropouts (8190),
    +-15 mm noise, 4% stray short echoes, a fixed offset on one side sensor,
    optional gyro bias that appears after start, optional mirrored world
    (maze on the mouse's RIGHT instead of left).

  python3 devtools/board_emu.py workspace/final_task/main.py /tmp/emu_left
  python3 devtools/board_emu.py workspace/final_task/main.py /tmp/emu_right --mirror
  python3 devtools/harness.py /tmp/emu_left/main.py --seeds 42,43,44,45,48,101,102,103,104,105 -j 5

  # board-faithful: the real HW_ profile on a board-like chassis (dead-band 61/62)
  python3 devtools/board_emu.py workspace/final_task/main.py /tmp/hw_l --hw
  python3 devtools/harness.py /tmp/hw_l/main.py --cfg devtools/cfg_board.json --max-time 300 --seeds ...

Everything must pass here before a build goes to the real mouse.
If a replace() target is missing, main.py's runtime constant block changed - update the table.
"""
import sys, os, argparse

ap = argparse.ArgumentParser()
ap.add_argument("src")
ap.add_argument("out")
ap.add_argument("--mirror", action="store_true", help="maze on the mouse's right")
ap.add_argument("--offset", type=float, default=16.0, help="mm added to the left ToF slot")
ap.add_argument("--drop", type=float, default=0.5, help="ToF dropout probability")
ap.add_argument("--gyro-drift", type=float, default=0.3, help="deg/s bias appearing after 4 s")
ap.add_argument("--tpmm", type=float, default=4.6, help="code's ticks/mm (sim truth 4.4)")
ap.add_argument("--hw", action="store_true",
                help="run the real HW_ profile; use with --cfg devtools/cfg_board.json")
ap.add_argument("--stiction", default=None,
                help="L,R PWM a wheel at rest needs to start (flight log: ~72-76; default with --hw)")
ap.add_argument("--enc-fault", default="",
                help="encoder fault on the physical wheel: lrev, lstuck+, lstuck-, lweakrev, rrev, rstuck+, rstuck-, rweakrev "
                     "(stuck = counts one way whichever way the wheel turns, as the v64 board's left one did)")
ap.add_argument("--compute-ms", type=int, default=None,
                help="board compute time per control step, ms (flight log: loop ~22 ms; default 10 with --hw)")
a = ap.parse_args()
if a.stiction is None:
    a.stiction = "72,76" if a.hw else "0,0"
if a.compute_ms is None:
    a.compute_ms = 10 if a.hw else 0
_bl, _br = [float(v) for v in a.stiction.split(",")]

inj = '''import uct_mouse
import random as _r
_tof0 = uct_mouse.get_tof
_g0 = uct_mouse.get_gyro
_m0 = uct_mouse.set_motors
_e0 = uct_mouse.get_encoders
_MIR = %(mir)s
_dl0 = uct_mouse.delay_ms
# Real board: a wheel at rest does not start below its breakaway PWM (static
# friction, worse on the right), and every control step costs compute time.
_BRK = (%(bl)r, %(br)r)
_CMP = %(cm)d
_last_e = [None]
_mov = [1, 1]
def _sm_phys(l, r):
    e = _e0()
    if _last_e[0] is not None:
        _mov[0] = 1 if abs(e[0] - _last_e[0][0]) > 0.3 else 0
        _mov[1] = 1 if abs(e[1] - _last_e[0][1]) > 0.3 else 0
    _last_e[0] = e
    # spinning on the spot (wheels opposite) the tyres scrub: below the
    # breakaway PWM the wheels stall even while turning (flight log: +-64..70
    # stalled, +-75 spun); driving forwards it only bites from rest
    spin = l * r < 0
    if (spin or not _mov[0]) and abs(l) < _BRK[0]:
        l = 0
    if (spin or not _mov[1]) and abs(r) < _BRK[1]:
        r = 0
    _m0(l, r)
uct_mouse.set_motors = _sm_phys
def _tof():
    if _CMP > 0:
        _dl0(_CMP)
    t = _tof0()
    if _MIR:
        t = (t[4], t[3], t[2], t[1], t[0])
    out = []
    k = 0
    for v in t:
        if _r.random() < 0.04:
            out.append(_r.randint(25, 60))
        elif v >= 8190 or _r.random() < %(drop)r:
            out.append(8190)
        else:
            out.append(max(5, int(v + _r.gauss(0, 15) + (%(off)r if k == 0 else 0))))
        k += 1
    return tuple(out)
uct_mouse.get_tof = _tof
_sg = -1.0 if _MIR else 1.0
uct_mouse.get_gyro = lambda: _sg * _g0() + (%(gd)r if uct_mouse.get_ticks_ms() > 4000 else 0.0)
if _MIR:
    uct_mouse.set_motors = lambda l, r: _sm_phys(r, l)
    def _enc():
        e = _e0()
        return (e[1], e[0])
    uct_mouse.get_encoders = _enc
_EF = %(ef)r
if _EF:
    _ge = uct_mouse.get_encoders
    _k = 0 if _EF[0] == "l" else 1
    _fs = [None, 0]
    def _encf():
        e = list(_ge())
        if _fs[0] is None:
            _fs[0] = e[_k]
            _fs[1] = e[_k]
        dd = e[_k] - _fs[0]
        _fs[0] = e[_k]
        if _EF[1:] == "revweak":
            _fs[1] -= dd * (0.2 if dd > 0 else 1.0)       # 31 Oct: counts reversed, and forwards mostly lost
        elif _EF[1:] == "weakrev":
            _fs[1] += dd * (0.15 if dd < 0 else 1.0)     # v71 board: counts backwards are mostly lost
        elif _EF[1:] == "rev":
            _fs[1] -= dd
        elif _EF[1:] == "stuck+":
            _fs[1] += abs(dd)
        else:
            _fs[1] -= abs(dd)
        e[_k] = _fs[1]
        return tuple(e)
    uct_mouse.get_encoders = _encf
''' % dict(ef=a.enc_fault, mir=a.mirror, drop=a.drop, off=a.offset, gd=a.gyro_drift, bl=_bl, br=_br, cm=a.compute_ms)

s = open(a.src).read()
s = s.replace("import uct_mouse\n", inj, 1)
# simulator-profile constant -> board-like value (sim keeps its own dead-band truth ~28)
table = [
    ("SYSID = 1 ", "SYSID = 0 "),
    ("TPMM = 5.0 ", "TPMM = %r " % a.tpmm),
    ("DB_L = 40.0 ", "DB_L = 30.0 "),
    ("DB_R = 40.0\n", "DB_R = 29.0\n"),
    ("SIDE_GUARD = 0.0 ", "SIDE_GUARD = 25.0 "),
    ("SIDE = 1\nSIDE_AUTO = 0\n", "SIDE = 1\nSIDE_AUTO = 1\n"),
    ("SIDE_WALL = 150.0", "SIDE_WALL = 100.0"),
    ("STOP_EACH = 0\n", "STOP_EACH = 1\n"),
    ("TOF_SIGMA = 6.0\n", "TOF_SIGMA = 15.0\n"),
    ("V_EXPLORE = 260.0\n", "V_EXPLORE = 160.0\n"),
    ("V_RETURN = 320.0\n", "V_RETURN = 250.0\n"),
    ("V_SPRINT = 360.0\n", "V_SPRINT = 330.0\n"),
    ("V_TURN = 215.0 ", "V_TURN = 170.0 "),
    ("W_TURN = 330.0\n", "W_TURN = 180.0\n"),
    ("ALPHA_TURN = 3000.0\n", "ALPHA_TURN = 1000.0\n"),
    ("TURN_TOL = 1.0\n", "TURN_TOL = 3.0\n"),
    ("KI_W = 1.2\n", "KI_W = 0.2\n"),
]
if a.hw:
    # Run the REAL board profile (every HW_ constant, board control code paths)
    # instead of copying values into the sim profile. Pair it with the
    # board-like chassis: --cfg devtools/cfg_board.json (dead-band 61/62,
    # 60 ms lag, 5600 true ticks/m vs the code's 5709).
    table = [
        ("    IS_SIM = on_simulator()\n    if IS_SIM == 1:\n        return\n", "    IS_SIM = 0\n"),
    ]
    import re
    for name, val in (("HW_ENC_POL_L", "1"), ("HW_ENC_POL_R", "1"), ("HW_WAIT_BUTTON", "0")):
        s, n = re.subn(r"^(%s\s*=\s*)-?\d+" % name, r"\g<1>" + val, s, count=1, flags=re.M)
        if n != 1:
            sys.exit("board_emu: %s not found in %s" % (name, a.src))
for old, new in table:
    if old not in s:
        sys.exit("board_emu: '%s' not found in %s - update the table" % (old.strip(), a.src))
    s = s.replace(old, new)
os.makedirs(a.out, exist_ok=True)
open(os.path.join(a.out, "main.py"), "w").write(s)
print("wrote", os.path.join(a.out, "main.py"), "(mirror)" if a.mirror else "")
