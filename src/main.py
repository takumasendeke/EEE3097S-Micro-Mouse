# =========================================================================
# EEE3097S Micromouse - Submission 4: Search, Celebrate & Sprint
#
# Author : Takudzwa Masendeke
# Date   : 19 October 2026
#
# AI assistance: developed with help from Gemini Pro 3.1 and Claude.
#
# Phase 1  floodfill exploration; walls classified on the move from the
#          three ToF sensors; corridor centring by the 2-state [y, psi]
#          Kalman filter from the GA3 report (side ToF + gyro + encoders).
# Phase 2  first 2x2 block with all four interior walls seen open is the
#          target room: stop, 360 deg clockwise pirouette on the gyro.
# Phase 3  turn-weighted Dijkstra over KNOWN-open walls back to (0,0),
#          then face the start heading ("North").
# Phase 4  3.0 s pause, then the same planner from (0,0) to the room;
#          straights merged into one accel-cruise-decel profile each.
#
# One file for co-simulator and board: on_simulator() picks the profile,
# so the FNV-1a hash stays valid against the physical run log. Only the
# HW_ constants affect the real mouse.
#
# Frames: the map grid is in "mouse frame" - x along the start heading,
# y to the arena side. Dir 0 = start heading (+x, called North in the
# brief), 1 = +y, 2 = -x, 3 = -y. Headings in degrees, CCW positive.
# =========================================================================
import uct_mouse

BUILD = "v15 19-Oct 18:20"

try:
    import math
    _sqrt = math.sqrt
    _sin = math.sin
    _cos = math.cos
except ImportError:
    def _sqrt(x):
        if x <= 0.0:
            return 0.0
        g = x if x > 1.0 else 1.0
        for _ in range(20):
            g = 0.5 * (g + x / g)
        return g

    def _sin(x):
        while x > 3.14159265:
            x -= 6.28318531
        while x < -3.14159265:
            x += 6.28318531
        x2 = x * x
        return x * (1.0 - x2 / 6.0 * (1.0 - x2 / 20.0 * (1.0 - x2 / 42.0 * (1.0 - x2 / 72.0))))

    def _cos(x):
        return _sin(x + 1.57079633)

# ---------------- hardware profile (this mouse) --------------------------
HW_MOTOR_POL_L    = 1             # flipped for the Oct-2026 MicroPython firmware
HW_MOTOR_POL_R    = 1
HW_ENC_POL_L      = -1
HW_ENC_POL_R      = 1
HW_LEFT_DEADBAND  = 61.0        # rolling threshold from pwm_sweep.py
HW_RIGHT_DEADBAND = 62.0
HW_TICKS_PER_M    = 5709.0      # measured (M1); start spins on new firmware gave 5460-5810
HW_GYRO_GAIN      = 1.0         # measured: 360 deg by hand reads +360 (0 = auto-measure at start)
HW_CAL_SAMPLES    = 150         # start-up gyro samples (mouse still)
HW_VMAX           = 500.0       # mm/s at effort 100 (46 effort ~ 0.23 m/s in M1)
HW_TAU            = 0.06        # s, motor lag used for accel feed-forward
HW_MAZE_ROWS      = 0           # 0 = ask uct_mouse.get_maze_dimensions()
HW_MAZE_COLS      = 0           # (board uses a square grid of the larger side)
HW_ARENA_SIDE     = 0           # 0 = detect from the first side opening (+1 maze on the LEFT of the start heading, -1 RIGHT)
HW_SIDE_X         = 50.0        # mm side ToF ahead of the axle
HW_FRONT_OFF      = 65.0        # mm front ToF ahead of the axle
HW_WALL_K         = 0.0         # deg/s of steering per mm off-centre, straight from side ToF
HW_GYRO_TRIM      = 0.0         # deg/s added to gyro bias. Drifts RIGHT -> make it NEGATIVE (see straight test)
HW_TEST_STRAIGHT  = 0           # >0: just drive this many cells straight and stop (calibration)
HW_STOP_EACH_CELL = 1           # 1 = stop at every cell centre and sense walls standing still
HW_SIDE_GUARD     = 35.0        # mm: side reflex - steer away harder the closer a wall is inside this
HW_FRONT_STOP     = 22.0        # mm: front reflex - forward motion blocked below this
HW_FRONT_CENTER   = 34.0        # mm: front reading with the axle on the cell centre (log: 32)
HW_TILT_LIM       = 0.25        # climb detector: accel tilt as a fraction of g (0 = off)
HW_SIDE_WALL      = 100.0       # mm: walls read 20-70, open sides 140+ (measured)
HW_FRONT_FIX_MAX  = 150.0       # mm: trust front ToF for distance only below this
HW_FRONT_SNAP     = 30.0        # mm: front fix only if within this of a wall line
HW_TOF_SIGMA      = 15.0        # mm, KF measurement noise (measured spread ~20-70)
HW_V_EXPLORE      = 160.0       # mm/s  EXPLORE SPEED
HW_V_RETURN       = 250.0       # mm/s
HW_V_SPRINT       = 330.0       # mm/s  SPRINT SPEED. Down if it wanders.
HW_V_TURN         = 170.0       # mm/s through smooth 90 deg arcs
HW_ACCEL          = 1200.0      # mm/s^2
HW_W_TURN         = 180.0       # deg/s in-place turn ceiling
HW_ALPHA_TURN     = 1000.0      # deg/s^2
HW_TURN_TOL       = 3.0         # deg
HW_ROOM_DOORS     = 2           # max openings in the target room's rim
HW_KP_V           = 0.10        # effort per mm/s (common mode)
HW_KI_V           = 0.6
HW_KP_W           = 0.06        # effort per deg/s (differential, on gyro)
HW_KI_W           = 0.2
HW_WAIT_BUTTON    = 1           # 1 = press SW1 to start (board only)
HW_DEBUG          = 1           # 1 = print sensing/decisions (run tethered via mpremote)
HW_TPMM_AUTO      = 0           # 1 = measure encoder ticks/mm at start (spin vs gyro)
HW_KF_HEADING     = 1           # 1 = walls also correct heading (needed: stops slip the gyro cannot see)
HW_SYSID          = 0           # 1 = measure dead-bands/ticks per mm with a start spin

# ---------------- runtime values (simulator defaults) --------------------
CELL = 200.0
DT_MS = 10
IS_SIM = 1
MOT_POL_L = 1
MOT_POL_R = 1
ENC_POL_L = 1
ENC_POL_R = 1
DB_L = 40.0                     # first guesses only - SYSID measures them
DB_R = 40.0
TPMM = 5.0                      # encoder ticks per mm
GYRO_GAIN = 1.0
TPMM_AUTO = 0
GYRO_AUTO = 0                   # 1 = measure gyro sign/scale at start (board default)
CAL_SAMPLES = 150
VMAX = 400.0
VMAX_L = 400.0                  # mm/s per wheel at PWM 100 (identified in sim)
VMAX_R = 400.0
TAU = 0.06
SYSID = 1                       # identify dead-bands, wheel gains, ticks/mm at start
MAZE_ROWS = 0
MAZE_COLS = 0
SIDE = 1
SIDE_AUTO = 0
SIDE_X = 50.0
FRONT_OFF = 65.0
SIDE_WALL = 150.0
FRONT_FIX_MAX = 2000.0
TOF_SIGMA = 6.0
V_EXPLORE = 260.0
V_RETURN = 320.0
V_SPRINT = 360.0
V_TURN = 215.0                  # mm/s through arcs (outer wheel = 1.54x)
ACCEL = 2000.0
W_TURN = 330.0
ALPHA_TURN = 3000.0
TURN_TOL = 1.0
KP_V = 0.10
KI_V = 0.8
KP_W = 0.12
KI_W = 1.2
HALF_AXLE = 54.0                # mm
NOM_L = 60.0                    # centred reading of each side sensor (measured at start)
NOM_R = 60.0
SIDE_NOM = 60.0                 # mm side reading when centred (re-measured at start)
ROOM_MAX_DOORS = 1              # openings in the room's rim (sim generator: always 1)
WALL_K = 0.0                    # direct side-wall steering (board)
GYRO_TRIM = 0.0
TEST_STRAIGHT = 0
STOP_EACH = 0
KF_HEADING = 1
FRONT_STOP = 0.0                # sim never needs the reflexes; board values above
FRONT_CENTER = 35.0             # sim: 100 - FRONT_OFF
TILT_LIM = 0.0
SIDE_GUARD = 0.0                # mm: steer away below this (board only; sim never needs it)
DECIDE_AHEAD = 60.0             # decide a cell this far before its centre
FRONT_SNAP = 45.0
FRONT_GAIN = 0.15
KLAT = 0.008                    # rad of heading demand per mm of offset
PSI_MAX = 0.14                  # rad
KPSI = 8.0                      # 1/s heading loop
ARC_KR = 4.0                    # deg/s per mm of radius error in arcs
ARC_KH = 8.0                    # 1/s on heading error to the arc tangent
DEBUG = 1

DX = (1, 0, -1, 0)
DY = (0, 1, 0, -1)
D2R = 0.0174532925


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def fabs(x):
    if x < 0.0:
        return -x
    return x


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def log(*args):
    if DEBUG:
        print(*args)


# 1 in co-simulation, 0 on the board - CPython has os.environ, MicroPython does not.
def on_simulator():
    try:
        import os
        os.environ
        return 1
    except AttributeError:
        return 0
    except Exception:
        pass
    return 0


def select_profile():
    global IS_SIM, MOT_POL_L, MOT_POL_R, ENC_POL_L, ENC_POL_R, DB_L, DB_R, TPMM
    global GYRO_GAIN, CAL_SAMPLES, VMAX, TAU, MAZE_ROWS, MAZE_COLS, SIDE, SIDE_X
    global FRONT_OFF, SIDE_WALL, FRONT_FIX_MAX, TOF_SIGMA, V_EXPLORE, V_RETURN
    global V_SPRINT, V_TURN, ACCEL, W_TURN, ALPHA_TURN, TURN_TOL, KP_V, KI_V, KP_W, KI_W
    global DEBUG, FRONT_STOP, FRONT_CENTER, TILT_LIM, KF_HEADING, STOP_EACH, WALL_K, GYRO_TRIM, TEST_STRAIGHT, SIDE_AUTO, SIDE_GUARD, TPMM_AUTO, FRONT_SNAP, GYRO_AUTO, ROOM_MAX_DOORS, VMAX_L, VMAX_R, SYSID
    IS_SIM = on_simulator()
    if IS_SIM == 1:
        return
    DEBUG = HW_DEBUG
    MOT_POL_L = HW_MOTOR_POL_L
    MOT_POL_R = HW_MOTOR_POL_R
    ENC_POL_L = HW_ENC_POL_L
    ENC_POL_R = HW_ENC_POL_R
    DB_L = HW_LEFT_DEADBAND
    DB_R = HW_RIGHT_DEADBAND
    TPMM = HW_TICKS_PER_M / 1000.0
    GYRO_GAIN = HW_GYRO_GAIN
    if GYRO_GAIN == 0.0:
        GYRO_GAIN = 1.0
        GYRO_AUTO = 1
    CAL_SAMPLES = HW_CAL_SAMPLES
    VMAX = HW_VMAX
    VMAX_L = HW_VMAX
    VMAX_R = HW_VMAX
    SYSID = HW_SYSID
    TPMM_AUTO = HW_TPMM_AUTO
    TAU = HW_TAU
    MAZE_ROWS = HW_MAZE_ROWS
    MAZE_COLS = HW_MAZE_COLS
    SIDE = HW_ARENA_SIDE
    if SIDE == 0:
        SIDE = 1
        SIDE_AUTO = 1
    SIDE_X = HW_SIDE_X
    FRONT_OFF = HW_FRONT_OFF
    SIDE_WALL = HW_SIDE_WALL
    SIDE_GUARD = HW_SIDE_GUARD
    FRONT_STOP = HW_FRONT_STOP
    FRONT_CENTER = HW_FRONT_CENTER
    TILT_LIM = HW_TILT_LIM
    KF_HEADING = HW_KF_HEADING
    STOP_EACH = HW_STOP_EACH_CELL
    WALL_K = HW_WALL_K
    GYRO_TRIM = HW_GYRO_TRIM
    TEST_STRAIGHT = HW_TEST_STRAIGHT
    FRONT_FIX_MAX = HW_FRONT_FIX_MAX
    TOF_SIGMA = HW_TOF_SIGMA
    FRONT_SNAP = HW_FRONT_SNAP
    V_EXPLORE = HW_V_EXPLORE
    V_RETURN = HW_V_RETURN
    V_SPRINT = HW_V_SPRINT
    V_TURN = HW_V_TURN
    ACCEL = HW_ACCEL
    W_TURN = HW_W_TURN
    ALPHA_TURN = HW_ALPHA_TURN
    TURN_TOL = HW_TURN_TOL
    ROOM_MAX_DOORS = HW_ROOM_DOORS
    KP_V = HW_KP_V
    KI_V = HW_KI_V
    KP_W = HW_KP_W
    KI_W = HW_KI_W


# =========================================================================
# MazeSolver: wall map, floodfill exploration, room detection, planning.
# Grid size comes from the arguments only - nothing assumes 4x6.
# =========================================================================
class MazeSolver:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.n = rows * cols
        self.wall = [0] * self.n        # bit d set = wall on side d
        self.known = [0] * self.n       # bit d set = side d has been observed
        self.visited = [0] * self.n
        self.room = None                # list of 4 cell indices once found
        x = 0
        while x < cols:
            self.set_wall(x, 0, 3, 1)
            self.set_wall(x, rows - 1, 1, 1)
            x += 1
        y = 0
        while y < rows:
            self.set_wall(0, y, 2, 1)
            self.set_wall(cols - 1, y, 0, 1)
            y += 1

    def inside(self, x, y):
        return 0 <= x < self.cols and 0 <= y < self.rows

    def set_wall(self, x, y, d, present):
        if not self.inside(x, y):
            return
        i = y * self.cols + x
        b = 1 << d
        if self.known[i] & b and (self.wall[i] & b) and not present:
            return                      # never erase a seen wall (outer walls)
        self.known[i] |= b
        if present:
            self.wall[i] |= b
        else:
            self.wall[i] &= ~b
        nx = x + DX[d]
        ny = y + DY[d]
        if self.inside(nx, ny):
            j = ny * self.cols + nx
            ob = 1 << ((d + 2) & 3)
            self.known[j] |= ob
            if present:
                self.wall[j] |= ob
            else:
                self.wall[j] &= ~ob

    def is_open(self, i, d, optimistic):
        b = 1 << d
        if self.wall[i] & b:
            return False
        if self.known[i] & b:
            return True
        return optimistic

    # 2x2 blocks with bottom-left (bx, by): the four interior walls
    def _block(self, bx, by):
        c = self.cols
        i0 = by * c + bx
        return ((i0, 0), (i0, 1), (i0 + 1, 1), (i0 + c, 0))

    # The 8 walls round the outside of the block (side d of cell i).
    def _rim(self, bx, by):
        c = self.cols
        i0 = by * c + bx
        return ((i0, 3), (i0, 2), (i0 + 1, 3), (i0 + 1, 0),
                (i0 + c, 1), (i0 + c, 2), (i0 + c + 1, 1), (i0 + c + 1, 0))

    # Count (open, unknown) walls in a list of (cell, side).
    def _count(self, sides):
        nopen = 0
        nunk = 0
        for i, d in sides:
            b = 1 << d
            if not (self.known[i] & b):
                nunk += 1
            elif not (self.wall[i] & b):
                nopen += 1
        return nopen, nunk

    # Target room = 2x2 block whose 4 interior walls are all seen open and
    # whose 8 rim walls are all seen, with at most ROOM_MAX_DOORS openings.
    # (Braided mazes can contain other open 2x2 areas; they have more doors.)
    def check_room(self):
        if self.room is not None:
            return self.room
        by = 0
        while by < self.rows - 1:
            bx = 0
            while bx < self.cols - 1:
                io, iu = self._count(self._block(bx, by))
                if io == 4:
                    ro, ru = self._count(self._rim(bx, by))
                    if ru == 0 and ro <= ROOM_MAX_DOORS:
                        i0 = by * self.cols + bx
                        self.room = [i0, i0 + 1, i0 + self.cols, i0 + self.cols + 1]
                        log("ROOM found at", bx, by, "doors", ro)
                        return self.room
                bx += 1
            by += 1
        return None

    # Seeds for the exploration floodfill: cells that could complete a
    # plausible room are cheapest, any other unvisited cell costs more.
    def _explore_seeds(self):
        if self.room is not None:
            return [(i, 0) for i in self.room]
        seeds = []
        best = [9] * self.n
        by = 0
        while by < self.rows - 1:
            bx = 0
            while bx < self.cols - 1:
                nopen, nunk = self._count(self._block(bx, by))
                bad = 0 if nopen + nunk == 4 else 1
                if not bad:
                    ro, ru = self._count(self._rim(bx, by))
                    if ro > ROOM_MAX_DOORS:
                        bad = 1
                if not bad and nopen >= 2:
                    i0 = by * self.cols + bx
                    cost = 0 if nopen >= 3 else 1
                    for i in (i0, i0 + 1, i0 + self.cols, i0 + self.cols + 1):
                        if not self.visited[i] and cost < best[i]:
                            best[i] = cost
                bx += 1
            by += 1
        i = 0
        while i < self.n:
            if not self.visited[i]:
                c = best[i]
                if c > 3:
                    c = 3
                seeds.append((i, c))
            i += 1
        return seeds

    # Multi-source floodfill (Dial's algorithm on unit edges).
    def flood(self, seeds, optimistic):
        INF = 30000
        dist = [INF] * self.n
        buckets = []
        for i, c in seeds:
            if c < dist[i]:
                dist[i] = c
                while len(buckets) <= c:
                    buckets.append([])
                buckets[c].append(i)
        lvl = 0
        while lvl < len(buckets):
            for i in buckets[lvl]:
                if dist[i] != lvl:
                    continue
                x = i % self.cols
                y = i // self.cols
                for d in range(4):
                    if self.is_open(i, d, optimistic):
                        nx = x + DX[d]
                        ny = y + DY[d]
                        if self.inside(nx, ny):
                            j = ny * self.cols + nx
                            if dist[j] > lvl + 1:
                                dist[j] = lvl + 1
                                while len(buckets) <= lvl + 1:
                                    buckets.append([])
                                buckets[lvl + 1].append(j)
            lvl += 1
        return dist

    # Next heading for exploration from (x, y) facing d; -1 when done.
    def next_dir(self, x, y, d):
        seeds = self._explore_seeds()
        if not seeds:
            return -1
        dist = self.flood(seeds, 1)
        i = y * self.cols + x
        best = -1
        bestv = 30000
        for k in (0, 1, 3, 2):          # straight first, then left/right, back last
            nd = (d + k) & 3
            if self.is_open(i, nd, 0):
                nx = x + DX[nd]
                ny = y + DY[nd]
                if self.inside(nx, ny):
                    v = dist[ny * self.cols + nx]
                    if v < bestv:
                        bestv = v
                        best = nd
        if bestv >= 30000:
            return -1
        return best

    # Turn-weighted Dijkstra over KNOWN-open walls. State = cell*4 + heading.
    # Cost: 4 per cell, +2 per 90 deg turn (a smooth arc), +7 for a 180.
    # Returns the heading of every cell-to-cell move, or None.
    def plan(self, x, y, d, goals):
        n4 = self.n * 4
        INF = 30000
        dist = [INF] * n4
        par = [-1] * n4
        s0 = (y * self.cols + x) * 4 + d
        dist[s0] = 0
        buckets = [[s0]]
        goal_state = -1
        lvl = 0
        while lvl < len(buckets) and goal_state < 0:
            for s in buckets[lvl]:
                if dist[s] != lvl:
                    continue
                i = s >> 2
                h = s & 3
                if i in goals:
                    goal_state = s
                    break
                cand = []
                if self.is_open(i, h, 0):
                    nx = i % self.cols + DX[h]
                    ny = i // self.cols + DY[h]
                    if self.inside(nx, ny):
                        cand.append(((ny * self.cols + nx) * 4 + h, 4))
                cand.append((i * 4 + ((h + 1) & 3), 2))
                cand.append((i * 4 + ((h + 3) & 3), 2))
                cand.append((i * 4 + ((h + 2) & 3), 7))
                for t, c in cand:
                    nv = lvl + c
                    if nv < dist[t]:
                        dist[t] = nv
                        par[t] = s
                        while len(buckets) <= nv:
                            buckets.append([])
                        buckets[nv].append(t)
            lvl += 1
        if goal_state < 0:
            return None
        chain = []
        s = goal_state
        while s >= 0:
            chain.append(s)
            s = par[s]
        chain.reverse()
        moves = []
        k = 1
        while k < len(chain):
            if (chain[k - 1] >> 2) != (chain[k] >> 2):
                moves.append(chain[k] & 3)
            k += 1
        return moves


# =========================================================================
# Mouse: sensing, pose estimate, velocity loops and motion primitives.
# =========================================================================
class Mouse:
    def __init__(self):
        self.th = 0.0           # heading, deg, map frame, CCW+
        self.X = CELL / 2.0     # mm, map frame
        self.Y = CELL / 2.0
        self.d = 0              # current axis
        self.cx = 0
        self.cy = 0
        self.el = 0
        self.er = 0
        self.v = 0.0            # measured forward speed mm/s
        self.rate = 0.0         # gyro rate deg/s, map frame
        self.rate_p = 0.0       # physical-frame rate
        self.iv = 0.0
        self.iw = 0.0
        self.bias = 0.0
        self.last_ms = 0
        self.dL = 8190
        self.dC = 8190
        self.dR = 8190
        self.ds = 0.0
        self.dt = 0.01
        self.vcmd_prev = 0.0
        self.wcmd_prev = 0.0
        self.p11 = 100.0        # KF covariance: lateral mm^2
        self.p12 = 0.0
        self.p22 = 0.0003       # heading rad^2
        self.t0 = 0
        self.vc = 0.0           # commanded forward speed (profile state)
        self.nv = [0, 0, 0, 0, 0]   # valid-reading count per ToF slot
        self.dead = 0
        self.wlast = 0.0
        self.hC = 8190
        self.aC = 99
        self.bump = 0
        self.tilt_n = 0
        self.a0 = None
        self.tk = 0
        self.hL = 8190
        self.hR = 8190
        self.aL = 99
        self.aR = 99
        self.el_last = 0
        self.er_last = 0

    # ---- sensing --------------------------------------------------------
    def now(self):
        try:
            return uct_mouse.get_ticks_ms()
        except Exception:
            return self.last_ms + DT_MS

    def sense(self):
        t = self.now()
        dt = (t - self.last_ms) * 0.001
        self.last_ms = t
        if dt <= 0.0:
            dt = DT_MS * 0.001
        elif dt > 0.25:
            dt = 0.25
        self.dt = dt
        e = uct_mouse.get_encoders()
        l = e[0] * ENC_POL_L
        r = e[1] * ENC_POL_R
        dl = (l - self.el) / TPMM
        dr = (r - self.er) / TPMM
        self.el = l
        self.er = r
        ds = 0.5 * (dl + dr)
        self.ds = ds
        self.v += 0.4 * (ds / dt - self.v)
        g = (uct_mouse.get_gyro() - self.bias) / GYRO_GAIN
        self.rate_p = g
        self.rate = g * SIDE
        thm = self.th + 0.5 * self.rate * dt          # mid-step heading
        self.th += self.rate * dt
        a = thm * D2R
        self.X += ds * _cos(a)
        self.Y += ds * _sin(a)
        # Five ToF slots (left, front-left, front, front-right, right). Some
        # boards report the side sensors in the front-left/right slots, so use
        # whichever slot of each pair has actually been returning readings.
        tof = uct_mouse.get_tof()
        nv = self.nv
        k = 0
        while k < 5:
            if 0 < tof[k] < 8190:
                nv[k] += 1
            k += 1
        a = tof[0] if nv[0] >= nv[1] else tof[1]
        b = tof[4] if nv[4] >= nv[3] else tof[3]
        if a <= 0:
            a = 8190
        if b <= 0:
            b = 8190
        if SIDE > 0:
            self.dL = a
            self.dR = b
        else:
            self.dL = b
            self.dR = a
        c = tof[2]
        self.dC = c if c > 0 else 8190
        # last valid side reading, held for up to 8 steps across dropouts
        if self.dL < 8190:
            self.hL = self.dL
            self.aL = 0
        else:
            self.aL += 1
        if self.dR < 8190:
            self.hR = self.dR
            self.aR = 0
        else:
            self.aR += 1
        if self.dC < 8190:
            self.hC = self.dC
            self.aC = 0
        else:
            self.aC += 1

    # ---- actuation ------------------------------------------------------
    @staticmethod
    def _pwm(eff, db):
        if eff > 0.0:
            p = db + eff * (100.0 - db) / 100.0
        elif eff < 0.0:
            p = -db + eff * (100.0 - db) / 100.0
        else:
            return 0
        return int(clamp(p, -100.0, 100.0))

    # One 10 ms control step: v in mm/s, w in deg/s (map frame).
    def step(self, v, w):
        dt = self.dt
        # FRONT REFLEX: never drive forward into a wall the front ToF can see
        if v > 0.0 and FRONT_STOP > 0.0 and self.aC < 4 and self.hC < FRONT_STOP:
            v = 0.0
        # CLIMB DETECTOR: body tilting = a wheel is riding up a wall
        if TILT_LIM > 0.0 and v > 0.0:
            self.tk += 1
            if self.tk % 2 == 0 and self.tilted():
                self.bump = 1
                v = 0.0
                w = 0.0
        if v == 0.0 and w == 0.0:
            uct_mouse.set_motors(0, 0)
            self.iv *= 0.5
            self.iw *= 0.5
            self.vcmd_prev = 0.0
            self.wcmd_prev = 0.0
        else:
            wp = w * SIDE
            ev = v - self.v
            ew = wp - self.rate_p
            self.iv = clamp(self.iv + KI_V * ev * dt, -25.0, 25.0)
            self.iw = clamp(self.iw + KI_W * ew * dt, -25.0, 25.0)
            uv = KP_V * ev + self.iv
            uw = KP_W * ew + self.iw
            wl = wp * D2R * HALF_AXLE
            cl = v - wl
            cr = v + wl
            # feed-forward of each wheel's commanded acceleration (motor lag)
            al = (cl - self.vcmd_prev) / dt
            ar = (cr - self.wcmd_prev) / dt
            self.vcmd_prev = cl
            self.wcmd_prev = cr
            el = (cl + TAU * al) * 100.0 / VMAX_L + uv - uw
            er = (cr + TAU * ar) * 100.0 / VMAX_R + uv + uw
            uct_mouse.set_motors(self._pwm(el, DB_L), self._pwm(er, DB_R))
            # encoder watchdog: wheels commanded but no counts at all -> stop
            if v >= 80.0:
                if self.ds == 0.0 and self.el == self.el_last and self.er == self.er_last:
                    self.dead += 1
                    if self.dead > 60:
                        uct_mouse.set_motors(0, 0)
                        raise RuntimeError("encoders not counting - check battery/encoder wiring")
                else:
                    self.dead = 0
            self.el_last = self.el
            self.er_last = self.er
        uct_mouse.delay_ms(DT_MS)
        self.sense()

    def hold(self, steps):
        n = 0
        while n < steps:
            self.step(0.0, 0.0)
            n += 1

    # Accelerometer tilt vs the resting gravity vector (board only).
    def tilted(self):
        try:
            t = uct_mouse.get_telemetry()
        except Exception:
            return 0
        if self.a0 is None:
            return 0
        dx = t[0] - self.a0[0]
        dy = t[1] - self.a0[1]
        if dx * dx + dy * dy > (TILT_LIM * self.a0[2]) ** 2:
            self.tilt_n += 1
        else:
            self.tilt_n = 0
        return 1 if self.tilt_n >= 3 else 0

    # Climbed / bumped: stop, reverse gently off the obstacle, stop.
    def back_off(self):
        log("BUMP - backing off")
        self.bump = 0
        self.tilt_n = 0
        n = 0
        while n < 35:
            self.step(-60.0, 0.0)
            n += 1
        self.settle()
        self.vc = 0.0

    # Before turning on the spot: if there is a wall ahead, creep until the
    # front ToF reads the centred distance, so the corners cannot touch while
    # spinning, and use that wall to reset the along-track position.
    def align_front(self):
        d = self.d
        n = 0
        good = 0
        seen = 0
        while n < 150:
            n += 1
            if self.aC < 6 and self.hC < 130:
                seen = 1
                e = self.hC - FRONT_CENTER
                if fabs(e) < 4.0:
                    good += 1
                    if good >= 4:
                        break
                    v = 0.0
                else:
                    good = 0
                    v = clamp(3.0 * e, -60.0, 60.0)
                    if 0.0 < v < 25.0:
                        v = 25.0
                    elif -25.0 < v < 0.0:
                        v = -25.0
            else:
                v = 0.0
                if n > 25 and not seen:
                    break
            psi = wrap180(self.th - 90.0 * d)
            self.step(v, clamp(-4.0 * psi, -40.0, 40.0))
        self.settle()
        if good >= 4:
            # axle is on the cell centre along this axis: correct the pose
            a = self.cell_along(self.cx, self.cy, d) - self.along(d)
            self.X += DX[d] * a
            self.Y += DY[d] * a

    # Stop and wait until the wheels are really still.
    def settle(self):
        n = 0
        quiet = 0
        while n < 40 and quiet < 4:
            self.step(0.0, 0.0)
            if fabs(self.v) < 5.0 and fabs(self.rate) < 8.0:
                quiet += 1
            else:
                quiet = 0
            n += 1

    def calibrate(self):
        uct_mouse.set_motors(0, 0)
        n = 0
        while n < 20:
            uct_mouse.delay_ms(DT_MS)
            n += 1
        tot = 0.0
        n = 0
        while n < CAL_SAMPLES:
            uct_mouse.delay_ms(DT_MS)
            tot += uct_mouse.get_gyro()
            n += 1
        self.bias = tot / CAL_SAMPLES + GYRO_TRIM * GYRO_GAIN
        try:
            ax = ay = az = 0.0
            n = 0
            while n < 10:
                t = uct_mouse.get_telemetry()
                ax += t[0]
                ay += t[1]
                az += t[2]
                uct_mouse.delay_ms(DT_MS)
                n += 1
            g = _sqrt(ax * ax + ay * ay + az * az) / 10.0
            if g > 0.1:
                self.a0 = (ax / 10.0, ay / 10.0, g)
        except Exception:
            self.a0 = None
        e = uct_mouse.get_encoders()
        self.el = e[0] * ENC_POL_L
        self.er = e[1] * ENC_POL_R
        self.last_ms = self.now()
        self.t0 = self.last_ms
        self.sense()
        self.th = 0.0
        self.X = CELL / 2.0
        self.Y = CELL / 2.0

    # Encoder scale (board): spin on the spot left then right open-loop. The
    # gyro gives the true rotation, each wheel travels HALF_AXLE * angle, so
    # ticks per mm = ticks / (HALF_AXLE * angle). No hand measurement needed.
    def odo_id(self):
        global TPMM
        p = int(clamp((DB_L if DB_L > DB_R else DB_R) + 14.0, 30.0, 95.0))
        tk = 0
        ang = 0.0
        ph = 0
        while ph < 2:
            sg = 1 if ph == 0 else -1
            n = 0
            while n < 55:
                if n < 40:
                    uct_mouse.set_motors(-sg * p, sg * p)
                else:
                    uct_mouse.set_motors(0, 0)
                uct_mouse.delay_ms(DT_MS)
                l0 = self.el
                r0 = self.er
                t0 = self.th
                self.sense()
                tk += abs(self.el - l0) + abs(self.er - r0)
                ang += fabs(self.th - t0)
                n += 1
            ph += 1
        uct_mouse.set_motors(0, 0)
        if tk < 20:
            raise RuntimeError("encoders not counting - check battery/encoder wiring")
        old = TPMM
        if ang > 30.0:
            TPMM = clamp(0.5 * tk / (HALF_AXLE * ang * D2R), 1.0, 20.0)
        log("ODO ticks", tk, "angle", ang, "ticks/mm", TPMM, "(was", old, ")")
        self.turn_by(wrap180(0.0 - self.th))
        self.th = 0.0
        self.X = CELL / 2.0
        self.Y = CELL / 2.0

    # Gyro sign and scale check (board, HW_GYRO_GAIN = 0): rock left then
    # right on the spot open-loop, and compare the integrated gyro with the
    # rotation the wheel encoders imply, (dR - dL) / (2 * HALF_AXLE). Firmware
    # versions differ in IMU chip, sign and scaling; this makes it not matter.
    def gyro_id(self):
        global GYRO_GAIN
        p = int(clamp((DB_L if DB_L > DB_R else DB_R) + 16.0, 30.0, 95.0))
        gs = 0.0
        es = 0.0
        gnet = 0.0
        ph = 0
        while ph < 2:
            sg = 1 if ph == 0 else -1
            n = 0
            while n < 50:
                if n < 35:
                    uct_mouse.set_motors(-sg * p, sg * p)
                else:
                    uct_mouse.set_motors(0, 0)
                uct_mouse.delay_ms(DT_MS)
                t = self.now()
                dt = (t - self.last_ms) * 0.001
                self.last_ms = t
                if dt <= 0.0 or dt > 0.25:
                    dt = DT_MS * 0.001
                e = uct_mouse.get_encoders()
                l = e[0] * ENC_POL_L
                r = e[1] * ENC_POL_R
                dl = (l - self.el) / TPMM
                dr = (r - self.er) / TPMM
                self.el = l
                self.er = r
                gr = (uct_mouse.get_gyro() - self.bias) * dt
                gnet += gr
                gs += sg * gr
                es += sg * (dr - dl) / (2.0 * HALF_AXLE) / D2R
                n += 1
            ph += 1
        uct_mouse.set_motors(0, 0)
        k = 1.94
        if fabs(es) > 20.0:
            a = fabs(gs / es)
            if 0.7 < a < 1.4:
                a = 1.0
            elif 1.6 < a < 2.5:
                a = 2.0
            if 0.3 < a < 4.0:
                k = a
            if gs * es < 0.0:
                k = -k
        GYRO_GAIN = k
        log("GYRO gain", k, "gyro", gs, "enc", es)
        self.sense()
        self.th = gnet / GYRO_GAIN * SIDE
        self.turn_by(-self.th)
        self.th = 0.0
        self.X = CELL / 2.0
        self.Y = CELL / 2.0

    # Start-up system identification (simulator profile): spin on the spot
    # at two PWM levels. Each wheel's steady tick rate is linear in PWM above
    # its dead-band, so two points give the dead-band (x-intercept) and the
    # gain; ticks counted against gyro-measured rotation give ticks per mm.
    def sysid(self):
        global DB_L, DB_R, TPMM, VMAX_L, VMAX_R
        pa = 72
        pb = 92
        rl = [0.0, 0.0]
        rr = [0.0, 0.0]
        tk = 0.0
        ang = 0.0
        ph = 0
        while ph < 2:
            p = pa if ph == 0 else pb
            n = 0
            while n < 34:
                uct_mouse.set_motors(p, -p)
                uct_mouse.delay_ms(DT_MS)
                l0 = self.el
                r0 = self.er
                th0 = self.th
                self.sense()
                if n >= 18:
                    rl[ph] += self.el - l0
                    rr[ph] += r0 - self.er
                    tk += fabs(self.el - l0) + fabs(self.er - r0)
                    ang += fabs(self.th - th0)
                n += 1
            ph += 1
        self.settle()
        if rl[1] > rl[0] + 10 and rr[1] > rr[0] + 10 and ang > 20.0:
            DB_L = clamp(pa - rl[0] * (pb - pa) / (rl[1] - rl[0]), 5.0, 80.0)
            DB_R = clamp(pa - rr[0] * (pb - pa) / (rr[1] - rr[0]), 5.0, 80.0)
            TPMM = clamp(0.5 * tk / (HALF_AXLE * ang * D2R), 2.0, 12.0)
            t = 16.0 * DT_MS * 0.001
            VMAX_L = (rl[1] - rl[0]) / (pb - pa) * (100.0 - DB_L) / TPMM / t
            VMAX_R = (rr[1] - rr[0]) / (pb - pa) * (100.0 - DB_R) / TPMM / t
        log("SYSID db", DB_L, DB_R, "tpmm", TPMM, "vmax", VMAX_L, VMAX_R)
        self.turn_by(wrap180(0.0 - self.th))
        self.X = CELL / 2.0
        self.Y = CELL / 2.0

    # Re-estimate gyro bias while parked (used during the 3 s pause).
    def zupt(self, steps):
        tot = 0.0
        n = 0
        while n < steps:
            uct_mouse.set_motors(0, 0)
            uct_mouse.delay_ms(DT_MS)
            tot += uct_mouse.get_gyro()
            n += 1
        est = tot / steps
        if fabs(est - self.bias) < 1.0:
            self.bias += 0.7 * (est - self.bias)
        self.last_ms = self.now()
        self.sense()

    # ---- geometry helpers ----------------------------------------------
    def along(self, d):
        return self.X * DX[d] + self.Y * DY[d]

    def lateral(self, d):
        return -self.X * DY[d] + self.Y * DX[d]

    def cell_along(self, x, y, d):
        return (x * CELL + CELL / 2.0) * DX[d] + (y * CELL + CELL / 2.0) * DY[d]

    def cell_lat(self, x, y, d):
        return -(x * CELL + CELL / 2.0) * DY[d] + (y * CELL + CELL / 2.0) * DX[d]

    # ---- Kalman corridor filter: state [lat (mm), psi (rad)] -------------
    def kf(self, latc):
        d = self.d
        ds = self.ds
        p11 = self.p11 + 2.0 * ds * self.p12 + ds * ds * self.p22 + 0.02 * fabs(ds)
        p12 = self.p12 + ds * self.p22
        p22 = self.p22 + 4e-8 + 2e-7 * fabs(ds)
        zs = 0.0
        nz = 0
        if self.dL < NOM_L + 35.0 and self.dL < SIDE_WALL:
            zs += NOM_L - self.dL
            nz += 1
        if self.dR < NOM_R + 35.0 and self.dR < SIDE_WALL:
            zs += self.dR - NOM_R
            nz += 1
        if nz > 0:
            z = zs / nz
            psi = wrap180(self.th - 90.0 * d) * D2R
            lat = self.lateral(d) - latc
            r = TOF_SIGMA * TOF_SIGMA / nz
            h1 = p11 + SIDE_X * p12
            h2 = p12 + SIDE_X * p22
            s = h1 + SIDE_X * h2 + r
            nu = z - (lat + SIDE_X * psi)
            if not KF_HEADING:
                # heading is trusted from the gyro: scalar update of the
                # sideways offset only
                h1 = p11
                h2 = 0.0
                s = p11 + r
            if nu * nu < 9.0 * s + 25.0:
                k1 = h1 / s
                k2 = h2 / s
                dl = k1 * nu
                self.X -= DY[d] * dl
                self.Y += DX[d] * dl
                self.th += k2 * nu / D2R
                p11 -= k1 * h1
                p12 -= k1 * h2
                p22 -= k2 * h2
        self.p11 = p11
        self.p12 = p12
        self.p22 = p22

    # Front ToF: every wall across the corridor sits on a multiple of CELL,
    # so snap the measured wall position and pull the along estimate onto it.
    def front_fix(self):
        if self.dC <= 0 or self.dC > FRONT_FIX_MAX:
            return
        if fabs(wrap180(self.th - 90.0 * self.d)) > 6.0:
            return
        d = self.d
        b = self.along(d) + FRONT_OFF + self.dC
        bs = CELL * float(round(b / CELL))
        err = bs - b
        if fabs(err) < FRONT_SNAP:
            c = FRONT_GAIN * err
            self.X += DX[d] * c
            self.Y += DY[d] * c

    # ---- in-place turn by a heading change (deg, map frame) --------------
    def turn_by(self, delta):
        self.iw = 0.0
        target = self.th + delta
        n = 0
        good = 0
        lim = int(3000 / DT_MS) + int(fabs(delta) * 4.0 / DT_MS)
        while n < lim:
            err = target - self.th
            if fabs(err) < TURN_TOL and fabs(self.rate) < 40.0:
                good += 1
                if good >= 2:
                    break
            else:
                good = 0
            mag = fabs(err)
            wr = _sqrt(2.0 * ALPHA_TURN * mag)
            if wr > W_TURN:
                wr = W_TURN
            w = wr + 6.0 * mag
            if w > W_TURN:
                w = W_TURN
            if err < 0.0:
                w = -w
            self.step(0.0, w)
            n += 1
        self.settle()
        self.p22 += 0.0005      # yaw slide when the wheels stop
        self.p11 += 25.0

    def turn_to(self, nd):
        delta = wrap180(90.0 * nd - self.th)
        if fabs(delta) < 3.0:
            self.d = nd
            return
        self.turn_by(delta)
        self.d = nd
        self.p12 = 0.0
        p = self.p11
        self.p11 = 100.0 if p < 100.0 else p

    # ---- straight driving ----------------------------------------------
    # One corridor-tracking step along axis d at commanded speed vc: lateral
    # offset -> heading demand -> heading loop -> gyro rate loop, then the
    # Kalman update from the side walls and the front-wall distance fix.
    def track(self, d, latc, vc):
        lat = self.lateral(d) - latc
        psi = wrap180(self.th - 90.0 * d) * D2R
        pref = clamp(-KLAT * lat, -PSI_MAX, PSI_MAX)
        w = clamp(KPSI * (pref - psi) / D2R, -120.0, 120.0)
        if WALL_K > 0.0:
            # classic wall following: lateral offset straight from side ToF
            # (+ = too far left), steer away from the nearer wall
            lim = SIDE_WALL - 20.0
            okL = self.aL < 8 and self.hL < lim
            okR = self.aR < 8 and self.hR < lim
            z = 0.0
            if okL and okR:
                z = 0.5 * ((self.hR - NOM_R) - (self.hL - NOM_L))
            elif okL:
                z = NOM_L - self.hL
            elif okR:
                z = self.hR - NOM_R
            w -= clamp(WALL_K * z, -60.0, 60.0)
        # SIDE REFLEX: the closer a side wall is inside SIDE_GUARD, the harder
        # the mouse turns away from it (held readings bridge ToF dropouts)
        if SIDE_GUARD > 0.0:
            if self.aR < 6 and self.hR < SIDE_GUARD:
                w += clamp(5.0 * (SIDE_GUARD - self.hR), 0.0, 120.0)
            if self.aL < 6 and self.hL < SIDE_GUARD:
                w -= clamp(5.0 * (SIDE_GUARD - self.hL), 0.0, 120.0)
        self.wlast = w
        self.step(vc, w)
        self.kf(latc)
        self.front_fix()

    # Trapezoidal profile: accelerate at ACCEL, cap at vmax, and never be
    # faster than what still allows reaching v_end at the end of `rem` mm.
    def profile(self, rem, v_end, vmax):
        if rem < 0.0:
            rem = 0.0
        vt = _sqrt(v_end * v_end + 2.0 * ACCEL * rem)
        if vt > vmax:
            vt = vmax
        vc = self.vc
        if vt > vc:
            vc += ACCEL * self.dt
            if vc > vt:
                vc = vt
        else:
            vc = vt
        if v_end == 0.0 and vc < 40.0 and rem > 3.0:
            vc = 40.0
        self.vc = vc
        return vc

    # Exploration drive from the centre of (cx, cy) along self.d. Each cell
    # it approaches is classified DECIDE_AHEAD mm before its centre; if the
    # solver wants to carry straight on, the goal is extended by a cell and
    # the mouse never stops. Ends stopped on a cell centre.
    def explore_drive(self, solver, vmax):
        d = self.d
        x0 = self.cx
        y0 = self.cy
        a0 = self.cell_along(x0, y0, d)
        latc = self.cell_lat(x0, y0, d)
        goal = 1
        k = 1                       # cell being approached (relative index)
        decided = 0
        vl = 0
        nl = 0
        vr = 0
        nr = 0
        fs = 0
        fo = 0
        tl = 0
        guard = 0
        self.vc = 0.0
        steps = 0
        while steps < 4000:
            rel = self.along(d) - a0
            s = rel + SIDE_X - CELL * k
            if -70.0 <= s <= 55.0:
                tl += 1
                # 8190 = no reading (dropout), not "open": only valid
                # readings vote, short = wall, long = open
                if self.dL < 8190:
                    nl += 1
                    if self.dL < SIDE_WALL:
                        vl += 1
                if self.dR < 8190:
                    nr += 1
                    if self.dR < SIDE_WALL:
                        vr += 1
                if self.dC < 8190:
                    if fabs(self.along(d) + FRONT_OFF + self.dC - (a0 + CELL * k + CELL / 2.0)) < 60.0:
                        fs += 1
                    else:
                        fo += 1
            if not decided and not STOP_EACH and rel >= CELL * k - DECIDE_AHEAD:
                # wall = short readings outnumber long ones AND appear in at
                # least a quarter of all samples (stray short echoes do not)
                wl = 1 if (2 * vl > nl and vl >= 2 and 4 * vl >= tl) else 0
                wr = 1 if (2 * vr > nr and vr >= 2 and 4 * vr >= tl) else 0
                wf = 1 if (fs > fo and fs >= 2 and 4 * fs >= tl) else 0
                wl, wr = self.side_check(wl, wr)
                nd = record_cell(solver, x0 + DX[d] * k, y0 + DY[d] * k, d, wl, wf, wr)
                if DEBUG:
                    log("cell", x0 + DX[d] * k, y0 + DY[d] * k, "d", d, "ToF L/C/R", self.dL, self.dC, self.dR,
                        "votes L", vl, "/", nl, "R", vr, "/", nr, "F", fs, "/", fs + fo, "-> wall L F R", wl, wf, wr, "next", nd,
                        "th", int(self.th))
                decided = 1
                if nd == d and wf == 0:
                    goal = k + 1
                    k += 1
                    decided = 0
                    vl = 0
                    nl = 0
                    vr = 0
                    nr = 0
                    fs = 0
                    fo = 0
                    tl = 0
            rem = CELL * goal - rel
            if rem <= self.v * TAU * 0.5 + 1.0:
                break
            if 0 < self.dC < 15:
                guard += 1
            else:
                guard = 0
            if self.bump:
                self.back_off()
                goal = int((self.along(d) - a0 + CELL / 2.0) / CELL)
                if goal < 0:
                    goal = 0
                solver.set_wall(x0 + DX[d] * goal, y0 + DY[d] * goal, d, 1)
                break
            if v_blocked(self) and rem > 40.0:
                guard = 3
            if guard >= 3:
                # unexpected wall right ahead: stop in the cell we are in and
                # put the wall on the map so the planner cannot send us back
                goal = int((rel + CELL / 2.0) / CELL)
                solver.set_wall(x0 + DX[d] * goal, y0 + DY[d] * goal, d, 1)
                log("FRONT GUARD")
                break
            self.track(d, latc, self.profile(rem, 0.0, vmax))
            steps += 1
        self.settle()
        self.vc = 0.0
        self.cx = x0 + DX[d] * goal
        self.cy = y0 + DY[d] * goal

    # Straight piece of a planned path: run along axis d (corridor line latc)
    # until the along-position reaches a_end, arriving at v_end. v_end = 0
    # means stop there.
    def straight(self, d, latc, a_end, v_end, vmax):
        self.d = d
        n = 0
        while n < 3000:
            rem = a_end - self.along(d)
            if v_end > 0.0:
                if rem <= 0.0:
                    break
            elif rem <= self.v * TAU * 0.5 + 1.0:
                break
            if 0 < self.dC < 15 or self.bump:
                log("FRONT GUARD")
                if self.bump:
                    self.back_off()
                break
            self.track(d, latc, self.profile(rem, v_end, vmax))
            if TEST_STRAIGHT and n % 8 == 0:
                # telemetry for the straight-line calibration run
                print("along", int(self.along(d)), "lat", int(self.lateral(d) - latc),
                      "head", int(10 * (self.th - 90.0 * d)) / 10.0, "w_cmd", int(self.wlast),
                      "rate", int(self.rate), "v", int(self.v), "ToF L/R", self.dL, self.dR)
            n += 1

    # Smooth 90 degree turn through cell (ex, ey): quarter circle of radius
    # CELL/2 from the entry-edge midpoint to the exit-edge midpoint, centred
    # on the inner corner post. Feed-forward w = v/R plus feedback on the
    # radius error and on the heading error to the circle tangent.
    def arc(self, din, dout, ex, ey, v):
        sg = 1.0 if dout == ((din + 1) & 3) else -1.0
        r0 = CELL / 2.0
        ox = ex * CELL + r0 - r0 * DX[din] + r0 * DX[dout]
        oy = ey * CELL + r0 - r0 * DY[din] + r0 * DY[dout]
        wff = sg * v / r0 / D2R
        th_end = self.th + wrap180(90.0 * dout - self.th)
        n = 0
        while n < 400:
            rx = self.X - ox
            ry = self.Y - oy
            r = _sqrt(rx * rx + ry * ry)
            if r < 1.0:
                r = 1.0
            # done once the radius points along the entry heading (exit edge
            # reached) or the heading has come round to the new axis
            if rx * DX[din] + ry * DY[din] >= 0.995 * r:
                break
            if sg * (self.th - th_end) > -1.0:
                break
            if sg > 0.0:
                tx = -ry / r
                ty = rx / r
            else:
                tx = ry / r
                ty = -rx / r
            a = self.th * D2R
            eh = _cos(a) * ty - _sin(a) * tx
            w = wff + sg * ARC_KR * (r - r0) + ARC_KH * eh / D2R
            self.vc = v
            self.step(v, w)
            n += 1
        self.d = dout
        self.p11 += 16.0
        self.p12 = 0.0
        self.p22 += 0.0001

    # Execute a planned list of move headings: one in-place turn to the first
    # heading, then straights merged into single profiles joined by arcs.
    def follow(self, moves, vmax, vturn):
        if not moves:
            return
        if moves[0] != self.d:
            self.align_front()
        self.turn_to(moves[0])
        self.vc = 0.0
        x = self.cx
        y = self.cy
        i = 0
        nmv = len(moves)
        while i < nmv:
            d = moves[i]
            j = i
            while j < nmv and moves[j] == d:
                j += 1
            ex = x + DX[d] * (j - i)
            ey = y + DY[d] * (j - i)
            latc = self.cell_lat(x, y, d)
            ce = self.cell_along(ex, ey, d)
            if j < nmv and moves[j] != ((d + 2) & 3):
                self.straight(d, latc, ce - CELL / 2.0, vturn, vmax)
                self.arc(d, moves[j], ex, ey, vturn)
            else:
                self.straight(d, latc, ce, 0.0, vmax)
                self.settle()
                self.vc = 0.0
                if j < nmv:
                    self.align_front()
                    self.turn_to(moves[j])
            x = ex
            y = ey
            i = j
        self.cx = x
        self.cy = y

    # Maze turned out to be on the other side of the start heading: mirror the
    # map frame. Only the straight start corridor (row 0) has been driven so
    # far, and it has walls on both sides, so the map itself is unchanged.
    def mirror(self):
        global SIDE
        SIDE = -SIDE
        self.th = -self.th
        self.Y = CELL - self.Y
        self.p12 = -self.p12
        t = self.dL
        self.dL = self.dR
        self.dR = t
        global NOM_L, NOM_R
        t = NOM_L
        NOM_L = NOM_R
        NOM_R = t
        log("maze is on the", "LEFT" if SIDE > 0 else "RIGHT", "of the start heading")

    # First side opening on the start corridor tells which side the maze is
    # on (the other side of that corridor is the arena's outer wall).
    def side_check(self, wl, wr):
        global SIDE_AUTO
        if SIDE_AUTO and (wl == 0 or wr == 0):
            SIDE_AUTO = 0
            if wl == 1 and wr == 0:
                self.mirror()
                return wr, wl
            log("maze is on the LEFT of the start heading")
        return wl, wr

    # Start cell: sit still for 0.6 s and vote on each wall from the valid
    # ToF readings (dropouts ignored). Also sets the centred side distance.
    def start_walls(self):
        return self.still_walls(60, 1)

    # Stand still at a cell centre and vote on each wall from valid ToF
    # readings (dropouts ignored). calib=1 also measures each side sensor's
    # centred reading (start cell only).
    def still_walls(self, steps, calib):
        global SIDE_NOM, NOM_L, NOM_R
        vl = nl = vr = nr = fs = fo = 0
        sl = []
        sr = []
        n = 0
        while n < steps:
            self.step(0.0, 0.0)
            if self.dL < 8190:
                nl += 1
                if self.dL < SIDE_WALL:
                    vl += 1
                    sl.append(self.dL)
            if self.dR < 8190:
                nr += 1
                if self.dR < SIDE_WALL:
                    vr += 1
                    sr.append(self.dR)
            if self.dC < 8190:
                if self.dC + FRONT_OFF < CELL / 2.0 + 60.0:
                    fs += 1
                else:
                    fo += 1
            n += 1
        wl = 1 if (2 * vl > nl and vl >= 2) else 0
        wr = 1 if (2 * vr > nr and vr >= 2) else 0
        wf = 1 if (fs > fo and fs >= 2) else 0
        if calib and wl and wr:
            # each sensor's own centred reading: VL53L0X units differ by 10-20
            # mm, and a shared value makes one-wall centring pull to one side
            # median, so stray short echoes and noise do not bias it
            sl.sort()
            sr.sort()
            # mean of the two sides: independent of where in the cell the
            # mouse was placed (an off-centre start must not become "centre")
            SIDE_NOM = clamp(0.5 * (sl[len(sl) // 2] + sr[len(sr) // 2]), 15.0, 95.0)
            NOM_L = SIDE_NOM
            NOM_R = SIDE_NOM
        return wl, wf, wr

    # Is there a wall across the corridor at along-position `b`?
    def front_wall(self, centre):
        if self.dC <= 0 or self.dC >= 8000:
            return 0
        b = self.along(self.d) + FRONT_OFF + self.dC
        if fabs(b - (centre + CELL / 2.0)) < 60.0:
            return 1
        return 0


# Front reflex is currently blocking forward motion.
def v_blocked(m):
    return FRONT_STOP > 0.0 and m.aC < 4 and m.hC < FRONT_STOP


# Record the walls sensed in cell (x, y) while heading d, mark it visited,
# check for the room, and return the heading the explorer wants next
# (-2 = stop here, this cell is in the room).
def record_cell(solver, x, y, d, wl, wf, wr):
    if solver.inside(x, y):
        solver.set_wall(x, y, (d + 1) & 3, wl)
        solver.set_wall(x, y, (d + 3) & 3, wr)
        solver.set_wall(x, y, d, wf)
        i = y * solver.cols + x
        b = (d + 2) & 3
        if not (solver.known[i] & (1 << b)):
            solver.set_wall(x, y, b, 0)
        solver.visited[i] = 1
    room = solver.check_room()
    if room is not None and (y * solver.cols + x) in room:
        return -2
    return solver.next_dir(x, y, d)


# On the board: wait for SW1 so the mouse can be placed, then 1 s to let go.
def wait_for_start():
    if IS_SIM == 1 or not HW_WAIT_BUTTON:
        return
    try:
        while uct_mouse.get_button() == 0:
            uct_mouse.delay_ms(50)
    except Exception:
        return
    uct_mouse.delay_ms(1000)


def mission():
    global SIDE_NOM
    if not uct_mouse.init():
        return
    print("S4 BUILD", BUILD)
    try:
        uct_mouse.display_text(1, "S4 " + BUILD[:3])
    except Exception:
        pass
    select_profile()
    uct_mouse.set_polarity(MOT_POL_L, MOT_POL_R)
    try:
        uct_mouse.set_encoder_polarity(1, 1)
    except Exception:
        pass
    rows = MAZE_ROWS
    cols = MAZE_COLS
    if rows <= 0 or cols <= 0:
        try:
            rc = uct_mouse.get_maze_dimensions()
            rows = int(rc[0])
            cols = int(rc[1])
        except Exception:
            rows = 4
            cols = 6
        if IS_SIM == 0:
            # board: the API cannot know which way the mouse was placed, so use
            # a square grid big enough either way (unused cells stay walled off)
            if cols > rows:
                rows = cols
            cols = rows
    solver = MazeSolver(rows, cols)
    m = Mouse()
    wait_for_start()
    m.calibrate()
    if SYSID:
        m.sysid()
    if GYRO_AUTO:
        m.gyro_id()
    elif TPMM_AUTO:
        m.odo_id()
    log("maze", rows, "x", cols, "side nom", SIDE_NOM, "start ToF L/C/R", m.dL, m.dC, m.dR,
        "slot counts", m.nv, "gyro gain", GYRO_GAIN)

    if TEST_STRAIGHT > 0:
        # calibration: drive straight, print heading every cell, stop
        m.follow([0] * TEST_STRAIGHT, V_EXPLORE, V_TURN)
        m.hold(50)
        print("STRAIGHT TEST done: heading", m.th, "deg  lateral est", m.Y - CELL / 2.0, "mm")
        print("Measure how far it ended up left/right of where it should be.")
        return
    # ---------------- Phase 1: explore ----------------
    wl, wf, wr = m.start_walls()
    wl, wr = m.side_check(wl, wr)
    log("start walls L F R", wl, wf, wr, "centred L/R", NOM_L, NOM_R)
    nd = record_cell(solver, 0, 0, 0, wl, wf, wr)
    moves_left = 6 * solver.n
    while nd >= 0 and moves_left > 0:
        moves_left -= 1
        if nd != m.d:
            m.align_front()
            m.turn_to(nd)
        m.explore_drive(solver, V_EXPLORE)
        if STOP_EACH:
            # stopped on the new cell's centre: sense standing still, then plan
            wl, wf, wr = m.still_walls(40, 0)
            wl, wr = m.side_check(wl, wr)
            record_cell(solver, m.cx, m.cy, m.d, wl, wf, wr)
            log("cell", m.cx, m.cy, "d", m.d, "walls L F R", wl, wf, wr)
        i = m.cy * solver.cols + m.cx
        room = solver.check_room()
        if room is not None and i in room:
            nd = -2
        else:
            nd = solver.next_dir(m.cx, m.cy, m.d)
        log("at", m.cx, m.cy, "d", m.d, "next", nd, "t", (m.now() - m.t0) / 1000.0)
    if solver.room is None:
        log("no room found")
    else:
        # ---------------- Phase 2: pirouette (clockwise) ----------------
        log("PIROUETTE at", m.cx, m.cy)
        m.turn_by(-360.0 * SIDE)
    # ---------------- Phase 3: return to start ----------------
    moves = solver.plan(m.cx, m.cy, m.d, [0])
    log("return", moves)
    if moves:
        m.follow(moves, V_RETURN, V_TURN)
    m.turn_to(0)
    # ---------------- Phase 4: pause, sprint ----------------
    m.zupt(int(3000 / DT_MS))
    if solver.room is not None:
        moves = solver.plan(0, 0, m.d, solver.room)
        log("sprint", moves)
        if moves:
            m.follow(moves, V_SPRINT, V_TURN)
    m.hold(30)
    uct_mouse.set_motors(0, 0)
    log("done t", (m.now() - m.t0) / 1000.0)


try:
    __name__
except NameError:
    __name__ = "__main__"

if __name__ == "__main__":
    try:
        mission()
    except Exception as e:
        try:
            uct_mouse.set_motors(0, 0)
        except Exception:
            pass
        raise e