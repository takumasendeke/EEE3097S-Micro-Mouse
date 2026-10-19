#!/usr/bin/env python3
"""Real-chassis clearance check for harness runs (the simulator only tests a 70 mm circle).

  python3 devtools/footprint.py <harness -v output>... [--front 70 --rear 35 --width 100]

The physical mouse is a 105 mm long rectangle (course spec), front ToF 65 mm ahead of
the axle, so most of it is ahead of the wheels: turning on the spot, the front
corners sweep ~85 mm round the axle. For every logged pose the rectangle is tested
against the wall segments (6 mm thick). Reports, per run, the closest approach and
what the mouse was doing then (straight / turning on the spot / moving turn).
"""
import sys, json, math, re, os, argparse

ap = argparse.ArgumentParser()
ap.add_argument("files", nargs="+")
ap.add_argument("--front", type=float, default=70.0)
ap.add_argument("--rear", type=float, default=35.0)
ap.add_argument("--width", type=float, default=100.0)
ap.add_argument("--wall", type=float, default=6.0)
a = ap.parse_args()


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def outline(x, y, th):
    # points along the rectangle edge (mm), world frame
    c, s = math.cos(th), math.sin(th)
    f, r, h = a.front, -a.rear, a.width / 2.0
    pts = []
    for k in range(9):
        u = r + (f - r) * k / 8.0
        pts += [(u, h), (u, -h)]
    for k in range(1, 8):
        v = -h + 2 * h * k / 8.0
        pts += [(f, v), (r, v)]
    return [(x + u * c - v * s, y + u * s + v * c) for u, v in pts]


def clearance(x, y, th, walls):
    best = 1e9
    for px, py in outline(x, y, th):
        for (ax, ay, bx, by) in walls:
            if abs(px - ax) > 260 and abs(px - bx) > 260:
                continue
            d = seg_dist(px, py, ax, ay, bx, by) - a.wall / 2.0
            if d < best:
                best = d
    return best


tot = []
for f in a.files:
    txt = open(f).read()
    dirs = re.findall(r"files: (\S+)", txt)
    seeds = re.findall(r"^seed\s+(\d+)", txt, re.M)
    print("==", f, "(front %g rear %g width %g)" % (a.front, a.rear, a.width))
    for sd, d in zip(seeds, dirs):
        try:
            data = json.load(open(os.path.join(d, "traj.json")))
        except Exception:
            continue
        W = [(p[0][0] * 1000, p[0][1] * 1000, p[1][0] * 1000, p[1][1] * 1000) for p in data["walls"]]
        tr = data["trajectory"]
        worst = (1e9, None)
        by = {"straight": 1e9, "pivot": 1e9, "moving-turn": 1e9}
        for k in range(1, len(tr)):
            x0, y0, th0, t0 = tr[k - 1][:4]
            x, y, th, t = tr[k][:4]
            c = clearance(x * 1000, y * 1000, th, W)
            mv = math.hypot(x - x0, y - y0) * 1000
            dth = abs((th - th0 + math.pi) % (2 * math.pi) - math.pi)
            kind = "pivot" if (mv < 2.0 and dth > 0.01) else ("moving-turn" if dth > 0.03 else "straight")
            by[kind] = min(by[kind], c)
            if c < worst[0]:
                worst = (c, (round(t, 1), kind, int(x * 1000), int(y * 1000), int(math.degrees(th))))
        hit = "HIT" if worst[0] <= 0 else ""
        print("seed %4s min clearance %6.1f mm at t %s (%s, x %d y %d th %d) | straight %5.1f pivot %5.1f moving-turn %5.1f %s" % (
            sd, worst[0], worst[1][0], worst[1][1], worst[1][2], worst[1][3], worst[1][4],
            by["straight"], by["pivot"], by["moving-turn"], hit))
        tot.append((worst[0], by))
if tot:
    n = len(tot)
    print("ALL runs %d  real-footprint hits %d  worst %.1f mm | worst straight %.1f pivot %.1f moving-turn %.1f" % (
        n, sum(1 for c, _ in tot if c <= 0), min(c for c, _ in tot),
        min(b["straight"] for _, b in tot), min(b["pivot"] for _, b in tot), min(b["moving-turn"] for _, b in tot)))
