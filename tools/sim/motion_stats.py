#!/usr/bin/env python3
"""Motion quality from harness runs: straight-line wobble and turn behaviour.

  python3 devtools/motion_stats.py <harness output file>...

Reads the "files: <dir>" lines the harness prints with -v, then for each run:
  * lateral offset from the corridor centre line while driving straight
    (true pose from the simulator's traj.json): RMS and max, mm
  * min clearance from the 70 mm collision circle to any wall, mm
  * the mouse's own "turn" / "align" debug lines: overshoot and time per turn
"""
import sys, json, math, re, os

CELL = 0.2


def walls_clearance(traj, walls, r=0.070):
    best = 1e9
    for p in traj[::3]:
        x, y = p[0], p[1]
        for (a, b) in walls:
            ax, ay = a
            bx, by = b
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = math.hypot(x - ax - t * dx, y - ay - t * dy)
            if d < best:
                best = d
    return (best - r) * 1000.0


def lateral_stats(traj):
    lat = []
    for k in range(1, len(traj)):
        x0, y0, th0 = traj[k - 1][:3]
        x1, y1, th1 = traj[k][:3]
        if math.hypot(x1 - x0, y1 - y0) < 0.004:
            continue                     # not moving
        q = round(th1 / (math.pi / 2))
        if abs(th1 - q * math.pi / 2) > math.radians(8):
            continue                     # turning
        perp = y1 if q % 2 == 0 else x1  # coordinate across the corridor
        c = (math.floor(perp / CELL) + 0.5) * CELL
        lat.append((perp - c) * 1000.0)
    if not lat:
        return 0.0, 0.0
    rms = math.sqrt(sum(v * v for v in lat) / len(lat))
    return rms, max(abs(v) for v in lat)


def main():
    for f in sys.argv[1:]:
        txt = open(f).read()
        dirs = re.findall(r"files: (\S+)", txt)
        seeds = re.findall(r"^seed\s+(\d+)", txt, re.M)
        print("==", f)
        agg = []
        for sd, d in zip(seeds, dirs):
            try:
                data = json.load(open(os.path.join(d, "traj.json")))
            except Exception:
                continue
            traj = data.get("trajectory") or data.get("path") or []
            walls = data.get("walls") or []
            rms, mx = lateral_stats(traj)
            clr = walls_clearance(traj, walls) if walls else float("nan")
            log = open(os.path.join(d, "mouse.log")).read() if os.path.exists(os.path.join(d, "mouse.log")) else ""
            ov = [float(v) for v in re.findall(r"^turn .* ovs (-?[\d.]+)", log, re.M)]
            tt = [float(v) for v in re.findall(r"^turn .* ms (\d+)", log, re.M)]
            al = [float(v) for v in re.findall(r"^align .* ms (\d+)", log, re.M)]
            ng = len(re.findall(r"FRONT GUARD|BUMP", log))
            print("seed %4s lat rms %4.1f max %4.1f mm  clearance %5.1f mm  turns %2d ovs max %4.1f avg %4.1f deg  "
                  "t avg %4.0f ms  align avg %4.0f ms  guards %d" % (
                      sd, rms, mx, clr, len(ov), max(ov) if ov else 0, sum(ov) / len(ov) if ov else 0,
                      sum(tt) / len(tt) if tt else 0, sum(al) / len(al) if al else 0, ng))
            agg.append((rms, mx, clr))
        if agg:
            print("ALL lat rms %.1f  worst max %.1f mm  worst clearance %.1f mm" % (
                sum(a[0] for a in agg) / len(agg), max(a[1] for a in agg), min(a[2] for a in agg)))


if __name__ == "__main__":
    main()
