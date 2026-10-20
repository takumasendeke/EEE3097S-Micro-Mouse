#!/usr/bin/env python3
"""Readable report of the mouse's built-in flight recorder.

  python3 tools/dump_logs.py -o flight.jsonl      # mouse plugged in, SW1 NOT pressed
  python3 devtools/flight_report.py flight.jsonl

The firmware logs motor PWM, encoders, gyro, ToF and accelerometer at 25 Hz from the
first motor command after power-on. The older board keeps only ~32 KB (about the first
15 s of a run); the newer one about 1 MB. Prints the reconstructed path (dead reckoning:
encoders + gyro), wheel speeds, ToF, and flags stalls and knocks.
"""
import json, math, sys

recs = []
bad = 0
for l in open(sys.argv[1], errors="replace"):
    l = "".join(ch for ch in l.strip() if ch >= " ")
    if not l.startswith("{"):
        continue
    try:
        recs.append(json.loads(l))
    except Exception:
        bad += 1
print("parsed", len(recs), "records, bad lines", bad)
hdr = recs[0]
print("header", hdr)
st = dict(l=0, r=0, tl=8190, tal=8190, tc=8190, tar=8190, tr=8190, le=0, re=0, g=0.0, v=0.0,
          ax=0.0, ay=0.0, az=9.8)
rows = []
t = 0
for r in recs[1:]:
    t += r.get("+t", 0)
    for k in st:
        if k in r:
            st[k] = r[k]
    rows.append((t, dict(st)))
TPM = 5709.0                    # HW_TICKS_PER_M
print("duration %.2f s" % (rows[-1][0] / 1000.0))
x = y = th = 0.0
prev = rows[0]
out = []
for (t, s) in rows[1:]:
    dt = (t - prev[0]) / 1000.0
    if dt <= 0:
        prev = (t, s)
        continue
    dl = -(s["le"] - prev[1]["le"]) / TPM * 1000     # left encoder polarity -1
    dr = (s["re"] - prev[1]["re"]) / TPM * 1000
    th += s["g"] * dt
    ds = 0.5 * (dl + dr)
    x += ds * math.cos(math.radians(th))
    y += ds * math.sin(math.radians(th))
    vl, vr = dl / dt, dr / dt
    stall = (abs(s["l"]) > 50 or abs(s["r"]) > 50) and abs(vl) < 15 and abs(vr) < 15
    acc = math.hypot(s["ax"], s["ay"])
    out.append((t, x, y, th, s, vl, vr, stall, acc))
    prev = (t, s)
step = max(1, len(out) // 120)
print("   t(s)    x mm  y mm  head | PWM L    R | v_L  v_R mm/s | gyro dps | ToF L    C    R | acc  batt")
for (t, x, y, th, s, vl, vr, stall, acc) in out[::step]:
    print("%7.2f %6.0f %5.0f %5.0f | %4d %4d | %5.0f %5.0f | %7.1f | %4d %4d %4d | %4.1f %4.2f %s" % (
        t / 1000, x, y, th, s["l"], s["r"], vl, vr, s["g"], s["tl"], s["tc"], s["tr"], acc, s["v"],
        "STALL" if stall else ""))
st_ev = [o for o in out if o[7]]
print("stall samples (PWM > 50, wheels not turning):", len(st_ev),
      "first at %.2f s" % (st_ev[0][0] / 1000) if st_ev else "")
big = sorted(out, key=lambda o: -o[8])[:5]
print("largest horizontal accel (knocks):", [(round(o[0] / 1000, 2), round(o[8], 1)) for o in big])
