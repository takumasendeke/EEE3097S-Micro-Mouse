#!/usr/bin/env python3
"""Score a main.py in the physics simulator exactly like the Gradescope autograder.

Runs tools/physics_sim.py headless for each seed (in parallel, one port each),
runs the student main.py against it, and scores the trajectory with the official
tools/autograder/assignments/milestone2_maze/test_suite.evaluate_run.

  python3 devtools/harness.py workspace/final_task/main.py            # the 5 graded seeds
  python3 devtools/harness.py <main.py> --seeds 101,102,103 -j 3 -v    # -v prints the mouse's log
  python3 devtools/harness.py <main.py> --rows 8 --cols 8 --cfg devtools/cfg_nomaze.json --max-time 240

Default config = tools/simulation_config.json, which is what the Submission 4
autograder bundles (build_zip.py). Note its "maze" block overrides --rows/--cols,
so for a true 8x8 use a copy of the config without the "maze" key.
"""
import os, sys, json, time, subprocess, argparse, importlib.util, tempfile
from concurrent.futures import ThreadPoolExecutor

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
SIM = os.path.join(REPO, "tools", "physics_sim.py")
spec = importlib.util.spec_from_file_location(
    "ts", os.path.join(REPO, "tools/autograder/assignments/milestone2_maze/test_suite.py"))
ts = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ts)


def run_one(main, seed, rows, cols, imb, slip, cfg, max_time, port):
    main = os.path.abspath(main)
    tdir = tempfile.mkdtemp(prefix="mm_")
    traj = os.path.join(tdir, "traj.json")
    simlog = open(os.path.join(tdir, "sim.log"), "w")
    cmd = [sys.executable, "-u", SIM, "--headless", "--map", "random", "--imbalance", str(imb),
           "--slip", str(slip), "--json-log", traj, "--max-time", str(max_time), "--seed", str(seed),
           "--rows", str(rows), "--cols", str(cols), "--port", str(port), "--config", cfg]
    sp = subprocess.Popen(cmd, stdout=simlog, stderr=simlog, text=True)
    t0 = time.time()
    while time.time() - t0 < 20:
        if "Waiting for student" in open(os.path.join(tdir, "sim.log")).read():
            break
        time.sleep(0.05)
    env = os.environ.copy()
    env["GRADESCOPE_AUTOGRADER"] = "1"          # fast (lock-step) simulation
    env["UCT_MICROMOUSE_PORT"] = str(port)
    env["PYTHONPATH"] = os.path.join(REPO, "python")
    cp = subprocess.Popen([sys.executable, main], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          env=env, cwd=os.path.dirname(main), text=True)
    try:
        out, _ = cp.communicate(timeout=max_time + 120)
    except subprocess.TimeoutExpired:
        cp.kill()
        out, _ = cp.communicate()
    open(os.path.join(tdir, "mouse.log"), "w").write(out or "")
    try:
        sp.wait(timeout=5)
    except subprocess.TimeoutExpired:
        sp.send_signal(2)
        sp.wait(timeout=5)
    try:
        score, fb = ts.evaluate_run(traj)
        data = json.load(open(traj))
    except Exception as e:
        score, fb, data = 0.0, str(e), {}
    return dict(seed=seed, rows=rows, cols=cols, imb=imb, slip=slip, score=score, fb=fb, out=out,
                wall=time.time() - t0, simt=data.get("time", 0), crashed=data.get("crashed"),
                tdir=tdir, target=data.get("target_room"))


def short(fb):
    keep = [l.strip() for l in fb.splitlines()
            if "Bonus" in l or "FAILED" in l or "PARTIAL" in l or "Crash" in l]
    return " | ".join(keep)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("main")
    ap.add_argument("--seeds", default="42,43,44,45,48")          # the 5 Gradescope seeds
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--imb", default="0.04,0.06,-0.06,0.04,0.05")  # Gradescope values per seed
    ap.add_argument("--slip", default="0.02,0.02,0.02,0.04,0.03")
    ap.add_argument("--cfg", default=os.path.join(REPO, "tools", "simulation_config.json"))
    ap.add_argument("--max-time", type=float, default=120.0)
    ap.add_argument("-j", type=int, default=5)
    ap.add_argument("-v", action="store_true")
    ap.add_argument("--port", type=int, default=8100)
    a = ap.parse_args()
    seeds = [int(s) for s in a.seeds.split(",")]
    imbs = [float(x) for x in a.imb.split(",")]
    slips = [float(x) for x in a.slip.split(",")]
    with ThreadPoolExecutor(a.j) as ex:
        jobs = [ex.submit(run_one, a.main, s, a.rows, a.cols, imbs[i % len(imbs)],
                          slips[i % len(slips)], a.cfg, a.max_time, a.port + i)
                for i, s in enumerate(seeds)]
    tot = 0.0
    for j in jobs:
        r = j.result()
        tot += r["score"]
        print("seed %5d %dx%d imb %+.2f slip %.2f -> %5.1f  sim %5.1fs  crash=%s  %s" % (
            r["seed"], r["rows"], r["cols"], r["imb"], r["slip"], r["score"], r["simt"],
            "YES" if r["crashed"] else "no", short(r["fb"])))
        if a.v or r["score"] < 90:
            print("    " + "\n    ".join(r["out"].strip().splitlines()[-30:]))
            print("    files:", r["tdir"])
    print("MEAN %.2f over %d" % (tot / len(jobs), len(jobs)))
