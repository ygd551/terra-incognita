"""Summary of a walking benchmark run (sim/bench_walk.py) from its trials.csv. Walking and standing still are scored
apart, they are different weaknesses: per test, speed and level the share of trials that
  walked         no fall before the end of the command and >= 80 % of the commanded distance (or turn)
  fell           fell before the end of the command (at the start or while walking)
  fell standing  fell later, standing still at a zero command
  python3 sim/bench_report.py logs/bench/NAME      -> prints the table, writes summary.csv and summary.txt there
  python3 sim/bench_report.py logs/bench/A logs/bench/B ...   -> side by side per test, speed and level (no files)
(bench_walk.py runs it at the end; run it again after changing it, no simulation needed.)
"""
import csv
import os
import sys

SUCCESS = 0.8


def report(out_dir):
    rows = list(csv.DictReader(open(os.path.join(out_dir, "trials.csv"))))
    cells = {}  # (test, level, speed) -> trials, in the order of the run
    for r in rows:
        cells.setdefault((r["test"], float(r["level"]), float(r["speed"])), []).append(r)
    summary = []
    for (test, level, speed), rs in cells.items():
        n = len(rs)
        fell = [r["fall_phase"] in ("settle", "walk") for r in rs]
        walked = [not f and float(r["progress"]) >= SUCCESS for f, r in zip(fell, rs)]
        mean = lambda key: sum(float(r[key]) for r in rs) / n  # noqa: E731
        summary.append(dict(test=test, level=level, speed=speed, n=n, walked=round(sum(walked) / n, 3),
                            fell=round(sum(fell) / n, 3),
                            fell_standing=round(sum(r["fall_phase"] == "stand" for r in rs) / n, 3),
                            progress=round(mean("progress"), 3), max_tilt_deg=round(mean("max_tilt_deg"), 1),
                            thigh_contact=round(mean("thigh_contact"), 3), vel_err=round(mean("vel_err"), 3)))
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)

    info = os.path.join(out_dir, "info.txt")
    lines = [open(info).read().strip() if os.path.exists(info) else out_dir,
             "walked % (fell %) per level; then fell % when standing still afterwards, where > 0", ""]
    for test in dict.fromkeys(s["test"] for s in summary):
        for v in dict.fromkeys(s["speed"] for s in summary if s["test"] == test):
            cs = [s for s in summary if s["test"] == test and s["speed"] == v]
            bad = [s["level"] for s in cs if s["walked"] < 0.95]
            reach = [s["level"] for s in cs if not bad or s["level"] < min(bad)]
            unit = "deg" if test.startswith("slope") else "m"
            reach = f"{max(reach):g} {unit}" if reach else "-"
            lines.append(f"{test:12s} {v:4.2f} m/s  walks up to {reach} at >= 95 %")
            lines.append("    " + "  ".join(f"{s['level']:g}:{s['walked'] * 100:.0f}({s['fell'] * 100:.0f})"
                                          for s in cs))
            stand = [f"{s['level']:g}:{s['fell_standing'] * 100:.0f}" for s in cs if s["fell_standing"] > 0]
            if stand:
                lines.append("    standing still, fell %: " + "  ".join(stand))
    n = len(rows)
    lines += ["", f"all {n} trials: fell {sum(r['fall_phase'] in ('settle', 'walk') for r in rows) / n * 100:.1f} %, "
                  f"fell standing still {sum(r['fall_phase'] == 'stand' for r in rows) / n * 100:.1f} %"]
    text = "\n".join(lines)
    print(text, flush=True)
    open(os.path.join(out_dir, "summary.txt"), "w").write(text + "\n")
    if os.geteuid() == 0:  # written from the container: hand the files to the owner of logs/
        st = os.stat(os.path.join(out_dir, "..", ".."))
        for p in [out_dir] + [os.path.join(out_dir, f) for f in os.listdir(out_dir)]:
            os.chown(p, st.st_uid, st.st_gid)


def compare(dirs):
    cells = {}  # (test, level, speed) -> run -> [n, walked, fell, fell standing]
    for d in dirs:
        for r in csv.DictReader(open(os.path.join(d, "trials.csv"))):
            c = cells.setdefault((r["test"], float(r["level"]), float(r["speed"])), {}).setdefault(d, [0, 0, 0, 0])
            fell = r["fall_phase"] in ("settle", "walk")
            c[0] += 1
            c[1] += not fell and float(r["progress"]) >= SUCCESS
            c[2] += fell
            c[3] += r["fall_phase"] == "stand"
    print("walked % (fell %) [fell standing still %]:  " + "  |  ".join(os.path.basename(d.rstrip("/")) for d in dirs))
    for test in dict.fromkeys(k[0] for k in cells):
        for v in dict.fromkeys(k[2] for k in cells if k[0] == test):
            print(f"\n{test} {v:.2f} m/s")
            for (t, lev, sp), runs in cells.items():
                if (t, sp) == (test, v):
                    print(f"  {lev:>5g}:  " + "  |  ".join(
                        f"{100 * w / n:3.0f} ({100 * f / n:3.0f}) [{100 * st / n:3.0f}]" if n else "-"
                        for n, w, f, st in (runs.get(d, [0] * 4) for d in dirs)))


if __name__ == "__main__":
    if len(sys.argv) > 2:
        compare(sys.argv[1:])
    else:
        report(sys.argv[1] if len(sys.argv) > 1 else sys.exit(__doc__))
