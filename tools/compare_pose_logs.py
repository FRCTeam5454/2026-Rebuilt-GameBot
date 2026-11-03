#!/usr/bin/env python3
"""compares a pathplanner run and a bline run and says pass/fail

needs x, y (meters) and heading (degrees) columns, works with:
  - the test logs in build/bline-compare/*.csv
  - advantagescope csv export of SwerveDriveTrain/Pose (rotation is radians, use --radians)

compares by spot on the field (closest point) not by time since bline and pathplanner
dont go the same speed

  python tools/compare_pose_logs.py pathplanner.csv bline.csv
  python tools/compare_pose_logs.py --radians pp_advantagescope.csv bline_advantagescope.csv
"""
import argparse
import csv
import math
import sys

FINAL_POS_TOL_M = 0.2
FINAL_HEADING_TOL_DEG = 5.0
SAMPLE_POS_TOL_M = 0.05
SAMPLE_HEADING_TOL_DEG = 3.0


def find_column(header, *needles):
    for needle in needles:
        for name in header:
            if needle in name.lower():
                return name
    raise SystemExit(f"no column matching {needles} in {header}")


def load(path, radians):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path} is empty")
    header = rows[0].keys()
    x = find_column(header, "x_m", "/x", "x")
    y = find_column(header, "y_m", "/y", "y")
    h = find_column(header, "heading", "rotation", "theta")
    poses = []
    for r in rows:
        if not r[x] or not r[y] or not r[h]:
            continue  # advantagescope leaves blanks
        heading = float(r[h])
        poses.append((float(r[x]), float(r[y]), math.degrees(heading) if radians else heading))
    return poses


def heading_err(a, b):
    return abs((a - b + 180) % 360 - 180)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("reference", help="PathPlanner run CSV")
    parser.add_argument("candidate", help="BLine run CSV")
    parser.add_argument("--radians", action="store_true", help="heading column is in radians")
    args = parser.parse_args()

    ref = load(args.reference, args.radians)
    cand = load(args.candidate, args.radians)

    pos_sq = head_sq = pos_max = head_max = 0.0
    for c in cand:
        r = min(ref, key=lambda p: math.dist(p[:2], c[:2]))
        d, h = math.dist(r[:2], c[:2]), heading_err(r[2], c[2])
        pos_sq += d * d
        head_sq += h * h
        pos_max, head_max = max(pos_max, d), max(head_max, h)
    n = len(cand)
    final_d = math.dist(ref[-1][:2], cand[-1][:2])
    final_h = heading_err(ref[-1][2], cand[-1][2])

    final_ok = final_d <= FINAL_POS_TOL_M and final_h <= FINAL_HEADING_TOL_DEG
    sample_ok = pos_max <= SAMPLE_POS_TOL_M and head_max <= SAMPLE_HEADING_TOL_DEG
    print(f"samples: {n}")
    print(f"position RMS {math.sqrt(pos_sq / n):.3f} m, max {pos_max:.3f} m")
    print(f"heading  RMS {math.sqrt(head_sq / n):.2f} deg, max {head_max:.2f} deg")
    print(f"final pose: {final_d:.3f} m, {final_h:.2f} deg -> "
          f"{'PASS' if final_ok else 'FAIL'} (limit {FINAL_POS_TOL_M} m, {FINAL_HEADING_TOL_DEG} deg)")
    print(f"per-sample: {'PASS' if sample_ok else 'FAIL'} "
          f"(limit {SAMPLE_POS_TOL_M} m, {SAMPLE_HEADING_TOL_DEG} deg)")
    sys.exit(0 if final_ok else 1)


if __name__ == "__main__":
    main()
