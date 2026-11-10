#!/usr/bin/env python3
"""turns pathplanner .path files into bline path json files

bline only drives straight lines between points, pathplanner drives bezier curves, so this
chops each curve up into points about every 0.25m so bline basically traces the curve.

what carries over:
  - all the pathplanner anchors (exact spots)
  - start rotation, rotation targets and end rotation (as bline waypoints)
  - max vel/accel and angular vel/accel
  - event markers (as bline event triggers, same name)

bline doesnt slow down for curves by itself so we also cap the speed at each point the same
way pathplanner does (sqrt(maxAccel * curve radius)) and make sure theres room to brake before
the tight parts

run it from the repo root:
  python tools/pp_to_bline.py "Double sweep out=DoubleSweepTrenchIO_out" ...

each arg is "<pathplanner name>=<bline name>", leave off the =... to keep the same name.
files go in src/main/deploy/autos/paths/
"""
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PP_PATHS = os.path.join(REPO, "src", "main", "deploy", "pathplanner", "paths")
PP_SETTINGS = os.path.join(REPO, "src", "main", "deploy", "pathplanner", "settings.json")
BLINE_PATHS = os.path.join(REPO, "src", "main", "deploy", "autos", "paths")

# distance between points along the curve (meters)
SAMPLE_SPACING_M = 0.25
# how close we get to a point before going to the next one, keep it smaller than the spacing
# or the robot cuts the corners
HANDOFF_RADIUS_M = 0.12
# how close to the end counts as done. pathplanner doesnt have this, it just stops when time
# runs out. making these bigger makes the auto faster
END_TRANSLATION_TOLERANCE_M = 0.03
END_ROTATION_TOLERANCE_DEG = 2.0


def bezier(p0, p1, p2, p3, t):
    u = 1 - t
    return (
        u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0],
        u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1],
    )


def bezier_curvature(p0, p1, p2, p3, t):
    u = 1 - t
    dx = 3 * u**2 * (p1[0] - p0[0]) + 6 * u * t * (p2[0] - p1[0]) + 3 * t**2 * (p3[0] - p2[0])
    dy = 3 * u**2 * (p1[1] - p0[1]) + 6 * u * t * (p2[1] - p1[1]) + 3 * t**2 * (p3[1] - p2[1])
    ddx = 6 * u * (p2[0] - 2 * p1[0] + p0[0]) + 6 * t * (p3[0] - 2 * p2[0] + p1[0])
    ddy = 6 * u * (p2[1] - 2 * p1[1] + p0[1]) + 6 * t * (p3[1] - 2 * p2[1] + p1[1])
    speed = math.hypot(dx, dy)
    return 0.0 if speed < 1e-9 else abs(dx * ddy - dy * ddx) / speed**3


def pt(d):
    return (d["x"], d["y"])


def segment_controls(waypoints, i):
    a, b = waypoints[i], waypoints[i + 1]
    return pt(a["anchor"]), pt(a["nextControl"]), pt(b["prevControl"]), pt(b["anchor"])


def arc_length_ts(ctrl, spacing):
    # t values spaced evenly by distance along the curve, 0 to 1
    n = 400
    pts = [bezier(*ctrl, k / n) for k in range(n + 1)]
    cum = [0.0]
    for k in range(1, n + 1):
        cum.append(cum[-1] + math.dist(pts[k - 1], pts[k]))
    total = cum[-1]
    count = max(1, round(total / spacing))
    ts = []
    k = 0
    for j in range(count + 1):
        s = total * j / count
        while k < n and cum[k + 1] < s:
            k += 1
        if k >= n:
            ts.append(1.0)
            continue
        span = cum[k + 1] - cum[k]
        frac = 0.0 if span == 0 else (s - cum[k]) / span
        ts.append((k + frac) / n)
    ts[0], ts[-1] = 0.0, 1.0
    return ts


def global_constraints(path):
    if path.get("useDefaultConstraints", False):
        with open(PP_SETTINGS) as f:
            s = json.load(f)
        return s["defaultMaxVel"], s["defaultMaxAccel"], s["defaultMaxAngVel"], s["defaultMaxAngAccel"]
    g = path["globalConstraints"]
    return g["maxVelocity"], g["maxAcceleration"], g["maxAngularVelocity"], g["maxAngularAcceleration"]


def convert(pp_name, bline_name):
    with open(os.path.join(PP_PATHS, pp_name + ".path")) as f:
        path = json.load(f)
    if path.get("constraintZones"):
        print(f"warning: {pp_name} has constraint zones; they are not converted", file=sys.stderr)
    if path.get("pointTowardsZones"):
        print(f"warning: {pp_name} has point-towards zones; they are not converted", file=sys.stderr)

    waypoints = path["waypoints"]
    num_segments = len(waypoints) - 1

    rotations = {}  # waypointRelativePos -> radians
    for rt in path.get("rotationTargets", []):
        rotations[rt["waypointRelativePos"]] = math.radians(rt["rotationDegrees"])
    events = sorted((m["waypointRelativePos"], m["name"]) for m in path.get("eventMarkers", []))

    # sample spots as waypointRelativePos (segment index + bezier t)
    positions = []
    for seg in range(num_segments):
        ctrl = segment_controls(waypoints, seg)
        ts = arc_length_ts(ctrl, SAMPLE_SPACING_M)
        positions += [seg + t for t in (ts if seg == 0 else ts[1:])]
    for pos in rotations:
        # move the closest point onto the rotation target so it lands exactly there
        nearest = min(range(1, len(positions) - 1), key=lambda k: abs(positions[k] - pos))
        positions[nearest] = pos
    positions = sorted(set(positions))

    def point_at(pos):
        seg = min(int(pos), num_segments - 1)
        return bezier(*segment_controls(waypoints, seg), pos - seg)

    max_v, max_a, max_w, max_alpha = global_constraints(path)

    def curvature_at(pos):
        seg = min(int(pos), num_segments - 1)
        return bezier_curvature(*segment_controls(waypoints, seg), pos - seg)

    # speed cap at each point from the tightest bit of curve around it (same as pathplanner)
    speed_caps = []
    for k, pos in enumerate(positions):
        lo, hi = positions[max(0, k - 1)], positions[min(len(positions) - 1, k + 1)]
        kappa = max(curvature_at(lo + (hi - lo) * j / 20) for j in range(21))
        speed_caps.append(max_v if kappa < 1e-9 else min(max_v, math.sqrt(max_a / kappa)))
    # go backwards so we can always brake in time for the next cap. dont cap the very end,
    # bline slows down for the last point on its own
    for k in range(len(positions) - 2, -1, -1):
        leg = math.dist(point_at(positions[k]), point_at(positions[k + 1]))
        speed_caps[k] = min(speed_caps[k], math.sqrt(speed_caps[k + 1] ** 2 + 2 * max_a * leg))

    elements = []
    start_rot = math.radians(path["idealStartingState"]["rotation"])
    end_rot = math.radians(path["goalEndState"]["rotation"])
    event_idx = 0
    for k, pos in enumerate(positions):
        # put event markers on the line going into this point, same spot along it
        while k > 0 and event_idx < len(events) and events[event_idx][0] <= pos:
            prev = positions[k - 1]
            t_ratio = 0.0 if pos == prev else (events[event_idx][0] - prev) / (pos - prev)
            elements.append({"type": "event_trigger", "t_ratio": round(t_ratio, 4),
                             "lib_key": events[event_idx][1]})
            event_idx += 1

        x, y = point_at(pos)
        translation = {"x_meters": round(x, 4), "y_meters": round(y, 4)}
        rot = None
        if k == 0:
            rot = start_rot
        elif k == len(positions) - 1:
            rot = end_rot
        elif pos in rotations:
            rot = rotations[pos]

        if k < len(positions) - 1:
            translation["intermediate_handoff_radius_meters"] = HANDOFF_RADIUS_M
        if rot is None:
            elements.append({"type": "translation", **translation})
        else:
            elements.append({
                "type": "waypoint",
                "translation_target": translation,
                "rotation_target": {
                    "rotation_radians": round(rot, 6),
                    "t_ratio": 0.0 if k == 0 else 1.0,
                    "profiled_rotation": True,
                },
            })

    last = len(elements) - 1
    # one speed cap per point, bline uses it while driving toward that point
    velocity_caps = [{"value": round(v, 3), "start_ordinal": k, "end_ordinal": k}
                     for k, v in enumerate(speed_caps)]
    out = {
        "_comment": f"made by tools/pp_to_bline.py from pathplanner/paths/{pp_name}.path, "
                    "rerun the script instead of editing this",
        "path_elements": elements,
        "constraints": {
            "max_velocity_meters_per_sec": velocity_caps,
            "max_acceleration_meters_per_sec2": [{"value": max_a, "start_ordinal": 0, "end_ordinal": last}],
            "max_velocity_deg_per_sec": [{"value": max_w, "start_ordinal": 0, "end_ordinal": last}],
            "max_acceleration_deg_per_sec2": [{"value": max_alpha, "start_ordinal": 0, "end_ordinal": last}],
            "end_translation_tolerance_meters": END_TRANSLATION_TOLERANCE_M,
            "end_rotation_tolerance_deg": END_ROTATION_TOLERANCE_DEG,
        },
    }
    os.makedirs(BLINE_PATHS, exist_ok=True)
    dest = os.path.join(BLINE_PATHS, bline_name + ".json")
    with open(dest, "w", newline="\n") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    print(f"{pp_name}.path -> {os.path.relpath(dest, REPO)} ({len(positions)} targets, "
          f"slowest cap {min(speed_caps):.2f} m/s)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for arg in sys.argv[1:]:
        pp, _, bl = arg.partition("=")
        convert(pp, bl or pp)
