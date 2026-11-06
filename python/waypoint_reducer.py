"""Pure-Python mirror of the C++ waypoint reducer (waypoint_navigation/src/waypoint_reducer.cpp).

Useful for experimenting with thresholds without ROS. Outputs match the C++ core.
"""
import math

EPS = 1e-9


def distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def turning_angle_deg(a, b, c):
    """Angle in degrees between direction a->b and b->c (0 = straight on)."""
    v1 = (b[0] - a[0], b[1] - a[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    if math.hypot(*v1) < EPS or math.hypot(*v2) < EPS:
        return 0.0
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    return math.degrees(math.atan2(abs(cross), dot))


def perpendicular_distance(p, a, b):
    """Distance from p to the infinite line through a and b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < EPS:
        return distance(p, a)
    return abs(dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / length


def angle_filter(path, angle_threshold_deg=5.0, mode="cumulative", lookahead=0.25):
    """mode="local": original per-step angle. mode="cumulative": direction since the last
    kept point vs direction over the next `lookahead` metres (robust to planner jitter)."""
    if len(path) <= 2:
        return list(path)
    out, ahead, i = [path[0]], 1, 1
    while i < len(path) - 1:
        if mode == "local":
            if turning_angle_deg(path[i - 1], path[i], path[i + 1]) >= angle_threshold_deg:
                out.append(path[i])
        elif distance(out[-1], path[i]) >= lookahead:
            ahead = max(ahead, i + 1)
            while ahead + 1 < len(path) and distance(path[i], path[ahead]) < lookahead:
                ahead += 1
            if turning_angle_deg(out[-1], path[i], path[ahead]) >= angle_threshold_deg:
                # keep the actual corner: the point in [i, ahead) furthest from the local chord
                behind = i
                while behind > 0 and distance(path[behind], path[i]) < lookahead:
                    behind -= 1
                i = max(range(i, ahead), key=lambda k: perpendicular_distance(path[k], path[behind], path[ahead]))
                out.append(path[i])
        i += 1
    if distance(out[-1], path[-1]) >= EPS:
        out.append(path[-1])
    return out


def distance_filter_with_recovery(path, min_distance=1.0, epsilon=0.3):
    if len(path) <= 2:
        return list(path)
    out, rejected = [path[0]], []
    for i in range(1, len(path)):
        is_goal = i == len(path) - 1
        if distance(out[-1], path[i]) >= min_distance or is_goal:
            # corner recovery: restore the dropped point that deviates most (if >= epsilon)
            best, best_pt = -1.0, None
            for r in rejected:
                d = perpendicular_distance(r, out[-1], path[i])
                if d >= epsilon and d > best:
                    best, best_pt = d, r
            if best_pt is not None:
                out.append(best_pt)
            rejected = []
            if distance(out[-1], path[i]) >= EPS:
                out.append(path[i])
        else:
            rejected.append(path[i])
    return out


def reduce_path(path, angle_threshold_deg=5.0, min_distance=1.0, epsilon=0.3, mode="cumulative",
                lookahead=0.25):
    return distance_filter_with_recovery(
        angle_filter(path, angle_threshold_deg, mode, lookahead), min_distance, epsilon
    )
