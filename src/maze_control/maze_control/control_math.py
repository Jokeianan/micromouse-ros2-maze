"""ROS-independent planar control and polyline geometry (metres, radians)."""
import json
import math


DEFAULT_WAYPOINTS = '[1.0,0.0,1.0,1.0,0.0,1.0,0.0,0.0]'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def positive(name, value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def parse_waypoints(raw):
    values = json.loads(raw)
    if not isinstance(values, list) or len(values) < 2 or len(values) % 2:
        raise ValueError('waypoints_json must be [x1,y1,x2,y2,...]')
    if any(isinstance(v, bool) or not isinstance(v, (float, int))
           or not math.isfinite(v) for v in values):
        raise ValueError('waypoints must contain finite numbers')
    return list(zip(values[::2], values[1::2]))


def p_command(x, y, yaw, goal, kp_dist, kp_ang, max_v, max_w):
    dx, dy = goal[0] - x, goal[1] - y
    distance = math.hypot(dx, dy)
    error = wrap(math.atan2(dy, dx) - yaw)
    v = min(max_v, kp_dist * distance) * max(0.0, math.cos(error))
    if abs(error) > 0.04:
        v = 0.0
    w = max(-max_w, min(max_w, kp_ang * error))
    return v, w, distance


class Polyline:
    """Local projection avoids jumping to the end of a closed path."""

    def __init__(self, points):
        self.points = [points[0]]
        self.lengths = [0.0]
        for point in points[1:]:
            length = math.dist(self.points[-1], point)
            if length > 1e-9:
                self.points.append(point)
                self.lengths.append(self.lengths[-1] + length)
        self.total = self.lengths[-1]

    def point_at(self, s):
        s = max(0.0, min(self.total, s))
        for i in range(len(self.points) - 1):
            if s <= self.lengths[i + 1]:
                t = (s - self.lengths[i]) / (self.lengths[i + 1] - self.lengths[i])
                a, b = self.points[i:i + 2]
                return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        return self.points[-1]

    def project(self, x, y, progress, window):
        best_dist, best_s = float('inf'), progress
        end = min(self.total, progress + window)
        for i in range(len(self.points) - 1):
            lo, hi = max(progress, self.lengths[i]), min(end, self.lengths[i + 1])
            if lo > hi:
                continue
            a, b = self.points[i:i + 2]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = self.lengths[i + 1] - self.lengths[i]
            t = ((x - a[0]) * dx + (y - a[1]) * dy) / length ** 2
            s = max(lo, min(hi, self.lengths[i] + t * length))
            px, py = self.point_at(s)
            distance = math.hypot(x - px, y - py)
            if distance < best_dist:
                best_dist, best_s = distance, s
        return best_s
