"""lab_geometry.py  --  path generators and a trapezoidal speed profile.

Pure numpy: no ROS imports, so `03_geometry.py --dry-run` works on any laptop.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

SHAPES = ("line", "square", "rectangle", "triangle", "circle", "ellipse", "figure8", "spiral")
PLANES = ("xy", "xz", "yz")


def _polyline(vertices: np.ndarray, u: np.ndarray, closed: bool) -> np.ndarray:
    """Point on a polyline at normalised arclength u in [0, 1]."""
    v = np.asarray(vertices, dtype=float)
    if closed:
        v = np.vstack([v, v[:1]])
    seg = np.linalg.norm(np.diff(v, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    d = np.clip(u, 0.0, 1.0) * cum[-1]
    a = np.interp(d, cum, v[:, 0])
    b = np.interp(d, cum, v[:, 1])
    return np.stack([a, b], axis=1)


def local_curve(shape: str, u: np.ndarray, w: float, h: float, turns: float = 2.0,
                r_inner: float = 0.0) -> tuple[np.ndarray, bool]:
    """2-D curve (a, b) in metres for u in [0, 1].  Returns (points (N,2), closed).

    w, h are the full width / height of the shape (diameter for a circle).
    """
    u = np.asarray(u, dtype=float)
    if shape == "line":
        a = (u - 0.5) * w
        return np.stack([a, np.zeros_like(a)], axis=1), False
    if shape in ("square", "rectangle"):
        hh = w if shape == "square" else h
        verts = [(-w / 2, -hh / 2), (w / 2, -hh / 2), (w / 2, hh / 2), (-w / 2, hh / 2)]
        return _polyline(np.array(verts), u, True), True
    if shape == "triangle":
        verts = [(-w / 2, -h / 2), (w / 2, -h / 2), (0.0, h / 2)]
        return _polyline(np.array(verts), u, True), True
    if shape == "circle":
        r = w / 2
        t = 2 * np.pi * u
        return np.stack([r * np.cos(t), r * np.sin(t)], axis=1), True
    if shape == "ellipse":
        t = 2 * np.pi * u
        return np.stack([(w / 2) * np.cos(t), (h / 2) * np.sin(t)], axis=1), True
    if shape == "figure8":
        t = 2 * np.pi * u
        # Lemniscate of Gerono: width w, height h, starts at the centre.
        return np.stack([(w / 2) * np.sin(t), h * np.sin(t) * np.cos(t)], axis=1), True
    if shape == "spiral":
        r = r_inner + (w / 2 - r_inner) * u
        t = 2 * np.pi * turns * u
        return np.stack([r * np.cos(t), r * np.sin(t)], axis=1), False
    raise ValueError(f"unknown shape {shape!r}; choose from {SHAPES}")


def to_plane(ab: np.ndarray, plane: str, center) -> np.ndarray:
    """Map local 2-D coordinates into 3-D base-frame coordinates."""
    cx, cy, cz = (float(c) for c in center)
    a, b = ab[:, 0], ab[:, 1]
    zeros = np.zeros_like(a)
    if plane == "xy":
        return np.stack([cx + a, cy + b, cz + zeros], axis=1)
    if plane == "xz":
        return np.stack([cx + a, cy + zeros, cz + b], axis=1)
    if plane == "yz":
        return np.stack([cx + zeros, cy + a, cz + b], axis=1)
    raise ValueError(f"unknown plane {plane!r}; choose from {PLANES}")


@dataclass
class Path3D:
    """A densely sampled 3-D path with arclength parametrisation."""

    pts: np.ndarray      # (N, 3)
    s: np.ndarray        # (N,) cumulative arclength [m]
    closed: bool

    @property
    def length(self) -> float:
        return float(self.s[-1])

    def point(self, d: float) -> np.ndarray:
        """Position after travelling distance d (loops closed paths, ping-pongs open ones)."""
        L = self.length
        if L <= 0:
            return self.pts[0].copy()
        if self.closed:
            f = d % L
        else:
            k = int(np.floor(d / L))
            f = d - k * L
            if k % 2 == 1:
                f = L - f
        return np.array([np.interp(f, self.s, self.pts[:, i]) for i in range(3)])

    def curvature_and_corners(self, window: float = 0.01) -> tuple[float, float]:
        """(largest smooth curvature [1/m], largest sharp-corner turning angle [rad]).

        The path is resampled uniformly and the heading change over `window` metres is
        measured.  Windows turning more than 0.5 rad are treated as corners (polygon
        vertices or an extremely tight turn), the rest as smooth curvature.
        """
        L = self.length
        if L <= 0:
            return 0.0, 0.0
        n = max(int(L / 0.002), 10)
        ds = L / n
        d = np.linspace(0.0, L, n + 1)
        P = np.stack([np.interp(d, self.s, self.pts[:, i]) for i in range(3)], axis=1)
        seg = np.diff(P, axis=0)
        t = seg / np.maximum(np.linalg.norm(seg, axis=1, keepdims=True), 1e-12)
        k = max(1, int(round(window / ds)))
        if len(t) <= k:
            return 0.0, 0.0
        ang = np.arccos(np.clip(np.sum(t[:-k] * t[k:], axis=1), -1.0, 1.0))
        corner = ang > 0.5
        kappa = float(np.max(ang[~corner]) / (k * ds)) if np.any(~corner) else 0.0
        return kappa, float(np.max(ang[corner])) if np.any(corner) else 0.0


def make_path(shape: str, plane: str, center, w: float, h: float, turns: float = 2.0,
              r_inner: float = 0.0, n: int = 4000) -> Path3D:
    u = np.linspace(0.0, 1.0, n + 1)
    ab, closed = local_curve(shape, u, w, h, turns, r_inner)
    pts = to_plane(ab, plane, center)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return Path3D(pts=pts, s=s, closed=closed)


class TrapezoidProfile:
    """Distance-vs-time law: accelerate at `accel` to `v_max`, cruise, decelerate to rest."""

    def __init__(self, total: float, v_max: float, accel: float):
        if total < 0 or v_max <= 0 or accel <= 0:
            raise ValueError("total >= 0, v_max > 0 and accel > 0 required")
        self.total, self.accel = total, accel
        t_acc = v_max / accel
        d_acc = 0.5 * accel * t_acc ** 2
        if 2 * d_acc >= total:                      # triangular profile
            t_acc = np.sqrt(total / accel) if total > 0 else 0.0
            self.v_max = accel * t_acc
            self.t_cruise = 0.0
        else:
            self.v_max = v_max
            self.t_cruise = (total - 2 * d_acc) / v_max
        self.t_acc = t_acc
        self.duration = 2 * t_acc + self.t_cruise

    def dist(self, t: float) -> float:
        t = float(np.clip(t, 0.0, self.duration))
        a, ta, tc, v = self.accel, self.t_acc, self.t_cruise, self.v_max
        if t < ta:
            return 0.5 * a * t * t
        if t < ta + tc:
            return 0.5 * a * ta * ta + v * (t - ta)
        td = t - ta - tc
        return self.total - 0.5 * a * (ta - td) ** 2 if td < ta else self.total
