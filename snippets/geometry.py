"""Obstacle geometry, gene sampling/mutation, nominal route, DTW, and scoring formulas.

_ROUTE_XY, _DESTINATION, and GENE_STEP are mutable state (not constants): ESGenerator
updates them via `geometry._ROUTE_XY = ...` / `geometry._DESTINATION = ...`
(module-qualified assignment, so it stays consistent for outside readers).
"""
import logging
import math
import os
import random
from typing import List, Optional

import numpy as np
from shapely.geometry import Point

from aerialist.px4.obstacle import Obstacle

from config import (
    ADAPTIVE,
    DEST_TOLERANCE,
    GENE_HIGH,
    GENE_LOW,
    MAX_POS,
    MIN_POS,
    POS_STEP_FRAC,
    POS_STEP_MAX,
    POS_STEP_MIN,
    ROUTE_JITTER,
    SAMPLE_X_MAX,
    SAMPLE_X_MIN,
    SAMPLE_Y_MAX,
    SAMPLE_Y_MIN,
    TRAJ_N,
    TRAP_JITTER,
    TRAP_SIZE_FRAC,
    TRAP_SPAN_MULT,
)

logger = logging.getLogger(__name__)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _gene_to_obstacle(gene: List[float]) -> Obstacle:
    l, w, h, x, y, r = gene
    return Obstacle.from_coordinates([l, w, h, x, y, 0.0, r])  # wants [l, w, h, x, y, z, r]


def _obstacle_corners(gene):
    """(x,y) corners of a rotated box obstacle. gene = [l,w,h,x,y,r], r in degrees."""
    l, w, _h, cx, cy, r = gene
    rad = math.radians(r)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    hl, hw = l / 2.0, w / 2.0
    local = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    corners = []
    for lx, ly in local:
        gx = cx + lx * cos_r - ly * sin_r
        gy = cy + lx * sin_r + ly * cos_r
        corners.append((gx, gy))
    return corners


def _projection_overlap(corners_a, corners_b, axis):
    """True if the projections of two polygons onto an axis overlap."""
    def project(corners):
        dots = [c[0] * axis[0] + c[1] * axis[1] for c in corners]
        return min(dots), max(dots)
    min_a, max_a = project(corners_a)
    min_b, max_b = project(corners_b)
    return not (max_a < min_b or max_b < min_a)


def _boxes_overlap(gene_a, gene_b):
    """Separating Axis Theorem for two rotated 2D boxes (top-down view)."""
    ca = _obstacle_corners(gene_a)
    cb = _obstacle_corners(gene_b)
    axes = []
    for corners in (ca, cb):
        for i in range(len(corners)):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % len(corners)]
            edge = (x2 - x1, y2 - y1)
            normal = (-edge[1], edge[0])
            length = math.hypot(*normal)
            if length > 1e-9:
                axes.append((normal[0] / length, normal[1] / length))
    for axis in axes:
        if not _projection_overlap(ca, cb, axis):
            return False
    return True


def _overlaps_any(gene, others):
    """True if 'gene' overlaps at least one of the obstacles in 'others'."""
    for g in others:
        if _boxes_overlap(gene, g):
            return True
    return False


_ROUTE_XY: Optional[List] = None       # (x,y) points of the nominal route, for adaptive sampling
_DESTINATION: Optional[tuple] = None   # last point of the nominal route, to check completion


def _load_route(case_study_file: str) -> Optional[List]:
    """(x,y) points of the mission's nominal route from its .ulg, filtered to
    the sampling y window. None if unavailable (falls back to a fixed box)."""
    candidates = [os.path.splitext(case_study_file)[0] + ".ulg"]
    try:
        import yaml
        with open(case_study_file) as f:
            cfg = yaml.safe_load(f)
        mission = cfg.get("drone", {}).get("mission_file")
        if mission:
            candidates.append(os.path.splitext(mission)[0] + ".ulg")
    except Exception:
        pass
    candidates = list(dict.fromkeys(candidates))  # yaml and mission_file may coincide
    log_path = next((c for c in candidates if os.path.exists(c)), None)
    if log_path is None:
        logger.warning(f"Nominal log not found {candidates}; falling back to fixed box SAMPLE_X/Y.")
        return None
    try:
        from aerialist.px4.trajectory import Trajectory
        traj = Trajectory.extract_from_log(log_path)
        global _DESTINATION
        if traj.positions:
            goal = traj.positions[-1]  # destination = last point of the nominal flight
            _DESTINATION = (goal.x, goal.y)
            logger.debug(f"Mission destination: (x={goal.x:.1f}, y={goal.y:.1f})")
        pts = [(p.x, p.y) for p in traj.positions
               if SAMPLE_Y_MIN <= p.y <= SAMPLE_Y_MAX]
        if len(pts) < 2:
            logger.warning("Route empty within the y window; falling back to fixed box SAMPLE_X/Y.")
            return None
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        logger.debug(
            f"Nominal route from {os.path.basename(log_path)}: {len(pts)} points, "
            f"x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}]"
        )
        return pts
    except Exception as e:
        logger.warning(f"Failed to read the route ({str(e)[:80]}); falling back to fixed box SAMPLE_X/Y.")
        return None


def _reached_destination(trajectory) -> bool:
    """True if the minimum distance from the nominal destination, over the
    whole flight, is within DEST_TOLERANCE. True also if the destination is unknown."""
    if _DESTINATION is None:
        return True
    dx, dy = _DESTINATION
    try:
        return min(math.hypot(p.x - dx, p.y - dy)
                   for p in trajectory.positions) <= DEST_TOLERANCE
    except Exception:
        return True


def _fit_center(gene: List[float]) -> List[float]:
    """Shifts the center (x,y) just enough so the WHOLE rotated box stays
    within the legal area, not just the center (half_x/half_y = half-extents
    of the rotated box's axis-aligned bounding box). If it doesn't fit in
    any orientation, centers on the area."""
    l, w, h, x, y, r = gene
    rad = math.radians(r)
    half_x = abs(l / 2.0 * math.cos(rad)) + abs(w / 2.0 * math.sin(rad))
    half_y = abs(l / 2.0 * math.sin(rad)) + abs(w / 2.0 * math.cos(rad))
    lo_x, hi_x = MIN_POS.x + half_x, MAX_POS.x - half_x
    lo_y, hi_y = MIN_POS.y + half_y, MAX_POS.y - half_y
    x = _clamp(x, lo_x, hi_x) if lo_x <= hi_x else (MIN_POS.x + MAX_POS.x) / 2.0
    y = _clamp(y, lo_y, hi_y) if lo_y <= hi_y else (MIN_POS.y + MAX_POS.y) / 2.0
    return [l, w, h, x, y, r]


def _random_gene() -> List[float]:
    """Samples a gene: in adaptive mode the center (x,y) is a point of the
    nominal route + gaussian jitter, otherwise a fixed box SAMPLE_X/Y.
    Size and rotation within the official ranges."""
    g = [random.uniform(GENE_LOW[i], GENE_HIGH[i]) for i in range(6)]
    if ADAPTIVE and _ROUTE_XY:
        rx, ry = random.choice(_ROUTE_XY)
        g[3] = _clamp(rx + random.gauss(0, ROUTE_JITTER), MIN_POS.x, MAX_POS.x)
        g[4] = _clamp(ry + random.gauss(0, ROUTE_JITTER), SAMPLE_Y_MIN, SAMPLE_Y_MAX)
    else:
        g[3] = random.uniform(SAMPLE_X_MIN, SAMPLE_X_MAX)
        g[4] = random.uniform(SAMPLE_Y_MIN, SAMPLE_Y_MAX)
    return _fit_center(g)


GENE_STEP = [GENE_HIGH[i] - GENE_LOW[i] for i in range(6)]  # per-dimension scale; _set_step_scale tightens it on (x,y)


def _set_step_scale(route_xy):
    """Tunes the position mutation step on the route's corridor: x = lateral
    width, y = longitudinal extent, within [POS_STEP_MIN, POS_STEP_MAX]. If
    the route isn't available, the legal span is kept."""
    if not route_xy:
        return
    xs = [p[0] for p in route_xy]
    ys = [p[1] for p in route_xy]
    GENE_STEP[3] = _clamp(POS_STEP_FRAC * (max(xs) - min(xs)), POS_STEP_MIN, POS_STEP_MAX)
    GENE_STEP[4] = _clamp(POS_STEP_FRAC * (max(ys) - min(ys)), POS_STEP_MIN, POS_STEP_MAX)
    logger.debug(
        f"Mutation step (std at sigma=1): x={GENE_STEP[3]:.1f}m y={GENE_STEP[4]:.1f}m "
        f"(was x={GENE_HIGH[3]-GENE_LOW[3]:.0f}m y={GENE_HIGH[4]-GENE_LOW[4]:.0f}m)"
    )


def _mutate_gene(gene: List[float], sigma: float) -> List[float]:
    """Gaussian mutation: step std = sigma * GENE_STEP[i]. _fit_center brings
    the whole rotated box back within the legal limits."""
    new = []
    for i in range(6):
        mutated = gene[i] + random.gauss(0, sigma * GENE_STEP[i])
        new.append(_clamp(mutated, GENE_LOW[i], GENE_HIGH[i]))
    return _fit_center(new)


def _point(min_dist: float) -> int:
    """Official Formula 1: score of a single simulation from the minimum
    drone-obstacle distance. <0.25->5, [0.25,1)->2, [1,1.5)->1, >=1.5->0 (no fail)."""
    if min_dist < 0.25:
        return 5
    if min_dist < 1.0:
        return 2
    if min_dist < 1.5:
        return 1
    return 0


def _ranking_score(ind: "Individual") -> float:
    """Official Formula 2, used ONLY for the delivery ranking (never in the
    search fitness): test_score = avg_point*10 / (#obstacles^2 * avg_time_min).
    Higher = better: more severe fail, fewer obstacles, shorter flight.
    avg_time is the flight duration, as a proxy for the real execution time.
    Fallback for old checkpoints: avg_point from the minimum distance's
    banding, neutral time (1.0) if the duration isn't available."""
    ap = ind.avg_point if ind.avg_point is not None else float(_point(ind.min_distance))
    n_obs = max(1, len(ind.genes))
    t = getattr(ind, "avg_flight_min", None)
    t = max(float(t), 0.1) if t else 1.0   # anti-blowup clamp on degenerate logs
    return (ap * 10.0) / (n_obs ** 2 * t)


def _resample_xy(trajectory, n: int = TRAJ_N) -> Optional[List[List[float]]]:
    """Resamples the trajectory to n (x,y) points, uniform in time: plain
    lists (JSON-safe) and a tractable O(n^2) DTW cost."""
    try:
        pts = trajectory.positions
        if not pts or len(pts) < 2:
            return None
        xs = np.array([p.x for p in pts], dtype=float)
        ys = np.array([p.y for p in pts], dtype=float)
        src = np.arange(len(pts))
        dst = np.linspace(0, len(pts) - 1, n)
        return [[round(float(x), 3), round(float(y), 3)]
                for x, y in zip(np.interp(dst, src, xs), np.interp(dst, src, ys))]
    except Exception:
        return None


def _dtw_mean(a, b) -> float:
    """DTW between two (x,y) trajectories, normalized over the alignment
    length -> mean deviation in meters."""
    A = np.asarray(a, dtype=float)
    B = np.asarray(b, dtype=float)
    n, m = len(A), len(B)
    if n == 0 or m == 0:
        return float("inf")
    cost = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)
    acc = np.full((n + 1, m + 1), np.inf)
    acc[0, 0] = 0.0
    for i in range(1, n + 1):
        ci = cost[i - 1]
        for j in range(1, m + 1):
            acc[i, j] = ci[j - 1] + min(acc[i - 1, j], acc[i, j - 1], acc[i - 1, j - 1])
    i, j, steps = n, m, 0
    while i > 0 and j > 0:          # backtracking for the alignment length
        steps += 1
        k = int(np.argmin((acc[i - 1, j - 1], acc[i - 1, j], acc[i, j - 1])))
        if k == 0:
            i, j = i - 1, j - 1
        elif k == 1:
            i -= 1
        else:
            j -= 1
    steps += i + j
    return float(acc[n, m] / max(steps, 1))


def _traj_gap(a: "Individual", b: "Individual") -> Optional[float]:
    """DTW between the trajectories of two individuals; None if one is
    missing (old checkpoint, failed sim) -> the caller falls back to IoU."""
    if not getattr(a, "traj", None) or not getattr(b, "traj", None):
        return None
    return _dtw_mean(a.traj, b.traj)


def _non_overlapping_gene(others, max_tries=20):
    """Samples a gene that doesn't overlap 'others'. None if it fails to find one."""
    for _ in range(max_tries):
        g = _random_gene()
        if not _overlaps_any(g, others):
            return g
    return None


def _trap_gene(ind: "Individual", others: List[List[float]], max_tries: int = 20) -> Optional[List[float]]:
    """'Plug' obstacle targeted from 'ind's trajectory: finds the point of
    the flight closest to the obstacles in 'others' (the gap the drone used
    to avoid them) and proposes a new gene centered there, oriented
    CROSSWISE to the local motion (blocks the gap instead of sitting next to
    it), sized on the measured gap. None if ind has no trajectory, 'others'
    is empty, or it can't find a free position within max_tries -> the
    caller falls back to _non_overlapping_gene (pure random)."""
    traj = getattr(ind, "traj", None)
    if not traj or len(traj) < 2 or not others:
        return None

    obs_geoms = [_gene_to_obstacle(g).geometry for g in others]
    best_i, best_d = None, float("inf")
    for i, (x, y) in enumerate(traj):
        d = min(geom.distance(Point(x, y)) for geom in obs_geoms)
        if d < best_d:
            best_d, best_i = d, i
    if best_i is None:
        return None
    tx, ty = traj[best_i]

    # local direction of motion (discrete tangent): the plug must be oriented
    # PERPENDICULAR, not aligned, to maximize how much it blocks the gap
    j, k = min(best_i + 1, len(traj) - 1), max(best_i - 1, 0)
    dx, dy = traj[j][0] - traj[k][0], traj[j][1] - traj[k][1]
    heading = math.degrees(math.atan2(dy, dx)) if (dx or dy) else 0.0
    perp = (heading + 90.0) % 180.0  # desired orientation, period 180

    cap_len = GENE_LOW[0] + TRAP_SIZE_FRAC * (GENE_HIGH[0] - GENE_LOW[0])
    long_side = _clamp(TRAP_SPAN_MULT * best_d, GENE_LOW[0], cap_len)

    for _ in range(max_tries):
        short_side = random.uniform(GENE_LOW[1], min(GENE_HIGH[1], long_side))
        # a box l x w rotated by r is identical to w x l rotated by r+90: so
        # 'perp' (in [0,180)) is always representable with an official r in [0,90]
        if perp <= 90.0:
            r, l, w = perp, long_side, short_side
        else:
            r, l, w = perp - 90.0, short_side, long_side
        h = random.uniform(GENE_LOW[2], GENE_HIGH[2])
        x = tx + random.gauss(0, TRAP_JITTER)
        y = ty + random.gauss(0, TRAP_JITTER)
        gene = _fit_center([l, w, h, x, y, r])
        if not _overlaps_any(gene, others):
            return gene
    return None


def _repair_overlaps(genes):
    """Removes obstacles overlapping others already accepted (always keeps the first)."""
    if not genes:
        return genes
    kept = [genes[0]]
    for g in genes[1:]:
        if not _overlaps_any(g, kept):
            kept.append(g)
    return kept
